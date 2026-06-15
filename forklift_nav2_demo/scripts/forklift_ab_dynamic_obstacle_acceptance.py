#!/usr/bin/python3

import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from forklift_msgs.msg import ForkliftControlCommand
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.parameter import Parameter
from rclpy.time import Time
import tf2_ros


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def yaw_to_quaternion(yaw):
    quat = Quaternion()
    quat.z = math.sin(yaw * 0.5)
    quat.w = math.cos(yaw * 0.5)
    return quat


def make_box_sdf(name, sx, sy, sz):
    return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>{sx:.3f} {sy:.3f} {sz:.3f}</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{sx:.3f} {sy:.3f} {sz:.3f}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.9 0.1 0.1 1.0</ambient>
          <diffuse>0.9 0.1 0.1 1.0</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""".format(name=name, sx=sx, sy=sy, sz=sz)


class DynamicObstacleAcceptance:
    def __init__(self):
        self.node = rclpy.create_node("forklift_ab_dynamic_obstacle_acceptance")
        self.set_use_sim_time()
        self.node.declare_parameter("action_name", "/navigate_to_pose")
        self.node.declare_parameter("timeout_sec", 120.0)
        self.node.declare_parameter("spawn_delay_sec", 1.2)
        self.node.declare_parameter("blocked_observation_sec", 6.0)
        self.node.declare_parameter("release_settle_sec", 4.0)
        self.node.declare_parameter("clear_costmaps_after_release", False)
        self.node.declare_parameter("reissue_goal_after_release", False)
        self.node.declare_parameter("goal_x", 1.20)
        self.node.declare_parameter("goal_y", -0.50)
        self.node.declare_parameter("goal_yaw", 0.0)
        self.node.declare_parameter("obstacle_name", "forklift_dynamic_obstacle")
        self.node.declare_parameter("obstacle_x", -0.60)
        self.node.declare_parameter("obstacle_y", -0.50)
        self.node.declare_parameter("obstacle_z", 0.50)
        self.node.declare_parameter("obstacle_size_x", 0.45)
        self.node.declare_parameter("obstacle_size_y", 1.00)
        self.node.declare_parameter("obstacle_size_z", 1.00)
        self.node.declare_parameter("moving_velocity_threshold", 0.03)
        self.node.declare_parameter("zero_velocity_threshold", 0.01)
        self.node.declare_parameter("publish_initial_pose", True)
        self.node.declare_parameter("initial_x", -2.0)
        self.node.declare_parameter("initial_y", -0.5)
        self.node.declare_parameter("initial_yaw", 0.0)
        self.node.declare_parameter("initial_pose_publish_interval_sec", 1.0)
        self.node.declare_parameter("initial_pose_wait_sec", 30.0)

        self.action_name = self.node.get_parameter("action_name").value
        self.timeout_sec = float(self.node.get_parameter("timeout_sec").value)
        self.spawn_delay_sec = float(self.node.get_parameter("spawn_delay_sec").value)
        self.blocked_observation_sec = float(
            self.node.get_parameter("blocked_observation_sec").value)
        self.release_settle_sec = float(self.node.get_parameter("release_settle_sec").value)
        self.clear_costmaps_after_release = bool(
            self.node.get_parameter("clear_costmaps_after_release").value)
        self.reissue_goal_after_release = bool(
            self.node.get_parameter("reissue_goal_after_release").value)
        self.goal_x = float(self.node.get_parameter("goal_x").value)
        self.goal_y = float(self.node.get_parameter("goal_y").value)
        self.goal_yaw = float(self.node.get_parameter("goal_yaw").value)
        self.obstacle_name = self.node.get_parameter("obstacle_name").value
        self.obstacle_x = float(self.node.get_parameter("obstacle_x").value)
        self.obstacle_y = float(self.node.get_parameter("obstacle_y").value)
        self.obstacle_z = float(self.node.get_parameter("obstacle_z").value)
        self.obstacle_size_x = float(self.node.get_parameter("obstacle_size_x").value)
        self.obstacle_size_y = float(self.node.get_parameter("obstacle_size_y").value)
        self.obstacle_size_z = float(self.node.get_parameter("obstacle_size_z").value)
        self.moving_velocity_threshold = float(
            self.node.get_parameter("moving_velocity_threshold").value)
        self.zero_velocity_threshold = float(
            self.node.get_parameter("zero_velocity_threshold").value)
        self.publish_initial_pose_enabled = bool(
            self.node.get_parameter("publish_initial_pose").value)
        self.initial_x = float(self.node.get_parameter("initial_x").value)
        self.initial_y = float(self.node.get_parameter("initial_y").value)
        self.initial_yaw = float(self.node.get_parameter("initial_yaw").value)
        self.initial_pose_publish_interval_sec = float(
            self.node.get_parameter("initial_pose_publish_interval_sec").value)
        self.initial_pose_wait_sec = float(
            self.node.get_parameter("initial_pose_wait_sec").value)

        self.phase = "before_obstacle"
        self.feedback_count = 0
        self.control_samples = 0
        self.sim_cmd_samples = 0
        self.forward_control_samples = 0
        self.reverse_control_samples = 0
        self.last_odom = None
        self.phase_max_control_velocity = {
            "before_obstacle": 0.0,
            "blocked": 0.0,
            "after_release": 0.0,
        }
        self.phase_zero_control_samples = {
            "before_obstacle": 0,
            "blocked": 0,
            "after_release": 0,
        }
        self.phase_max_sim_linear_x = {
            "before_obstacle": 0.0,
            "blocked": 0.0,
            "after_release": 0.0,
        }
        self.phase_zero_sim_samples = {
            "before_obstacle": 0,
            "blocked": 0,
            "after_release": 0,
        }

        self.node.create_subscription(
            ForkliftControlCommand, "/forklift/control_cmd", self.on_control_cmd, 10)
        self.node.create_subscription(Twist, "/forklift/sim_cmd_vel", self.on_sim_cmd, 10)
        self.node.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.initial_pose_pub = self.node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        self.action_client = ActionClient(self.node, NavigateToPose, self.action_name)
        self.spawn_client = self.node.create_client(SpawnEntity, "/spawn_entity")
        self.delete_client = self.node.create_client(DeleteEntity, "/delete_entity")
        self.clear_global_client = self.node.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap")
        self.clear_local_client = self.node.create_client(
            ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap")

    def set_use_sim_time(self):
        try:
            self.node.declare_parameter("use_sim_time", True)
        except ParameterAlreadyDeclaredException:
            self.node.set_parameters([
                Parameter("use_sim_time", Parameter.Type.BOOL, True)
            ])

    def on_control_cmd(self, msg):
        self.control_samples += 1
        self.phase_max_control_velocity[self.phase] = max(
            self.phase_max_control_velocity[self.phase], abs(msg.velocity_mps))
        if abs(msg.velocity_mps) <= self.zero_velocity_threshold:
            self.phase_zero_control_samples[self.phase] += 1
        if msg.forward and not msg.reverse:
            self.forward_control_samples += 1
        if msg.reverse and not msg.forward:
            self.reverse_control_samples += 1

    def on_sim_cmd(self, msg):
        self.sim_cmd_samples += 1
        self.phase_max_sim_linear_x[self.phase] = max(
            self.phase_max_sim_linear_x[self.phase], abs(msg.linear.x))
        if abs(msg.linear.x) <= self.zero_velocity_threshold:
            self.phase_zero_sim_samples[self.phase] += 1

    def on_odom(self, msg):
        self.last_odom = msg

    def make_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.goal_x
        goal.pose.pose.position.y = self.goal_y
        goal.pose.pose.orientation = yaw_to_quaternion(self.goal_yaw)
        return goal

    def send_goal(self):
        goal = self.make_goal()
        self.node.get_logger().info(
            "NavigateToPose goal: ({:.2f},{:.2f},{:.2f})".format(
                self.goal_x, self.goal_y, self.goal_yaw))
        send_future = self.action_client.send_goal_async(
            goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("{} rejected the goal".format(self.action_name))
            return None
        self.node.get_logger().info("{} accepted: true".format(self.action_name))
        return goal_handle

    def on_feedback(self, _feedback_msg):
        self.feedback_count += 1

    def wait_for_services(self):
        services = [
            (self.spawn_client, "/spawn_entity"),
            (self.delete_client, "/delete_entity"),
        ]
        for client, name in services:
            if not client.wait_for_service(timeout_sec=10.0):
                self.node.get_logger().error("{} service is not available".format(name))
                return False
        return True

    def spawn_obstacle(self):
        request = SpawnEntity.Request()
        request.name = self.obstacle_name
        request.xml = make_box_sdf(
            self.obstacle_name,
            self.obstacle_size_x,
            self.obstacle_size_y,
            self.obstacle_size_z,
        )
        request.robot_namespace = ""
        request.reference_frame = "world"
        pose = Pose()
        pose.position.x = self.obstacle_x
        pose.position.y = self.obstacle_y
        pose.position.z = self.obstacle_z
        pose.orientation.w = 1.0
        request.initial_pose = pose
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        response = future.result()
        if response is None:
            self.node.get_logger().error("spawn obstacle timed out")
            return False
        if not response.success:
            self.node.get_logger().error(
                "spawn obstacle failed: {}".format(response.status_message))
            return False
        self.node.get_logger().info(
            "spawned obstacle {} at ({:.2f},{:.2f})".format(
                self.obstacle_name, self.obstacle_x, self.obstacle_y))
        return True

    def delete_obstacle(self):
        request = DeleteEntity.Request()
        request.name = self.obstacle_name
        future = self.delete_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        response = future.result()
        if response is None:
            self.node.get_logger().warning("delete obstacle timed out")
            return False
        if not response.success:
            self.node.get_logger().warning(
                "delete obstacle: {}".format(response.status_message))
            return False
        self.node.get_logger().info("deleted obstacle {}".format(self.obstacle_name))
        return True

    def clear_costmaps(self):
        for client, name in [
            (self.clear_global_client, "global"),
            (self.clear_local_client, "local"),
        ]:
            if not client.wait_for_service(timeout_sec=2.0):
                self.node.get_logger().warning("{} costmap clear service unavailable".format(name))
                continue
            future = client.call_async(ClearEntireCostmap.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            if future.result() is None:
                self.node.get_logger().warning("{} costmap clear timed out".format(name))
            else:
                self.node.get_logger().info("{} costmap cleared".format(name))

    def spin_for(self, duration_sec):
        deadline = time.monotonic() + max(0.0, duration_sec)
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def wait_for_result(self, result_future, timeout_sec):
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if result_future.done():
                return result_future.result()
        return None

    def publish_initial_pose(self):
        self.node.get_logger().info(
            "Publishing initial pose ({:.2f},{:.2f},{:.2f})".format(
                self.initial_x, self.initial_y, self.initial_yaw))
        wait_deadline = time.monotonic() + max(0.1, self.initial_pose_wait_sec)
        next_publish_time = 0.0
        while time.monotonic() < wait_deadline and rclpy.ok():
            if self.tf_buffer.can_transform(
                    "map", "base_link", Time(), timeout=Duration(seconds=0.1)):
                self.node.get_logger().info("Initial pose accepted; map->base_link is available")
                return True

            now = time.monotonic()
            if now >= next_publish_time:
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = "map"
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.pose.pose.position.x = self.initial_x
                msg.pose.pose.position.y = self.initial_y
                msg.pose.pose.orientation = yaw_to_quaternion(self.initial_yaw)
                msg.pose.covariance[0] = 0.25
                msg.pose.covariance[7] = 0.25
                msg.pose.covariance[35] = 0.0685
                self.initial_pose_pub.publish(msg)
                next_publish_time = now + max(0.2, self.initial_pose_publish_interval_sec)

            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.get_logger().error("Timed out waiting for map->base_link after initial pose")
        return False

    def run(self):
        if not self.wait_for_services():
            return 2
        self.delete_obstacle()
        if self.publish_initial_pose_enabled and not self.publish_initial_pose():
            return 14
        if not self.action_client.wait_for_server(timeout_sec=15.0):
            self.node.get_logger().error("{} action server is not available".format(
                self.action_name))
            return 3

        goal_handle = self.send_goal()
        if goal_handle is None:
            return 4
        result_future = goal_handle.get_result_async()

        self.phase = "before_obstacle"
        self.spin_for(self.spawn_delay_sec)
        if result_future.done():
            self.node.get_logger().error("goal finished before obstacle was spawned")
            return 5
        if not self.spawn_obstacle():
            return 6

        self.phase = "blocked"
        self.spin_for(self.blocked_observation_sec)
        self.delete_obstacle()
        if self.clear_costmaps_after_release:
            self.clear_costmaps()

        self.phase = "after_release"
        if self.reissue_goal_after_release:
            self.node.get_logger().info("canceling and reissuing goal after release")
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=5.0)
            goal_handle = self.send_goal()
            if goal_handle is None:
                return 7
            result_future = goal_handle.get_result_async()

        result = self.wait_for_result(
            result_future, max(0.0, self.timeout_sec - self.spawn_delay_sec))
        self.spin_for(self.release_settle_sec)
        self.print_summary(result)

        if result is None:
            self.node.get_logger().error("{} timed out".format(self.action_name))
            return 8
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            return 9
        if self.control_samples == 0 or self.sim_cmd_samples == 0:
            self.node.get_logger().error("bridge/controller command samples were not observed")
            return 10
        if self.phase_max_control_velocity["before_obstacle"] < self.moving_velocity_threshold:
            self.node.get_logger().error("movement before obstacle was not observed")
            return 11
        if self.phase_zero_control_samples["blocked"] == 0:
            self.node.get_logger().error("zero/brake control samples while blocked were not observed")
            return 12
        if self.phase_max_sim_linear_x["after_release"] < self.moving_velocity_threshold:
            self.node.get_logger().error("movement after obstacle release was not observed")
            return 13

        self.node.get_logger().info("dynamic_obstacle_acceptance=PASS")
        return 0

    def print_summary(self, result):
        if result is None:
            self.node.get_logger().info("{} status: TIMEOUT".format(self.action_name))
        else:
            status_name = STATUS_NAMES.get(result.status, "STATUS_{}".format(result.status))
            self.node.get_logger().info(
                "{} status: {} {}".format(self.action_name, result.status, status_name))
        self.node.get_logger().info("feedback_count={}".format(self.feedback_count))
        self.node.get_logger().info(
            "control_samples={} sim_cmd_samples={} forward={} reverse={}".format(
                self.control_samples,
                self.sim_cmd_samples,
                self.forward_control_samples,
                self.reverse_control_samples,
            ))
        self.node.get_logger().info(
            "phase_control_max before={:.3f} blocked={:.3f} after={:.3f}".format(
                self.phase_max_control_velocity["before_obstacle"],
                self.phase_max_control_velocity["blocked"],
                self.phase_max_control_velocity["after_release"],
            ))
        self.node.get_logger().info(
            "phase_control_zero before={} blocked={} after={}".format(
                self.phase_zero_control_samples["before_obstacle"],
                self.phase_zero_control_samples["blocked"],
                self.phase_zero_control_samples["after_release"],
            ))
        self.node.get_logger().info(
            "phase_sim_max before={:.3f} blocked={:.3f} after={:.3f}".format(
                self.phase_max_sim_linear_x["before_obstacle"],
                self.phase_max_sim_linear_x["blocked"],
                self.phase_max_sim_linear_x["after_release"],
            ))
        self.node.get_logger().info(
            "phase_sim_zero before={} blocked={} after={}".format(
                self.phase_zero_sim_samples["before_obstacle"],
                self.phase_zero_sim_samples["blocked"],
                self.phase_zero_sim_samples["after_release"],
            ))
        if self.last_odom is not None:
            position = self.last_odom.pose.pose.position
            self.node.get_logger().info(
                "odom_final x={:.3f} y={:.3f}".format(position.x, position.y))


def main():
    rclpy.init()
    runner = DynamicObstacleAcceptance()
    try:
        return_code = runner.run()
    finally:
        runner.delete_obstacle()
        runner.action_client.destroy()
        runner.node.destroy_node()
        rclpy.shutdown()
    sys.exit(return_code)


if __name__ == "__main__":
    main()
