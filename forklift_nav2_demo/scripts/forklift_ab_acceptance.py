#!/usr/bin/python3

import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from forklift_msgs.msg import ForkliftControlCommand
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import DeleteEntity, SetEntityState, SpawnEntity
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


SCENARIOS = {
    "forward_ab": {
        "initial": (-2.0, -0.5, 0.0),
        "goal": (1.2, -0.5, 0.0),
        "timeout_sec": 120.0,
        "min_forward_samples": 1,
        "max_reverse_samples": 0,
        "min_final_x": 0.75,
    },
    "sparse_90_turn_ab": {
        "initial": (-2.0, -0.5, 0.0),
        "goal": (-1.3, 0.2, math.pi * 0.5),
        "timeout_sec": 120.0,
        "min_forward_samples": 1,
        "max_reverse_samples": 0,
    },
    "reverse_ab": {
        "initial": (-2.0, -0.5, 0.0),
        "goal": (-2.35, -0.5, 0.0),
        "timeout_sec": 90.0,
        "min_reverse_samples": 1,
        "expect_negative_sim_cmd": True,
        "max_forward_samples": 4,
    },
    "dynamic_stop_release_ab": {
        "initial": (-2.0, -0.5, 0.0),
        "goal": (1.2, -0.5, 0.0),
        "timeout_sec": 140.0,
        "dynamic_obstacle": True,
        "spawn_delay_sec": 1.2,
        "spawn_trigger_odom_x": -1.55,
        "spawn_trigger_timeout_sec": 15.0,
        "blocked_observation_sec": 6.0,
        "release_settle_sec": 4.0,
        "obstacle": (-0.6, -0.5, 0.5),
        "obstacle_size": (0.45, 1.0, 1.0),
        "min_forward_samples": 1,
        "max_reverse_samples": 0,
        "require_blocked_zero_samples": True,
        "require_after_release_motion": True,
        "clear_costmaps_after_release": True,
        "reissue_goal_after_release": True,
        "min_final_x": 0.75,
    },
}


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


class AbAcceptance:
    def __init__(self):
        self.node = rclpy.create_node("forklift_ab_acceptance")
        self.set_use_sim_time()
        self.node.declare_parameter("scenario", "forward_ab")
        self.node.declare_parameter("action_name", "/navigate_to_pose")
        self.node.declare_parameter("robot_entity_name", "forklift")
        self.node.declare_parameter("reset_entity_pose", True)
        self.node.declare_parameter("publish_initial_pose", True)
        self.node.declare_parameter("initial_pose_publish_interval_sec", 1.0)
        self.node.declare_parameter("initial_pose_wait_sec", 30.0)
        self.node.declare_parameter("initial_pose_settle_sec", 2.0)
        self.node.declare_parameter("moving_velocity_threshold", 0.03)
        self.node.declare_parameter("zero_velocity_threshold", 0.01)
        self.node.declare_parameter("obstacle_name", "forklift_ab_acceptance_obstacle")
        self.node.declare_parameter("clear_costmaps_after_release", False)

        self.scenario_name = self.node.get_parameter("scenario").value
        self.action_name = self.node.get_parameter("action_name").value
        self.robot_entity_name = self.node.get_parameter("robot_entity_name").value
        self.reset_entity_pose_enabled = bool(
            self.node.get_parameter("reset_entity_pose").value)
        self.publish_initial_pose_enabled = bool(
            self.node.get_parameter("publish_initial_pose").value)
        self.initial_pose_publish_interval_sec = float(
            self.node.get_parameter("initial_pose_publish_interval_sec").value)
        self.initial_pose_wait_sec = float(
            self.node.get_parameter("initial_pose_wait_sec").value)
        self.initial_pose_settle_sec = float(
            self.node.get_parameter("initial_pose_settle_sec").value)
        self.moving_velocity_threshold = float(
            self.node.get_parameter("moving_velocity_threshold").value)
        self.zero_velocity_threshold = float(
            self.node.get_parameter("zero_velocity_threshold").value)
        self.obstacle_name = self.node.get_parameter("obstacle_name").value
        self.clear_costmaps_after_release = bool(
            self.node.get_parameter("clear_costmaps_after_release").value)

        self.phase = "normal"
        self.feedback_count = 0
        self.control_samples = 0
        self.sim_cmd_samples = 0
        self.forward_control_samples = 0
        self.reverse_control_samples = 0
        self.max_control_velocity = 0.0
        self.min_signed_sim_linear_x = 0.0
        self.max_signed_sim_linear_x = 0.0
        self.last_odom = None
        self.phase_max_control_velocity = {
            "normal": 0.0,
            "before_obstacle": 0.0,
            "blocked": 0.0,
            "after_release": 0.0,
        }
        self.phase_zero_control_samples = {
            "normal": 0,
            "before_obstacle": 0,
            "blocked": 0,
            "after_release": 0,
        }
        self.phase_max_sim_linear_x = {
            "normal": 0.0,
            "before_obstacle": 0.0,
            "blocked": 0.0,
            "after_release": 0.0,
        }

        self.node.create_subscription(
            ForkliftControlCommand, "/forklift/control_cmd", self.on_control_cmd, 10)
        self.node.create_subscription(Twist, "/forklift/sim_cmd_vel", self.on_sim_cmd, 10)
        self.node.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.initial_pose_pub = self.node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)

        self.action_client = ActionClient(self.node, NavigateToPose, self.action_name)
        self.set_state_client = self.node.create_client(SetEntityState, "/set_entity_state")
        self.spawn_client = self.node.create_client(SpawnEntity, "/spawn_entity")
        self.delete_client = self.node.create_client(DeleteEntity, "/delete_entity")
        self.clear_global_client = self.node.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap")
        self.clear_local_client = self.node.create_client(
            ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def set_use_sim_time(self):
        try:
            self.node.declare_parameter("use_sim_time", True)
        except ParameterAlreadyDeclaredException:
            self.node.set_parameters([
                Parameter("use_sim_time", Parameter.Type.BOOL, True)
            ])

    def on_control_cmd(self, msg):
        self.control_samples += 1
        self.max_control_velocity = max(self.max_control_velocity, abs(msg.velocity_mps))
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
        self.min_signed_sim_linear_x = min(self.min_signed_sim_linear_x, msg.linear.x)
        self.max_signed_sim_linear_x = max(self.max_signed_sim_linear_x, msg.linear.x)
        self.phase_max_sim_linear_x[self.phase] = max(
            self.phase_max_sim_linear_x[self.phase], abs(msg.linear.x))

    def on_odom(self, msg):
        self.last_odom = msg

    def run(self):
        scenario = SCENARIOS.get(self.scenario_name)
        if scenario is None:
            self.node.get_logger().error(
                "Unknown scenario '{}'; valid scenarios: {}".format(
                    self.scenario_name, ", ".join(sorted(SCENARIOS))))
            return 2

        initial = scenario["initial"]
        goal = scenario["goal"]
        self.node.get_logger().info("A-B acceptance scenario: {}".format(self.scenario_name))
        self.delete_obstacle()
        if self.reset_entity_pose_enabled and not self.reset_entity_pose(initial):
            return 3
        if self.publish_initial_pose_enabled and not self.publish_initial_pose(initial):
            return 4
        if not self.action_client.wait_for_server(timeout_sec=15.0):
            self.node.get_logger().error("{} action server is not available".format(
                self.action_name))
            return 5

        if scenario.get("dynamic_obstacle", False):
            result = self.run_dynamic_scenario(scenario, goal)
        else:
            result = self.run_plain_scenario(scenario, goal)
        self.print_summary(result)

        if result is None:
            self.node.get_logger().error("{} timed out".format(self.action_name))
            return 6
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            return 7
        return self.validate_scenario(scenario)

    def run_plain_scenario(self, scenario, goal):
        goal_handle = self.send_goal(goal)
        if goal_handle is None:
            return None
        return self.wait_for_result(goal_handle.get_result_async(), scenario["timeout_sec"])

    def run_dynamic_scenario(self, scenario, goal):
        if not self.wait_for_gazebo_services():
            return None
        goal_handle = self.send_goal(goal)
        if goal_handle is None:
            return None
        result_future = goal_handle.get_result_async()

        self.phase = "before_obstacle"
        if "spawn_trigger_odom_x" in scenario:
            self.spin_until_odom_x(
                float(scenario["spawn_trigger_odom_x"]),
                float(scenario.get("spawn_trigger_timeout_sec", 15.0)))
        else:
            self.spin_for(float(scenario.get("spawn_delay_sec", 1.2)))
        if result_future.done():
            self.node.get_logger().error("goal finished before obstacle was spawned")
            return result_future.result()
        if not self.spawn_obstacle(scenario):
            return None

        self.phase = "blocked"
        self.spin_for(float(scenario.get("blocked_observation_sec", 6.0)))
        self.delete_obstacle()
        if self.clear_costmaps_after_release or scenario.get("clear_costmaps_after_release", False):
            self.clear_costmaps()

        self.phase = "after_release"
        if scenario.get("reissue_goal_after_release", False):
            self.node.get_logger().info("canceling and reissuing goal after release")
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=5.0)
            goal_handle = self.send_goal(goal)
            if goal_handle is None:
                return None
            result_future = goal_handle.get_result_async()
        result = self.wait_for_result(result_future, scenario["timeout_sec"])
        self.spin_for(float(scenario.get("release_settle_sec", 4.0)))
        return result

    def send_goal(self, goal_spec):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_spec[0]
        goal.pose.pose.position.y = goal_spec[1]
        goal.pose.pose.orientation = yaw_to_quaternion(goal_spec[2])
        self.node.get_logger().info(
            "NavigateToPose goal: ({:.2f},{:.2f},{:.2f})".format(*goal_spec))
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

    def reset_entity_pose(self, initial):
        if not self.set_state_client.wait_for_service(timeout_sec=10.0):
            self.node.get_logger().error("/set_entity_state service is not available")
            return False
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = self.robot_entity_name
        request.state.reference_frame = "world"
        request.state.pose.position.x = initial[0]
        request.state.pose.position.y = initial[1]
        request.state.pose.position.z = 0.05
        request.state.pose.orientation = yaw_to_quaternion(initial[2])
        request.state.twist.linear.x = 0.0
        request.state.twist.linear.y = 0.0
        request.state.twist.angular.z = 0.0
        future = self.set_state_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        response = future.result()
        if response is None:
            self.node.get_logger().error("reset entity pose timed out")
            return False
        if not response.success:
            self.node.get_logger().error(
                "reset entity pose failed: {}".format(response.status_message))
            return False
        self.node.get_logger().info(
            "reset {} to ({:.2f},{:.2f},{:.2f})".format(
                self.robot_entity_name, initial[0], initial[1], initial[2]))
        self.spin_for(0.5)
        return True

    def publish_initial_pose(self, initial):
        self.node.get_logger().info(
            "Publishing initial pose ({:.2f},{:.2f},{:.2f})".format(*initial))
        wait_deadline = time.monotonic() + max(0.1, self.initial_pose_wait_sec)
        next_publish_time = 0.0
        published_once = False
        first_publish_time = None
        while time.monotonic() < wait_deadline and rclpy.ok():
            settled = (
                first_publish_time is not None and
                time.monotonic() - first_publish_time >= max(0.0, self.initial_pose_settle_sec))
            if published_once and settled and self.tf_buffer.can_transform(
                    "map", "base_link", Time(), timeout=Duration(seconds=0.1)):
                self.node.get_logger().info("Initial pose accepted; map->base_link is available")
                return True

            now = time.monotonic()
            if now >= next_publish_time:
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = "map"
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.pose.pose.position.x = initial[0]
                msg.pose.pose.position.y = initial[1]
                msg.pose.pose.orientation = yaw_to_quaternion(initial[2])
                msg.pose.covariance[0] = 0.25
                msg.pose.covariance[7] = 0.25
                msg.pose.covariance[35] = 0.0685
                self.initial_pose_pub.publish(msg)
                published_once = True
                if first_publish_time is None:
                    first_publish_time = now
                next_publish_time = now + max(0.2, self.initial_pose_publish_interval_sec)

            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.get_logger().error("Timed out waiting for map->base_link after initial pose")
        return False

    def wait_for_gazebo_services(self):
        for client, name in [
            (self.spawn_client, "/spawn_entity"),
            (self.delete_client, "/delete_entity"),
        ]:
            if not client.wait_for_service(timeout_sec=10.0):
                self.node.get_logger().error("{} service is not available".format(name))
                return False
        return True

    def spawn_obstacle(self, scenario):
        obstacle = scenario["obstacle"]
        size = scenario["obstacle_size"]
        request = SpawnEntity.Request()
        request.name = self.obstacle_name
        request.xml = make_box_sdf(self.obstacle_name, size[0], size[1], size[2])
        request.robot_namespace = ""
        request.reference_frame = "world"
        request.initial_pose = Pose()
        request.initial_pose.position.x = obstacle[0]
        request.initial_pose.position.y = obstacle[1]
        request.initial_pose.position.z = obstacle[2]
        request.initial_pose.orientation.w = 1.0
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
                self.obstacle_name, obstacle[0], obstacle[1]))
        return True

    def delete_obstacle(self):
        if not self.delete_client.wait_for_service(timeout_sec=1.0):
            return False
        request = DeleteEntity.Request()
        request.name = self.obstacle_name
        future = self.delete_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        return future.result() is not None

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

    def spin_until_odom_x(self, target_x, timeout_sec):
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.last_odom is not None:
                if self.last_odom.pose.pose.position.x >= target_x:
                    self.node.get_logger().info(
                        "spawn trigger reached: odom_x={:.3f} target_x={:.3f}".format(
                            self.last_odom.pose.pose.position.x, target_x))
                    return True
        self.node.get_logger().warning(
            "spawn trigger timed out waiting for odom_x >= {:.3f}".format(target_x))
        return False

    def wait_for_result(self, result_future, timeout_sec):
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if result_future.done():
                return result_future.result()
        return None

    def validate_scenario(self, scenario):
        errors = []
        if self.control_samples == 0 or self.sim_cmd_samples == 0:
            errors.append("bridge/controller command samples were not observed")
        if self.forward_control_samples < int(scenario.get("min_forward_samples", 0)):
            errors.append(
                "forward samples {} below min {}".format(
                    self.forward_control_samples, scenario.get("min_forward_samples")))
        if self.reverse_control_samples < int(scenario.get("min_reverse_samples", 0)):
            errors.append(
                "reverse samples {} below min {}".format(
                    self.reverse_control_samples, scenario.get("min_reverse_samples")))
        max_reverse = scenario.get("max_reverse_samples")
        if max_reverse is not None and self.reverse_control_samples > int(max_reverse):
            errors.append(
                "reverse samples {} exceed max {}".format(
                    self.reverse_control_samples, max_reverse))
        max_forward = scenario.get("max_forward_samples")
        if max_forward is not None and self.forward_control_samples > int(max_forward):
            errors.append(
                "forward samples {} exceed max {}".format(
                    self.forward_control_samples, max_forward))
        if scenario.get("expect_negative_sim_cmd", False) and self.min_signed_sim_linear_x >= -0.01:
            errors.append("negative sim cmd velocity was not observed")
        if scenario.get("require_blocked_zero_samples", False):
            if self.phase_zero_control_samples["blocked"] == 0:
                errors.append("zero/brake control samples while blocked were not observed")
        if scenario.get("require_after_release_motion", False):
            if self.phase_max_sim_linear_x["after_release"] < self.moving_velocity_threshold:
                errors.append("movement after obstacle release was not observed")
        min_final_x = scenario.get("min_final_x")
        if min_final_x is not None and self.last_odom is not None:
            if self.last_odom.pose.pose.position.x < float(min_final_x):
                errors.append(
                    "final odom x {:.3f} below min {:.3f}".format(
                        self.last_odom.pose.pose.position.x, float(min_final_x)))

        if errors:
            for error in errors:
                self.node.get_logger().error(error)
            return 8
        self.node.get_logger().info("ab_acceptance=PASS scenario={}".format(self.scenario_name))
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
            "sim_cmd min_signed_linear_x={:.3f} max_signed_linear_x={:.3f}".format(
                self.min_signed_sim_linear_x, self.max_signed_sim_linear_x))
        self.node.get_logger().info(
            "phase_control_max normal={:.3f} before={:.3f} blocked={:.3f} after={:.3f}".format(
                self.phase_max_control_velocity["normal"],
                self.phase_max_control_velocity["before_obstacle"],
                self.phase_max_control_velocity["blocked"],
                self.phase_max_control_velocity["after_release"],
            ))
        self.node.get_logger().info(
            "phase_control_zero normal={} before={} blocked={} after={}".format(
                self.phase_zero_control_samples["normal"],
                self.phase_zero_control_samples["before_obstacle"],
                self.phase_zero_control_samples["blocked"],
                self.phase_zero_control_samples["after_release"],
            ))
        self.node.get_logger().info(
            "phase_sim_max normal={:.3f} before={:.3f} blocked={:.3f} after={:.3f}".format(
                self.phase_max_sim_linear_x["normal"],
                self.phase_max_sim_linear_x["before_obstacle"],
                self.phase_max_sim_linear_x["blocked"],
                self.phase_max_sim_linear_x["after_release"],
            ))
        if self.last_odom is not None:
            position = self.last_odom.pose.pose.position
            self.node.get_logger().info(
                "odom_final x={:.3f} y={:.3f}".format(position.x, position.y))


def main():
    rclpy.init()
    runner = AbAcceptance()
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
