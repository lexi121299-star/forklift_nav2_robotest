#!/usr/bin/python3

import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from forklift_msgs.msg import ForkliftControlCommand
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
import tf2_ros


SCENARIOS = {
    "forward_straight": {
        "goal": (-0.90, -0.50, 0.0),
        "run_follow_path": True,
        "max_planner_reverse_segments": 0,
        "max_controller_reverse_samples": 0,
        "min_controller_forward_samples": 1,
    },
    "forward_with_goal_heading": {
        "goal": (-0.90, -0.50, 0.75),
        "run_follow_path": False,
        "max_planner_reverse_segments": 0,
        "max_planner_gear_switches": 0,
    },
    "sparse_90_turn": {
        "goal": (-1.30, 0.20, math.pi * 0.5),
        "run_follow_path": True,
        "max_planner_reverse_segments": 0,
        "max_controller_reverse_samples": 0,
        "min_controller_forward_samples": 1,
    },
    "reverse_straight": {
        "goal": (-2.35, -0.50, 0.0),
        "run_follow_path": True,
        "min_planner_reverse_segments": 1,
        "min_controller_reverse_samples": 1,
        "expect_negative_sim_cmd": True,
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


def quaternion_to_yaw(quat):
    return math.atan2(
        2.0 * (quat.w * quat.z + quat.x * quat.y),
        1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))


def shortest_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def make_goal_pose(node, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation = yaw_to_quaternion(yaw)
    return pose


def infer_path_direction(path):
    forward_segments = 0
    reverse_segments = 0
    gear_switches = 0
    heading_jumps = 0
    previous_direction = 0
    max_heading_delta = 0.0

    poses = path.poses
    for index in range(len(poses) - 1):
        current = poses[index].pose
        next_pose = poses[index + 1].pose
        dx = next_pose.position.x - current.position.x
        dy = next_pose.position.y - current.position.y
        distance = math.hypot(dx, dy)
        if distance < 1e-4:
            continue

        tangent = math.atan2(dy, dx)
        yaw = quaternion_to_yaw(current.orientation)
        heading_error = shortest_angle(tangent - yaw)
        direction = -1 if abs(heading_error) > math.pi * 0.5 else 1
        if direction > 0:
            forward_segments += 1
        else:
            reverse_segments += 1

        if previous_direction and previous_direction != direction:
            gear_switches += 1
        previous_direction = direction

        next_yaw = quaternion_to_yaw(next_pose.orientation)
        yaw_delta = abs(shortest_angle(next_yaw - yaw))
        max_heading_delta = max(max_heading_delta, yaw_delta)
        if yaw_delta > math.radians(60.0):
            heading_jumps += 1

    return {
        "forward_segments": forward_segments,
        "reverse_segments": reverse_segments,
        "gear_switches": gear_switches,
        "heading_jumps": heading_jumps,
        "max_heading_delta": max_heading_delta,
    }


def clear_path_stamps(path):
    path.header.stamp.sec = 0
    path.header.stamp.nanosec = 0
    for pose in path.poses:
        pose.header.stamp.sec = 0
        pose.header.stamp.nanosec = 0
    return path


class P6ReverseAcceptance:
    def __init__(self):
        self.node = rclpy.create_node("forklift_p6_reverse_acceptance")
        self.node.declare_parameter("scenario", "forward_straight")
        self.node.declare_parameter("planner_id", "GridBased")
        self.node.declare_parameter("compute_path_action_name", "/compute_path_to_pose")
        self.node.declare_parameter("follow_path_action_name", "/follow_path")
        self.node.declare_parameter("controller_id", "FollowPath")
        self.node.declare_parameter("goal_checker_id", "general_goal_checker")
        self.node.declare_parameter("compute_timeout_sec", 30.0)
        self.node.declare_parameter("follow_timeout_sec", 90.0)
        self.node.declare_parameter("post_result_observation_sec", 1.0)
        self.node.declare_parameter("require_follow_path", True)
        self.node.declare_parameter("publish_initial_pose", True)
        self.node.declare_parameter("initial_x", -2.0)
        self.node.declare_parameter("initial_y", -0.5)
        self.node.declare_parameter("initial_yaw", 0.0)
        self.node.declare_parameter("initial_pose_publish_interval_sec", 1.0)
        self.node.declare_parameter("initial_pose_wait_sec", 15.0)

        self.scenario = self.node.get_parameter("scenario").value
        self.planner_id = self.node.get_parameter("planner_id").value
        self.compute_path_action_name = self.node.get_parameter("compute_path_action_name").value
        self.follow_path_action_name = self.node.get_parameter("follow_path_action_name").value
        self.controller_id = self.node.get_parameter("controller_id").value
        self.goal_checker_id = self.node.get_parameter("goal_checker_id").value
        self.compute_timeout_sec = float(self.node.get_parameter("compute_timeout_sec").value)
        self.follow_timeout_sec = float(self.node.get_parameter("follow_timeout_sec").value)
        self.post_result_observation_sec = float(
            self.node.get_parameter("post_result_observation_sec").value)
        self.require_follow_path = bool(self.node.get_parameter("require_follow_path").value)
        self.publish_initial_pose_enabled = bool(
            self.node.get_parameter("publish_initial_pose").value)
        self.initial_x = float(self.node.get_parameter("initial_x").value)
        self.initial_y = float(self.node.get_parameter("initial_y").value)
        self.initial_yaw = float(self.node.get_parameter("initial_yaw").value)
        self.initial_pose_publish_interval_sec = float(
            self.node.get_parameter("initial_pose_publish_interval_sec").value)
        self.initial_pose_wait_sec = float(
            self.node.get_parameter("initial_pose_wait_sec").value)

        self.control_samples = 0
        self.forward_control_samples = 0
        self.reverse_control_samples = 0
        self.max_control_velocity = 0.0
        self.sim_cmd_samples = 0
        self.min_signed_sim_linear_x = 0.0
        self.max_signed_sim_linear_x = 0.0
        self.max_sim_linear_x = 0.0
        self.last_odom = None

        self.node.create_subscription(
            ForkliftControlCommand, "/forklift/control_cmd", self.on_control_cmd, 10)
        self.node.create_subscription(Twist, "/forklift/sim_cmd_vel", self.on_sim_cmd, 10)
        self.node.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.initial_pose_pub = self.node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)

        self.compute_client = ActionClient(
            self.node, ComputePathToPose, self.compute_path_action_name)
        self.follow_client = ActionClient(
            self.node, FollowPath, self.follow_path_action_name)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def on_control_cmd(self, msg):
        self.control_samples += 1
        self.max_control_velocity = max(self.max_control_velocity, abs(msg.velocity_mps))
        if msg.forward and not msg.reverse:
            self.forward_control_samples += 1
        if msg.reverse and not msg.forward:
            self.reverse_control_samples += 1

    def on_sim_cmd(self, msg):
        self.sim_cmd_samples += 1
        self.max_sim_linear_x = max(self.max_sim_linear_x, abs(msg.linear.x))
        self.min_signed_sim_linear_x = min(self.min_signed_sim_linear_x, msg.linear.x)
        self.max_signed_sim_linear_x = max(self.max_signed_sim_linear_x, msg.linear.x)

    def on_odom(self, msg):
        self.last_odom = msg

    def run(self):
        scenario_spec = SCENARIOS.get(self.scenario)
        if scenario_spec is None:
            self.node.get_logger().error(
                "Unknown scenario '{}'; valid scenarios: {}".format(
                    self.scenario, ", ".join(sorted(SCENARIOS))))
            return 2

        if self.publish_initial_pose_enabled and not self.publish_initial_pose():
            return 9

        path = self.compute_path(scenario_spec)
        if path is None:
            return 3

        summary = infer_path_direction(path)
        self.log_path_summary(path, summary)
        validation_code = self.validate_planner_summary(scenario_spec, summary)
        if validation_code != 0:
            return validation_code

        if bool(scenario_spec.get("run_follow_path", False)):
            path = clear_path_stamps(path)
            follow_code = self.follow_path(path)
            if follow_code != 0:
                return follow_code
            self.spin_for_observation()
            self.log_command_summary()
            return self.validate_follow_summary(scenario_spec)

        self.node.get_logger().info(
            "follow_path skipped for planner-only scenario '{}'".format(self.scenario))
        return 0

    def compute_path(self, scenario_spec):
        if not self.compute_client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().error(
                "{} action server is not available".format(self.compute_path_action_name))
            return None

        x, y, yaw = scenario_spec["goal"]
        goal = ComputePathToPose.Goal()
        goal.pose = make_goal_pose(self.node, x, y, yaw)
        goal.planner_id = self.planner_id
        self.node.get_logger().info(
            "ComputePathToPose scenario={} goal=({:.2f},{:.2f},{:.2f})".format(
                self.scenario, x, y, yaw))

        send_future = self.compute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("{} rejected the goal".format(
                self.compute_path_action_name))
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node, result_future, timeout_sec=self.compute_timeout_sec)
        result = result_future.result()
        if result is None:
            self.node.get_logger().error("{} timed out".format(self.compute_path_action_name))
            return None

        status_name = STATUS_NAMES.get(result.status, "STATUS_{}".format(result.status))
        self.node.get_logger().info(
            "{} status: {} {}".format(
                self.compute_path_action_name, result.status, status_name))
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            return None
        if not result.result.path.poses:
            self.node.get_logger().error("planner returned an empty path")
            return None
        return result.result.path

    def publish_initial_pose(self):
        self.node.get_logger().info(
            "Publishing initial pose ({:.2f},{:.2f},{:.2f})".format(
                self.initial_x, self.initial_y, self.initial_yaw))
        wait_deadline = time.monotonic() + max(0.1, self.initial_pose_wait_sec)
        next_publish_time = 0.0
        while time.monotonic() < wait_deadline:
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

    def follow_path(self, path):
        if not self.follow_client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().error(
                "{} action server is not available".format(self.follow_path_action_name))
            return 10 if self.require_follow_path else 0

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.controller_id
        if hasattr(goal, "goal_checker_id"):
            goal.goal_checker_id = self.goal_checker_id

        send_future = self.follow_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("{} rejected the goal".format(
                self.follow_path_action_name))
            return 11

        self.node.get_logger().info("{} accepted: true".format(self.follow_path_action_name))
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node, result_future, timeout_sec=self.follow_timeout_sec)
        result = result_future.result()
        if result is None:
            self.node.get_logger().error("{} timed out".format(self.follow_path_action_name))
            return 12

        status_name = STATUS_NAMES.get(result.status, "STATUS_{}".format(result.status))
        self.node.get_logger().info(
            "{} status: {} {}".format(
                self.follow_path_action_name, result.status, status_name))
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            return 13
        return 0

    def validate_planner_summary(self, scenario_spec, summary):
        max_reverse = scenario_spec.get("max_planner_reverse_segments")
        if max_reverse is not None and summary["reverse_segments"] > int(max_reverse):
            self.node.get_logger().error(
                "planner reverse_segments={} exceeds max {}".format(
                    summary["reverse_segments"], max_reverse))
            return 20

        min_reverse = scenario_spec.get("min_planner_reverse_segments")
        if min_reverse is not None and summary["reverse_segments"] < int(min_reverse):
            self.node.get_logger().error(
                "planner reverse_segments={} below min {}".format(
                    summary["reverse_segments"], min_reverse))
            return 21

        max_switches = scenario_spec.get("max_planner_gear_switches")
        if max_switches is not None and summary["gear_switches"] > int(max_switches):
            self.node.get_logger().error(
                "planner gear_switches={} exceeds max {}".format(
                    summary["gear_switches"], max_switches))
            return 22

        if summary["heading_jumps"] > int(scenario_spec.get("max_heading_jumps", 0)):
            self.node.get_logger().error(
                "path heading_jumps={} exceeds max {}".format(
                    summary["heading_jumps"], scenario_spec.get("max_heading_jumps", 0)))
            return 23

        return 0

    def validate_follow_summary(self, scenario_spec):
        min_forward = scenario_spec.get("min_controller_forward_samples")
        if min_forward is not None and self.forward_control_samples < int(min_forward):
            self.node.get_logger().error(
                "controller forward samples={} below min {}".format(
                    self.forward_control_samples, min_forward))
            return 30

        max_reverse = scenario_spec.get("max_controller_reverse_samples")
        if max_reverse is not None and self.reverse_control_samples > int(max_reverse):
            self.node.get_logger().error(
                "controller reverse samples={} exceeds max {}".format(
                    self.reverse_control_samples, max_reverse))
            return 31

        min_reverse = scenario_spec.get("min_controller_reverse_samples")
        if min_reverse is not None and self.reverse_control_samples < int(min_reverse):
            self.node.get_logger().error(
                "controller reverse samples={} below min {}".format(
                    self.reverse_control_samples, min_reverse))
            return 32

        if self.control_samples == 0 or self.sim_cmd_samples == 0:
            self.node.get_logger().error("controller/bridge command samples were not observed")
            return 33

        if bool(scenario_spec.get("expect_negative_sim_cmd", False)) and \
                self.min_signed_sim_linear_x >= -0.01:
            self.node.get_logger().error("negative sim_cmd_vel.linear.x was not observed")
            return 34

        return 0

    def log_path_summary(self, path, summary):
        self.node.get_logger().info(
            "path_summary poses={} forward_segments={} reverse_segments={} "
            "gear_switches={} heading_jumps={} max_heading_delta_deg={:.1f}".format(
                len(path.poses),
                summary["forward_segments"],
                summary["reverse_segments"],
                summary["gear_switches"],
                summary["heading_jumps"],
                math.degrees(summary["max_heading_delta"])))

    def log_command_summary(self):
        self.node.get_logger().info(
            "control_samples={} max_velocity_mps={:.3f}".format(
                self.control_samples, self.max_control_velocity))
        self.node.get_logger().info(
            "control_direction_samples forward={} reverse={}".format(
                self.forward_control_samples, self.reverse_control_samples))
        self.node.get_logger().info(
            "sim_cmd_samples={} max_linear_x={:.3f} min_signed_linear_x={:.3f} "
            "max_signed_linear_x={:.3f}".format(
                self.sim_cmd_samples,
                self.max_sim_linear_x,
                self.min_signed_sim_linear_x,
                self.max_signed_sim_linear_x))
        if self.last_odom is not None:
            position = self.last_odom.pose.pose.position
            self.node.get_logger().info(
                "odom_final x={:.3f} y={:.3f}".format(position.x, position.y))

    def spin_for_observation(self):
        deadline = time.monotonic() + max(0.0, self.post_result_observation_sec)
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)


def main():
    rclpy.init()
    runner = P6ReverseAcceptance()
    try:
        return_code = runner.run()
    finally:
        runner.compute_client.destroy()
        runner.follow_client.destroy()
        runner.node.destroy_node()
        rclpy.shutdown()
    sys.exit(return_code)


if __name__ == "__main__":
    main()
