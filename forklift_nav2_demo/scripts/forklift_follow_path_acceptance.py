#!/usr/bin/python3

import math
import sys

import rclpy
from action_msgs.msg import GoalStatus
from forklift_msgs.msg import ForkliftControlCommand
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient


SCENARIOS = {
    "short_straight": [(-2.0, -0.5), (-1.3, -0.5)],
    "gentle_arc": [
        (-2.0, -0.5),
        (-1.9, -0.48),
        (-1.8, -0.45),
        (-1.7, -0.42),
        (-1.6, -0.39),
        (-1.5, -0.36),
        (-1.4, -0.34),
    ],
    "sparse_90_turn": [(-2.0, -0.5), (-1.3, -0.5), (-1.3, 0.2)],
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
    pose = PoseStamped()
    pose.pose.orientation.z = math.sin(yaw * 0.5)
    pose.pose.orientation.w = math.cos(yaw * 0.5)
    return pose.pose.orientation


def make_path(node, points):
    path = Path()
    path.header.frame_id = "map"

    for index, point in enumerate(points):
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        if index + 1 < len(points):
            next_point = points[index + 1]
            yaw = math.atan2(next_point[1] - point[1], next_point[0] - point[0])
        elif index > 0:
            prev_point = points[index - 1]
            yaw = math.atan2(point[1] - prev_point[1], point[0] - prev_point[0])
        else:
            yaw = 0.0
        pose.pose.orientation = yaw_to_quaternion(yaw)
        path.poses.append(pose)

    node.get_logger().info(
        "FollowPath path: " +
        " -> ".join("({:.2f},{:.2f})".format(x, y) for x, y in points))
    return path


class FollowPathAcceptance:
    def __init__(self):
        self.node = rclpy.create_node("forklift_follow_path_acceptance")
        self.node.declare_parameter("scenario", "short_straight")
        self.node.declare_parameter("timeout_sec", 60.0)
        self.node.declare_parameter("controller_id", "FollowPath")
        self.node.declare_parameter("goal_checker_id", "general_goal_checker")
        self.node.declare_parameter("action_name", "/follow_path")

        self.scenario = self.node.get_parameter("scenario").value
        self.timeout_sec = float(self.node.get_parameter("timeout_sec").value)
        self.controller_id = self.node.get_parameter("controller_id").value
        self.goal_checker_id = self.node.get_parameter("goal_checker_id").value
        self.action_name = self.node.get_parameter("action_name").value

        self.control_samples = 0
        self.sim_cmd_samples = 0
        self.max_control_velocity = 0.0
        self.max_sim_linear_x = 0.0
        self.last_odom = None

        self.node.create_subscription(
            ForkliftControlCommand, "/forklift/control_cmd", self.on_control_cmd, 10)
        self.node.create_subscription(Twist, "/forklift/sim_cmd_vel", self.on_sim_cmd, 10)
        self.node.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.action_client = ActionClient(self.node, FollowPath, self.action_name)

    def on_control_cmd(self, msg):
        self.control_samples += 1
        self.max_control_velocity = max(self.max_control_velocity, abs(msg.velocity_mps))

    def on_sim_cmd(self, msg):
        self.sim_cmd_samples += 1
        self.max_sim_linear_x = max(self.max_sim_linear_x, abs(msg.linear.x))

    def on_odom(self, msg):
        self.last_odom = msg

    def run(self):
        if self.scenario not in SCENARIOS:
            self.node.get_logger().error(
                "Unknown scenario '{}'; valid scenarios: {}".format(
                    self.scenario, ", ".join(sorted(SCENARIOS))))
            return 2

        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().error("{} action server is not available".format(self.action_name))
            return 3

        goal = FollowPath.Goal()
        goal.path = make_path(self.node, SCENARIOS[self.scenario])
        goal.controller_id = self.controller_id
        if hasattr(goal, "goal_checker_id"):
            goal.goal_checker_id = self.goal_checker_id

        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("/follow_path rejected the goal")
            return 4

        self.node.get_logger().info("/follow_path accepted: true")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=self.timeout_sec)
        result = result_future.result()
        if result is None:
            self.node.get_logger().error("/follow_path timed out")
            return 5

        status_name = STATUS_NAMES.get(result.status, "STATUS_{}".format(result.status))
        self.node.get_logger().info("/follow_path status: {} {}".format(result.status, status_name))
        self.node.get_logger().info(
            "control_samples={} max_velocity_mps={:.3f}".format(
                self.control_samples, self.max_control_velocity))
        self.node.get_logger().info(
            "sim_cmd_samples={} max_linear_x={:.3f}".format(
                self.sim_cmd_samples, self.max_sim_linear_x))
        if self.last_odom is not None:
            position = self.last_odom.pose.pose.position
            self.node.get_logger().info(
                "odom_final x={:.3f} y={:.3f}".format(position.x, position.y))

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            return 6
        if self.control_samples == 0 or self.sim_cmd_samples == 0:
            self.node.get_logger().error("bridge/controller command samples were not observed")
            return 7
        return 0


def main():
    rclpy.init()
    runner = FollowPathAcceptance()
    try:
        return_code = runner.run()
    finally:
        runner.action_client.destroy()
        runner.node.destroy_node()
        rclpy.shutdown()
    sys.exit(return_code)


if __name__ == "__main__":
    main()
