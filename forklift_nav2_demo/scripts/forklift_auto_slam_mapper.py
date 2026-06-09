#!/usr/bin/python3

import math
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry


def clamp(value, low, high):
    return max(low, min(value, high))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def parse_relative_waypoints(text):
    waypoints = []
    for item in text.split(";"):
        item = item.strip()
        if not item:
            continue
        x_text, y_text = item.split(",", 1)
        waypoints.append((float(x_text), float(y_text)))
    return waypoints


class AutoSlamMapper:
    def __init__(self):
        self.node = rclpy.create_node("forklift_auto_slam_mapper")
        self.node.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.node.declare_parameter("odom_topic", "/odom")
        self.node.declare_parameter("map_topic", "/map")
        self.node.declare_parameter("map_path", "/home/pl/robotest/forklift_factory_auto_map")
        self.node.declare_parameter(
            "relative_waypoints",
            "-7.0,0.0;-7.0,6.5;2.0,6.5;10.0,6.5;12.0,0.0;"
            "10.0,-6.5;2.0,-6.5;-7.0,-6.5;-7.0,0.0;"
            "14.0,0.0;23.0,0.0;23.0,4.0;23.0,-4.0;14.0,0.0;2.0,0.0",
        )
        self.node.declare_parameter("max_linear_velocity", 0.35)
        self.node.declare_parameter("max_angular_velocity", 0.55)
        self.node.declare_parameter("waypoint_tolerance", 0.35)
        self.node.declare_parameter("heading_tolerance", 0.25)
        self.node.declare_parameter("spin_at_waypoints", True)
        self.node.declare_parameter("spin_angular_velocity", 0.45)
        self.node.declare_parameter("wait_for_map_timeout", 60.0)
        self.node.declare_parameter("save_map", True)

        self.cmd_vel_topic = self.node.get_parameter("cmd_vel_topic").value
        self.map_path = self.node.get_parameter("map_path").value
        self.relative_waypoints = parse_relative_waypoints(
            self.node.get_parameter("relative_waypoints").value)
        self.max_linear = float(self.node.get_parameter("max_linear_velocity").value)
        self.max_angular = float(self.node.get_parameter("max_angular_velocity").value)
        self.waypoint_tolerance = float(self.node.get_parameter("waypoint_tolerance").value)
        self.heading_tolerance = float(self.node.get_parameter("heading_tolerance").value)
        self.spin_at_waypoints = bool(self.node.get_parameter("spin_at_waypoints").value)
        self.spin_angular = float(self.node.get_parameter("spin_angular_velocity").value)
        self.wait_for_map_timeout = float(self.node.get_parameter("wait_for_map_timeout").value)
        self.save_map = bool(self.node.get_parameter("save_map").value)

        self.odom = None
        self.map_received = False

        self.pub = self.node.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.node.create_subscription(
            Odometry,
            self.node.get_parameter("odom_topic").value,
            self.odom_callback,
            10,
        )
        self.node.create_subscription(
            OccupancyGrid,
            self.node.get_parameter("map_topic").value,
            self.map_callback,
            10,
        )

    def odom_callback(self, msg):
        self.odom = msg

    def map_callback(self, _msg):
        self.map_received = True

    def publish_cmd(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub.publish(msg)

    def wait_for_inputs(self):
        start = time.monotonic()
        while rclpy.ok() and self.odom is None:
            self.node.get_logger().info("Waiting for /odom...")
            rclpy.spin_once(self.node, timeout_sec=1.0)

        while rclpy.ok() and not self.map_received:
            if time.monotonic() - start > self.wait_for_map_timeout:
                self.node.get_logger().warn("No /map received yet; starting route anyway")
                return
            self.node.get_logger().info("Waiting for SLAM /map...")
            rclpy.spin_once(self.node, timeout_sec=1.0)

    def pose(self):
        pose = self.odom.pose.pose
        return pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)

    def spin_scan(self):
        if not self.spin_at_waypoints:
            return
        duration = 2.0 * math.pi / max(abs(self.spin_angular), 0.05)
        end_time = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < end_time:
            self.publish_cmd(0.0, self.spin_angular)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.publish_cmd()

    def drive_to(self, target_x, target_y):
        deadline = time.monotonic() + 90.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            x, y, yaw = self.pose()
            dx = target_x - x
            dy = target_y - y
            distance = math.hypot(dx, dy)
            if distance < self.waypoint_tolerance:
                self.publish_cmd()
                return True

            desired_yaw = math.atan2(dy, dx)
            heading_error = normalize_angle(desired_yaw - yaw)
            angular = clamp(1.2 * heading_error, -self.max_angular, self.max_angular)

            if abs(heading_error) > self.heading_tolerance:
                linear = 0.0
            else:
                linear = min(self.max_linear, max(0.08, 0.35 * distance))

            self.publish_cmd(linear, angular)

        self.publish_cmd()
        return False

    def save_current_map(self):
        if not self.save_map:
            return
        self.node.get_logger().info(f"Saving map to {self.map_path}")
        result = subprocess.run(
            ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", self.map_path],
            check=False,
        )
        if result.returncode == 0:
            self.node.get_logger().info("Map saved successfully")
        else:
            self.node.get_logger().error(
                f"map_saver_cli failed with exit code {result.returncode}")

    def run(self):
        self.wait_for_inputs()
        start_x, start_y, _ = self.pose()
        targets = [(start_x + dx, start_y + dy) for dx, dy in self.relative_waypoints]

        self.node.get_logger().info(f"Starting automatic SLAM route with {len(targets)} waypoints")
        self.spin_scan()
        for index, (target_x, target_y) in enumerate(targets, start=1):
            self.node.get_logger().info(
                f"Waypoint {index}/{len(targets)}: ({target_x:.2f}, {target_y:.2f})")
            reached = self.drive_to(target_x, target_y)
            if not reached:
                self.node.get_logger().warn(f"Timed out before waypoint {index}")
            self.spin_scan()

        self.publish_cmd()
        self.save_current_map()


def main():
    rclpy.init()
    mapper = AutoSlamMapper()
    try:
        mapper.run()
    finally:
        mapper.publish_cmd()
        mapper.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
