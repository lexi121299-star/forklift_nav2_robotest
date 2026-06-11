#!/usr/bin/python3

import math
import select
import sys

import rclpy
from builtin_interfaces.msg import Duration as BuiltinDuration
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclpyDuration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray


HELP_TEXT = """
Forklift manual path follower
-----------------------------
RViz:
  1. Select Publish Point
  2. Click path points in order on the map

Commands:
  send        send clicked path to /follow_path
  clear       clear all clicked points
  pop         remove the last point
  list        print clicked points
  help        show this help
  q           quit
"""


def yaw_to_quaternion(yaw):
    q = PoseStamped().pose.orientation
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


class ManualPathFollower:
    def __init__(self):
        self.node = rclpy.create_node("forklift_manual_path_follower")
        self.node.declare_parameter("clicked_point_topic", "/clicked_point")
        self.node.declare_parameter("path_topic", "/manual_path")
        self.node.declare_parameter("marker_topic", "/waypoints")
        self.node.declare_parameter("follow_path_action", "/follow_path")
        self.node.declare_parameter("frame_id", "map")
        self.node.declare_parameter("robot_base_frame", "base_link")
        self.node.declare_parameter("controller_id", "FollowPath")
        self.node.declare_parameter("goal_checker_id", "general_goal_checker")
        self.node.declare_parameter("prepend_robot_pose", True)
        self.node.declare_parameter("prepend_if_farther_than", 0.2)
        self.node.declare_parameter("tf_timeout", 0.5)
        self.node.declare_parameter("stamp_paths_with_current_time", False)
        self.node.declare_parameter("interpolation_resolution", 0.2)
        self.node.declare_parameter("min_point_spacing", 0.05)
        self.node.declare_parameter("marker_lifetime_sec", 0.0)

        self.clicked_point_topic = self.node.get_parameter("clicked_point_topic").value
        self.path_topic = self.node.get_parameter("path_topic").value
        self.marker_topic = self.node.get_parameter("marker_topic").value
        self.follow_path_action = self.node.get_parameter("follow_path_action").value
        self.frame_id = self.node.get_parameter("frame_id").value
        self.robot_base_frame = self.node.get_parameter("robot_base_frame").value
        self.controller_id = self.node.get_parameter("controller_id").value
        self.goal_checker_id = self.node.get_parameter("goal_checker_id").value
        self.prepend_robot_pose = bool(self.node.get_parameter("prepend_robot_pose").value)
        self.prepend_if_farther_than = float(
            self.node.get_parameter("prepend_if_farther_than").value)
        self.tf_timeout = float(self.node.get_parameter("tf_timeout").value)
        self.stamp_paths_with_current_time = bool(
            self.node.get_parameter("stamp_paths_with_current_time").value)
        self.interpolation_resolution = float(
            self.node.get_parameter("interpolation_resolution").value)
        self.min_point_spacing = float(self.node.get_parameter("min_point_spacing").value)
        self.marker_lifetime_sec = float(self.node.get_parameter("marker_lifetime_sec").value)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.points = []
        self.active_goal_handle = None

        self.path_pub = self.node.create_publisher(Path, self.path_topic, qos)
        self.marker_pub = self.node.create_publisher(MarkerArray, self.marker_topic, qos)
        self.node.create_subscription(
            PointStamped, self.clicked_point_topic, self.clicked_point_callback, 10)
        self.action_client = ActionClient(self.node, FollowPath, self.follow_path_action)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def clicked_point_callback(self, msg):
        point = Point()
        point.x = msg.point.x
        point.y = msg.point.y
        point.z = msg.point.z

        if self.points and distance(point, self.points[-1]) < self.min_point_spacing:
            self.node.get_logger().info("Ignored point too close to previous point")
            return

        self.points.append(point)
        self.publish_visuals()
        self.node.get_logger().info(
            f"Added point {len(self.points)}: x={point.x:.3f}, y={point.y:.3f}")

    def current_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.robot_base_frame,
                Time(),
                timeout=RclpyDuration(seconds=self.tf_timeout),
            )
        except Exception as exc:
            self.node.get_logger().warn(
                f"Could not get {self.frame_id}->{self.robot_base_frame} transform: {exc}")
            return None

        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        if self.stamp_paths_with_current_time:
            pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def densify_points(self, points):
        if len(points) < 2 or self.interpolation_resolution <= 0.0:
            return list(points)

        dense_points = []
        for index in range(len(points) - 1):
            start = points[index]
            end = points[index + 1]
            segment_length = distance(start, end)
            steps = max(1, int(math.ceil(segment_length / self.interpolation_resolution)))

            for step in range(steps):
                ratio = float(step) / float(steps)
                point = Point()
                point.x = start.x + ratio * (end.x - start.x)
                point.y = start.y + ratio * (end.y - start.y)
                point.z = start.z + ratio * (end.z - start.z)
                dense_points.append(point)

        dense_points.append(points[-1])
        return dense_points

    def make_path(self, include_robot_pose=False, densify=True):
        path = Path()
        path.header.frame_id = self.frame_id
        if self.stamp_paths_with_current_time:
            path.header.stamp = self.node.get_clock().now().to_msg()

        if not self.points:
            return path

        path_points = self.densify_points(self.points) if densify else list(self.points)
        robot_pose = self.current_robot_pose() if include_robot_pose else None
        if robot_pose is not None:
            robot_point = robot_pose.pose.position
            if distance(robot_point, path_points[0]) > self.prepend_if_farther_than:
                path.poses.append(robot_pose)
                self.node.get_logger().info(
                    "Prepended current robot pose to manual path: "
                    f"x={robot_point.x:.3f}, y={robot_point.y:.3f}")

        for index, point in enumerate(path_points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = point.z

            if index + 1 < len(path_points):
                next_point = path_points[index + 1]
                yaw = math.atan2(next_point.y - point.y, next_point.x - point.x)
            elif index > 0:
                prev_point = path_points[index - 1]
                yaw = math.atan2(point.y - prev_point.y, point.x - prev_point.x)
            else:
                yaw = 0.0

            pose.pose.orientation = yaw_to_quaternion(yaw)
            path.poses.append(pose)

        return path

    def publish_visuals(self):
        path = self.make_path()
        self.path_pub.publish(path)
        self.marker_pub.publish(self.make_markers(self.make_path(densify=False)))

    def make_markers(self, path):
        markers = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        lifetime = BuiltinDuration()
        if self.marker_lifetime_sec > 0.0:
            lifetime.sec = int(self.marker_lifetime_sec)
            lifetime.nanosec = int((self.marker_lifetime_sec - lifetime.sec) * 1e9)

        for index, pose in enumerate(path.poses):
            sphere = Marker()
            sphere.header = path.header
            sphere.ns = "manual_path_points"
            sphere.id = index
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = pose.pose
            sphere.scale.x = 0.22
            sphere.scale.y = 0.22
            sphere.scale.z = 0.08
            sphere.color.r = 0.1
            sphere.color.g = 0.6
            sphere.color.b = 1.0
            sphere.color.a = 0.9
            sphere.lifetime = lifetime
            markers.markers.append(sphere)

            label = Marker()
            label.header = path.header
            label.ns = "manual_path_labels"
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = pose.pose
            label.pose.position.z += 0.35
            label.scale.z = 0.28
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 0.95
            label.text = str(index + 1)
            label.lifetime = lifetime
            markers.markers.append(label)

        if len(path.poses) >= 2:
            line = Marker()
            line.header = path.header
            line.ns = "manual_path_line"
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.07
            line.color.r = 0.1
            line.color.g = 0.6
            line.color.b = 1.0
            line.color.a = 0.8
            line.points = [pose.pose.position for pose in path.poses]
            line.lifetime = lifetime
            markers.markers.append(line)

        return markers

    def clear_points(self):
        self.points.clear()
        self.publish_visuals()
        self.node.get_logger().info("Cleared manual path")

    def pop_point(self):
        if not self.points:
            self.node.get_logger().info("No points to remove")
            return

        removed = self.points.pop()
        self.publish_visuals()
        self.node.get_logger().info(
            f"Removed point: x={removed.x:.3f}, y={removed.y:.3f}")

    def print_points(self):
        if not self.points:
            print("No clicked points yet")
            return

        for index, point in enumerate(self.points, start=1):
            print(f"{index:02d}: x={point.x:.3f}, y={point.y:.3f}, z={point.z:.3f}")

    def send_path(self):
        if len(self.points) < 2:
            self.node.get_logger().warn("Need at least 2 points before sending FollowPath")
            return

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(f"Action server {self.follow_path_action} is not available")
            return

        goal = FollowPath.Goal()
        goal.path = self.make_path(include_robot_pose=self.prepend_robot_pose)
        goal.controller_id = self.controller_id
        if hasattr(goal, "goal_checker_id"):
            goal.goal_checker_id = self.goal_checker_id

        self.node.get_logger().info(
            f"Sending manual path with {len(self.points)} clicked points, "
            f"{len(goal.path.poses)} path poses, controller_id='{self.controller_id}', "
            f"goal_checker_id='{self.goal_checker_id}'")
        future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("FollowPath goal was rejected")
            return

        self.active_goal_handle = goal_handle
        self.node.get_logger().info("FollowPath goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        status = result.status
        self.node.get_logger().info(f"FollowPath finished with status {status}")
        self.active_goal_handle = None

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.node.get_logger().info(
            f"distance_to_goal={feedback.distance_to_goal:.2f} speed={feedback.speed:.2f}")

    def handle_command(self, command):
        command = command.strip()
        if not command:
            return True
        if command == "send":
            self.send_path()
        elif command == "clear":
            self.clear_points()
        elif command == "pop":
            self.pop_point()
        elif command == "list":
            self.print_points()
        elif command in ("help", "h", "?"):
            print(HELP_TEXT)
        elif command in ("q", "quit", "exit"):
            return False
        else:
            print(f"Unknown command: {command}")
            print("Type 'help' for commands")
        return True

    def spin(self):
        print(HELP_TEXT)
        self.publish_visuals()
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            if ready:
                if not self.handle_command(sys.stdin.readline()):
                    break


def main():
    rclpy.init()
    follower = ManualPathFollower()
    try:
        follower.spin()
    finally:
        follower.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
