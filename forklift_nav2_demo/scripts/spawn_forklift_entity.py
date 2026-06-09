#!/usr/bin/python3

import argparse
import math
import sys

import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose


def make_pose(x, y, z, yaw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.z = math.sin(yaw * 0.5)
    pose.orientation.w = math.cos(yaw * 0.5)
    return pose


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--entity", default="forklift")
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--robot-namespace", default="")
    parser.add_argument("--reference-frame", default="world")
    parser.add_argument("--service-timeout", type=float, default=30.0)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = rclpy.create_node("spawn_forklift_entity")
    node.declare_parameter("robot_description", "")
    robot_description = node.get_parameter("robot_description").value

    if not robot_description:
        node.get_logger().error("robot_description parameter is empty")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    client = node.create_client(SpawnEntity, "/spawn_entity")
    node.get_logger().info("Waiting for Gazebo /spawn_entity service")
    if not client.wait_for_service(timeout_sec=args.service_timeout):
        node.get_logger().error("Timed out waiting for Gazebo /spawn_entity service")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    request = SpawnEntity.Request()
    request.name = args.entity
    request.xml = robot_description
    request.robot_namespace = args.robot_namespace
    request.initial_pose = make_pose(args.x, args.y, args.z, args.yaw)
    request.reference_frame = args.reference_frame

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=args.service_timeout)
    if not future.done():
        node.get_logger().error("Timed out calling Gazebo /spawn_entity service")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    response = future.result()
    if response is None:
        node.get_logger().error("Gazebo /spawn_entity service returned no response")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if not response.success:
        node.get_logger().error(f"Failed to spawn {args.entity}: {response.status_message}")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    node.get_logger().info(f"Spawned {args.entity}: {response.status_message}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
