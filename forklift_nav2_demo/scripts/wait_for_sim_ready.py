#!/usr/bin/python3

import argparse
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
import tf2_ros


class SimReadyWaiter(Node):
    def __init__(self, odom_topic, odom_frame, base_frame, quiet):
        super().__init__("wait_for_sim_ready")
        self.odom_topic = odom_topic
        self.odom_frame = odom_frame
        self.base_frame = base_frame
        self.quiet = quiet

        self.have_clock = False
        self.have_odom = False
        self.last_status = ""
        self.timeout_reported = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        clock_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Clock, "/clock", self.clock_callback, clock_qos)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

    def clock_callback(self, _msg):
        self.have_clock = True

    def odom_callback(self, _msg):
        self.have_odom = True

    def has_base_tf(self):
        try:
            self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
            return True
        except Exception:
            return False

    def ready(self):
        return self.have_clock and self.have_odom and self.has_base_tf()

    def report_waiting(self):
        missing = []
        if not self.have_clock:
            missing.append("/clock")
        if not self.have_odom:
            missing.append(self.odom_topic)
        if not self.has_base_tf():
            missing.append(f"{self.odom_frame}->{self.base_frame}")

        status = ", ".join(missing) if missing else "ready"
        if status != self.last_status and not self.quiet:
            self.get_logger().info(f"Waiting for simulation readiness: {status}")
            self.last_status = status


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.0,
        help="Seconds to wait before logging a timeout warning. Values <= 0 wait forever.",
    )
    parser.add_argument("--exit-on-timeout", action="store_true")
    parser.add_argument("--quiet", action="store_true")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = SimReadyWaiter(args.odom_topic, args.odom_frame, args.base_frame, args.quiet)

    if not args.quiet:
        node.get_logger().info(
            "Waiting for /clock, "
            f"{args.odom_topic}, and {args.odom_frame}->{args.base_frame} TF")

    start_time = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.ready():
                if not args.quiet:
                    node.get_logger().info("Simulation readiness checks passed")
                return 0

            if args.timeout > 0.0 and time.monotonic() - start_time > args.timeout:
                if not node.timeout_reported:
                    node.report_waiting()
                    node.get_logger().error("Timed out waiting for simulation readiness")
                    node.timeout_reported = True
                    if args.exit_on_timeout:
                        return 1

            node.report_waiting()
    except KeyboardInterrupt:
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 1


if __name__ == "__main__":
    sys.exit(main())
