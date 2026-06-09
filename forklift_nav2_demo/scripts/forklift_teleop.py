#!/usr/bin/python3

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist


HELP_TEXT = """
Forklift teleop
---------------
Moving around:
        w
   a    s    d
        x

w/x: increase/decrease linear velocity
a/d: increase/decrease angular velocity
space or s: stop
q: quit
"""


def clamp(value, low, high):
    return max(low, min(value, high))


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    ready, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if ready else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def publish(pub, linear, angular):
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular
    pub.publish(msg)


def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = rclpy.create_node("forklift_teleop")
    node.declare_parameter("cmd_vel_topic", "/cmd_vel")
    node.declare_parameter("max_linear_velocity", 0.60)
    node.declare_parameter("max_angular_velocity", 1.00)
    node.declare_parameter("linear_step", 0.05)
    node.declare_parameter("angular_step", 0.10)

    topic = node.get_parameter("cmd_vel_topic").value
    max_linear = float(node.get_parameter("max_linear_velocity").value)
    max_angular = float(node.get_parameter("max_angular_velocity").value)
    linear_step = float(node.get_parameter("linear_step").value)
    angular_step = float(node.get_parameter("angular_step").value)

    pub = node.create_publisher(Twist, topic, 10)
    linear = 0.0
    angular = 0.0

    try:
        print(HELP_TEXT)
        print(f"limits: linear +/-{max_linear:.2f} m/s, angular +/-{max_angular:.2f} rad/s")
        while rclpy.ok():
            key = get_key(settings)
            if key == "w":
                linear = clamp(linear + linear_step, -max_linear, max_linear)
            elif key == "x":
                linear = clamp(linear - linear_step, -max_linear, max_linear)
            elif key == "a":
                angular = clamp(angular + angular_step, -max_angular, max_angular)
            elif key == "d":
                angular = clamp(angular - angular_step, -max_angular, max_angular)
            elif key == " " or key == "s":
                linear = 0.0
                angular = 0.0
            elif key == "q" or key == "\x03":
                break
            elif key == "":
                publish(pub, linear, angular)
                rclpy.spin_once(node, timeout_sec=0.0)
                continue
            else:
                continue

            publish(pub, linear, angular)
            print(f"linear: {linear:.2f} m/s, angular: {angular:.2f} rad/s")
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        publish(pub, 0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
