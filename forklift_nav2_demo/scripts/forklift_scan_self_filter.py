#!/usr/bin/python3

import ast
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def parse_footprint(value):
    if isinstance(value, str):
        value = ast.literal_eval(value)
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        raise ValueError('footprint must contain at least three points')
    return [(float(point[0]), float(point[1])) for point in value]


def point_in_polygon(x, y, polygon):
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        crosses = (y1 > y) != (y2 > y)
        if crosses:
            edge_x = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < edge_x:
                inside = not inside
        previous = current
    return inside


def filtered_ranges(msg, footprint, sensor_x, sensor_y, sensor_yaw):
    ranges = list(msg.ranges)
    cos_sensor = math.cos(sensor_yaw)
    sin_sensor = math.sin(sensor_yaw)
    filtered_count = 0
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance):
            continue
        angle = msg.angle_min + index * msg.angle_increment
        scan_x = distance * math.cos(angle)
        scan_y = distance * math.sin(angle)
        base_x = sensor_x + scan_x * cos_sensor - scan_y * sin_sensor
        base_y = sensor_y + scan_x * sin_sensor + scan_y * cos_sensor
        if point_in_polygon(base_x, base_y, footprint):
            ranges[index] = float('inf')
            filtered_count += 1
    return ranges, filtered_count


class ForkliftScanSelfFilter(Node):
    def __init__(self):
        super().__init__('forklift_scan_self_filter')
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('sensor_x', 0.0)
        self.declare_parameter('sensor_y', 0.0)
        self.declare_parameter('sensor_yaw', 0.0)
        self.declare_parameter(
            'footprint',
            '[[1.709, 0.610], [1.709, -0.610], [-1.590, -0.610], [-1.590, 0.610]]')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.sensor_x = float(self.get_parameter('sensor_x').value)
        self.sensor_y = float(self.get_parameter('sensor_y').value)
        self.sensor_yaw = float(self.get_parameter('sensor_yaw').value)
        self.footprint = parse_footprint(self.get_parameter('footprint').value)
        self.message_count = 0

        output_qos = QoSProfile(depth=10)
        output_qos.reliability = ReliabilityPolicy.RELIABLE
        self.publisher = self.create_publisher(LaserScan, output_topic, output_qos)
        self.subscription = self.create_subscription(
            LaserScan, input_topic, self.on_scan, qos_profile_sensor_data)
        self.get_logger().info(
            'Filtering simulated self returns: {} -> {}'.format(
                input_topic, output_topic))

    def on_scan(self, msg):
        ranges, filtered_count = filtered_ranges(
            msg, self.footprint, self.sensor_x, self.sensor_y, self.sensor_yaw)
        output = LaserScan()
        output.header = msg.header
        output.angle_min = msg.angle_min
        output.angle_max = msg.angle_max
        output.angle_increment = msg.angle_increment
        output.time_increment = msg.time_increment
        output.scan_time = msg.scan_time
        output.range_min = msg.range_min
        output.range_max = msg.range_max
        output.ranges = ranges
        output.intensities = list(msg.intensities)
        self.publisher.publish(output)

        self.message_count += 1
        if self.message_count % 50 == 0:
            self.get_logger().info(
                'Filtered {} self-return samples from latest scan'.format(
                    filtered_count))


def main(args=None):
    rclpy.init(args=args)
    node = ForkliftScanSelfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
