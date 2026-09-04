#!/usr/bin/python3

import math
import sys
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quat: Quaternion) -> float:
    siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
    cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    quat = Quaternion()
    half_yaw = 0.5 * yaw
    quat.z = math.sin(half_yaw)
    quat.w = math.cos(half_yaw)
    return quat


def map_to_odom_2d(
    map_to_base_xy_yaw: Tuple[float, float, float],
    odom_to_base_xy_yaw: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    map_base_x, map_base_y, map_base_yaw = map_to_base_xy_yaw
    odom_base_x, odom_base_y, odom_base_yaw = odom_to_base_xy_yaw

    map_odom_yaw = normalize_angle(map_base_yaw - odom_base_yaw)
    cos_yaw = math.cos(map_odom_yaw)
    sin_yaw = math.sin(map_odom_yaw)
    map_odom_x = map_base_x - (cos_yaw * odom_base_x - sin_yaw * odom_base_y)
    map_odom_y = map_base_y - (sin_yaw * odom_base_x + cos_yaw * odom_base_y)
    return map_odom_x, map_odom_y, map_odom_yaw


def apply_map_frame_offset_2d(
    map_to_base_xy_yaw: Tuple[float, float, float],
    offset_xy_yaw: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    x, y, yaw = map_to_base_xy_yaw
    offset_x, offset_y, offset_yaw = offset_xy_yaw
    return (
        x + offset_x,
        y + offset_y,
        normalize_angle(yaw + offset_yaw),
    )


class MapOdomLocalizationAdapter(Node):
    """Convert map-frame base pose messages into a standard map->odom TF.

    The external localization topic is expected to describe map->base_link as a
    nav_msgs/Odometry pose. The vehicle interface owns the odom-frame base pose,
    commonly odom->base_footprint. This node combines both poses and publishes
    map->odom:

        map_T_odom = map_T_localized_base * inverse(odom_T_odom_base)
    """

    def __init__(self) -> None:
        super().__init__('map_odom_localization_adapter')

        self.declare_parameter('localization_topic', '/fusion/localization')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('localization_base_frame_id', 'base_link')
        self.declare_parameter('odom_base_frame_id', 'base_footprint')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('stamp_with_current_time', False)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('localization_offset_x_m', 0.0)
        self.declare_parameter('localization_offset_y_m', 0.0)
        self.declare_parameter('localization_offset_yaw_rad', 0.0)
        self.declare_parameter('warn_on_frame_mismatch', True)

        self._localization_topic = str(self.get_parameter('localization_topic').value)
        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._map_frame_id = str(self.get_parameter('map_frame_id').value)
        self._odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self._localization_base_frame_id = str(
            self.get_parameter('localization_base_frame_id').value)
        self._odom_base_frame_id = str(self.get_parameter('odom_base_frame_id').value)
        # Backward compatibility for older launch files that only set base_frame_id.
        legacy_base_frame_id = str(self.get_parameter('base_frame_id').value)
        if self._localization_base_frame_id == 'base_link' and legacy_base_frame_id != 'base_link':
            self._localization_base_frame_id = legacy_base_frame_id
        if self._odom_base_frame_id == 'base_footprint' and legacy_base_frame_id != 'base_link':
            self._odom_base_frame_id = legacy_base_frame_id
        self._stamp_with_current_time = bool(
            self.get_parameter('stamp_with_current_time').value)
        self._publish_rate_hz = self._positive_param('publish_rate_hz', 20.0)
        self._localization_offset = (
            float(self.get_parameter('localization_offset_x_m').value),
            float(self.get_parameter('localization_offset_y_m').value),
            float(self.get_parameter('localization_offset_yaw_rad').value),
        )
        self._warn_on_frame_mismatch = bool(
            self.get_parameter('warn_on_frame_mismatch').value)

        self._warned_frame_mismatch = False
        self._last_error = ''
        self._latest_localization: Optional[Odometry] = None
        self._latest_odom: Optional[Odometry] = None

        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            self._localization_topic,
            self._on_localization,
            10,
        )
        self.create_subscription(
            Odometry,
            self._odom_topic,
            self._on_odom,
            10,
        )
        self.create_timer(1.0 / self._publish_rate_hz, self._publish_latest_transform)

        self.get_logger().info(
            'map_odom_localization_adapter ready: '
            f'{self._localization_topic} '
            f'({self._map_frame_id}->{self._localization_base_frame_id}) '
            f'+ {self._odom_topic} '
            f'({self._odom_frame_id}->{self._odom_base_frame_id}) -> '
            f'{self._map_frame_id}->{self._odom_frame_id} TF '
            f'at {self._publish_rate_hz:.1f} Hz; '
            f'localization offset={self._localization_offset}'
        )

    def _positive_param(self, name: str, fallback: float) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            self.get_logger().warning(
                f'Parameter {name} must be positive; using fallback {fallback}.')
            return fallback
        return value

    def _on_localization(self, msg: Odometry) -> None:
        if self._warn_on_frame_mismatch:
            self._check_frames(
                msg,
                self._map_frame_id,
                self._localization_base_frame_id,
                'Localization',
            )
        self._latest_localization = msg

    def _on_odom(self, msg: Odometry) -> None:
        if self._warn_on_frame_mismatch:
            self._check_frames(
                msg,
                self._odom_frame_id,
                self._odom_base_frame_id,
                'Odometry',
            )
        self._latest_odom = msg

    def _publish_latest_transform(self) -> None:
        localization = self._latest_localization
        odom = self._latest_odom
        if localization is None or odom is None:
            return

        map_pose = localization.pose.pose
        map_to_base = (
            float(map_pose.position.x),
            float(map_pose.position.y),
            yaw_from_quaternion(map_pose.orientation),
        )
        map_to_base = apply_map_frame_offset_2d(
            map_to_base,
            self._localization_offset,
        )
        odom_pose = odom.pose.pose
        odom_to_base = (
            float(odom_pose.position.x),
            float(odom_pose.position.y),
            yaw_from_quaternion(odom_pose.orientation),
        )
        map_odom_x, map_odom_y, map_odom_yaw = map_to_odom_2d(
            map_to_base,
            odom_to_base,
        )

        transform = TransformStamped()
        transform.header.stamp = (
            self.get_clock().now().to_msg()
            if self._stamp_with_current_time else localization.header.stamp
        )
        transform.header.frame_id = self._map_frame_id
        transform.child_frame_id = self._odom_frame_id
        transform.transform.translation.x = map_odom_x
        transform.transform.translation.y = map_odom_y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = quaternion_from_yaw(map_odom_yaw)
        self._tf_broadcaster.sendTransform(transform)
        self._last_error = ''

    def _check_frames(
        self,
        msg: Odometry,
        expected_frame_id: str,
        expected_child_frame_id: str,
        source_name: str,
    ) -> None:
        frame_ok = msg.header.frame_id == expected_frame_id
        child_ok = msg.child_frame_id == expected_child_frame_id
        if frame_ok and child_ok:
            return
        if self._warned_frame_mismatch:
            return
        self.get_logger().warning(
            f'{source_name} frame mismatch: expected '
            f'header.frame_id={expected_frame_id}, '
            f'child_frame_id={expected_child_frame_id}; got '
            f'header.frame_id={msg.header.frame_id}, '
            f'child_frame_id={msg.child_frame_id}.'
        )
        self._warned_frame_mismatch = True


def main(args=None) -> int:
    rclpy.init(args=args)
    node = MapOdomLocalizationAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
