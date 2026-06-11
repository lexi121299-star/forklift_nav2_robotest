from __future__ import annotations

import math
from typing import List, Optional, Tuple

import rclpy
from forklift_msgs.msg import (
    ForkliftControlCommand,
    ForkliftFaultState,
    ForkliftVehicleState,
)
from forklift_msgs.srv import SetControlMode, SetEmergencyStop
from geometry_msgs.msg import Twist
from rclpy.node import Node


class SimCommandBridge(Node):
    """Adapt shared forklift control commands to Gazebo's /cmd_vel topic."""

    def __init__(self) -> None:
        super().__init__('sim_command_bridge')

        self.declare_parameter('wheel_base', 1.2)
        self.declare_parameter('max_velocity_mps', 0.4)
        self.declare_parameter('max_steering_angle_rad', 0.55)
        self.declare_parameter('max_angular_velocity_radps', 0.8)
        self.declare_parameter('allow_pivot_turn', False)
        self.declare_parameter('pivot_steering_angle_rad', math.pi / 2.0)
        self.declare_parameter('pivot_steering_tolerance_rad', 0.03)
        self.declare_parameter('pivot_turn_radius', 0.6)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publish_tf', False)

        self._wheel_base = self._positive_param('wheel_base', 1.2)
        self._max_velocity_mps = self._positive_param('max_velocity_mps', 0.4)
        self._max_steering_angle_rad = self._positive_param('max_steering_angle_rad', 0.55)
        self._max_angular_velocity_radps = self._positive_param(
            'max_angular_velocity_radps', 0.8
        )
        self._allow_pivot_turn = self._bool_param('allow_pivot_turn', False)
        self._pivot_steering_angle_rad = min(
            self._positive_param('pivot_steering_angle_rad', math.pi / 2.0),
            self._max_steering_angle_rad,
        )
        self._pivot_steering_tolerance_rad = max(
            0.0,
            min(
                float(self.get_parameter('pivot_steering_tolerance_rad').value),
                self._pivot_steering_angle_rad,
            ),
        )
        self._pivot_turn_radius = self._positive_param('pivot_turn_radius', 0.6)
        self._command_timeout_sec = self._positive_param('command_timeout_sec', 0.5)
        self._cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        control_rate_hz = self._positive_param('control_rate_hz', 20.0)

        if self.get_parameter('publish_tf').value:
            self.get_logger().warning(
                'publish_tf is ignored by sim_command_bridge; Gazebo remains the odom/TF source.'
            )

        self._mode = 'auto'
        self._emergency_stop = False
        self._last_command: Optional[ForkliftControlCommand] = None
        self._last_command_time = self.get_clock().now()
        self._last_stop_reason = ''

        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._vehicle_state_pub = self.create_publisher(
            ForkliftVehicleState, '/forklift/vehicle_state', 10
        )
        self._fault_state_pub = self.create_publisher(
            ForkliftFaultState, '/forklift/fault_state', 10
        )

        self.create_subscription(
            ForkliftControlCommand,
            '/forklift/control_cmd',
            self._on_command,
            10,
        )
        self.create_service(
            SetEmergencyStop,
            '/forklift/set_emergency_stop',
            self._on_set_emergency_stop,
        )
        self.create_service(
            SetControlMode,
            '/forklift/set_control_mode',
            self._on_set_control_mode,
        )

        self.create_timer(1.0 / control_rate_hz, self._on_timer)
        self.get_logger().info(
            f'sim_command_bridge ready: /forklift/control_cmd -> {self._cmd_vel_topic}'
        )

    def _positive_param(self, name: str, fallback: float) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            self.get_logger().warning(
                f'Parameter {name} must be positive; using fallback {fallback}.'
            )
            return fallback
        return value

    def _bool_param(self, name: str, fallback: bool) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in {'true', '1', 'yes', 'on'}:
                return True
            if normalized in {'false', '0', 'no', 'off'}:
                return False
        self.get_logger().warning(
            f'Parameter {name} must be boolean; using fallback {fallback}.'
        )
        return fallback

    def _on_command(self, msg: ForkliftControlCommand) -> None:
        self._last_command = msg
        self._last_command_time = self.get_clock().now()

    def _on_set_emergency_stop(
        self,
        request: SetEmergencyStop.Request,
        response: SetEmergencyStop.Response,
    ) -> SetEmergencyStop.Response:
        self._emergency_stop = bool(request.emergency_stop)
        response.success = True
        response.message = 'emergency stop enabled' if self._emergency_stop else 'emergency stop cleared'
        self.get_logger().warning(response.message)
        return response

    def _on_set_control_mode(
        self,
        request: SetControlMode.Request,
        response: SetControlMode.Response,
    ) -> SetControlMode.Response:
        mode = request.mode.strip().lower()
        if mode not in {'auto', 'manual', 'standby'}:
            response.success = False
            response.message = 'mode must be one of: auto, manual, standby'
            return response
        self._mode = mode
        response.success = True
        response.message = f'mode set to {mode}'
        self.get_logger().info(response.message)
        return response

    def _on_timer(self) -> None:
        twist, stop_reason = self._twist_from_latest_command()
        self._cmd_vel_pub.publish(twist)
        self._publish_debug_state(twist, stop_reason)
        self._log_stop_reason(stop_reason)

    def _twist_from_latest_command(self) -> Tuple[Twist, str]:
        twist = Twist()
        command = self._last_command
        if command is None:
            return twist, 'waiting for first command'

        age_sec = (self.get_clock().now() - self._last_command_time).nanoseconds / 1e9
        if age_sec > self._command_timeout_sec:
            return twist, 'command timeout'
        if self._emergency_stop:
            return twist, 'emergency stop'
        if not command.enable:
            return twist, 'command disabled'
        if command.brake:
            return twist, 'brake command'

        direction = self._direction(command)
        if direction == 0:
            return twist, 'invalid direction'

        speed = min(abs(command.velocity_mps), self._max_velocity_mps)
        steering = max(
            -self._max_steering_angle_rad,
            min(self._max_steering_angle_rad, command.steering_angle_rad),
        )
        twist.linear.x = direction * speed
        if self._is_pivot_turn(speed, steering):
            twist.linear.x = 0.0
            twist.angular.z = direction * speed / self._pivot_turn_radius
            if steering < 0.0:
                twist.angular.z *= -1.0
        else:
            twist.angular.z = math.tan(steering) * twist.linear.x / self._wheel_base
        twist.angular.z = max(
            -self._max_angular_velocity_radps,
            min(self._max_angular_velocity_radps, twist.angular.z),
        )
        return twist, ''

    def _is_pivot_turn(self, speed: float, steering: float) -> bool:
        if not self._allow_pivot_turn or speed <= 1e-6:
            return False
        return (
            abs(steering)
            >= self._pivot_steering_angle_rad - self._pivot_steering_tolerance_rad
        )

    @staticmethod
    def _direction(command: ForkliftControlCommand) -> int:
        if command.forward and not command.reverse:
            return 1
        if command.reverse and not command.forward:
            return -1
        return 0

    def _publish_debug_state(self, twist: Twist, stop_reason: str) -> None:
        now = self.get_clock().now().to_msg()
        command = self._last_command

        vehicle_state = ForkliftVehicleState()
        vehicle_state.header.stamp = now
        vehicle_state.header.frame_id = 'base_footprint'
        vehicle_state.enabled = bool(command.enable) if command is not None else False
        vehicle_state.auto_mode = self._mode == 'auto'
        vehicle_state.emergency_stopped = self._emergency_stop
        vehicle_state.soft_emergency_stop = self._emergency_stop
        vehicle_state.parking_brake = bool(command.brake) if command is not None else False
        vehicle_state.interlock = vehicle_state.enabled and not self._emergency_stop
        vehicle_state.velocity_mps = twist.linear.x
        if command is not None:
            direction = self._direction(command)
            signed_rpm = direction * abs(command.drive_rpm)
            vehicle_state.left_drive_rpm = signed_rpm
            vehicle_state.right_drive_rpm = signed_rpm
            vehicle_state.steering_angle_rad = command.steering_angle_rad
            vehicle_state.steering_angle_deg = (
                command.steering_angle_deg
                if abs(command.steering_angle_deg) > 1e-9
                else math.degrees(command.steering_angle_rad)
            )
        vehicle_state.mode = self._mode
        self._vehicle_state_pub.publish(vehicle_state)

        fault_state = ForkliftFaultState()
        fault_state.header.stamp = now
        fault_state.has_fault = stop_reason in {'invalid direction', 'emergency stop'}
        fault_state.summary = stop_reason
        self._fault_state_pub.publish(fault_state)

    def _log_stop_reason(self, stop_reason: str) -> None:
        if stop_reason == self._last_stop_reason:
            return
        self._last_stop_reason = stop_reason
        if stop_reason == 'invalid direction':
            self.get_logger().warning('Stopping: forward/reverse command is invalid.')
        elif stop_reason in {'command timeout', 'emergency stop'}:
            self.get_logger().warning(f'Stopping: {stop_reason}.')
        elif stop_reason:
            self.get_logger().info(f'Stopping: {stop_reason}.')


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SimCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
