from __future__ import annotations

import math
from typing import List, Optional, Tuple

import rclpy
from forklift_msgs.msg import ForkliftControlCommand, ForkliftFaultState, ForkliftVehicleState
from forklift_msgs.srv import SetEmergencyStop
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


def positive(value: float, fallback: float) -> float:
    return value if value > 0.0 else fallback


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def direction(command: ForkliftControlCommand) -> int:
    if command.forward and not command.reverse:
        return 1
    if command.reverse and not command.forward:
        return -1
    return 0


def stop_command(stamp=None) -> ForkliftControlCommand:
    command = ForkliftControlCommand()
    if stamp is not None:
        command.header.stamp = stamp
    command.enable = False
    command.brake = True
    command.forward = False
    command.reverse = False
    command.velocity_mps = 0.0
    command.drive_rpm = 0.0
    command.steering_angle_rad = 0.0
    command.steering_angle_deg = 0.0
    return command


def clamp_control_command(
    command: ForkliftControlCommand,
    max_forward_velocity_mps: float,
    max_reverse_velocity_mps: float,
    max_steering_angle_rad: float,
) -> ForkliftControlCommand:
    gated = ForkliftControlCommand()
    gated.header = command.header
    gated.enable = command.enable
    gated.brake = command.brake
    gated.forward = command.forward
    gated.reverse = command.reverse
    gated.accel_time_sec = command.accel_time_sec
    gated.decel_time_sec = command.decel_time_sec
    gated.pump_rpm = command.pump_rpm
    gated.lift_valve_ma = command.lift_valve_ma
    gated.lower_valve_ma = command.lower_valve_ma
    gated.side_shift_left_valve_ma = command.side_shift_left_valve_ma
    gated.side_shift_right_valve_ma = command.side_shift_right_valve_ma
    gated.tilt_forward_valve_ma = command.tilt_forward_valve_ma
    gated.tilt_backward_valve_ma = command.tilt_backward_valve_ma
    gated.horn = command.horn
    gated.light = command.light

    travel_direction = direction(command)
    speed_limit = max_forward_velocity_mps if travel_direction >= 0 else max_reverse_velocity_mps
    speed = clamp(abs(command.velocity_mps), 0.0, speed_limit)
    gated.velocity_mps = speed
    gated.drive_rpm = math.copysign(abs(command.drive_rpm), command.drive_rpm)
    gated.steering_angle_rad = clamp(
        command.steering_angle_rad,
        -max_steering_angle_rad,
        max_steering_angle_rad,
    )
    gated.steering_angle_deg = math.degrees(gated.steering_angle_rad)
    return gated


def recovery_command_from_twist(
    twist: Twist,
    max_recovery_velocity_mps: float,
    max_recovery_angular_velocity_radps: float,
    wheel_base: float,
    pivot_turn_radius: float,
    pivot_steering_angle_rad: float,
    allow_recovery_backoff: bool,
    allow_recovery_pivot: bool,
    stamp=None,
) -> Tuple[ForkliftControlCommand, str]:
    linear = clamp(
        twist.linear.x,
        -max_recovery_velocity_mps,
        max_recovery_velocity_mps,
    )
    angular = clamp(
        twist.angular.z,
        -max_recovery_angular_velocity_radps,
        max_recovery_angular_velocity_radps,
    )

    if abs(linear) <= 1e-6 and abs(angular) <= 1e-6:
        return stop_command(stamp), 'recovery wait'

    if abs(linear) > 1e-6:
        if linear < 0.0 and not allow_recovery_backoff:
            return stop_command(stamp), 'recovery backoff disabled'
        command = ForkliftControlCommand()
        if stamp is not None:
            command.header.stamp = stamp
        command.enable = True
        command.brake = False
        command.forward = linear > 0.0
        command.reverse = linear < 0.0
        command.velocity_mps = abs(linear)
        if abs(angular) > 1e-6:
            steering = math.atan2(angular * wheel_base, linear)
            command.steering_angle_rad = clamp(
                steering,
                -pivot_steering_angle_rad,
                pivot_steering_angle_rad,
            )
            command.steering_angle_deg = math.degrees(command.steering_angle_rad)
        return command, 'recovery backoff' if linear < 0.0 else 'recovery forward'

    if not allow_recovery_pivot:
        return stop_command(stamp), 'recovery pivot disabled'

    command = ForkliftControlCommand()
    if stamp is not None:
        command.header.stamp = stamp
    command.enable = True
    command.brake = False
    command.forward = True
    command.reverse = False
    command.velocity_mps = min(
        max_recovery_velocity_mps,
        abs(angular) * positive(pivot_turn_radius, 0.6),
    )
    command.steering_angle_rad = (
        pivot_steering_angle_rad if angular > 0.0 else -pivot_steering_angle_rad
    )
    command.steering_angle_deg = math.degrees(command.steering_angle_rad)
    return command, 'recovery pivot'


class SafetyCommandGate(Node):
    """Gate all motion commands before they reach the vehicle interface."""

    def __init__(self) -> None:
        super().__init__('safety_command_gate')

        self.declare_parameter('enabled', True)
        self.declare_parameter('raw_command_topic', '/forklift/control_cmd_raw')
        self.declare_parameter('gated_command_topic', '/forklift/control_cmd')
        self.declare_parameter('recovery_twist_topic', '/cmd_vel')
        self.declare_parameter('vehicle_state_topic', '/forklift/vehicle_state')
        self.declare_parameter('fault_state_topic', '/forklift/fault_state')
        self.declare_parameter('localization_topic', '/odom')
        self.declare_parameter('status_topic', '/forklift/safety_gate/status')
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('recovery_timeout_sec', 0.5)
        self.declare_parameter('vehicle_state_timeout_sec', 0.5)
        self.declare_parameter('fault_state_timeout_sec', 0.5)
        self.declare_parameter('localization_timeout_sec', 0.5)
        self.declare_parameter('require_vehicle_state', False)
        self.declare_parameter('require_fault_state', False)
        self.declare_parameter('require_localization', False)
        self.declare_parameter('emergency_stop_active', False)
        self.declare_parameter('allow_recovery_twist', True)
        self.declare_parameter('allow_recovery_backoff', True)
        self.declare_parameter('allow_recovery_pivot', True)
        self.declare_parameter('max_forward_velocity_mps', 0.45)
        self.declare_parameter('max_reverse_velocity_mps', 0.15)
        self.declare_parameter('max_recovery_velocity_mps', 0.10)
        self.declare_parameter('max_recovery_angular_velocity_radps', 0.30)
        self.declare_parameter('max_steering_angle_rad', math.pi / 2.0)
        self.declare_parameter('wheel_base', 1.2)
        self.declare_parameter('pivot_turn_radius', 0.6)
        self.declare_parameter('pivot_steering_angle_rad', math.pi / 2.0)
        self.declare_parameter('control_rate_hz', 20.0)

        self._enabled = bool(self.get_parameter('enabled').value)
        self._raw_command_topic = str(self.get_parameter('raw_command_topic').value)
        self._gated_command_topic = str(self.get_parameter('gated_command_topic').value)
        self._recovery_twist_topic = str(self.get_parameter('recovery_twist_topic').value)
        self._vehicle_state_topic = str(self.get_parameter('vehicle_state_topic').value)
        self._fault_state_topic = str(self.get_parameter('fault_state_topic').value)
        self._localization_topic = str(self.get_parameter('localization_topic').value)
        self._status_topic = str(self.get_parameter('status_topic').value)
        self._command_timeout_sec = self._positive_param('command_timeout_sec', 0.5)
        self._recovery_timeout_sec = self._positive_param('recovery_timeout_sec', 0.5)
        self._vehicle_state_timeout_sec = self._positive_param('vehicle_state_timeout_sec', 0.5)
        self._fault_state_timeout_sec = self._positive_param('fault_state_timeout_sec', 0.5)
        self._localization_timeout_sec = self._positive_param('localization_timeout_sec', 0.5)
        self._require_vehicle_state = bool(self.get_parameter('require_vehicle_state').value)
        self._require_fault_state = bool(self.get_parameter('require_fault_state').value)
        self._require_localization = bool(self.get_parameter('require_localization').value)
        self._emergency_stop = bool(self.get_parameter('emergency_stop_active').value)
        self._allow_recovery_twist = bool(self.get_parameter('allow_recovery_twist').value)
        self._allow_recovery_backoff = bool(self.get_parameter('allow_recovery_backoff').value)
        self._allow_recovery_pivot = bool(self.get_parameter('allow_recovery_pivot').value)
        self._max_forward_velocity_mps = self._positive_param('max_forward_velocity_mps', 0.45)
        self._max_reverse_velocity_mps = self._positive_param('max_reverse_velocity_mps', 0.15)
        self._max_recovery_velocity_mps = self._positive_param('max_recovery_velocity_mps', 0.10)
        self._max_recovery_angular_velocity_radps = self._positive_param(
            'max_recovery_angular_velocity_radps',
            0.30,
        )
        self._max_steering_angle_rad = self._positive_param(
            'max_steering_angle_rad',
            math.pi / 2.0,
        )
        self._wheel_base = self._positive_param('wheel_base', 1.2)
        self._pivot_turn_radius = self._positive_param('pivot_turn_radius', 0.6)
        self._pivot_steering_angle_rad = min(
            self._positive_param('pivot_steering_angle_rad', math.pi / 2.0),
            self._max_steering_angle_rad,
        )
        control_rate_hz = self._positive_param('control_rate_hz', 20.0)

        self._last_raw_command: Optional[ForkliftControlCommand] = None
        self._last_raw_command_time = self.get_clock().now()
        self._last_recovery_twist: Optional[Twist] = None
        self._last_recovery_twist_time = self.get_clock().now()
        self._last_vehicle_state: Optional[ForkliftVehicleState] = None
        self._last_vehicle_state_time = self.get_clock().now()
        self._last_fault_state: Optional[ForkliftFaultState] = None
        self._last_fault_state_time = self.get_clock().now()
        self._last_localization_time = self.get_clock().now()
        self._last_reason = ''

        self._command_pub = self.create_publisher(
            ForkliftControlCommand,
            self._gated_command_topic,
            10,
        )
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self.create_subscription(
            ForkliftControlCommand,
            self._raw_command_topic,
            self._on_raw_command,
            10,
        )
        if self._recovery_twist_topic:
            self.create_subscription(Twist, self._recovery_twist_topic, self._on_recovery_twist, 10)
        if self._vehicle_state_topic:
            self.create_subscription(
                ForkliftVehicleState,
                self._vehicle_state_topic,
                self._on_vehicle_state,
                10,
            )
        if self._fault_state_topic:
            self.create_subscription(
                ForkliftFaultState,
                self._fault_state_topic,
                self._on_fault_state,
                10,
            )
        if self._localization_topic:
            self.create_subscription(Odometry, self._localization_topic, self._on_localization, 10)
        self.create_service(
            SetEmergencyStop,
            '/forklift_safety/set_emergency_stop',
            self._on_set_emergency_stop,
        )
        self.create_timer(1.0 / control_rate_hz, self._on_timer)
        self.get_logger().info(
            f'safety_command_gate ready: {self._raw_command_topic} -> '
            f'{self._gated_command_topic}, recovery={self._recovery_twist_topic or "disabled"}'
        )

    def _positive_param(self, name: str, fallback: float) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            self.get_logger().warning(
                f'Parameter {name} must be positive; using fallback {fallback}.'
            )
            return fallback
        return value

    def _on_raw_command(self, msg: ForkliftControlCommand) -> None:
        self._last_raw_command = msg
        self._last_raw_command_time = self.get_clock().now()

    def _on_recovery_twist(self, msg: Twist) -> None:
        self._last_recovery_twist = msg
        self._last_recovery_twist_time = self.get_clock().now()

    def _on_vehicle_state(self, msg: ForkliftVehicleState) -> None:
        self._last_vehicle_state = msg
        self._last_vehicle_state_time = self.get_clock().now()

    def _on_fault_state(self, msg: ForkliftFaultState) -> None:
        self._last_fault_state = msg
        self._last_fault_state_time = self.get_clock().now()

    def _on_localization(self, _msg: Odometry) -> None:
        self._last_localization_time = self.get_clock().now()

    def _on_set_emergency_stop(
        self,
        request: SetEmergencyStop.Request,
        response: SetEmergencyStop.Response,
    ) -> SetEmergencyStop.Response:
        self._emergency_stop = bool(request.emergency_stop)
        response.success = True
        response.message = (
            'safety gate emergency stop enabled'
            if self._emergency_stop else
            'safety gate emergency stop cleared'
        )
        self.get_logger().warning(response.message)
        return response

    def _on_timer(self) -> None:
        command, reason = self._latest_safe_command()
        self._command_pub.publish(command)
        self._publish_status(reason)
        self._log_reason(reason)

    def _latest_safe_command(self) -> Tuple[ForkliftControlCommand, str]:
        stamp = self.get_clock().now().to_msg()
        stop_reason = self._stop_reason_from_health()
        if stop_reason:
            return stop_command(stamp), stop_reason

        raw_age = (self.get_clock().now() - self._last_raw_command_time).nanoseconds / 1e9
        if self._last_raw_command is not None and raw_age <= self._command_timeout_sec:
            command = clamp_control_command(
                self._last_raw_command,
                self._max_forward_velocity_mps,
                self._max_reverse_velocity_mps,
                self._max_steering_angle_rad,
            )
            command.header.stamp = stamp
            if not self._enabled:
                return command, 'bypass'
            if not command.enable or command.brake:
                return command, 'raw stop'
            if direction(command) == 0:
                return stop_command(stamp), 'invalid direction'
            return command, 'raw command'

        if self._allow_recovery_twist and self._last_recovery_twist is not None:
            recovery_age = (
                self.get_clock().now() - self._last_recovery_twist_time
            ).nanoseconds / 1e9
            if recovery_age <= self._recovery_timeout_sec:
                command, reason = recovery_command_from_twist(
                    self._last_recovery_twist,
                    self._max_recovery_velocity_mps,
                    self._max_recovery_angular_velocity_radps,
                    self._wheel_base,
                    self._pivot_turn_radius,
                    self._pivot_steering_angle_rad,
                    self._allow_recovery_backoff,
                    self._allow_recovery_pivot,
                    stamp,
                )
                return command, reason

        if self._last_raw_command is None:
            return stop_command(stamp), 'waiting for first command'
        return stop_command(stamp), 'command timeout'

    def _stop_reason_from_health(self) -> str:
        now = self.get_clock().now()
        if self._emergency_stop:
            return 'emergency stop'

        vehicle_state = self._last_vehicle_state
        if vehicle_state is None:
            if self._require_vehicle_state:
                return 'vehicle state missing'
        else:
            age = (now - self._last_vehicle_state_time).nanoseconds / 1e9
            if self._require_vehicle_state and age > self._vehicle_state_timeout_sec:
                return 'vehicle state timeout'
            if vehicle_state.emergency_stopped or vehicle_state.soft_emergency_stop:
                return 'vehicle emergency stop'
            if self._require_vehicle_state and vehicle_state.parking_brake:
                return 'parking brake'
            if not vehicle_state.interlock and self._require_vehicle_state:
                return 'vehicle interlock open'

        fault_state = self._last_fault_state
        if fault_state is None:
            if self._require_fault_state:
                return 'fault state missing'
        else:
            age = (now - self._last_fault_state_time).nanoseconds / 1e9
            if self._require_fault_state and age > self._fault_state_timeout_sec:
                return 'fault state timeout'
            if fault_state.has_fault:
                return 'vehicle fault'

        if self._require_localization:
            age = (now - self._last_localization_time).nanoseconds / 1e9
            if age > self._localization_timeout_sec:
                return 'localization timeout'

        return ''

    def _publish_status(self, reason: str) -> None:
        status = String()
        status.data = reason
        self._status_pub.publish(status)

    def _log_reason(self, reason: str) -> None:
        if reason == self._last_reason:
            return
        self._last_reason = reason
        if reason in {
            'emergency stop',
            'vehicle emergency stop',
            'vehicle fault',
            'invalid direction',
            'localization timeout',
        }:
            self.get_logger().warning(f'Safety gate stopping: {reason}.')
        elif reason in {'raw command', 'recovery backoff', 'recovery pivot', 'recovery forward'}:
            self.get_logger().info(f'Safety gate passing: {reason}.')
        elif reason:
            self.get_logger().info(f'Safety gate state: {reason}.')


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SafetyCommandGate()
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
