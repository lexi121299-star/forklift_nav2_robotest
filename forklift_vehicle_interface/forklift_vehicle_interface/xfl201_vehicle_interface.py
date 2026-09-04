from __future__ import annotations

import math
from typing import Iterable, List, Optional, Tuple

import rclpy
from forklift_msgs.msg import (
    ForkliftControlCommand,
    ForkliftFaultState,
    ForkliftIoState,
    ForkliftVehicleState,
)
from forklift_msgs.srv import SetControlMode, SetEmergencyStop
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from forklift_vehicle_interface.curtis_can_transport import SocketCanTransport
from forklift_vehicle_interface.xfl201_vehicle_state import Xfl201FeedbackState
from forklift_vehicle_interface.zhongli_can_codec import (
    FRAME_FORK_COMMAND,
    FRAME_HEARTBEAT,
    FRAME_TRAVEL_COMMAND,
    encode_0x231,
    encode_0x232,
    encode_0x233,
    encode_0x233_from_control,
    format_frame,
)


class Xfl201VehicleInterface(Node):
    """XFL201 vehicle interface with dry-run CAN logging and watchdog stop."""

    def __init__(self) -> None:
        super().__init__('xfl201_vehicle_interface')

        self.declare_parameter('dry_run', True)
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('feedback_poll_rate_hz', 50.0)
        self.declare_parameter('state_publish_rate_hz', 20.0)
        self.declare_parameter('feedback_timeout_sec', 0.5)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('drive_wheel_radius_m', 0.225)
        self.declare_parameter('drive_gear_ratio', 26.75)
        self.declare_parameter('drive_wheel_base_m', 1.47)
        self.declare_parameter('left_meter_per_pulse', 0.0002)
        self.declare_parameter('right_meter_per_pulse', 0.0002)
        self.declare_parameter('left_motor_sign', 1.0)
        self.declare_parameter('right_motor_sign', 1.0)
        self.declare_parameter('left_encoder_sign', 1.0)
        self.declare_parameter('right_encoder_sign', 1.0)
        self.declare_parameter('body_positive_is_fork_reverse', True)
        self.declare_parameter('pivot_steering_angle_rad', math.pi / 2.0)
        self.declare_parameter('pivot_turn_radius_m', 1.743)
        self.declare_parameter('pivot_center_x_offset_m', 0.0)
        self.declare_parameter('max_motor_rpm', 3000.0)
        self.declare_parameter('min_motor_rpm', 100.0)
        self.declare_parameter('rpm_accel_time_sec', 1.0)
        self.declare_parameter('rpm_decel_time_sec', 1.0)
        self.declare_parameter('max_brake_force', 0)
        self.declare_parameter('normal_brake_force', 0)
        self.declare_parameter('send_fork_stop_frame', False)
        self.declare_parameter('fork_control_enabled', False)
        self.declare_parameter('fork_command_full_scale_ma', 800.0)
        self.declare_parameter('steering_angle_fixed_deg', 0.0)
        self.declare_parameter('use_command_steering_angle', True)
        self.declare_parameter('max_integration_dt_sec', 0.20)
        self.declare_parameter('max_rx_frames_per_cycle', 32)

        self._dry_run = self._bool_param('dry_run', True)
        self._can_interface = str(self.get_parameter('can_interface').value)
        self._command_timeout_sec = self._positive_param('command_timeout_sec', 0.5)
        self._feedback_timeout_sec = self._positive_param('feedback_timeout_sec', 0.5)
        self._publish_tf = self._bool_param('publish_tf', True)
        self._odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self._base_frame_id = str(self.get_parameter('base_frame_id').value)
        self._drive_wheel_radius_m = self._positive_param('drive_wheel_radius_m', 0.225)
        self._drive_gear_ratio = self._positive_param('drive_gear_ratio', 26.75)
        self._drive_wheel_base_m = self._positive_param('drive_wheel_base_m', 1.47)
        self._left_motor_sign = self._nonzero_param('left_motor_sign', 1.0)
        self._right_motor_sign = self._nonzero_param('right_motor_sign', 1.0)
        self._body_positive_is_fork_reverse = self._bool_param(
            'body_positive_is_fork_reverse', True
        )
        self._pivot_steering_angle_rad = self._positive_param(
            'pivot_steering_angle_rad', math.pi / 2.0
        )
        self._pivot_turn_radius_m = self._positive_param('pivot_turn_radius_m', 1.743)
        self._pivot_center_x_offset_m = float(
            self.get_parameter('pivot_center_x_offset_m').value
        )
        self._max_motor_rpm = self._positive_param('max_motor_rpm', 3000.0)
        self._min_motor_rpm = max(0.0, float(self.get_parameter('min_motor_rpm').value))
        self._rpm_accel_time_sec = self._positive_param('rpm_accel_time_sec', 1.0)
        self._rpm_decel_time_sec = self._positive_param('rpm_decel_time_sec', 1.0)
        self._max_brake_force = self._byte_param('max_brake_force', 255)
        self._normal_brake_force = self._byte_param('normal_brake_force', 0)
        self._send_fork_stop_frame = self._bool_param('send_fork_stop_frame', False)
        self._fork_control_enabled = self._bool_param('fork_control_enabled', False)
        self._fork_command_full_scale_ma = self._positive_param(
            'fork_command_full_scale_ma', 800.0
        )
        self._steering_angle_fixed_deg = float(self.get_parameter('steering_angle_fixed_deg').value)
        self._use_command_steering_angle = self._bool_param('use_command_steering_angle', True)
        self._max_rx_frames_per_cycle = max(
            1, int(self.get_parameter('max_rx_frames_per_cycle').value)
        )

        self._mode = 'auto'
        self._emergency_stop = False
        self._last_command: Optional[ForkliftControlCommand] = None
        self._last_command_time = self.get_clock().now()
        self._last_stop_reason = ''
        self._last_logged_tx = ''
        self._transport_error = ''
        self._last_left_command_rpm = 0.0
        self._last_right_command_rpm = 0.0
        self._last_tx_sec: Optional[float] = None

        self._feedback = Xfl201FeedbackState(
            drive_wheel_base_m=self._drive_wheel_base_m,
            pivot_steering_angle_rad=self._pivot_steering_angle_rad,
            pivot_turn_radius_m=self._pivot_turn_radius_m,
            pivot_center_x_offset_m=self._pivot_center_x_offset_m,
            left_meter_per_pulse=max(0.0, float(self.get_parameter('left_meter_per_pulse').value)),
            right_meter_per_pulse=max(0.0, float(self.get_parameter('right_meter_per_pulse').value)),
            left_encoder_sign=self._nonzero_param('left_encoder_sign', 1.0),
            right_encoder_sign=self._nonzero_param('right_encoder_sign', 1.0),
            max_integration_dt_sec=self._positive_param('max_integration_dt_sec', 0.20),
        )

        self._transport: Optional[SocketCanTransport] = None
        if not self._dry_run:
            self._transport = SocketCanTransport(self._can_interface)
            self._transport.open()
            self.get_logger().info(f'xfl201_vehicle_interface opened SocketCAN {self._can_interface}.')
        else:
            self.get_logger().warning(
                'xfl201_vehicle_interface running in dry_run mode; CAN frames are logged only.'
            )

        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._vehicle_state_pub = self.create_publisher(
            ForkliftVehicleState, '/forklift/vehicle_state', 10
        )
        self._fault_state_pub = self.create_publisher(
            ForkliftFaultState, '/forklift/fault_state', 10
        )
        self._io_state_pub = self.create_publisher(ForkliftIoState, '/forklift/io_state', 10)

        self.create_subscription(ForkliftControlCommand, '/forklift/control_cmd', self._on_command, 10)
        self.create_service(SetEmergencyStop, '/forklift/set_emergency_stop', self._on_set_emergency_stop)
        self.create_service(SetControlMode, '/forklift/set_control_mode', self._on_set_control_mode)

        self.create_timer(1.0 / self._positive_param('control_rate_hz', 20.0), self._on_tx_timer)
        if not self._dry_run:
            self.create_timer(
                1.0 / self._positive_param('feedback_poll_rate_hz', 50.0),
                self._on_rx_timer,
            )
        self.create_timer(
            1.0 / self._positive_param('state_publish_rate_hz', 20.0),
            self._on_publish_timer,
        )

    def destroy_node(self) -> bool:
        transport = self._transport
        self._transport = None
        if transport is not None:
            transport.close()
        return super().destroy_node()

    def _positive_param(self, name: str, fallback: float) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            self.get_logger().warning(
                f'Parameter {name} must be positive; using fallback {fallback}.'
            )
            return fallback
        return value

    def _nonzero_param(self, name: str, fallback: float) -> float:
        value = float(self.get_parameter(name).value)
        if abs(value) <= 1e-9:
            self.get_logger().warning(
                f'Parameter {name} must be non-zero; using fallback {fallback}.'
            )
            return fallback
        return value

    def _byte_param(self, name: str, fallback: int) -> int:
        value = int(round(float(self.get_parameter(name).value)))
        if value < 0 or value > 255:
            self.get_logger().warning(
                f'Parameter {name} must be in [0, 255]; using fallback {fallback}.'
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
        self._send_frames(self._stop_command(), response.message)
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
        if mode != 'auto':
            self._send_frames(self._stop_command(), response.message)
        return response

    def _on_tx_timer(self) -> None:
        command, stop_reason = self._latest_safe_command()
        self._send_frames(command, stop_reason)
        self._log_stop_reason(stop_reason)

    def _on_rx_timer(self) -> None:
        transport = self._transport
        if transport is None:
            return
        try:
            frames = transport.receive_available(self._max_rx_frames_per_cycle)
        except OSError as exc:
            self._transport_error = f'CAN receive failed: {exc}'
            self.get_logger().error(self._transport_error)
            return

        now_sec = self._now_sec()
        for frame in frames:
            try:
                accepted = self._feedback.update_frame(frame.can_id, frame.data, now_sec)
            except ValueError as exc:
                self.get_logger().warning(
                    f'Ignoring malformed XFL201 feedback 0x{frame.can_id:X}: {exc}'
                )
                continue
            if not accepted:
                self.get_logger().debug(f'Ignoring unrelated CAN frame 0x{frame.can_id:X}.')

    def _on_publish_timer(self) -> None:
        now = self.get_clock().now().to_msg()
        feedback_timeout = self._feedback.feedback_timed_out(
            self._now_sec(), self._feedback_timeout_sec
        )
        self._publish_vehicle_state(now)
        self._publish_io_state(now)
        self._publish_fault_state(now, feedback_timeout)
        self._publish_odom(now)

    def _latest_safe_command(self) -> Tuple[ForkliftControlCommand, str]:
        command = self._last_command
        if command is None:
            return self._stop_command(), 'waiting for first command'

        age_sec = (self.get_clock().now() - self._last_command_time).nanoseconds / 1e9
        if age_sec > self._command_timeout_sec:
            return self._stop_command(), 'command timeout'
        if self._emergency_stop:
            return self._stop_command(), 'emergency stop'
        if self._mode != 'auto':
            return self._stop_command(), f'mode {self._mode}'
        if self._invalid_travel_direction(command):
            return self._stop_command(), 'invalid direction'
        return command, ''

    @staticmethod
    def _invalid_travel_direction(command: ForkliftControlCommand) -> bool:
        wants_travel = abs(command.velocity_mps) > 1e-6 or abs(command.drive_rpm) > 1e-6
        if not wants_travel:
            return False
        return command.forward == command.reverse

    def _send_frames(self, command: ForkliftControlCommand, stop_reason: str) -> None:
        left_rpm, right_rpm, steering_deg = self._motor_command_from_control(command)
        immediate_stop = (
            stop_reason in {'emergency stop', 'command timeout'} or
            command.brake or
            not command.enable
        )
        left_rpm, right_rpm = self._slew_motor_rpms(left_rpm, right_rpm, immediate_stop)
        travel = {
            'left_motor_rpm': left_rpm,
            'right_motor_rpm': right_rpm,
            'steering_angle_deg': steering_deg,
            'brake_force': self._max_brake_force if command.brake else self._normal_brake_force,
            'auto_enable': command.enable and self._mode == 'auto' and not self._emergency_stop,
        }
        frame_231 = encode_0x231(travel)
        frame_232 = encode_0x232()
        frame_233 = None
        if self._fork_control_enabled:
            frame_233 = encode_0x233_from_control(
                command,
                self._fork_command_full_scale_ma,
            )
        elif self._send_fork_stop_frame:
            frame_233 = encode_0x233({})
        if self._dry_run:
            self._log_tx_frames(frame_231, frame_232, frame_233, stop_reason)
            return

        transport = self._transport
        if transport is None:
            return
        try:
            transport.send(FRAME_TRAVEL_COMMAND, frame_231)
            transport.send(FRAME_HEARTBEAT, frame_232)
            if frame_233 is not None:
                transport.send(FRAME_FORK_COMMAND, frame_233)
            self._transport_error = ''
        except OSError as exc:
            self._transport_error = f'CAN send failed: {exc}'
            self.get_logger().error(self._transport_error)

    def _motor_command_from_control(self, command: ForkliftControlCommand) -> Tuple[float, float, float]:
        if not command.enable or command.brake:
            return 0.0, 0.0, self._steering_deg_for_command(command)

        travel_speed = self._travel_speed_from_control(command)
        motor_rpm = self._speed_to_motor_rpm(travel_speed)
        left_rpm = motor_rpm * self._left_motor_sign
        right_rpm = motor_rpm * self._right_motor_sign
        return (
            self._clamp_motor_rpm(left_rpm),
            self._clamp_motor_rpm(right_rpm),
            self._steering_deg_for_command(command),
        )

    def _travel_speed_from_control(self, command: ForkliftControlCommand) -> float:
        direction = self._direction(command)
        if direction == 0:
            return 0.0

        speed = abs(command.velocity_mps)
        if abs(command.drive_rpm) > 1e-6 and speed <= 1e-6:
            speed = self._motor_rpm_to_speed(abs(command.drive_rpm))
        if speed <= 1e-9:
            return 0.0

        v = direction * speed
        if self._body_positive_is_fork_reverse:
            v *= -1.0
        return v

    def _direction(self, command: ForkliftControlCommand) -> int:
        if command.forward and not command.reverse:
            return 1
        if command.reverse and not command.forward:
            return -1
        return 0

    def _steering_deg_for_command(self, command: ForkliftControlCommand) -> float:
        if not self._use_command_steering_angle:
            return self._steering_angle_fixed_deg
        if abs(command.steering_angle_deg) > 1e-9:
            return command.steering_angle_deg
        return math.degrees(command.steering_angle_rad)

    def _speed_to_motor_rpm(self, speed_mps: float) -> float:
        if abs(speed_mps) <= 1e-9:
            return 0.0
        wheel_rpm = speed_mps * 60.0 / (2.0 * math.pi * self._drive_wheel_radius_m)
        motor_rpm = wheel_rpm * self._drive_gear_ratio
        if abs(motor_rpm) < self._min_motor_rpm:
            motor_rpm = math.copysign(self._min_motor_rpm, motor_rpm)
        return motor_rpm

    def _motor_rpm_to_speed(self, motor_rpm: float) -> float:
        wheel_rpm = motor_rpm / self._drive_gear_ratio
        return wheel_rpm * (2.0 * math.pi * self._drive_wheel_radius_m) / 60.0

    def _clamp_motor_rpm(self, rpm: float) -> float:
        return max(-self._max_motor_rpm, min(self._max_motor_rpm, rpm))

    def _slew_motor_rpms(
        self,
        target_left_rpm: float,
        target_right_rpm: float,
        immediate_stop: bool = False,
    ) -> Tuple[float, float]:
        if immediate_stop:
            self._last_left_command_rpm = target_left_rpm
            self._last_right_command_rpm = target_right_rpm
            self._last_tx_sec = self._now_sec()
            return target_left_rpm, target_right_rpm

        now_sec = self._now_sec()
        if self._last_tx_sec is None:
            self._last_tx_sec = now_sec
            self._last_left_command_rpm = target_left_rpm
            self._last_right_command_rpm = target_right_rpm
            return target_left_rpm, target_right_rpm

        dt = max(0.0, now_sec - self._last_tx_sec)
        self._last_tx_sec = now_sec
        accel_step = self._max_motor_rpm * dt / max(1e-6, self._rpm_accel_time_sec)
        decel_step = self._max_motor_rpm * dt / max(1e-6, self._rpm_decel_time_sec)

        self._last_left_command_rpm = self._slew_one_rpm(
            self._last_left_command_rpm, target_left_rpm, accel_step, decel_step)
        self._last_right_command_rpm = self._slew_one_rpm(
            self._last_right_command_rpm, target_right_rpm, accel_step, decel_step)
        return self._last_left_command_rpm, self._last_right_command_rpm

    @staticmethod
    def _slew_one_rpm(current: float, target: float, accel_step: float, decel_step: float) -> float:
        delta = target - current
        if abs(delta) <= 1e-9:
            return target
        step = decel_step if abs(target) < abs(current) else accel_step
        if abs(delta) <= step:
            return target
        return current + math.copysign(step, delta)

    def _log_tx_frames(
        self,
        frame_231: Iterable[int],
        frame_232: Iterable[int],
        frame_233: Optional[Iterable[int]],
        stop_reason: str,
    ) -> None:
        text = (
            f'0x231 [{format_frame(frame_231)}], '
            f'0x232 [{format_frame(frame_232)}]'
        )
        if frame_233 is not None:
            text = f'{text}, 0x233 [{format_frame(frame_233)}]'
        if stop_reason:
            text = f'{text}, stop_reason={stop_reason}'
        if text == self._last_logged_tx:
            return
        self._last_logged_tx = text
        self.get_logger().info(f'XFL201 TX dry-run: {text}')

    @staticmethod
    def _stop_command() -> ForkliftControlCommand:
        command = ForkliftControlCommand()
        command.enable = False
        command.brake = True
        command.forward = False
        command.reverse = False
        command.velocity_mps = 0.0
        command.drive_rpm = 0.0
        command.steering_angle_rad = 0.0
        command.steering_angle_deg = 0.0
        command.accel_time_sec = 0.0
        command.decel_time_sec = 0.0
        command.pump_rpm = 0.0
        return command

    def _publish_vehicle_state(self, stamp) -> None:
        msg = ForkliftVehicleState()
        msg.header.stamp = stamp
        msg.header.frame_id = self._base_frame_id
        msg.enabled = self._feedback.enabled
        msg.auto_mode = self._mode == 'auto' and self._feedback.auto_mode
        msg.emergency_stopped = self._emergency_stop or self._feedback.emergency_stopped
        msg.soft_emergency_stop = self._feedback.soft_emergency_stop
        msg.parking_brake = self._feedback.parking_brake
        msg.interlock = self._feedback.interlock and not self._emergency_stop
        msg.velocity_mps = self._feedback.odom.velocity_mps
        msg.left_drive_rpm = self._feedback.left_drive_rpm
        msg.right_drive_rpm = self._feedback.right_drive_rpm
        msg.steering_angle_rad = self._feedback.steering_angle_rad
        msg.steering_angle_deg = self._feedback.steering_angle_deg
        msg.battery_percent = self._feedback.battery_percent
        msg.mode = self._mode
        self._vehicle_state_pub.publish(msg)

    def _publish_io_state(self, stamp) -> None:
        msg = ForkliftIoState()
        msg.header.stamp = stamp
        msg.main_contactor = self._feedback.enabled
        msg.auto_mode_input = self._feedback.auto_mode
        msg.soft_emergency_stop_input = self._feedback.soft_emergency_stop
        msg.parking_brake_input = self._feedback.parking_brake
        self._io_state_pub.publish(msg)

    def _publish_fault_state(self, stamp, feedback_timeout: bool) -> None:
        msg = ForkliftFaultState()
        msg.header.stamp = stamp
        msg.left_drive_fault_code = self._feedback.travel_fault_code
        msg.right_drive_fault_code = self._feedback.travel_fault_code
        msg.steering_fault_code = self._feedback.steering_fault_code
        msg.lift_fault_code = self._feedback.vcm_valve_fault_code
        extra = ''
        if feedback_timeout and not self._dry_run:
            extra = 'feedback timeout'
        if self._transport_error:
            extra = f'{extra}, {self._transport_error}' if extra else self._transport_error
        if self._emergency_stop:
            extra = f'{extra}, emergency stop' if extra else 'emergency stop'
        msg.has_fault = self._feedback.has_fault() or bool(extra)
        msg.summary = self._feedback.fault_summary(extra)
        self._fault_state_pub.publish(msg)

    def _publish_odom(self, stamp) -> None:
        odom = self._feedback.odom
        quat = _yaw_to_quaternion(odom.yaw)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._odom_frame_id
        msg.child_frame_id = self._base_frame_id
        msg.pose.pose.position.x = odom.x
        msg.pose.pose.position.y = odom.y
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        msg.twist.twist.linear.x = odom.velocity_mps
        msg.twist.twist.angular.z = odom.angular_velocity_radps
        self._odom_pub.publish(msg)

        if self._tf_broadcaster is None:
            return
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._odom_frame_id
        transform.child_frame_id = self._base_frame_id
        transform.transform.translation.x = odom.x
        transform.transform.translation.y = odom.y
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        self._tf_broadcaster.sendTransform(transform)

    def _log_stop_reason(self, stop_reason: str) -> None:
        if stop_reason == self._last_stop_reason:
            return
        self._last_stop_reason = stop_reason
        if stop_reason in {'command timeout', 'emergency stop', 'invalid direction'}:
            self.get_logger().warning(f'Stopping XFL201 command output: {stop_reason}.')
        elif stop_reason:
            self.get_logger().info(f'Stopping XFL201 command output: {stop_reason}.')

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def _yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = Xfl201VehicleInterface()
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
