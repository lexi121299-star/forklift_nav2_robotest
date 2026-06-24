from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Optional

from forklift_vehicle_interface.curtis_can_codec import (
    decode_0x183,
    decode_0x283,
    decode_0x383,
    decode_0x483,
)


FRAME_LEFT_DRIVE = 0x183
FRAME_IO_STATE = 0x283
FRAME_STEERING_RIGHT_DRIVE = 0x383
FRAME_LIFT = 0x483


@dataclass
class CurtisOdomState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    velocity_mps: float = 0.0
    angular_velocity_radps: float = 0.0


@dataclass
class CurtisFeedbackState:
    drive_wheel_radius_m: float = 0.2285
    drive_gear_ratio: float = 26.75
    drive_track_width_m: float = 0.937
    max_integration_dt_sec: float = 0.20

    enabled: bool = False
    auto_mode: bool = False
    emergency_stopped: bool = False
    soft_emergency_stop: bool = False
    parking_brake: bool = False
    interlock: bool = False

    left_drive_rpm: float = 0.0
    right_drive_rpm: float = 0.0
    left_drive_current_a: float = 0.0
    right_drive_current_a: float = 0.0
    steering_angle_rad: float = 0.0
    steering_angle_deg: float = 0.0
    battery_percent: float = 0.0
    drive_controller_temperature_c: float = 0.0
    lift_controller_temperature_c: float = 0.0
    lift_motor_rpm: float = 0.0
    lift_motor_current_a: float = 0.0

    left_drive_fault_code: int = 0
    right_drive_fault_code: int = 0
    steering_fault_code: int = 0
    lift_fault_code: int = 0

    lower_valve_output: bool = False
    lift_valve_output: bool = False
    retract_valve_output: bool = False
    extend_valve_output: bool = False
    side_shift_left_output: bool = False
    side_shift_right_output: bool = False
    tilt_forward_output: bool = False
    tilt_backward_output: bool = False
    brake_relay: bool = False
    reverse_relay: bool = False
    main_contactor: bool = False
    auto_mode_input: bool = False
    slowdown_switch_input: bool = False

    odom: CurtisOdomState = field(default_factory=CurtisOdomState)
    last_feedback_sec: Optional[float] = None
    _last_odom_update_sec: Optional[float] = None

    def update_frame(self, can_id: int, data: Iterable[int], stamp_sec: float) -> bool:
        can_id = int(can_id)
        if can_id == FRAME_LEFT_DRIVE:
            decoded = decode_0x183(data)
            self._integrate_odom(stamp_sec)
            self.left_drive_rpm = float(decoded['left_drive_rpm'])
            self.left_drive_current_a = float(decoded['left_drive_current_a'])
            self.left_drive_fault_code = int(decoded['left_drive_fault_code'])
            self.battery_percent = float(decoded['battery_percent'])
            self.drive_controller_temperature_c = float(
                decoded['drive_controller_temperature_c']
            )
        elif can_id == FRAME_IO_STATE:
            decoded = decode_0x283(data)
            self._apply_io(decoded)
        elif can_id == FRAME_STEERING_RIGHT_DRIVE:
            decoded = decode_0x383(data)
            self._integrate_odom(stamp_sec)
            self.steering_angle_deg = float(decoded['steering_angle_deg'])
            self.steering_angle_rad = float(decoded['steering_angle_rad'])
            self.steering_fault_code = int(decoded['steering_fault_code'])
            self.right_drive_rpm = float(decoded['right_drive_rpm'])
            self.right_drive_current_a = float(decoded['right_drive_current_a'])
            self.right_drive_fault_code = int(decoded['right_drive_fault_code'])
        elif can_id == FRAME_LIFT:
            decoded = decode_0x483(data)
            self.lift_motor_rpm = float(decoded['lift_motor_rpm'])
            self.lift_motor_current_a = float(decoded['lift_motor_current_a'])
            self.lift_controller_temperature_c = float(
                decoded['lift_controller_temperature_c']
            )
            self.lift_fault_code = int(decoded['lift_fault_code'])
        else:
            return False

        self.last_feedback_sec = stamp_sec
        self.enabled = self.main_contactor
        self.auto_mode = self.auto_mode_input
        self.emergency_stopped = self.soft_emergency_stop
        self.interlock = self.main_contactor and self.auto_mode_input and not self.emergency_stopped
        return True

    def feedback_timed_out(self, now_sec: float, timeout_sec: float) -> bool:
        if self.last_feedback_sec is None:
            return True
        return now_sec - self.last_feedback_sec > timeout_sec

    def has_fault(self) -> bool:
        return any(
            code != 0
            for code in (
                self.left_drive_fault_code,
                self.right_drive_fault_code,
                self.steering_fault_code,
                self.lift_fault_code,
            )
        )

    def fault_summary(self, extra: str = '') -> str:
        parts = []
        if self.left_drive_fault_code:
            parts.append(f'left_drive={self.left_drive_fault_code}')
        if self.right_drive_fault_code:
            parts.append(f'right_drive={self.right_drive_fault_code}')
        if self.steering_fault_code:
            parts.append(f'steering={self.steering_fault_code}')
        if self.lift_fault_code:
            parts.append(f'lift={self.lift_fault_code}')
        if extra:
            parts.append(extra)
        return ', '.join(parts)

    def _apply_io(self, decoded: dict) -> None:
        self.lower_valve_output = bool(decoded['lower_valve_output'])
        self.brake_relay = bool(decoded['brake_relay'])
        self.reverse_relay = bool(decoded['reverse_relay'])
        self.main_contactor = bool(decoded['main_contactor'])
        self.auto_mode_input = bool(decoded['auto_mode_input'])
        self.soft_emergency_stop = bool(decoded['soft_emergency_stop_input'])
        self.parking_brake = bool(decoded['parking_brake_input'])
        self.slowdown_switch_input = bool(decoded['slowdown_switch_input'])
        self.lift_valve_output = bool(decoded['lift_valve_output'])
        self.retract_valve_output = bool(decoded['retract_valve_output'])
        self.extend_valve_output = bool(decoded['extend_valve_output'])
        self.side_shift_left_output = bool(decoded['side_shift_left_output'])
        self.side_shift_right_output = bool(decoded['side_shift_right_output'])
        self.tilt_forward_output = bool(decoded['tilt_forward_output'])
        self.tilt_backward_output = bool(decoded['tilt_backward_output'])

    def _integrate_odom(self, stamp_sec: float) -> None:
        if self._last_odom_update_sec is None:
            self._last_odom_update_sec = stamp_sec
            return

        dt = stamp_sec - self._last_odom_update_sec
        self._last_odom_update_sec = stamp_sec
        if dt <= 0.0:
            return
        dt = min(dt, self.max_integration_dt_sec)

        left = self._rpm_to_mps(self.left_drive_rpm)
        right = self._rpm_to_mps(self.right_drive_rpm)
        linear = 0.5 * (left + right)
        angular = 0.0
        if self.drive_track_width_m > 1e-6:
            angular = (right - left) / self.drive_track_width_m

        heading = self.odom.yaw + 0.5 * angular * dt
        self.odom.x += linear * math.cos(heading) * dt
        self.odom.y += linear * math.sin(heading) * dt
        self.odom.yaw = _normalize_angle(self.odom.yaw + angular * dt)
        self.odom.velocity_mps = linear
        self.odom.angular_velocity_radps = angular

    def _rpm_to_mps(self, rpm: float) -> float:
        gear_ratio = max(self.drive_gear_ratio, 1e-6)
        wheel_rpm = rpm / gear_ratio
        return wheel_rpm * (2.0 * math.pi * self.drive_wheel_radius_m) / 60.0


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))
