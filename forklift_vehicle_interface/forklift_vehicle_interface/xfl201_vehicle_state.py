from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Optional

from forklift_vehicle_interface.zhongli_can_codec import (
    FRAME_FAULT_FEEDBACK,
    FRAME_LEFT_ENCODER,
    FRAME_RIGHT_ENCODER,
    FRAME_VCM_FAULT,
    FRAME_VEHICLE_FEEDBACK,
    decode_0x206,
    decode_0x207,
    decode_0x209,
    decode_0x20a,
    decode_0x6db,
)


@dataclass
class Xfl201OdomState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    velocity_mps: float = 0.0
    angular_velocity_radps: float = 0.0


@dataclass
class Xfl201FeedbackState:
    drive_track_width_m: float = 0.80
    left_meter_per_pulse: float = 0.0
    right_meter_per_pulse: float = 0.0
    left_encoder_sign: float = 1.0
    right_encoder_sign: float = 1.0
    max_integration_dt_sec: float = 0.20

    enabled: bool = False
    auto_mode: bool = False
    manual_mode: bool = False
    emergency_stopped: bool = False
    soft_emergency_stop: bool = False
    parking_brake: bool = False
    interlock: bool = False

    left_drive_rpm: float = 0.0
    right_drive_rpm: float = 0.0
    steering_angle_rad: float = 0.0
    steering_angle_deg: float = 0.0
    battery_percent: float = 0.0

    travel_fault_code: int = 0
    steering_fault_code: int = 0
    vcm_valve_fault_code: int = 0

    left_motor_pulse_count: Optional[int] = None
    right_motor_pulse_count: Optional[int] = None

    odom: Xfl201OdomState = field(default_factory=Xfl201OdomState)
    last_feedback_sec: Optional[float] = None
    _last_left_motor_pulse_count: Optional[int] = None
    _last_right_motor_pulse_count: Optional[int] = None
    _last_odom_update_sec: Optional[float] = None
    _left_encoder_dirty: bool = False
    _right_encoder_dirty: bool = False

    def update_frame(self, can_id: int, data: Iterable[int], stamp_sec: float) -> bool:
        can_id = int(can_id)
        if can_id == FRAME_VEHICLE_FEEDBACK:
            decoded = decode_0x206(data)
            self.left_drive_rpm = float(decoded['left_motor_rpm'])
            self.right_drive_rpm = float(decoded['right_motor_rpm'])
            self.steering_angle_deg = float(decoded['steering_angle_deg'])
            self.steering_angle_rad = float(decoded['steering_angle_rad'])
            self.battery_percent = float(decoded['battery_percent'])
            self.manual_mode = bool(decoded['manual_mode'])
            self.auto_mode = bool(decoded['auto_mode'])
        elif can_id == FRAME_FAULT_FEEDBACK:
            decoded = decode_0x207(data)
            self.travel_fault_code = int(decoded['travel_fault_code'])
            self.steering_fault_code = int(decoded['steering_fault_code'])
        elif can_id == FRAME_LEFT_ENCODER:
            decoded = decode_0x209(data)
            self.left_motor_pulse_count = int(decoded['left_motor_pulse_count'])
            self._left_encoder_dirty = True
            self._integrate_encoder_delta(stamp_sec)
        elif can_id == FRAME_RIGHT_ENCODER:
            decoded = decode_0x20a(data)
            self.right_motor_pulse_count = int(decoded['right_motor_pulse_count'])
            self._right_encoder_dirty = True
            self._integrate_encoder_delta(stamp_sec)
        elif can_id == FRAME_VCM_FAULT:
            decoded = decode_0x6db(data)
            self.vcm_valve_fault_code = int(decoded['vcm_valve_fault_code'])
        else:
            return False

        self.last_feedback_sec = stamp_sec
        self.enabled = self.auto_mode
        self.interlock = self.auto_mode and not self.emergency_stopped
        return True

    def feedback_timed_out(self, now_sec: float, timeout_sec: float) -> bool:
        if self.last_feedback_sec is None:
            return True
        return now_sec - self.last_feedback_sec > timeout_sec

    def odom_enabled(self) -> bool:
        return (
            self.drive_track_width_m > 1e-6 and
            self.left_meter_per_pulse > 0.0 and
            self.right_meter_per_pulse > 0.0
        )

    def has_fault(self) -> bool:
        return any(
            code != 0
            for code in (
                self.travel_fault_code,
                self.steering_fault_code,
                self.vcm_valve_fault_code,
            )
        )

    def fault_summary(self, extra: str = '') -> str:
        parts = []
        if self.travel_fault_code:
            parts.append(f'travel={self.travel_fault_code}')
        if self.steering_fault_code:
            parts.append(f'steering={self.steering_fault_code}')
        if self.vcm_valve_fault_code:
            parts.append(f'vcm_valve={self.vcm_valve_fault_code}')
        if extra:
            parts.append(extra)
        return ', '.join(parts)

    def _integrate_encoder_delta(self, stamp_sec: float) -> None:
        left = self.left_motor_pulse_count
        right = self.right_motor_pulse_count
        if left is None or right is None:
            return

        if self._last_left_motor_pulse_count is None or self._last_right_motor_pulse_count is None:
            self._last_left_motor_pulse_count = left
            self._last_right_motor_pulse_count = right
            self._last_odom_update_sec = stamp_sec
            self._left_encoder_dirty = False
            self._right_encoder_dirty = False
            return

        if not self._left_encoder_dirty or not self._right_encoder_dirty:
            return

        delta_left_count = left - self._last_left_motor_pulse_count
        delta_right_count = right - self._last_right_motor_pulse_count
        self._left_encoder_dirty = False
        self._right_encoder_dirty = False
        if delta_left_count == 0 and delta_right_count == 0:
            self._last_odom_update_sec = stamp_sec
            return

        self._last_left_motor_pulse_count = left
        self._last_right_motor_pulse_count = right

        if not self.odom_enabled():
            self._last_odom_update_sec = stamp_sec
            return

        dt = 0.0
        if self._last_odom_update_sec is not None:
            dt = max(0.0, min(stamp_sec - self._last_odom_update_sec, self.max_integration_dt_sec))
        self._last_odom_update_sec = stamp_sec

        left_distance = (
            float(delta_left_count) * self.left_meter_per_pulse * self.left_encoder_sign
        )
        right_distance = (
            float(delta_right_count) * self.right_meter_per_pulse * self.right_encoder_sign
        )

        delta_s = 0.5 * (left_distance + right_distance)
        delta_yaw = (right_distance - left_distance) / self.drive_track_width_m
        heading = self.odom.yaw + 0.5 * delta_yaw
        self.odom.x += delta_s * math.cos(heading)
        self.odom.y += delta_s * math.sin(heading)
        self.odom.yaw = _normalize_angle(self.odom.yaw + delta_yaw)
        if dt > 1e-9:
            self.odom.velocity_mps = delta_s / dt
            self.odom.angular_velocity_radps = delta_yaw / dt


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))
