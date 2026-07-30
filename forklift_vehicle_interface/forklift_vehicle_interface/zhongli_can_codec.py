"""Zhongli XFL201 CAN frame codec helpers.

The XFL201 protocol uses standard 8-byte CAN frames at 125 kbit/s.  These
helpers are intentionally free of ROS dependencies so the byte layout can be
unit-tested before the vehicle interface is connected to SocketCAN.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Iterable, List


FRAME_SIZE = 8

FRAME_TRAVEL_COMMAND = 0x231
FRAME_HEARTBEAT = 0x232
FRAME_FORK_COMMAND = 0x233
FRAME_VEHICLE_FEEDBACK = 0x206
FRAME_FAULT_FEEDBACK = 0x207
FRAME_LEFT_ENCODER = 0x209
FRAME_RIGHT_ENCODER = 0x20A
FRAME_VCM_FAULT = 0x6DB


def _get(source: Any, name: str, default: Any = 0) -> Any:
    if isinstance(source, dict):
        return source.get(name, default)
    return getattr(source, name, default)


def _first(source: Any, names: Iterable[str], default: Any = 0) -> Any:
    for name in names:
        value = _get(source, name, None)
        if value is not None:
            return value
    return default


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _frame(data: Iterable[int]) -> List[int]:
    frame = [int(byte) & 0xFF for byte in data]
    if len(frame) != FRAME_SIZE:
        raise ValueError(f'Zhongli CAN frames must be {FRAME_SIZE} bytes, got {len(frame)}')
    return frame


def _i16_be(value: int) -> List[int]:
    value = int(_clamp(value, -32768, 32767)) & 0xFFFF
    return [(value >> 8) & 0xFF, value & 0xFF]


def _read_i16_be(data: Iterable[int], offset: int) -> int:
    frame = _frame(data)
    value = (frame[offset] << 8) | frame[offset + 1]
    if value & 0x8000:
        value -= 0x10000
    return value


def _read_i64_le(data: Iterable[int]) -> int:
    frame = _frame(data)
    value = 0
    for index, byte in enumerate(frame):
        value |= byte << (8 * index)
    if value & (1 << 63):
        value -= 1 << 64
    return value


def _steering_deg(command: Any) -> float:
    steering_deg = float(_first(command, ('steering_angle_deg', 'steering_deg'), 0.0))
    steering_rad = float(_first(command, ('steering_angle_rad', 'steering_rad'), 0.0))
    if abs(steering_deg) < 1e-9 and abs(steering_rad) > 1e-9:
        steering_deg = math.degrees(steering_rad)
    return _clamp(steering_deg, -90.0, 90.0)


def _byte_from_percent_or_raw(source: Any, percent_name: str, raw_name: str) -> int:
    raw_value = _get(source, raw_name, None)
    if raw_value is not None:
        return int(_clamp(round(float(raw_value)), 0, 255))

    percent = float(_get(source, percent_name, 0.0))
    return int(_clamp(round(percent * 255.0 / 100.0), 0, 255))


def encode_0x231(command: Any) -> List[int]:
    """Encode XFL201 travel command frame 0x231.

    Protocol fields:
    - BYTE0-1: left motor speed, signed int16 RPM, high byte first.
    - BYTE2-3: right motor speed, signed int16 RPM, high byte first.
    - BYTE4-5: steering wheel angle, signed int16, 0.01 deg, high byte first.
    - BYTE6: brake force, 0..255.
    - BYTE7 bit0: automatic enable / interlock.
    """

    left_rpm = int(round(float(_first(command, ('left_motor_rpm', 'left_drive_rpm'), 0.0))))
    right_rpm = int(round(float(_first(command, ('right_motor_rpm', 'right_drive_rpm'), 0.0))))
    steering_centideg = int(round(_steering_deg(command) * 100.0))

    brake_raw = _get(command, 'brake_force', None)
    if brake_raw is None:
        brake_raw = _get(command, 'brake_force_raw', None)
    if brake_raw is None:
        brake_raw = 255 if bool(_get(command, 'brake', False)) else 0
    brake = int(_clamp(round(float(brake_raw)), 0, 255))

    byte7 = 0
    if bool(_first(command, ('auto_enable', 'enable', 'interlock'), False)):
        byte7 |= 1 << 0

    frame = [0] * FRAME_SIZE
    frame[0:2] = _i16_be(left_rpm)
    frame[2:4] = _i16_be(right_rpm)
    frame[4:6] = _i16_be(steering_centideg)
    frame[6] = brake
    frame[7] = byte7
    return frame


def encode_0x232(heartbeat: int = 0x05) -> List[int]:
    """Encode XFL201 heartbeat frame 0x232."""

    frame = [0] * FRAME_SIZE
    frame[0] = int(heartbeat) & 0xFF
    return frame


def encode_0x233(command: Any) -> List[int]:
    """Encode XFL201 fork command frame 0x233.

    Protocol fields:
    - BYTE0: fork speed, 0..255 maps to 0..100%.
    - BYTE1: lower speed, 0..255 maps to 0..100%.
    - BYTE2 bits: lift, lower, left, right, tilt forward, tilt backward, open, close.
    """

    byte2 = 0
    if bool(_first(command, ('lift', 'raise_fork', 'fork_lift'), False)):
        byte2 |= 1 << 0
    if bool(_first(command, ('lower', 'lower_fork', 'fork_lower'), False)):
        byte2 |= 1 << 1
    if bool(_first(command, ('side_shift_left', 'left'), False)):
        byte2 |= 1 << 2
    if bool(_first(command, ('side_shift_right', 'right'), False)):
        byte2 |= 1 << 3
    if bool(_first(command, ('tilt_forward', 'forward_tilt'), False)):
        byte2 |= 1 << 4
    if bool(_first(command, ('tilt_backward', 'backward_tilt'), False)):
        byte2 |= 1 << 5
    if bool(_first(command, ('clamp_open', 'open', 'fork_open'), False)):
        byte2 |= 1 << 6
    if bool(_first(command, ('clamp_close', 'close', 'fork_close'), False)):
        byte2 |= 1 << 7

    frame = [0] * FRAME_SIZE
    frame[0] = _byte_from_percent_or_raw(command, 'fork_speed_percent', 'fork_speed')
    frame[1] = _byte_from_percent_or_raw(command, 'lower_speed_percent', 'lower_speed')
    frame[2] = byte2
    return frame


def decode_0x206(data: Iterable[int]) -> Dict[str, Any]:
    """Decode XFL201 vehicle feedback frame 0x206."""

    frame = _frame(data)
    steering_deg = float(_read_i16_be(frame, 4)) / 100.0
    mode_bits = frame[7]
    return {
        'left_motor_rpm': float(_read_i16_be(frame, 0)),
        'right_motor_rpm': float(_read_i16_be(frame, 2)),
        'steering_angle_deg': steering_deg,
        'steering_angle_rad': math.radians(steering_deg),
        'battery_percent': float(frame[6]),
        'manual_mode': bool(mode_bits & (1 << 0)),
        'auto_mode': bool(mode_bits & (1 << 1)),
    }


def decode_0x207(data: Iterable[int]) -> Dict[str, Any]:
    """Decode XFL201 drive/steering fault frame 0x207."""

    frame = _frame(data)
    travel_fault_code = frame[0]
    steering_fault_code = frame[1]
    return {
        'travel_fault_code': travel_fault_code,
        'steering_fault_code': steering_fault_code,
        'has_fault': travel_fault_code != 0 or steering_fault_code != 0,
    }


def decode_0x209(data: Iterable[int]) -> Dict[str, Any]:
    """Decode XFL201 left motor pulse counter frame 0x209."""

    return {'left_motor_pulse_count': _read_i64_le(data)}


def decode_0x20a(data: Iterable[int]) -> Dict[str, Any]:
    """Decode XFL201 right motor pulse counter frame 0x20A."""

    return {'right_motor_pulse_count': _read_i64_le(data)}


def decode_0x20A(data: Iterable[int]) -> Dict[str, Any]:  # noqa: N802
    """Compatibility alias using the protocol's uppercase CAN ID spelling."""

    return decode_0x20a(data)


def decode_0x6db(data: Iterable[int]) -> Dict[str, Any]:
    """Decode XFL201 VCM valve fault frame 0x6DB."""

    frame = _frame(data)
    fault_code = frame[6]
    return {
        'vcm_valve_fault_code': fault_code,
        'has_fault': fault_code != 0,
    }


def decode_0x6DB(data: Iterable[int]) -> Dict[str, Any]:  # noqa: N802
    """Compatibility alias using the protocol's uppercase CAN ID spelling."""

    return decode_0x6db(data)


def format_frame(frame: Iterable[int]) -> str:
    """Return a compact uppercase hex dump for logs and acceptance notes."""

    return ' '.join(f'{byte:02X}' for byte in _frame(frame))
