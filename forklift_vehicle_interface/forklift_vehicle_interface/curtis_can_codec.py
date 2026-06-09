"""Curtis MK320 CAN frame codec helpers.

The helpers intentionally work with any object that exposes command-like
attributes, including ROS messages, dictionaries, and SimpleNamespace objects.
That keeps the protocol mapping testable without a running ROS graph.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Iterable, List


FRAME_SIZE = 8


def _get(source: Any, name: str, default: Any = 0) -> Any:
    if isinstance(source, dict):
        return source.get(name, default)
    return getattr(source, name, default)


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _u16_le(value: int) -> List[int]:
    value = int(_clamp(value, 0, 0xFFFF))
    return [value & 0xFF, (value >> 8) & 0xFF]


def _i16_le(value: int) -> List[int]:
    value = int(_clamp(value, -32768, 32767)) & 0xFFFF
    return [value & 0xFF, (value >> 8) & 0xFF]


def _read_u16_le(data: Iterable[int], offset: int) -> int:
    frame = _frame(data)
    return frame[offset] | (frame[offset + 1] << 8)


def _read_i16_le(data: Iterable[int], offset: int) -> int:
    value = _read_u16_le(data, offset)
    if value & 0x8000:
        value -= 0x10000
    return value


def _frame(data: Iterable[int]) -> List[int]:
    frame = [int(byte) & 0xFF for byte in data]
    if len(frame) != FRAME_SIZE:
        raise ValueError(f'Curtis CAN frames must be {FRAME_SIZE} bytes, got {len(frame)}')
    return frame


def _time_to_tenths(seconds: float) -> int:
    if seconds <= 0.0:
        return 0
    return int(_clamp(round(seconds * 10.0), 2, 255))


def _steering_deg(command: Any) -> float:
    steering_deg = float(_get(command, 'steering_angle_deg', 0.0))
    steering_rad = float(_get(command, 'steering_angle_rad', 0.0))
    if abs(steering_deg) < 1e-9 and abs(steering_rad) > 1e-9:
        steering_deg = math.degrees(steering_rad)
    return _clamp(steering_deg, -90.0, 90.0)


def _valve_ma_to_byte(current_ma: float) -> int:
    return int(_clamp(round(current_ma / 10.0), 0, 200))


def encode_0x203(command: Any) -> List[int]:
    """Encode travel control frame 0x203."""

    byte0 = 0
    if bool(_get(command, 'enable', False)):
        byte0 |= 1 << 0
    if bool(_get(command, 'forward', False)):
        byte0 |= 1 << 1
    if bool(_get(command, 'reverse', False)):
        byte0 |= 1 << 2
    if bool(_get(command, 'brake', False)):
        byte0 |= 1 << 3

    drive_rpm = int(_clamp(round(float(_get(command, 'drive_rpm', 0.0))), 0, 4000))
    accel = _time_to_tenths(float(_get(command, 'accel_time_sec', 0.0)))
    decel = _time_to_tenths(float(_get(command, 'decel_time_sec', 0.0)))
    steering_centideg = int(round(_steering_deg(command) * 100.0))

    frame = [0] * FRAME_SIZE
    frame[0] = byte0
    frame[1:3] = _u16_le(drive_rpm)
    frame[3] = accel
    frame[4] = decel
    frame[6:8] = _i16_le(steering_centideg)
    return frame


def encode_0x303(command: Any) -> List[int]:
    """Encode pump motor and discrete output frame 0x303."""

    pump_rpm = int(_clamp(round(float(_get(command, 'pump_rpm', 0.0))), 0, 3000))
    byte3 = 0
    if bool(_get(command, 'light', False)):
        byte3 |= 1 << 0
    if bool(_get(command, 'horn', False)):
        byte3 |= 1 << 1
    if float(_get(command, 'lower_valve_ma', 0.0)) > 0.0:
        byte3 |= 1 << 2

    frame = [0] * FRAME_SIZE
    frame[0:2] = _u16_le(pump_rpm)
    frame[3] = byte3
    return frame


def encode_0x403(command: Any) -> List[int]:
    """Encode proportional valve frame 0x403."""

    return [
        _valve_ma_to_byte(float(_get(command, 'lower_valve_ma', 0.0))),
        _valve_ma_to_byte(float(_get(command, 'lift_valve_ma', 0.0))),
        0,
        0,
        _valve_ma_to_byte(float(_get(command, 'side_shift_left_valve_ma', 0.0))),
        _valve_ma_to_byte(float(_get(command, 'side_shift_right_valve_ma', 0.0))),
        _valve_ma_to_byte(float(_get(command, 'tilt_forward_valve_ma', 0.0))),
        _valve_ma_to_byte(float(_get(command, 'tilt_backward_valve_ma', 0.0))),
    ]


def decode_0x183(data: Iterable[int]) -> Dict[str, Any]:
    """Decode main/left drive feedback frame 0x183."""

    frame = _frame(data)
    fault_code = frame[4]
    return {
        'left_drive_rpm': float(_read_i16_le(frame, 0)),
        'left_drive_current_a': float(_read_u16_le(frame, 2)) / 10.0,
        'left_drive_fault_code': fault_code,
        'battery_percent': float(frame[5]),
        'drive_controller_temperature_c': float(_read_i16_le(frame, 6)) / 10.0,
        'has_fault': fault_code != 0,
    }


def decode_0x283(data: Iterable[int]) -> Dict[str, Any]:
    """Decode IO state frame 0x283."""

    frame = _frame(data)
    output_1351 = frame[4]
    relays = frame[5]
    inputs = frame[6]
    return {
        'lower_valve_output': bool(relays & (1 << 0)),
        'brake_relay': bool(relays & (1 << 1)),
        'reverse_relay': bool(relays & (1 << 4)),
        'main_contactor': bool(relays & (1 << 3)),
        'auto_mode_input': bool(inputs & (1 << 1)),
        'soft_emergency_stop_input': bool(inputs & (1 << 2)),
        'parking_brake_input': bool(inputs & (1 << 3)),
        'slowdown_switch_input': bool(inputs & (1 << 4)),
        'lift_valve_output': bool(output_1351 & (1 << 0)),
        'retract_valve_output': bool(output_1351 & (1 << 1)),
        'extend_valve_output': bool(output_1351 & (1 << 2)),
        'side_shift_left_output': bool(output_1351 & (1 << 3)),
        'side_shift_right_output': bool(output_1351 & (1 << 4)),
        'tilt_forward_output': bool(output_1351 & (1 << 5)),
        'tilt_backward_output': bool(output_1351 & (1 << 6)),
    }


def decode_0x383(data: Iterable[int]) -> Dict[str, Any]:
    """Decode steering and right-drive feedback frame 0x383."""

    frame = _frame(data)
    steering_fault = frame[2]
    right_drive_fault = frame[7]
    steering_deg = float(_read_i16_le(frame, 0)) / 100.0
    return {
        'steering_angle_deg': steering_deg,
        'steering_angle_rad': math.radians(steering_deg),
        'steering_fault_code': steering_fault,
        'right_drive_rpm': float(_read_i16_le(frame, 3)),
        'right_drive_current_a': float(_read_u16_le(frame, 5)) / 10.0,
        'right_drive_fault_code': right_drive_fault,
        'has_fault': steering_fault != 0 or right_drive_fault != 0,
    }


def decode_0x483(data: Iterable[int]) -> Dict[str, Any]:
    """Decode lift-controller feedback frame 0x483."""

    frame = _frame(data)
    fault_code = frame[6]
    relays = frame[7]
    return {
        'lift_motor_rpm': float(_read_i16_le(frame, 0)),
        'lift_motor_current_a': float(_read_u16_le(frame, 2)) / 10.0,
        'lift_controller_temperature_c': float(_read_i16_le(frame, 4)) / 10.0,
        'lift_fault_code': fault_code,
        'light_relay': bool(relays & (1 << 0)),
        'horn_relay': bool(relays & (1 << 1)),
        'has_fault': fault_code != 0,
    }


def format_frame(frame: Iterable[int]) -> str:
    """Return a compact uppercase hex dump for logs and acceptance notes."""

    return ' '.join(f'{byte:02X}' for byte in _frame(frame))
