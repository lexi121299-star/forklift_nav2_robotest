import math
from types import SimpleNamespace

import pytest

from forklift_vehicle_interface.curtis_can_codec import (
    decode_0x183,
    decode_0x283,
    decode_0x383,
    decode_0x483,
    encode_0x203,
    encode_0x303,
    encode_0x403,
    format_frame,
)


def command(**overrides):
    values = {
        'enable': False,
        'brake': False,
        'forward': False,
        'reverse': False,
        'drive_rpm': 0.0,
        'steering_angle_rad': 0.0,
        'steering_angle_deg': 0.0,
        'accel_time_sec': 0.0,
        'decel_time_sec': 0.0,
        'pump_rpm': 0.0,
        'lift_valve_ma': 0.0,
        'lower_valve_ma': 0.0,
        'side_shift_left_valve_ma': 0.0,
        'side_shift_right_valve_ma': 0.0,
        'tilt_forward_valve_ma': 0.0,
        'tilt_backward_valve_ma': 0.0,
        'horn': False,
        'light': False,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_pdf_example_drive_forward_with_45_degree_steering():
    cmd = command(
        enable=True,
        forward=True,
        drive_rpm=1500,
        steering_angle_deg=45.0,
        accel_time_sec=3.0,
        decel_time_sec=3.0,
    )

    assert encode_0x203(cmd) == [0x03, 0xDC, 0x05, 0x1E, 0x1E, 0x00, 0x94, 0x11]
    assert encode_0x303(cmd) == [0x00] * 8
    assert encode_0x403(cmd) == [0x00] * 8


def test_pdf_example_lift_forks_with_pump_and_lift_valve():
    cmd = command(enable=True, pump_rpm=1500, lift_valve_ma=400)

    assert encode_0x203(cmd) == [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    assert encode_0x303(cmd) == [0xDC, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    assert encode_0x403(cmd) == [0x00, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]


@pytest.mark.parametrize(
    ('angle_deg', 'expected_bytes'),
    [
        (-90.0, [0xD8, 0xDC]),
        (0.0, [0x00, 0x00]),
        (45.0, [0x94, 0x11]),
        (90.0, [0x28, 0x23]),
    ],
)
def test_steering_angle_scaling(angle_deg, expected_bytes):
    frame = encode_0x203(command(enable=True, steering_angle_deg=angle_deg))
    assert frame[6:8] == expected_bytes


def test_steering_angle_can_be_supplied_in_radians():
    frame = encode_0x203(command(enable=True, steering_angle_rad=math.radians(45.0)))
    assert frame[6:8] == [0x94, 0x11]


def test_valve_current_scaling_and_clamping():
    frame = encode_0x403(
        command(
            lower_valve_ma=400,
            lift_valve_ma=2000,
            side_shift_left_valve_ma=2500,
            tilt_backward_valve_ma=-5,
        )
    )

    assert frame == [40, 200, 0, 0, 200, 0, 0, 0]


def test_decode_feedback_frames():
    assert decode_0x183([0x18, 0xFC, 0xE8, 0x03, 0x02, 0x64, 0xC4, 0x09]) == {
        'left_drive_rpm': -1000.0,
        'left_drive_current_a': 100.0,
        'left_drive_fault_code': 2,
        'battery_percent': 100.0,
        'drive_controller_temperature_c': 250.0,
        'has_fault': True,
    }

    assert decode_0x283([0, 0, 0, 0, 0b01000001, 0b00011010, 0b00001110, 0]) == {
        'lower_valve_output': False,
        'brake_relay': True,
        'reverse_relay': True,
        'main_contactor': True,
        'auto_mode_input': True,
        'soft_emergency_stop_input': True,
        'parking_brake_input': True,
        'slowdown_switch_input': False,
        'lift_valve_output': True,
        'retract_valve_output': False,
        'extend_valve_output': False,
        'side_shift_left_output': False,
        'side_shift_right_output': False,
        'tilt_forward_output': False,
        'tilt_backward_output': True,
    }

    steering = decode_0x383([0x94, 0x11, 0x00, 0xE8, 0x03, 0x20, 0x03, 0x04])
    assert steering['steering_angle_deg'] == 45.0
    assert steering['right_drive_rpm'] == 1000.0
    assert steering['right_drive_current_a'] == 80.0
    assert steering['right_drive_fault_code'] == 4
    assert steering['has_fault'] is True

    lift = decode_0x483([0xDC, 0x05, 0xF4, 0x01, 0xFA, 0x00, 0x00, 0x03])
    assert lift['lift_motor_rpm'] == 1500.0
    assert lift['lift_motor_current_a'] == 50.0
    assert lift['lift_controller_temperature_c'] == 25.0
    assert lift['light_relay'] is True
    assert lift['horn_relay'] is True


def test_format_frame_rejects_wrong_length():
    with pytest.raises(ValueError):
        format_frame([0x00])
