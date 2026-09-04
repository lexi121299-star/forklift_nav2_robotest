import math
from types import SimpleNamespace

import pytest

from forklift_vehicle_interface.zhongli_can_codec import (
    decode_0x206,
    decode_0x207,
    decode_0x209,
    decode_0x20A,
    decode_0x20a,
    decode_0x6DB,
    decode_0x6db,
    encode_0x231,
    encode_0x232,
    encode_0x233,
    encode_0x233_from_control,
    format_frame,
)


def command(**overrides):
    values = {
        'left_motor_rpm': 0.0,
        'right_motor_rpm': 0.0,
        'steering_angle_deg': 0.0,
        'steering_angle_rad': 0.0,
        'brake_force': 0,
        'auto_enable': False,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_encode_0x231_uses_big_endian_signed_motor_rpm_and_centideg_steering():
    cmd = command(
        left_motor_rpm=1500,
        right_motor_rpm=-1200,
        steering_angle_deg=45.0,
        brake_force=128,
        auto_enable=True,
    )

    assert encode_0x231(cmd) == [
        0x05, 0xDC,
        0xFB, 0x50,
        0x11, 0x94,
        0x80,
        0x01,
    ]


def test_encode_0x231_accepts_existing_command_field_names_and_radian_steering():
    cmd = SimpleNamespace(
        left_drive_rpm=-5000,
        right_drive_rpm=5000,
        steering_angle_rad=math.radians(-90.0),
        brake=True,
        enable=True,
    )

    assert encode_0x231(cmd) == [
        0xEC, 0x78,
        0x13, 0x88,
        0xDC, 0xD8,
        0xFF,
        0x01,
    ]


def test_encode_0x231_clamps_protocol_limits():
    frame = encode_0x231(
        command(
            left_motor_rpm=60000,
            right_motor_rpm=-60000,
            steering_angle_deg=120.0,
            brake_force=999,
        )
    )

    assert frame[0:2] == [0x7F, 0xFF]
    assert frame[2:4] == [0x80, 0x00]
    assert frame[4:6] == [0x23, 0x28]
    assert frame[6] == 0xFF


def test_encode_0x232_heartbeat_defaults_to_protocol_value():
    assert encode_0x232() == [0x05, 0, 0, 0, 0, 0, 0, 0]
    assert encode_0x232(0x7A) == [0x7A, 0, 0, 0, 0, 0, 0, 0]


def test_encode_0x233_uses_speed_percent_and_direction_bits():
    fork = SimpleNamespace(
        fork_speed_percent=50.0,
        lower_speed_percent=25.0,
        lift=True,
        side_shift_right=True,
        tilt_backward=True,
        clamp_close=True,
    )

    assert encode_0x233(fork) == [128, 64, 0b10101001, 0, 0, 0, 0, 0]


def test_encode_0x233_accepts_raw_speed_bytes_and_aliases():
    fork = {
        'fork_speed': 10,
        'lower_speed': 300,
        'lower_fork': True,
        'left': True,
        'forward_tilt': True,
        'open': True,
    }

    assert encode_0x233(fork) == [10, 255, 0b01010110, 0, 0, 0, 0, 0]


def test_encode_0x233_from_shared_hydraulic_command_normalizes_currents():
    control = SimpleNamespace(
        lift_valve_ma=400.0,
        lower_valve_ma=0.0,
        side_shift_left_valve_ma=0.0,
        side_shift_right_valve_ma=800.0,
        tilt_forward_valve_ma=0.0,
        tilt_backward_valve_ma=0.0,
    )

    assert encode_0x233_from_control(control, valve_full_scale_ma=800.0) == [
        255, 0, 0b00001001, 0, 0, 0, 0, 0,
    ]


def test_encode_0x233_from_shared_hydraulic_command_uses_lower_speed_byte():
    control = SimpleNamespace(
        lift_valve_ma=0.0,
        lower_valve_ma=200.0,
        side_shift_left_valve_ma=0.0,
        side_shift_right_valve_ma=0.0,
        tilt_forward_valve_ma=0.0,
        tilt_backward_valve_ma=0.0,
    )

    assert encode_0x233_from_control(control, valve_full_scale_ma=800.0) == [
        0, 64, 0b00000010, 0, 0, 0, 0, 0,
    ]


def test_decode_0x206_vehicle_feedback():
    feedback = decode_0x206([0x05, 0xDC, 0xFB, 0x50, 0x11, 0x94, 87, 0b00000010])

    assert feedback['left_motor_rpm'] == 1500.0
    assert feedback['right_motor_rpm'] == -1200.0
    assert feedback['steering_angle_deg'] == 45.0
    assert feedback['steering_angle_rad'] == pytest.approx(math.radians(45.0))
    assert feedback['battery_percent'] == 87.0
    assert feedback['manual_mode'] is False
    assert feedback['auto_mode'] is True


def test_decode_0x207_fault_feedback():
    assert decode_0x207([3, 4, 0, 0, 0, 0, 0, 0]) == {
        'travel_fault_code': 3,
        'steering_fault_code': 4,
        'has_fault': True,
    }
    assert decode_0x207([0, 0, 0, 0, 0, 0, 0, 0])['has_fault'] is False


@pytest.mark.parametrize(
    ('frame', 'expected'),
    [
        ([0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01], 0x0102030405060708),
        ([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF], -1),
    ],
)
def test_decode_encoder_counts_are_little_endian_int64(frame, expected):
    assert decode_0x209(frame) == {'left_motor_pulse_count': expected}
    assert decode_0x20a(frame) == {'right_motor_pulse_count': expected}
    assert decode_0x20A(frame) == {'right_motor_pulse_count': expected}


def test_decode_0x6db_vcm_fault():
    assert decode_0x6db([0, 0, 0, 0, 0, 0, 9, 0]) == {
        'vcm_valve_fault_code': 9,
        'has_fault': True,
    }
    assert decode_0x6DB([0, 0, 0, 0, 0, 0, 0, 0])['has_fault'] is False


def test_format_frame_rejects_wrong_length():
    assert format_frame([0, 1, 2, 3, 4, 5, 6, 255]) == '00 01 02 03 04 05 06 FF'

    with pytest.raises(ValueError):
        format_frame([0x00])
