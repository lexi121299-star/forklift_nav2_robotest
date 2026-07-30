import math

import pytest

from forklift_vehicle_interface.xfl201_vehicle_state import Xfl201FeedbackState


def i64_le(value):
    value &= 0xFFFFFFFFFFFFFFFF
    return [(value >> (8 * index)) & 0xFF for index in range(8)]


def test_feedback_state_updates_vehicle_mode_and_faults():
    state = Xfl201FeedbackState()

    assert state.update_frame(0x206, [0x05, 0xDC, 0xFB, 0x50, 0x11, 0x94, 88, 0b10], 1.0)
    assert state.left_drive_rpm == 1500.0
    assert state.right_drive_rpm == -1200.0
    assert state.steering_angle_deg == 45.0
    assert state.steering_angle_rad == pytest.approx(math.radians(45.0))
    assert state.battery_percent == 88.0
    assert state.auto_mode is True
    assert state.manual_mode is False
    assert state.enabled is True
    assert state.interlock is True

    assert state.update_frame(0x207, [7, 3, 0, 0, 0, 0, 0, 0], 1.1)
    assert state.update_frame(0x6DB, [0, 0, 0, 0, 0, 0, 9, 0], 1.2)
    assert state.has_fault() is True
    assert state.fault_summary() == 'travel=7, steering=3, vcm_valve=9'


def test_encoder_odometry_uses_both_wheels():
    state = Xfl201FeedbackState(
        drive_track_width_m=0.80,
        left_meter_per_pulse=0.001,
        right_meter_per_pulse=0.001,
        max_integration_dt_sec=10.0,
    )

    state.update_frame(0x209, i64_le(0), 0.0)
    state.update_frame(0x20A, i64_le(0), 0.0)
    state.update_frame(0x209, i64_le(1000), 1.0)
    state.update_frame(0x20A, i64_le(1000), 1.0)

    assert state.odom.x == pytest.approx(1.0)
    assert state.odom.y == pytest.approx(0.0)
    assert state.odom.yaw == pytest.approx(0.0)

    state.update_frame(0x209, i64_le(1000), 2.0)
    state.update_frame(0x20A, i64_le(1400), 2.0)

    assert state.odom.yaw == pytest.approx(0.5)


def test_encoder_odometry_stays_zero_until_supplier_scale_is_configured():
    state = Xfl201FeedbackState(
        drive_track_width_m=0.80,
        left_meter_per_pulse=0.0,
        right_meter_per_pulse=0.0,
    )

    state.update_frame(0x209, i64_le(0), 0.0)
    state.update_frame(0x20A, i64_le(0), 0.0)
    state.update_frame(0x209, i64_le(1000), 1.0)
    state.update_frame(0x20A, i64_le(1000), 1.0)

    assert state.odom_enabled() is False
    assert state.odom.x == pytest.approx(0.0)
    assert state.odom.yaw == pytest.approx(0.0)


def test_feedback_timeout_starts_true_until_first_known_frame():
    state = Xfl201FeedbackState()

    assert state.feedback_timed_out(10.0, 0.5) is True
    state.update_frame(0x206, [0, 0, 0, 0, 0, 0, 50, 0], 10.0)
    assert state.feedback_timed_out(10.4, 0.5) is False
    assert state.feedback_timed_out(10.6, 0.5) is True
