import math

import pytest

from forklift_vehicle_interface.curtis_can_transport import _normalize_data
from forklift_vehicle_interface.curtis_vehicle_state import (
    FRAME_IO_STATE,
    FRAME_LEFT_DRIVE,
    FRAME_STEERING_RIGHT_DRIVE,
    CurtisFeedbackState,
)


def test_socketcan_payload_requires_eight_bytes():
    assert _normalize_data([0, 1, 2, 3, 4, 5, 6, 7]) == [0, 1, 2, 3, 4, 5, 6, 7]

    with pytest.raises(ValueError):
        _normalize_data([0, 1, 2])


def test_feedback_state_integrates_straight_odom_from_drive_rpm():
    state = CurtisFeedbackState(
        drive_wheel_radius_m=0.10,
        drive_gear_ratio=1.0,
        drive_track_width_m=0.70,
        max_integration_dt_sec=10.0,
    )

    state.update_frame(FRAME_LEFT_DRIVE, [60, 0, 0, 0, 0, 100, 0, 0], 0.0)
    state.update_frame(FRAME_STEERING_RIGHT_DRIVE, [0, 0, 0, 60, 0, 0, 0, 0], 0.0)
    state.update_frame(FRAME_LEFT_DRIVE, [60, 0, 0, 0, 0, 100, 0, 0], 1.0)

    assert state.odom.velocity_mps == pytest.approx(2.0 * math.pi * 0.10)
    assert state.odom.angular_velocity_radps == pytest.approx(0.0)
    assert state.odom.x == pytest.approx(2.0 * math.pi * 0.10)
    assert state.odom.y == pytest.approx(0.0)


def test_feedback_state_can_invert_drive_direction_for_odom_only():
    state = CurtisFeedbackState(
        drive_wheel_radius_m=0.10,
        drive_gear_ratio=1.0,
        drive_track_width_m=0.70,
        drive_feedback_sign=-1.0,
        max_integration_dt_sec=10.0,
    )

    state.update_frame(FRAME_LEFT_DRIVE, [60, 0, 0, 0, 0, 100, 0, 0], 0.0)
    state.update_frame(FRAME_STEERING_RIGHT_DRIVE, [0, 0, 0, 60, 0, 0, 0, 0], 0.0)
    state.update_frame(FRAME_LEFT_DRIVE, [60, 0, 0, 0, 0, 100, 0, 0], 1.0)

    assert state.left_drive_rpm == pytest.approx(60.0)
    assert state.right_drive_rpm == pytest.approx(60.0)
    assert state.odom.velocity_mps == pytest.approx(-2.0 * math.pi * 0.10)
    assert state.odom.angular_velocity_radps == pytest.approx(0.0)
    assert state.odom.x == pytest.approx(-2.0 * math.pi * 0.10)
    assert state.odom.y == pytest.approx(0.0)


def test_feedback_state_can_invert_odom_yaw_without_changing_linear_speed():
    state = CurtisFeedbackState(
        drive_wheel_radius_m=0.10,
        drive_gear_ratio=1.0,
        drive_track_width_m=0.70,
        odom_angular_scale=-1.0,
        max_integration_dt_sec=10.0,
    )

    state.update_frame(FRAME_LEFT_DRIVE, [0, 0, 0, 0, 0, 100, 0, 0], 0.0)
    state.update_frame(FRAME_STEERING_RIGHT_DRIVE, [0, 0, 0, 60, 0, 0, 0, 0], 0.0)
    state.update_frame(FRAME_LEFT_DRIVE, [0, 0, 0, 0, 0, 100, 0, 0], 1.0)

    assert state.odom.velocity_mps > 0.0
    assert state.odom.angular_velocity_radps < 0.0
    assert state.odom.yaw < 0.0


def test_feedback_state_reports_io_interlock_and_faults():
    state = CurtisFeedbackState()

    state.update_frame(FRAME_IO_STATE, [0, 0, 0, 0, 0, 0b00001000, 0b00000110, 0], 1.0)
    assert state.main_contactor is True
    assert state.auto_mode_input is True
    assert state.soft_emergency_stop_input is True
    assert state.soft_emergency_stop is False
    assert state.emergency_stopped is False
    assert state.interlock is True

    state.update_frame(FRAME_LEFT_DRIVE, [0, 0, 0, 0, 7, 100, 0, 0], 1.1)
    assert state.has_fault() is True
    assert state.fault_summary() == 'left_drive=7'


def test_feedback_state_treats_open_soft_estop_input_as_emergency_stop():
    state = CurtisFeedbackState()

    state.update_frame(FRAME_IO_STATE, [0, 0, 0, 0, 0, 0b00001000, 0b00000010, 0], 1.0)

    assert state.main_contactor is True
    assert state.auto_mode_input is True
    assert state.soft_emergency_stop_input is False
    assert state.soft_emergency_stop is True
    assert state.emergency_stopped is True
    assert state.interlock is False


def test_feedback_timeout_starts_true_until_first_known_frame():
    state = CurtisFeedbackState()

    assert state.feedback_timed_out(10.0, 0.5) is True
    state.update_frame(FRAME_IO_STATE, [0, 0, 0, 0, 0, 0, 0, 0], 10.0)
    assert state.feedback_timed_out(10.4, 0.5) is False
    assert state.feedback_timed_out(10.6, 0.5) is True
