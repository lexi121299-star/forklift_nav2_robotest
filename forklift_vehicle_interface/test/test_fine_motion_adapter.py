import pytest

from forklift_vehicle_interface.fine_motion_adapter import (
    motion_command,
    nonnegative_finite,
    pivot_command,
    pivot_yaw_is_settled,
    pivot_step_hold_enabled,
    steering_center_command,
    steering_center_is_settled,
    steering_center_settle_started_at,
    shortest_angle,
)


def test_reverse_motion_command_uses_reverse_direction():
    command = motion_command(-0.1, 1.0, 1.0)

    assert command.enable is True
    assert command.brake is False
    assert command.forward is False
    assert command.reverse is True
    assert command.velocity_mps == -0.1
    assert command.steering_angle_rad == 0.0


def test_zero_motion_command_is_a_stop():
    command = motion_command(0.0, 1.0, 1.0)

    assert command.enable is False
    assert command.brake is True
    assert command.forward is False
    assert command.reverse is False


def test_steering_center_command_has_no_traction_but_stays_enabled():
    command = steering_center_command(1.0, 1.0)

    assert command.enable is True
    assert command.brake is False
    assert command.forward is False
    assert command.reverse is False
    assert command.velocity_mps == 0.0
    assert command.steering_angle_rad == 0.0


def test_steering_center_requires_continuous_in_tolerance_feedback():
    started_at = steering_center_settle_started_at(
        steering_angle_rad=0.04,
        max_steering_angle_rad=0.10,
        now_sec=10.0,
        previous_started_at_sec=None,
    )
    assert started_at == 10.0
    assert not steering_center_is_settled(started_at, 10.39, 0.4)

    started_at = steering_center_settle_started_at(
        steering_angle_rad=0.11,
        max_steering_angle_rad=0.10,
        now_sec=10.40,
        previous_started_at_sec=started_at,
    )
    assert started_at is None

    started_at = steering_center_settle_started_at(
        steering_angle_rad=0.03,
        max_steering_angle_rad=0.10,
        now_sec=10.45,
        previous_started_at_sec=started_at,
    )
    assert started_at == 10.45
    assert steering_center_is_settled(started_at, 10.85, 0.4)


def test_shortest_angle_wraps():
    assert abs(shortest_angle(6.283185307179586)) < 1e-9


def test_zero_pivot_step_hold_is_valid():
    assert nonnegative_finite(0.0, 'pivot_step_hold_sec') == 0.0
    assert pivot_step_hold_enabled(0.0) is False
    assert pivot_step_hold_enabled(0.2) is True
    with pytest.raises(ValueError, match='non-negative'):
        nonnegative_finite(-0.1, 'pivot_step_hold_sec')


def test_pivot_requires_yaw_and_yaw_rate_to_settle():
    assert pivot_yaw_is_settled(0.02, 0.02, 0.025, 0.03)
    assert not pivot_yaw_is_settled(0.03, 0.0, 0.025, 0.03)
    assert not pivot_yaw_is_settled(0.0, 0.04, 0.025, 0.03)


def test_positive_pivot_command_uses_forward_and_left_steering():
    command = pivot_command(0.08, 1.0, 1.5707963267948966, 1.0, 1.0)

    assert command.enable is True
    assert command.forward is True
    assert command.reverse is False
    assert command.velocity_mps == 0.08
    assert command.steering_angle_rad > 1.5


def test_negative_pivot_command_uses_right_steering():
    command = pivot_command(0.06, -1.0, 1.5707963267948966, 1.0, 1.0)

    assert command.forward is True
    assert command.steering_angle_rad < -1.5
