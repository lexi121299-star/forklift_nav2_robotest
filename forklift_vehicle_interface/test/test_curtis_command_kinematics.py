import math

import pytest

from forklift_vehicle_interface.curtis_command_kinematics import drive_rpm_from_command
from forklift_vehicle_interface.curtis_vehicle_state import CurtisFeedbackState


GEOM = dict(
    wheel_base_m=1.2,
    track_width_m=0.70,
    wheel_radius_m=0.10,
    gear_ratio=1.0,
    pivot_steering_angle_rad=math.pi / 2.0,
    pivot_turn_radius_m=0.6,
    max_drive_rpm=2500.0,
)


def _expected_rpm(mps: float, radius: float = 0.10, gear: float = 1.0) -> float:
    return mps * 60.0 / (2.0 * math.pi * radius) * gear


def test_zero_velocity_gives_zero_rpm():
    assert drive_rpm_from_command(0.0, 0.5, **GEOM) == 0.0


def test_straight_drive_matches_wheel_surface_speed():
    rpm = drive_rpm_from_command(0.2, 0.0, **GEOM)
    assert rpm == pytest.approx(_expected_rpm(0.2))


def test_straight_drive_is_inverse_of_odom_rpm_to_mps():
    # The command rpm must round-trip back through the odom integrator.
    rpm = drive_rpm_from_command(0.2, 0.0, **GEOM)
    state = CurtisFeedbackState(drive_wheel_radius_m=0.10, drive_gear_ratio=1.0)
    assert state._rpm_to_mps(rpm) == pytest.approx(0.2)


def test_turning_outer_wheel_is_faster_than_centre():
    straight = drive_rpm_from_command(0.2, 0.0, **GEOM)
    turning = drive_rpm_from_command(0.2, 0.5, **GEOM)
    assert turning > straight


def test_pivot_sends_nonzero_outer_wheel_rpm():
    # A 90-degree pivot has zero body linear velocity but the outer wheel
    # must still spin, otherwise the real vehicle never rotates.
    rpm = drive_rpm_from_command(0.1, math.pi / 2.0, **GEOM)
    omega = 0.1 / 0.6
    expected = _expected_rpm(omega * 0.5 * 0.70)
    assert rpm == pytest.approx(expected)
    assert rpm > 0.0


def test_result_is_clamped_to_max_drive_rpm():
    rpm = drive_rpm_from_command(50.0, 0.0, **GEOM)
    assert rpm == pytest.approx(2500.0)
