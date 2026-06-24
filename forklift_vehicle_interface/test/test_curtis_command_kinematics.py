import math

import pytest

from forklift_vehicle_interface.curtis_command_kinematics import drive_rpm_from_command
from forklift_vehicle_interface.curtis_vehicle_state import CurtisFeedbackState


# Real 2MKC20M30LV205 values (spec drawing + ZF gear ratio). ``min_drive_rpm`` is
# disabled here so the proportionality tests stay pure; the floor has its own tests.
GEOM = dict(
    wheel_base_m=1.4,
    track_width_m=0.937,
    wheel_radius_m=0.2285,
    gear_ratio=26.75,
    pivot_steering_angle_rad=math.pi / 2.0,
    pivot_turn_radius_m=0.6,
    max_drive_rpm=2485.0,
    min_drive_rpm=0.0,
)


def _expected_rpm(mps: float, radius: float = 0.2285, gear: float = 26.75) -> float:
    return mps * 60.0 / (2.0 * math.pi * radius) * gear


def test_zero_velocity_gives_zero_rpm():
    assert drive_rpm_from_command(0.0, 0.5, **GEOM) == 0.0


def test_straight_drive_matches_wheel_surface_speed():
    rpm = drive_rpm_from_command(0.2, 0.0, **GEOM)
    assert rpm == pytest.approx(_expected_rpm(0.2))


def test_straight_drive_is_inverse_of_odom_rpm_to_mps():
    # The command rpm must round-trip back through the odom integrator.
    rpm = drive_rpm_from_command(0.2, 0.0, **GEOM)
    state = CurtisFeedbackState(drive_wheel_radius_m=0.2285, drive_gear_ratio=26.75)
    assert state._rpm_to_mps(rpm) == pytest.approx(0.2)


def test_top_speed_maps_to_envelope_rpm():
    # 8 km/h operating cap (2.2222 m/s straight) lands at the ~2485 rpm envelope.
    rpm = drive_rpm_from_command(8.0 / 3.6, 0.0, **GEOM)
    assert rpm == pytest.approx(2485.0, abs=2.0)


def test_turning_outer_wheel_is_faster_than_centre():
    straight = drive_rpm_from_command(0.2, 0.0, **GEOM)
    turning = drive_rpm_from_command(0.2, 0.5, **GEOM)
    assert turning > straight


def test_pivot_sends_nonzero_outer_wheel_rpm():
    # A 90-degree pivot has zero body linear velocity but the outer wheel
    # must still spin, otherwise the real vehicle never rotates.
    rpm = drive_rpm_from_command(0.1, math.pi / 2.0, **GEOM)
    omega = 0.1 / 0.6
    expected = _expected_rpm(omega * 0.5 * 0.937)
    assert rpm == pytest.approx(expected)
    assert rpm > 0.0


def test_result_is_clamped_to_max_drive_rpm():
    rpm = drive_rpm_from_command(50.0, 0.0, **GEOM)
    assert rpm == pytest.approx(2485.0)


def test_min_drive_rpm_floor_raises_low_command():
    # 0.05 m/s straight maps to ~56 rpm, below the 100 rpm stiction floor.
    rpm = drive_rpm_from_command(0.05, 0.0, **dict(GEOM, min_drive_rpm=100.0))
    assert _expected_rpm(0.05) < 100.0
    assert rpm == pytest.approx(100.0)


def test_min_drive_rpm_floor_keeps_zero_at_zero():
    rpm = drive_rpm_from_command(0.0, 0.0, **dict(GEOM, min_drive_rpm=100.0))
    assert rpm == 0.0


def test_min_drive_rpm_floor_leaves_commands_above_it_proportional():
    # 0.2 m/s maps well above the floor, so it must stay proportional.
    rpm = drive_rpm_from_command(0.2, 0.0, **dict(GEOM, min_drive_rpm=100.0))
    assert rpm == pytest.approx(_expected_rpm(0.2))
