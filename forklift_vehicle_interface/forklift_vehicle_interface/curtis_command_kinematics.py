"""Convert a (velocity, steering) ROS command into the Curtis 0x203 行驶速度.

The MK320 dual-drive forklift drives from two differential wheels on a common
axle (URDF: wheels at base_link x=-0.34, the MPC ``rear_axle_x_offset`` point).
Per the AGV protocol「注意事项 5」the ``行驶速度`` field (0x203 BYTE1-2,
0..4000 rpm) carries the **outer** drive wheel speed during a turn; the Curtis
controller derives the inner wheel itself.

Because the two wheels are a differential pair, the instantaneous centre of
rotation lies on their common axle, so the wheel speeds are fully determined by
the body twist at the drive-axle centre::

    v_left  = v_x - omega * track / 2
    v_right = v_x + omega * track / 2

and ``drive_rpm`` corresponds to the faster (outer) wheel. The motor-rpm
conversion is the exact inverse of ``CurtisFeedbackState._rpm_to_mps`` so command
and odometry stay consistent.

This is a pure helper (no ROS deps) so the kinematics stay unit-testable.
"""

from __future__ import annotations

import math


def _positive(value: float, fallback: float) -> float:
    return value if value > 1e-9 else fallback


def drive_rpm_from_command(
    velocity_mps: float,
    steering_angle_rad: float,
    *,
    wheel_base_m: float,
    track_width_m: float,
    wheel_radius_m: float,
    gear_ratio: float = 1.0,
    pivot_steering_angle_rad: float = math.pi / 2.0,
    pivot_turn_radius_m: float = 0.6,
    max_drive_rpm: float = 2500.0,
) -> float:
    """Return the unsigned outer-wheel motor rpm for the Curtis 0x203 frame.

    Travel direction is conveyed by the forward/reverse bits, so the returned
    value is a non-negative magnitude clamped to ``[0, max_drive_rpm]``.
    """

    speed = abs(float(velocity_mps))
    if speed <= 1e-9:
        return 0.0

    wheel_radius_m = _positive(float(wheel_radius_m), 0.10)
    gear_ratio = _positive(float(gear_ratio), 1.0)
    track_width_m = max(0.0, float(track_width_m))
    steering = max(-math.pi / 2.0, min(math.pi / 2.0, float(steering_angle_rad)))

    # Reconstruct the body twist consistently with ForkliftVehicleModel: a pivot
    # spins about the drive axle (v_x == 0), otherwise it follows the bicycle model.
    is_pivot = abs(steering) >= abs(pivot_steering_angle_rad) - 1e-3
    if is_pivot:
        v_x = 0.0
        omega = speed / _positive(float(pivot_turn_radius_m), 0.6)
    else:
        v_x = speed
        omega = speed * math.tan(steering) / _positive(float(wheel_base_m), 1.2)

    # Outer wheel speed: max(|v_x - w*t/2|, |v_x + w*t/2|) == |v_x| + |w|*t/2.
    v_outer = abs(v_x) + abs(omega) * 0.5 * track_width_m

    wheel_rpm = v_outer * 60.0 / (2.0 * math.pi * wheel_radius_m)
    drive_rpm = wheel_rpm * gear_ratio
    return max(0.0, min(drive_rpm, float(max_drive_rpm)))
