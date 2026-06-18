import math

import pytest
from forklift_msgs.msg import ForkliftControlCommand
from geometry_msgs.msg import Twist

from forklift_safety.safety_command_gate import (
    clamp_control_command,
    direction,
    recovery_command_from_twist,
    stop_command,
)


def test_direction_rejects_ambiguous_commands():
    command = ForkliftControlCommand()
    command.forward = True
    command.reverse = True
    assert direction(command) == 0

    command.reverse = False
    assert direction(command) == 1

    command.forward = False
    command.reverse = True
    assert direction(command) == -1


def test_stop_command_brakes_and_disables_motion():
    command = stop_command()

    assert command.enable is False
    assert command.brake is True
    assert command.forward is False
    assert command.reverse is False
    assert command.velocity_mps == pytest.approx(0.0)


def test_clamp_control_command_limits_speed_and_steering():
    command = ForkliftControlCommand()
    command.enable = True
    command.forward = True
    command.velocity_mps = 2.0
    command.steering_angle_rad = 2.0

    gated = clamp_control_command(command, 0.45, 0.15, 1.0)

    assert gated.velocity_mps == pytest.approx(0.45)
    assert gated.steering_angle_rad == pytest.approx(1.0)
    assert gated.steering_angle_deg == pytest.approx(math.degrees(1.0))


def test_recovery_backoff_is_low_speed_reverse_command():
    twist = Twist()
    twist.linear.x = -0.5

    command, reason = recovery_command_from_twist(
        twist,
        max_recovery_velocity_mps=0.10,
        max_recovery_angular_velocity_radps=0.30,
        wheel_base=1.2,
        pivot_turn_radius=0.6,
        pivot_steering_angle_rad=math.pi / 2.0,
        allow_recovery_backoff=True,
        allow_recovery_pivot=True,
    )

    assert reason == 'recovery backoff'
    assert command.enable is True
    assert command.reverse is True
    assert command.forward is False
    assert command.velocity_mps == pytest.approx(0.10)


def test_recovery_pivot_is_whitelisted_low_speed_command():
    twist = Twist()
    twist.angular.z = 1.0

    command, reason = recovery_command_from_twist(
        twist,
        max_recovery_velocity_mps=0.10,
        max_recovery_angular_velocity_radps=0.30,
        wheel_base=1.2,
        pivot_turn_radius=0.6,
        pivot_steering_angle_rad=math.pi / 2.0,
        allow_recovery_backoff=True,
        allow_recovery_pivot=True,
    )

    assert reason == 'recovery pivot'
    assert command.enable is True
    assert command.forward is True
    assert command.velocity_mps == pytest.approx(0.10)
    assert command.steering_angle_rad == pytest.approx(math.pi / 2.0)


def test_recovery_pivot_uses_steering_sign_for_turn_direction():
    twist = Twist()
    twist.angular.z = -1.0

    command, reason = recovery_command_from_twist(
        twist,
        max_recovery_velocity_mps=0.10,
        max_recovery_angular_velocity_radps=0.30,
        wheel_base=1.2,
        pivot_turn_radius=0.6,
        pivot_steering_angle_rad=math.pi / 2.0,
        allow_recovery_backoff=True,
        allow_recovery_pivot=True,
    )

    assert reason == 'recovery pivot'
    assert command.forward is True
    assert command.reverse is False
    assert command.steering_angle_rad == pytest.approx(-math.pi / 2.0)


def test_recovery_pivot_can_be_disabled():
    twist = Twist()
    twist.angular.z = 0.2

    command, reason = recovery_command_from_twist(
        twist,
        max_recovery_velocity_mps=0.10,
        max_recovery_angular_velocity_radps=0.30,
        wheel_base=1.2,
        pivot_turn_radius=0.6,
        pivot_steering_angle_rad=math.pi / 2.0,
        allow_recovery_backoff=True,
        allow_recovery_pivot=False,
    )

    assert reason == 'recovery pivot disabled'
    assert command.brake is True
