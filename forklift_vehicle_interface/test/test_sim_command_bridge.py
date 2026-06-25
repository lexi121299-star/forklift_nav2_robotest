import math

import pytest
from forklift_msgs.msg import ForkliftControlCommand

from forklift_vehicle_interface.sim_command_bridge import SimCommandBridge


def make_bridge():
    bridge = object.__new__(SimCommandBridge)
    bridge._wheel_base = 1.2
    bridge._max_velocity_mps = 0.45
    bridge._max_steering_angle_rad = math.pi / 2.0
    bridge._max_angular_velocity_radps = 0.8
    bridge._allow_pivot_turn = True
    bridge._pivot_steering_angle_rad = math.pi / 2.0
    bridge._pivot_steering_tolerance_rad = 0.03
    bridge._pivot_angular_velocity_radps = 0.5
    return bridge


def make_command(speed=0.0, steering=0.0, forward=True, reverse=False):
    command = ForkliftControlCommand()
    command.enable = True
    command.forward = forward
    command.reverse = reverse
    command.velocity_mps = speed
    command.steering_angle_rad = steering
    return command


def test_zero_speed_pivot_uses_fixed_angular_velocity():
    bridge = make_bridge()
    twist, reason = bridge._twist_from_command(
        make_command(speed=0.0, steering=math.pi / 2.0)
    )

    assert reason == ''
    assert twist.linear.x == pytest.approx(0.0)
    assert twist.angular.z == pytest.approx(0.5)


def test_low_speed_pivot_does_not_lose_angular_velocity():
    bridge = make_bridge()
    twist, _ = bridge._twist_from_command(
        make_command(speed=0.01, steering=math.pi / 2.0)
    )

    assert twist.linear.x == pytest.approx(0.0)
    assert twist.angular.z == pytest.approx(0.5)


@pytest.mark.parametrize(
    'forward,reverse,steering,expected',
    [
        (True, False, -math.pi / 2.0, -0.5),
        (False, True, math.pi / 2.0, -0.5),
        (False, True, -math.pi / 2.0, 0.5),
    ],
)
def test_pivot_sign_preserves_direction_and_steering_convention(
    forward, reverse, steering, expected
):
    bridge = make_bridge()
    twist, _ = bridge._twist_from_command(
        make_command(
            steering=steering,
            forward=forward,
            reverse=reverse,
        )
    )

    assert twist.angular.z == pytest.approx(expected)


def test_pivot_angular_velocity_is_clamped():
    bridge = make_bridge()
    bridge._pivot_angular_velocity_radps = 1.2
    twist, _ = bridge._twist_from_command(
        make_command(steering=math.pi / 2.0)
    )

    assert twist.angular.z == pytest.approx(0.8)


def test_zero_speed_straight_command_remains_stopped():
    bridge = make_bridge()
    twist, reason = bridge._twist_from_command(make_command())

    assert reason == ''
    assert twist.linear.x == pytest.approx(0.0)
    assert twist.angular.z == pytest.approx(0.0)


def test_disabled_or_braking_command_remains_stopped():
    bridge = make_bridge()
    command = make_command(steering=math.pi / 2.0)
    command.enable = False
    disabled, disabled_reason = bridge._twist_from_command(command)

    command.enable = True
    command.brake = True
    braking, braking_reason = bridge._twist_from_command(command)

    assert disabled_reason == 'command disabled'
    assert disabled.linear.x == pytest.approx(0.0)
    assert disabled.angular.z == pytest.approx(0.0)
    assert braking_reason == 'brake command'
    assert braking.linear.x == pytest.approx(0.0)
    assert braking.angular.z == pytest.approx(0.0)
