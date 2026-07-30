import math

import pytest
from forklift_msgs.msg import ForkliftControlCommand

from forklift_vehicle_interface.xfl201_vehicle_interface import Xfl201VehicleInterface


def make_interface():
    interface = object.__new__(Xfl201VehicleInterface)
    interface._drive_wheel_radius_m = 0.10
    interface._drive_gear_ratio = 1.0
    interface._drive_track_width_m = 0.80
    interface._drive_wheel_base_m = 1.47
    interface._left_motor_sign = 1.0
    interface._right_motor_sign = 1.0
    interface._body_positive_is_fork_reverse = False
    interface._pivot_steering_angle_rad = math.pi / 2.0
    interface._pivot_turn_radius_m = 0.40
    interface._max_motor_rpm = 5000.0
    interface._min_motor_rpm = 0.0
    interface._steering_angle_fixed_deg = 0.0
    interface._use_command_steering_angle = True
    return interface


def make_command(speed=0.0, steering=0.0, forward=True, reverse=False):
    command = ForkliftControlCommand()
    command.enable = True
    command.forward = forward
    command.reverse = reverse
    command.velocity_mps = speed
    command.steering_angle_rad = steering
    return command


def test_straight_command_maps_to_equal_motor_rpm():
    interface = make_interface()

    left, right, steering = interface._motor_command_from_control(make_command(speed=0.5))

    expected_rpm = 0.5 * 60.0 / (2.0 * math.pi * 0.10)
    assert left == pytest.approx(expected_rpm)
    assert right == pytest.approx(expected_rpm)
    assert steering == pytest.approx(0.0)


def test_pivot_command_maps_to_opposite_motor_rpm():
    interface = make_interface()

    left, right, _ = interface._motor_command_from_control(
        make_command(speed=0.2, steering=math.pi / 2.0)
    )

    assert left == pytest.approx(-right)
    assert left < 0.0
    assert right > 0.0


def test_body_positive_direction_can_be_flipped_for_protocol_convention():
    interface = make_interface()
    interface._body_positive_is_fork_reverse = True

    left, right, _ = interface._motor_command_from_control(make_command(speed=0.5))

    assert left < 0.0
    assert right < 0.0


def test_minimum_motor_rpm_floor_keeps_nonzero_commands_moving():
    interface = make_interface()
    interface._min_motor_rpm = 100.0

    left, right, _ = interface._motor_command_from_control(make_command(speed=0.001))

    assert left == pytest.approx(100.0)
    assert right == pytest.approx(100.0)


def test_brake_or_disabled_command_outputs_zero_rpm():
    interface = make_interface()
    command = make_command(speed=0.5)
    command.brake = True

    assert interface._motor_command_from_control(command)[0:2] == (0.0, 0.0)

    command.brake = False
    command.enable = False
    assert interface._motor_command_from_control(command)[0:2] == (0.0, 0.0)
