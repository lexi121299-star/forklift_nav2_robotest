from forklift_vehicle_interface.fine_motion_adapter import (
    motion_command,
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


def test_shortest_angle_wraps():
    assert abs(shortest_angle(6.283185307179586)) < 1e-9
