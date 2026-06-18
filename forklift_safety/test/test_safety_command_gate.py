import math

import pytest
from forklift_msgs.msg import ForkliftControlCommand
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

from forklift_safety.safety_command_gate import (
    clamp_control_command,
    costmap_error,
    costmap_stop_reason,
    direction,
    footprint_collision_at_pose,
    footprint_sweep_collision,
    parse_footprint,
    recovery_command_from_twist,
    stop_command,
)


def make_costmap(width=20, height=20, resolution=0.1, origin_x=-1.0, origin_y=-1.0):
    costmap = OccupancyGrid()
    costmap.info.width = width
    costmap.info.height = height
    costmap.info.resolution = resolution
    costmap.info.origin.position.x = origin_x
    costmap.info.origin.position.y = origin_y
    costmap.info.origin.orientation.w = 1.0
    costmap.data = [0] * (width * height)
    return costmap


def set_cost(costmap, x, y, cost):
    mx = int(math.floor((x - costmap.info.origin.position.x) / costmap.info.resolution))
    my = int(math.floor((y - costmap.info.origin.position.y) / costmap.info.resolution))
    costmap.data[my * costmap.info.width + mx] = cost


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


def test_costmap_error_rejects_empty_or_truncated_maps():
    costmap = make_costmap(width=0, height=10)
    assert costmap_error(costmap) == 'empty dimensions'

    costmap = make_costmap(width=4, height=4)
    costmap.data = [0] * 15
    assert costmap_error(costmap) == 'truncated data 15/16'


def test_costmap_stop_reason_blocks_missing_timeout_and_invalid_data():
    assert costmap_stop_reason(True, 0.6, False, '', 0.5) == 'costmap missing'
    assert costmap_stop_reason(True, 0.6, True, '', 0.5) == 'costmap timeout'
    assert (
        costmap_stop_reason(True, 0.1, True, 'empty dimensions', 0.5)
        == 'costmap invalid: empty dimensions'
    )
    assert costmap_stop_reason(False, 2.0, False, 'empty dimensions', 0.5) == ''


def test_parse_footprint_matches_foxy_yaml_string():
    footprint = parse_footprint(
        '[[0.843, 0.58], [0.843, -0.58], [-2.043, -0.58], [-2.043, 0.58]]'
    )

    assert footprint[0] == pytest.approx((0.843, 0.58))
    assert footprint[2] == pytest.approx((-2.043, -0.58))


def test_footprint_collision_at_pose_blocks_lethal_edge_cell():
    costmap = make_costmap()
    footprint = parse_footprint('[[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]]')
    set_cost(costmap, 0.1, 0.0, 100)

    collision, reason = footprint_collision_at_pose(
        costmap,
        footprint,
        (0.0, 0.0, 0.0),
        sample_spacing=0.05,
        cost_threshold=100,
        unknown_is_collision=True,
    )

    assert collision is True
    assert reason.startswith('footprint collision: cost')


def test_footprint_collision_can_treat_unknown_as_blocked():
    costmap = make_costmap()
    footprint = parse_footprint('[[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]]')
    set_cost(costmap, 0.1, 0.0, -1)

    collision, reason = footprint_collision_at_pose(
        costmap,
        footprint,
        (0.0, 0.0, 0.0),
        sample_spacing=0.05,
        cost_threshold=100,
        unknown_is_collision=True,
    )

    assert collision is True
    assert reason == 'footprint collision: unknown costmap cell'


def test_footprint_sweep_collision_checks_predicted_forward_pose():
    costmap = make_costmap()
    footprint = parse_footprint('[[-0.05, -0.05], [0.05, -0.05], [0.05, 0.05], [-0.05, 0.05]]')
    set_cost(costmap, 0.2, 0.0, 100)
    command = ForkliftControlCommand()
    command.enable = True
    command.forward = True
    command.velocity_mps = 0.2

    collision, reason = footprint_sweep_collision(
        costmap,
        footprint,
        (0.0, 0.0, 0.0),
        command,
        wheel_base=1.2,
        pivot_turn_radius=0.6,
        horizon_sec=1.0,
        time_step_sec=0.2,
        pivot_steering_angle_rad=math.pi / 2.0,
        sample_spacing=0.05,
        cost_threshold=100,
        unknown_is_collision=True,
    )

    assert collision is True
    assert reason.startswith('footprint collision: cost')


def test_footprint_sweep_collision_allows_clear_backoff():
    costmap = make_costmap()
    footprint = parse_footprint('[[-0.05, -0.05], [0.05, -0.05], [0.05, 0.05], [-0.05, 0.05]]')
    command = ForkliftControlCommand()
    command.enable = True
    command.reverse = True
    command.velocity_mps = 0.1

    collision, reason = footprint_sweep_collision(
        costmap,
        footprint,
        (0.0, 0.0, 0.0),
        command,
        wheel_base=1.2,
        pivot_turn_radius=0.6,
        horizon_sec=1.0,
        time_step_sec=0.2,
        pivot_steering_angle_rad=math.pi / 2.0,
        sample_spacing=0.05,
        cost_threshold=100,
        unknown_is_collision=True,
    )

    assert collision is False
    assert reason == 'footprint sweep clear'
