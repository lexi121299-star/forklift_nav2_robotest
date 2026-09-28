import math
import random
import time
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from rclpy.time import Time

from forklift_safety import safety_command_gate as gate
from test_safety_command_gate import make_costmap


@pytest.mark.parametrize('yaw', [0., .37, -1.4])
def test_prepared_lookup_matches_original_rotated_grid(yaw):
    grid = make_costmap()
    grid.info.origin.orientation.z = math.sin(yaw / 2.)
    grid.info.origin.orientation.w = math.cos(yaw / 2.)
    prepared = gate.PreparedCostmap(grid)
    rng = random.Random(24)
    points = [(rng.uniform(-3., 3.), rng.uniform(-3., 3.)) for _ in range(2000)]
    points += [(grid.info.origin.position.x, grid.info.origin.position.y)]
    for x, y in points:
        assert gate.world_to_map(prepared, x, y) == gate.world_to_map(grid, x, y)
        assert gate.cost_at_world(prepared, x, y) == gate.cost_at_world(grid, x, y)
    assert prepared.data is grid.data


@pytest.mark.parametrize('reason,obstacle', [
    ('costmap coverage insufficient: point=(2,3)', False),
    ('costmap pose transform unavailable', False),
    ('collision check deadline exceeded', False),
    ('costmap invalid: empty dimensions', False),
    ('footprint collision: cost 253 >= 253', True),
    ('footprint collision: unknown costmap cell', True),
    ('scan footprint sweep collision', True),
])
def test_stop_classification(reason, obstacle):
    assert gate.obstacle_stop_reason(reason) is obstacle


def test_coverage_failure_still_stops_without_latching_obstacle():
    from test_obstacle_release import ObstacleReleaseState
    command = gate.ForkliftControlCommand()
    command.enable = command.forward = True
    command.velocity_mps = .5
    node = SimpleNamespace(
        _collision_check_enabled=True, _obstacle_release=ObstacleReleaseState(),
        _protected_speed=lambda _: .5,
        _collision_check_reason=lambda _: 'costmap coverage insufficient: point=(2,3)')
    assert gate.SafetyCommandGate._collision_stop_reason(node, command).startswith('costmap coverage')
    assert not node._obstacle_release.active
    node._obstacle_release.blocked(.7)
    gate.SafetyCommandGate._collision_stop_reason(node, command)
    assert node._obstacle_release.active  # Never erase an earlier real obstacle.
    assert node._obstacle_release.speed_floor == .7


def test_reverse_escape_cannot_skip_missing_coverage():
    grid = make_costmap()
    command = gate.ForkliftControlCommand()
    command.enable = command.reverse = True
    command.velocity_mps = .1
    collision, reason = gate.footprint_sweep_collision(
        grid, [(-.1, -.1), (.1, -.1), (.1, .1), (-.1, .1)],
        (100., 100., 0.), command, 1.4, .6, 0., 1., .1, math.pi / 2,
        .05, 100, True, allow_initial_collision_escape=True,
        prediction_poses=[(100., 100., 0.), (0., 0., 0.)])
    assert collision and reason.startswith('costmap coverage insufficient')
    assert 'sweep_pose=' in reason and 'point=' in reason


@pytest.mark.parametrize('field', ['resolution', 'origin'])
def test_invalid_grid_stops_without_exception(field):
    grid = make_costmap()
    if field == 'resolution':
        grid.info.resolution = float('nan')
    else:
        grid.info.origin.position.x = float('inf')
    prepared = gate.PreparedCostmap(grid)
    collision, reason = gate.footprint_collision_at_pose(
        prepared, [(-.1, -.1), (.1, -.1), (.1, .1)], (0., 0., 0.), .05, 100, True)
    assert collision and reason.startswith('costmap invalid')


def pose_node(source='odom', child='base_link'):
    msg = Odometry()
    msg.header.frame_id, msg.child_frame_id = source, child
    msg.header.stamp = Time(seconds=9.95).to_msg()
    msg.pose.pose.position.x = 1.
    msg.pose.pose.orientation.w = 1.
    calls = []

    def lookup(target, origin, stamp):
        calls.append((target, origin, stamp.nanoseconds))
        tf = TransformStamped()
        tf.transform.rotation.w = 1.
        if origin == 'odom':
            tf.transform.translation.y = 3.
            tf.transform.rotation.z = math.sin(math.pi / 4)
            tf.transform.rotation.w = math.cos(math.pi / 4)
        else:
            tf.transform.translation.x = .2
        return tf

    node = SimpleNamespace(
        _last_localization=msg, _base_frame_id='base_link', _localization_timeout_sec=.5,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=10.)),
        _tf_buffer=SimpleNamespace(lookup_transform=lookup))
    return node, calls


def test_same_frame_pose_needs_no_tf():
    node, calls = pose_node()
    assert gate.SafetyCommandGate._pose_in_costmap(node, 'odom') == (1., 0., 0.)
    assert not calls


def test_pose_composes_child_and_costmap_frames_at_source_time():
    node, calls = pose_node(child='base_footprint')
    pose = gate.SafetyCommandGate._pose_in_costmap(node, 'map')
    assert pose == pytest.approx((0., 4.2, math.pi / 2))
    assert [(c[0], c[1]) for c in calls] == [
        ('base_footprint', 'base_link'), ('map', 'odom')]
    assert all(c[2] == Time.from_msg(node._last_localization.header.stamp).nanoseconds for c in calls)


@pytest.mark.parametrize('failure', ['stale', 'frame', 'tf', 'nan'])
def test_invalid_pose_fails_closed(failure):
    node, _ = pose_node()
    if failure == 'stale':
        node._last_localization.header.stamp = Time(seconds=8.).to_msg()
    elif failure == 'frame':
        node._last_localization.header.frame_id = ''
    elif failure == 'nan':
        node._last_localization.pose.pose.position.x = float('nan')
    else:
        def unavailable(*args):
            raise RuntimeError('missing transform')
        node._tf_buffer.lookup_transform = unavailable
    with pytest.raises(ValueError, match='costmap pose'):
        gate.SafetyCommandGate._pose_in_costmap(node, 'map')


def test_sweep_prepares_metadata_once(monkeypatch):
    grid = make_costmap()
    calls = []
    original = gate.costmap_metadata

    def metadata(msg):
        calls.append(1)
        return original(msg)

    monkeypatch.setattr(gate, 'costmap_metadata', metadata)
    command = gate.ForkliftControlCommand()
    collision, _ = gate.footprint_sweep_collision(
        grid, [(-.1, -.1), (.1, -.1), (.1, .1), (-.1, .1)],
        (0., 0., 0.), command, 1.4, .6, 0., 1., .1, math.pi / 2,
        .05, 100, True, prediction_poses=[(0., 0., i * .01) for i in range(100)])
    assert not collision
    assert len(calls) == 2  # Validation and preparation, independent of samples.


def legacy_collision(grid, footprint, pose, spacing, threshold, unknown, exemption=None):
    points = [gate.transform_point(p, pose) for p in footprint]
    for i, start in enumerate(points):
        for x, y in gate.sampled_segment_points(start, points[(i + 1) % len(points)], spacing):
            cost = gate.cost_at_world(grid, x, y)
            if cost is None or (cost < 0 and unknown):
                return True
            if cost >= threshold and not gate.point_in_pallet_exemption((x, y), exemption):
                return True
    return False


@pytest.mark.parametrize('unknown', [False, True])
def test_collision_decisions_match_legacy_sampling(unknown):
    grid = make_costmap()
    rng = random.Random(2409)
    grid.data = [rng.choice([0] * 99 + [100, -1]) for _ in grid.data]
    footprint = [(-.2, -.1), (.2, -.1), (.2, .1), (-.2, .1)]
    exemption = gate.PalletExemptionZone(0., 0., .3, .2, .2)
    for _ in range(250):
        pose = (rng.uniform(-1.1, 1.1), rng.uniform(-1.1, 1.1), rng.uniform(-math.pi, math.pi))
        result, _ = gate.footprint_collision_at_pose(grid, footprint, pose, .05, 100, unknown, exemption)
        assert result == legacy_collision(grid, footprint, pose, .05, 100, unknown, exemption)


def test_real_sized_nav2_costmap_and_benchmark():
    grid = Costmap()
    grid.metadata.size_x = grid.metadata.size_y = 360
    grid.metadata.resolution = .05
    grid.metadata.origin.position.x = grid.metadata.origin.position.y = -9.
    grid.metadata.origin.orientation.w = 1.
    grid.data = [0] * (360 * 360)
    footprint = [(1.709, .610), (1.709, -.610), (-1.590, -.610), (-1.590, .610)]
    poses = [(i * .025, 0., i * .001) for i in range(100)]
    start = time.perf_counter()
    old = [legacy_collision(grid, footprint, p, .05, 253, True) for p in poses]
    legacy_seconds = time.perf_counter() - start
    prepared = gate.PreparedCostmap(grid)
    start = time.perf_counter()
    new = [gate.footprint_collision_at_pose(prepared, footprint, p, .05, 253, True)[0] for p in poses]
    prepared_seconds = time.perf_counter() - start
    assert old == new == [False] * len(poses)
    print('100-pose costmap benchmark: legacy={:.4f}s prepared={:.4f}s'.format(
        legacy_seconds, prepared_seconds))


def test_gate_coverage_failure_recovers_on_valid_snapshot():
    rclpy.init()
    node = gate.SafetyCommandGate()
    try:
        node._scan_protection_enabled = False
        grid = Costmap()
        grid.header.frame_id = 'odom'
        grid.metadata.size_x = grid.metadata.size_y = 360
        grid.metadata.resolution = .05
        grid.metadata.origin.position.x = grid.metadata.origin.position.y = -9.
        grid.metadata.origin.orientation.w = 1.
        grid.data = [0] * (360 * 360)
        node._on_costmap(grid)
        msg = Odometry()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = node._base_frame_id
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.orientation.w = 1.
        msg.pose.pose.position.x = 50.
        node._on_localization(msg)
        command = gate.ForkliftControlCommand()
        command.enable = command.forward = True
        command.velocity_mps = .5
        assert node._collision_stop_reason(command).startswith('costmap coverage insufficient')
        assert not node._obstacle_release.active
        msg.pose.pose.position.x = 0.
        msg.header.stamp = node.get_clock().now().to_msg()
        node._on_localization(msg)
        node._on_costmap(grid)
        assert node._collision_stop_reason(command) == ''
        assert node._checked_costmap[0] is grid
    finally:
        node.destroy_node()
        rclpy.shutdown()
