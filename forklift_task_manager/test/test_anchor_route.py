import math
from types import SimpleNamespace

import pytest

from forklift_task_manager.anchor_route import (
    AnchorRouteConfig,
    anchor_pose_is_safe,
    select_route_anchors,
    validate_anchor_config,
)
from forklift_task_manager.route_model import PoseTarget


def _costmap(width=240, height=120, resolution=0.1):
    orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    origin = SimpleNamespace(
        position=SimpleNamespace(x=-2.0, y=-6.0),
        orientation=orientation,
    )
    metadata = SimpleNamespace(
        size_x=width, size_y=height, resolution=resolution, origin=origin
    )
    return SimpleNamespace(metadata=metadata, data=[0] * (width * height))


def _path(points):
    poses = []
    for x, y in points:
        poses.append(SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )))
    return SimpleNamespace(poses=poses)


def _mark(costmap, x, y, value=254):
    origin = costmap.metadata.origin.position
    resolution = costmap.metadata.resolution
    mx = int(math.floor((x - origin.x) / resolution))
    my = int(math.floor((y - origin.y) / resolution))
    costmap.data[my * costmap.metadata.size_x + mx] = value


def _config(**overrides):
    values = dict(
        preferred_spacing_m=4.0,
        min_spacing_m=1.0,
        max_spacing_m=5.0,
        min_clearance_m=0.1,
        approach_clearance_m=0.5,
        footprint=((0.5, 0.3), (0.5, -0.3), (-0.5, -0.3), (-0.5, 0.3)),
    )
    values.update(overrides)
    return AnchorRouteConfig(**values)


def test_validate_anchor_config_rejects_inverted_spacing():
    with pytest.raises(ValueError):
        validate_anchor_config(
            _config(min_spacing_m=4.5, preferred_spacing_m=4.0)
        )


def test_selects_spaced_safe_anchors_and_preserves_final_goal():
    path = _path([(float(index), 0.0) for index in range(13)])
    goal = PoseTarget('goal', 12.0, 0.0, 1.2, 'map')

    anchors = select_route_anchors(path, _costmap(), goal, _config())

    assert [round(anchor.x, 1) for anchor in anchors] == [4.0, 8.0, 12.0]
    assert anchors[-1] == goal
    assert anchors[0].yaw == pytest.approx(0.0)


def test_blocked_preferred_anchor_uses_nearby_clear_pose():
    costmap = _costmap()
    for x in (3.5, 4.0, 4.5):
        _mark(costmap, x, 0.0)
    path = _path([(index * 0.5, 0.0) for index in range(17)])
    goal = PoseTarget('goal', 8.0, 0.0, 0.0, 'map')

    anchors = select_route_anchors(path, costmap, goal, _config())

    assert anchors[-1] == goal
    assert anchors[0].x not in (3.5, 4.0, 4.5)
    assert anchor_pose_is_safe(
        costmap, anchors[0].x, anchors[0].y, anchors[0].yaw, _config()
    )


def test_short_topology_returns_only_final_goal():
    goal = PoseTarget('goal', 1.0, 0.0, -0.4, 'map')
    anchors = select_route_anchors(
        _path([(0.0, 0.0), (1.0, 0.0)]), _costmap(), goal, _config()
    )
    assert anchors == [goal]


def test_anchor_route_never_exceeds_configured_segment_length():
    path = _path([(index * 0.5, 0.0) for index in range(25)])
    goal = PoseTarget('goal', 12.0, 0.0, 0.0, 'map')

    anchors = select_route_anchors(path, _costmap(), goal, _config())

    positions = [0.0] + [anchor.x for anchor in anchors]
    assert max(
        positions[index] - positions[index - 1]
        for index in range(1, len(positions))
    ) <= 5.0


def test_anchor_route_interpolates_sparse_topology_segments():
    path = _path([(0.0, 0.0), (10.0, 0.0), (10.0, 4.0)])
    goal = PoseTarget('goal', 10.0, 4.0, 0.0, 'map')

    anchors = select_route_anchors(path, _costmap(), goal, _config())

    assert anchors[0].x == pytest.approx(4.0, abs=0.3)
    positions = [(0.0, 0.0)] + [(anchor.x, anchor.y) for anchor in anchors]
    assert all(
        math.hypot(
            positions[index][0] - positions[index - 1][0],
            positions[index][1] - positions[index - 1][1],
        ) <= 5.0
        for index in range(1, len(positions))
    )


def test_grid_corner_does_not_create_extra_anchor_by_default():
    path = _path([(0.0, 0.0), (3.0, 0.0), (3.0, 0.5), (8.0, 0.5)])
    goal = PoseTarget('goal', 8.0, 0.5, 0.0, 'map')

    anchors = select_route_anchors(path, _costmap(), goal, _config())

    assert len(anchors) == 2
    assert anchors[-1] == goal


def test_anchor_route_rejects_unbridgeable_long_gap():
    costmap = _costmap()
    for x_index in range(10, 111):
        _mark(costmap, x_index * 0.1, 0.0)
    path = _path([(index * 0.5, 0.0) for index in range(25)])
    goal = PoseTarget('goal', 12.0, 0.0, 0.0, 'map')

    anchors = select_route_anchors(path, costmap, goal, _config())

    assert anchors == []
