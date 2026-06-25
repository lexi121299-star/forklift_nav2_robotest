import math

import pytest
import yaml

from forklift_task_manager.route_model import (
    RouteConfigError,
    derive_segment_poses,
    load_routes,
    load_stations,
)


def write_yaml(path, data):
    path.write_text(yaml.safe_dump(data), encoding='utf-8')
    return str(path)


def test_derive_segment_poses_right_angle_points_toward_next_point():
    poses = derive_segment_poses([[0, 0], [2, 0], [2, 2]])

    assert len(poses) == 3
    assert poses[0][2] == pytest.approx(0.0)
    assert poses[1][2] == pytest.approx(math.pi / 2.0)


def test_derive_segment_poses_explicit_yaw_overrides_automatic_value():
    poses = derive_segment_poses([[0, 0], [2, 0, -0.25], [2, 2]])

    assert poses[1][2] == pytest.approx(-0.25)


def test_derive_segment_poses_final_point_follows_last_segment():
    poses = derive_segment_poses([[0, 0], [2, 0], [2, 2]])

    assert poses[-1][2] == pytest.approx(math.pi / 2.0)


def test_derive_segment_poses_collinear_points_do_not_add_false_turn():
    poses = derive_segment_poses([[0, 0], [1, 0], [3, 0]])

    assert [pose[2] for pose in poses] == pytest.approx([0.0, 0.0, 0.0])


def test_derive_segment_poses_drops_adjacent_duplicate_without_interpolation():
    with pytest.warns(UserWarning, match='dropping duplicate waypoint'):
        poses = derive_segment_poses([[0, 0], [0.0001, 0], [2, 0]])

    assert len(poses) == 2
    assert poses[0] == pytest.approx((0.0, 0.0, 0.0))
    assert poses[1] == pytest.approx((2.0, 0.0, 0.0))


def test_derive_segment_poses_rejects_single_point_without_yaw():
    with pytest.raises(RouteConfigError, match='single-point route must provide yaw'):
        derive_segment_poses([[1, 2]])


def test_load_stations_and_named_and_sparse_routes(tmp_path):
    stations_path = write_yaml(
        tmp_path / 'stations.yaml',
        {
            'stations': {
                'A': {'x': 1, 'y': 2, 'yaw': 0.5, 'frame_id': 'map'},
                'B': {'x': 3, 'y': 4, 'yaw': 1.0},
            }
        },
    )
    routes_path = write_yaml(
        tmp_path / 'routes.yaml',
        {
            'routes': {
                'named': {
                    'loop': False,
                    'segments': [{'to': 'A', 'type': 'drive'}, {'to': 'B'}],
                },
                'sparse': {
                    'loop': True,
                    'waypoints': [[0, 0], [2, 0], [2, 2]],
                },
            }
        },
    )

    stations = load_stations(stations_path)
    routes = load_routes(routes_path, stations)

    assert routes['named'].targets[0].yaw == pytest.approx(0.5)
    assert routes['named'].targets[1].frame_id == 'map'
    assert routes['sparse'].loop is True
    assert len(routes['sparse'].targets) == 3
    assert routes['sparse'].targets[1].yaw == pytest.approx(math.pi / 2.0)


def test_load_routes_rejects_segments_and_waypoints_together(tmp_path):
    routes_path = write_yaml(
        tmp_path / 'routes.yaml',
        {
            'bad': {
                'segments': [{'to': 'A'}],
                'waypoints': [[0, 0, 0]],
            }
        },
    )

    with pytest.raises(RouteConfigError, match='exactly one'):
        load_routes(routes_path, {})


def test_load_routes_rejects_unknown_station(tmp_path):
    routes_path = write_yaml(
        tmp_path / 'routes.yaml',
        {'bad': {'segments': [{'to': 'missing'}]}},
    )

    with pytest.raises(RouteConfigError, match='unknown station'):
        load_routes(routes_path, {})
