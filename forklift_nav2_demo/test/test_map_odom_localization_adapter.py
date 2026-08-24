import importlib.util
import math
from pathlib import Path

import pytest


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / 'scripts'
    / 'map_odom_localization_adapter.py'
)
SPEC = importlib.util.spec_from_file_location('map_odom_localization_adapter', SCRIPT)
adapter = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(adapter)


def compose_2d(a, b):
    ax, ay, ayaw = a
    bx, by, byaw = b
    cos_yaw = math.cos(ayaw)
    sin_yaw = math.sin(ayaw)
    return (
        ax + cos_yaw * bx - sin_yaw * by,
        ay + sin_yaw * bx + cos_yaw * by,
        adapter.normalize_angle(ayaw + byaw),
    )


def assert_pose_close(actual, expected):
    assert actual[0] == pytest.approx(expected[0])
    assert actual[1] == pytest.approx(expected[1])
    assert actual[2] == pytest.approx(expected[2])


def test_map_to_odom_identity_when_odom_and_map_base_match():
    map_to_base = (2.0, -1.0, 0.25)
    odom_to_base = (2.0, -1.0, 0.25)

    assert_pose_close(
        adapter.map_to_odom_2d(map_to_base, odom_to_base),
        (0.0, 0.0, 0.0),
    )


def test_map_to_odom_translation_offsets_odom_origin():
    map_to_base = (12.0, -4.0, 0.0)
    odom_to_base = (2.0, 3.0, 0.0)

    map_to_odom = adapter.map_to_odom_2d(map_to_base, odom_to_base)

    assert_pose_close(map_to_odom, (10.0, -7.0, 0.0))
    assert_pose_close(compose_2d(map_to_odom, odom_to_base), map_to_base)


def test_map_to_odom_handles_rotated_odom_frame():
    map_to_base = (5.0, 7.0, math.pi / 2.0)
    odom_to_base = (2.0, 0.0, 0.0)

    map_to_odom = adapter.map_to_odom_2d(map_to_base, odom_to_base)

    assert_pose_close(map_to_odom, (5.0, 5.0, math.pi / 2.0))
    assert_pose_close(compose_2d(map_to_odom, odom_to_base), map_to_base)


def test_apply_map_frame_offset_2d():
    corrected = adapter.apply_map_frame_offset_2d(
        (1.0, 2.0, math.pi - 0.1),
        (0.5, -0.25, 0.2),
    )

    assert_pose_close(corrected, (1.5, 1.75, -math.pi + 0.1))
