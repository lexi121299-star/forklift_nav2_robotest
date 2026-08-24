import math

import pytest

from forklift_task_manager.pallet_approach import (
    PalletApproachConfig,
    PalletApproachError,
    build_pallet_approach,
    build_pallet_approach_route,
    pallet_exemption_active_for_distance,
)


def test_rear_forks_align_outward_then_reverse_to_stop():
    geometry = build_pallet_approach(
        pallet_x=10.0,
        pallet_y=4.0,
        pallet_yaw=0.0,
        frame_id='map',
        config=PalletApproachConfig(),
    )

    assert geometry.stop.x == pytest.approx(11.8)
    assert geometry.alignment.x == pytest.approx(12.2)
    assert geometry.pre_approach.x == pytest.approx(12.8)
    assert geometry.final_motion.distance_m == pytest.approx(-1.0)
    assert geometry.final_motion.expected_start_x == pytest.approx(12.8)
    assert geometry.final_motion.expected_start_yaw == pytest.approx(0.0)
    assert geometry.final_motion.pallet_exemption_x == pytest.approx(10.0)
    assert geometry.final_motion.pallet_exemption_y == pytest.approx(4.0)
    assert geometry.final_motion.pallet_exemption_yaw == pytest.approx(0.0)
    assert geometry.pre_approach.yaw == pytest.approx(0.0)

    route = build_pallet_approach_route(geometry)
    assert [target.name for target in route.targets] == [
        'pallet_alignment',
        'pallet_pre_approach',
        'pallet_final_approach',
    ]


def test_rotated_pallet_geometry_uses_arrow_direction():
    geometry = build_pallet_approach(
        pallet_x=2.0,
        pallet_y=3.0,
        pallet_yaw=math.pi / 2.0,
        frame_id='map',
        config=PalletApproachConfig(),
    )

    assert geometry.stop.x == pytest.approx(2.0)
    assert geometry.stop.y == pytest.approx(4.8)
    assert geometry.pre_approach.y == pytest.approx(5.8)
    assert geometry.pre_approach.yaw == pytest.approx(math.pi / 2.0)


def test_front_forks_use_forward_final_motion():
    geometry = build_pallet_approach(
        pallet_x=0.0,
        pallet_y=0.0,
        pallet_yaw=0.0,
        frame_id='map',
        config=PalletApproachConfig(forks_on_negative_x=False),
    )

    assert geometry.final_motion.distance_m == pytest.approx(1.0)
    assert abs(geometry.pre_approach.yaw) == pytest.approx(math.pi)
    assert geometry.alignment.x > geometry.pre_approach.x


def test_runup_must_be_shorter_than_final_motion():
    with pytest.raises(PalletApproachError):
        build_pallet_approach(
            pallet_x=0.0,
            pallet_y=0.0,
            pallet_yaw=0.0,
            frame_id='map',
            config=PalletApproachConfig(
                final_approach_distance_m=0.5,
                alignment_runup_distance_m=0.5,
            ),
        )


def test_pallet_exemption_distance_hysteresis():
    assert pallet_exemption_active_for_distance(
        currently_active=False,
        distance_m=4.1,
        activation_distance_m=4.0,
        deactivation_distance_m=4.5,
    ) is False
    assert pallet_exemption_active_for_distance(
        currently_active=False,
        distance_m=4.0,
        activation_distance_m=4.0,
        deactivation_distance_m=4.5,
    ) is True
    assert pallet_exemption_active_for_distance(
        currently_active=True,
        distance_m=4.2,
        activation_distance_m=4.0,
        deactivation_distance_m=4.5,
    ) is True
    assert pallet_exemption_active_for_distance(
        currently_active=True,
        distance_m=4.5,
        activation_distance_m=4.0,
        deactivation_distance_m=4.5,
    ) is False
