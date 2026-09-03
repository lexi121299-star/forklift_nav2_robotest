from types import SimpleNamespace

import pytest

from forklift_task_manager.pallet_maneuver import (
    PalletManeuverConfig,
    circular_turn_is_clear,
    ranked_candidate_offsets,
    select_clear_candidates,
)


def fake_costmap(size=300, resolution=0.10):
    origin = SimpleNamespace(
        position=SimpleNamespace(x=-10.0, y=-10.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    metadata = SimpleNamespace(
        size_x=size,
        size_y=size,
        resolution=resolution,
        origin=origin,
    )
    return SimpleNamespace(metadata=metadata, data=[0] * (size * size))


def set_cost(costmap, x, y, cost):
    origin = costmap.metadata.origin.position
    resolution = costmap.metadata.resolution
    mx = int((x - origin.x) // resolution)
    my = int((y - origin.y) // resolution)
    costmap.data[my * costmap.metadata.size_x + mx] = cost


def test_candidate_order_uses_near_direct_then_near_lateral_then_far():
    offsets = ranked_candidate_offsets(PalletManeuverConfig())

    assert offsets[0] == pytest.approx((2.5, 0.0))
    assert offsets[6] == pytest.approx((4.0, 0.0))
    assert offsets[7] == pytest.approx((2.5, 0.25))
    far_direct_index = offsets.index((4.25, 0.0))
    assert far_direct_index > offsets.index((4.0, -1.5))


def test_empty_costmap_respects_pallet_turn_keepout_before_selecting_candidate():
    candidates = select_clear_candidates(
        costmap=fake_costmap(),
        pallet_x=0.0,
        pallet_y=0.0,
        outward_yaw=0.0,
        frame_id='map',
        config=PalletManeuverConfig(),
    )

    assert candidates
    assert candidates[0].distance_m == pytest.approx(3.5)
    assert candidates[0].lateral_m == pytest.approx(0.0)
    assert candidates[0].direct_final_approach is True


def test_candidate_limit_returns_after_first_clear_candidate():
    candidates = select_clear_candidates(
        costmap=fake_costmap(),
        pallet_x=0.0,
        pallet_y=0.0,
        outward_yaw=0.0,
        frame_id='map',
        config=PalletManeuverConfig(),
        max_candidates=1,
    )

    assert len(candidates) == 1
    assert candidates[0].distance_m == pytest.approx(3.5)


def test_turn_envelope_rejects_lethal_cell_and_unknown_space():
    costmap = fake_costmap()
    assert circular_turn_is_clear(costmap, (2.5, 0.0), 1.9, 254)

    set_cost(costmap, 2.5, 0.0, 254)
    assert not circular_turn_is_clear(costmap, (2.5, 0.0), 1.9, 254)


def test_turn_envelope_only_ignores_lethal_cells_inside_selected_pallet_zone():
    costmap = fake_costmap()
    set_cost(costmap, 2.5, 0.0, 254)

    exemption = (2.5, 0.0, 0.0, 0.70, 0.65)
    assert circular_turn_is_clear(
        costmap, (2.5, 0.0), 1.9, 254, pallet_exemption=exemption
    )

    set_cost(costmap, 4.0, 0.0, 254)
    assert not circular_turn_is_clear(
        costmap, (2.5, 0.0), 1.9, 254, pallet_exemption=exemption
    )

    set_cost(costmap, 2.5, 0.0, 255)
    assert not circular_turn_is_clear(costmap, (2.5, 0.0), 1.9, 254)


def test_invalid_search_bounds_are_rejected():
    with pytest.raises(ValueError, match='min <= preferred <= max'):
        ranked_candidate_offsets(
            PalletManeuverConfig(
                turn_min_distance_m=2.5,
                turn_preferred_max_distance_m=5.5,
                turn_max_distance_m=5.0,
            )
        )
