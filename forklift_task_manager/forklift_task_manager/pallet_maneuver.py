"""Costmap-aware staging selection for an RViz pallet approach."""

import math
from dataclasses import dataclass
from typing import Any, Iterable, List, Optional, Sequence, Tuple

from .route_model import PoseTarget

Point2D = Tuple[float, float]
PalletExemption = Tuple[float, float, float, float, float]


@dataclass(frozen=True)
class PalletManeuverConfig:
    """Search bounds and vehicle geometry for a pallet staging maneuver."""

    fork_tip_offset_m: float = 1.59
    stop_clearance_m: float = 0.30
    turn_min_distance_m: float = 2.50
    turn_preferred_max_distance_m: float = 4.00
    turn_max_distance_m: float = 5.00
    turn_distance_step_m: float = 0.25
    turn_lateral_max_m: float = 1.50
    turn_lateral_step_m: float = 0.25
    turn_runup_distance_m: float = 0.60
    turn_clearance_padding_m: float = 0.10
    pallet_turn_keepout_radius_m: float = 1.50
    collision_cost_threshold: int = 254
    footprint: Tuple[Point2D, ...] = (
        (1.709, 0.610),
        (1.709, -0.610),
        (-1.590, -0.610),
        (-1.590, 0.610),
    )

    @property
    def stop_base_distance_m(self) -> float:
        return self.fork_tip_offset_m + self.stop_clearance_m


@dataclass(frozen=True)
class PalletStagingCandidate:
    """One ranked staging pose and its outward runup pose."""

    distance_m: float
    lateral_m: float
    runup: PoseTarget
    staging: PoseTarget
    direct_final_approach: bool


def _positive(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError('{} must be positive and finite'.format(name))
    return result


def validate_maneuver_config(config: PalletManeuverConfig) -> None:
    """Raise when search bounds or vehicle geometry are inconsistent."""

    minimum = _positive(config.turn_min_distance_m, 'turn_min_distance_m')
    preferred = _positive(
        config.turn_preferred_max_distance_m,
        'turn_preferred_max_distance_m',
    )
    maximum = _positive(config.turn_max_distance_m, 'turn_max_distance_m')
    if not minimum <= preferred <= maximum:
        raise ValueError('turn distances must satisfy min <= preferred <= max')
    if minimum <= config.stop_base_distance_m:
        raise ValueError('turn_min_distance_m must exceed the final base stop distance')
    _positive(config.turn_distance_step_m, 'turn_distance_step_m')
    _positive(config.turn_lateral_step_m, 'turn_lateral_step_m')
    _positive(config.turn_runup_distance_m, 'turn_runup_distance_m')
    _positive(config.pallet_turn_keepout_radius_m, 'pallet_turn_keepout_radius_m')
    if config.turn_lateral_max_m < 0.0:
        raise ValueError('turn_lateral_max_m must not be negative')
    if len(config.footprint) < 3:
        raise ValueError('footprint must contain at least three points')
    if not 1 <= int(config.collision_cost_threshold) <= 255:
        raise ValueError('collision_cost_threshold must be in [1, 255]')


def _inclusive_values(start: float, stop: float, step: float) -> List[float]:
    if stop < start - 1e-9:
        return []
    count = int(math.floor((stop - start) / step + 1e-9))
    values = [start + index * step for index in range(count + 1)]
    if not values or values[-1] < stop - 1e-9:
        values.append(stop)
    return values


def ranked_candidate_offsets(config: PalletManeuverConfig) -> List[Tuple[float, float]]:
    """Return direct/near/far candidate offsets in deterministic preference order."""

    validate_maneuver_config(config)
    step = config.turn_distance_step_m
    near = _inclusive_values(
        config.turn_min_distance_m,
        config.turn_preferred_max_distance_m,
        step,
    )
    far_start = config.turn_preferred_max_distance_m + step
    far = _inclusive_values(far_start, config.turn_max_distance_m, step)
    lateral_values: List[float] = []
    magnitude = config.turn_lateral_step_m
    while magnitude <= config.turn_lateral_max_m + 1e-9:
        lateral_values.extend((magnitude, -magnitude))
        magnitude += config.turn_lateral_step_m

    result: List[Tuple[float, float]] = []
    result.extend((distance, 0.0) for distance in near)
    result.extend((distance, lateral) for lateral in lateral_values for distance in near)
    result.extend((distance, 0.0) for distance in far)
    result.extend((distance, lateral) for lateral in lateral_values for distance in far)
    return result


def build_candidate(
    *,
    pallet_x: float,
    pallet_y: float,
    outward_yaw: float,
    frame_id: str,
    distance_m: float,
    lateral_m: float,
    runup_distance_m: float,
) -> PalletStagingCandidate:
    """Build a staging pose from pallet-normal and lateral offsets."""

    nx = math.cos(outward_yaw)
    ny = math.sin(outward_yaw)
    tx = -ny
    ty = nx

    def pose(name: str, distance: float) -> PoseTarget:
        return PoseTarget(
            name=name,
            x=pallet_x + distance * nx + lateral_m * tx,
            y=pallet_y + distance * ny + lateral_m * ty,
            yaw=math.atan2(math.sin(outward_yaw + math.pi),
                           math.cos(outward_yaw + math.pi)),
            frame_id=frame_id,
        )

    return PalletStagingCandidate(
        distance_m=float(distance_m),
        lateral_m=float(lateral_m),
        runup=pose('pallet_staging_runup', distance_m + runup_distance_m),
        staging=pose('pallet_staging', distance_m),
        direct_final_approach=abs(lateral_m) <= 1e-9,
    )


def _metadata(costmap: Any):
    metadata = costmap.metadata
    return (
        int(metadata.size_x),
        int(metadata.size_y),
        float(metadata.resolution),
        metadata.origin,
    )


def _world_to_map(costmap: Any, x: float, y: float):
    width, height, resolution, origin = _metadata(costmap)
    if width <= 0 or height <= 0 or resolution <= 0.0:
        return None
    q = origin.orientation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    dx = x - origin.position.x
    dy = y - origin.position.y
    local_x = dx * math.cos(yaw) + dy * math.sin(yaw)
    local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)
    mx = int(math.floor(local_x / resolution))
    my = int(math.floor(local_y / resolution))
    if mx < 0 or my < 0 or mx >= width or my >= height:
        return None
    return mx, my


def cost_at(costmap: Any, x: float, y: float) -> int:
    cell = _world_to_map(costmap, x, y)
    if cell is None:
        return 255
    width, _height, _resolution, _origin = _metadata(costmap)
    index = cell[1] * width + cell[0]
    if index >= len(costmap.data):
        return 255
    return int(costmap.data[index])


def circular_turn_is_clear(
    costmap: Any,
    center: Point2D,
    radius_m: float,
    cost_threshold: int,
    pallet_exemption: Optional[PalletExemption] = None,
) -> bool:
    """Check the complete fixed-center rotation envelope against a costmap.

    ``pallet_exemption`` is an oriented (x, y, yaw, half-length, half-width)
    rectangle.  It is deliberately scoped to the selected pallet so a pivot
    may overlap that pallet's scan returns without ignoring nearby obstacles.
    """

    _width, _height, resolution, _origin = _metadata(costmap)
    spacing = max(0.05, resolution)
    cells = int(math.ceil(radius_m / spacing))
    for ix in range(-cells, cells + 1):
        dx = ix * spacing
        max_dy = math.sqrt(max(0.0, radius_m * radius_m - dx * dx))
        y_cells = int(math.ceil(max_dy / spacing))
        for iy in range(-y_cells, y_cells + 1):
            point = (center[0] + dx, center[1] + iy * spacing)
            if _point_in_pallet_exemption(point, pallet_exemption):
                continue
            if cost_at(costmap, point[0], point[1]) >= cost_threshold:
                return False
    return True


def _point_in_pallet_exemption(
    point: Point2D,
    pallet_exemption: Optional[PalletExemption],
) -> bool:
    if pallet_exemption is None:
        return False
    x, y, yaw, half_length, half_width = pallet_exemption
    dx = point[0] - x
    dy = point[1] - y
    local_x = dx * math.cos(yaw) + dy * math.sin(yaw)
    local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return abs(local_x) <= half_length and abs(local_y) <= half_width


def _footprint_samples(
    footprint: Sequence[Point2D],
    spacing: float,
) -> Iterable[Point2D]:
    min_x = min(point[0] for point in footprint)
    max_x = max(point[0] for point in footprint)
    min_y = min(point[1] for point in footprint)
    max_y = max(point[1] for point in footprint)
    x = min_x
    while x <= max_x + 1e-9:
        y = min_y
        while y <= max_y + 1e-9:
            yield x, y
            y += spacing
        x += spacing


def straight_sweep_is_clear(
    costmap: Any,
    start: Point2D,
    end: Point2D,
    yaw: float,
    footprint: Sequence[Point2D],
    padding_m: float,
    cost_threshold: int,
) -> bool:
    """Check a straight vehicle-body sweep using filled local footprint samples."""

    length = math.hypot(end[0] - start[0], end[1] - start[1])
    pose_steps = max(1, int(math.ceil(length / 0.10)))
    padded = tuple(
        (
            x + math.copysign(padding_m, x) if abs(x) > 1e-9 else x,
            y + math.copysign(padding_m, y) if abs(y) > 1e-9 else y,
        )
        for x, y in footprint
    )
    local_samples = tuple(_footprint_samples(padded, 0.10))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    for index in range(pose_steps + 1):
        ratio = float(index) / float(pose_steps)
        px = start[0] + ratio * (end[0] - start[0])
        py = start[1] + ratio * (end[1] - start[1])
        for local_x, local_y in local_samples:
            world_x = px + local_x * cos_yaw - local_y * sin_yaw
            world_y = py + local_x * sin_yaw + local_y * cos_yaw
            if cost_at(costmap, world_x, world_y) >= cost_threshold:
                return False
    return True


def select_clear_candidates(
    *,
    costmap: Any,
    pallet_x: float,
    pallet_y: float,
    outward_yaw: float,
    frame_id: str,
    config: PalletManeuverConfig,
    max_candidates: Optional[int] = None,
) -> List[PalletStagingCandidate]:
    """Return ranked clear candidates, optionally stopping at a fixed count."""

    validate_maneuver_config(config)
    if max_candidates is not None and int(max_candidates) <= 0:
        raise ValueError('max_candidates must be positive when provided')
    turn_radius = max(
        math.hypot(x, y) for x, y in config.footprint
    ) + config.turn_clearance_padding_m
    # A selected pallet is deliberately not treated as free space while the
    # vehicle pivots.  The staging base-link point must leave enough radial
    # room for the complete rotating vehicle envelope, the pallet footprint,
    # and localisation margin represented by ``pallet_turn_keepout_radius_m``.
    pivot_keepout_radius = turn_radius + config.pallet_turn_keepout_radius_m
    nx = math.cos(outward_yaw)
    ny = math.sin(outward_yaw)
    stop = (
        pallet_x + config.stop_base_distance_m * nx,
        pallet_y + config.stop_base_distance_m * ny,
    )
    candidates: List[PalletStagingCandidate] = []
    for distance, lateral in ranked_candidate_offsets(config):
        candidate = build_candidate(
            pallet_x=pallet_x,
            pallet_y=pallet_y,
            outward_yaw=outward_yaw,
            frame_id=frame_id,
            distance_m=distance,
            lateral_m=lateral,
            runup_distance_m=config.turn_runup_distance_m,
        )
        center = (candidate.staging.x, candidate.staging.y)
        if math.hypot(center[0] - pallet_x, center[1] - pallet_y) < (
            pivot_keepout_radius
        ):
            continue
        if not circular_turn_is_clear(
            costmap, center, turn_radius, config.collision_cost_threshold
        ):
            continue
        # Nav2 stops at ``runup`` before task-level fine motion takes over.
        # Usually it already has the infeed heading, but a local corrective
        # pivot may be needed before the fixed 0.6 m infeed. Validate that
        # recovery pivot here rather than discovering an obstructed turn after
        # leaving Nav2 control.
        runup_center = (candidate.runup.x, candidate.runup.y)
        if not circular_turn_is_clear(
            costmap, runup_center, turn_radius, config.collision_cost_threshold
        ):
            continue
        if not straight_sweep_is_clear(
            costmap,
            (candidate.runup.x, candidate.runup.y),
            center,
            candidate.staging.yaw,
            config.footprint,
            config.turn_clearance_padding_m,
            config.collision_cost_threshold,
        ):
            continue
        if candidate.direct_final_approach and not straight_sweep_is_clear(
            costmap,
            center,
            stop,
            outward_yaw,
            config.footprint,
            config.turn_clearance_padding_m,
            config.collision_cost_threshold,
        ):
            continue
        candidates.append(candidate)
        if max_candidates is not None and len(candidates) >= max_candidates:
            return candidates
    return candidates
