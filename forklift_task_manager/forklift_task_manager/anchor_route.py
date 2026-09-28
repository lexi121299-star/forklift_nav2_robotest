"""Select safe intermediate goals from a validated topology path."""

import math
from dataclasses import dataclass
from typing import Any, List, Sequence, Tuple

from .route_model import PoseTarget


Point2D = Tuple[float, float]


@dataclass(frozen=True)
class AnchorRouteConfig:
    """Geometry and retry limits for hierarchical navigation fallback."""

    preferred_spacing_m: float = 4.0
    min_spacing_m: float = 1.5
    max_spacing_m: float = 5.0
    min_clearance_m: float = 0.25
    approach_clearance_m: float = 0.50
    max_count: int = 20
    split_max_depth: int = 4
    segment_retry_count: int = 2
    # Nav2 can leave an intermediate action active at the last few decimetres
    # even after the vehicle has stopped inside its configured goal tolerance.
    # These values are only used to advance a non-final anchor safely.
    arrival_tolerance_m: float = 0.35
    arrival_speed_mps: float = 0.03
    arrival_settle_sec: float = 0.30
    # Let Nav2 finish processing a cancellation before the next anchor action
    # is sent. Foxy BT can otherwise acknowledge only the preempt request.
    arrival_transition_delay_sec: float = 0.25
    collision_cost_threshold: int = 254
    corner_angle_rad: float = 0.35
    force_corner_anchors: bool = False
    topology_sample_spacing_m: float = 0.25
    footprint: Tuple[Point2D, ...] = (
        (1.709, 0.610),
        (1.709, -0.610),
        (-1.590, -0.610),
        (-1.590, 0.610),
    )


def validate_anchor_config(config: AnchorRouteConfig) -> None:
    if not 0.0 < config.min_spacing_m <= config.preferred_spacing_m:
        raise ValueError('anchor spacing must satisfy 0 < min <= preferred')
    if config.preferred_spacing_m > config.max_spacing_m:
        raise ValueError('anchor spacing must satisfy preferred <= max')
    if config.min_clearance_m < 0.0 or config.approach_clearance_m < 0.0:
        raise ValueError('anchor clearances must not be negative')
    if config.max_count <= 0 or config.split_max_depth < 0:
        raise ValueError('anchor count/depth parameters are invalid')
    if config.segment_retry_count < 0:
        raise ValueError('anchor_segment_retry_count must not be negative')
    if config.arrival_tolerance_m <= 0.0:
        raise ValueError('anchor_arrival_tolerance_m must be positive')
    if (
        config.arrival_speed_mps < 0.0
        or config.arrival_settle_sec < 0.0
        or config.arrival_transition_delay_sec < 0.0
    ):
        raise ValueError('anchor arrival timing values must not be negative')
    if config.topology_sample_spacing_m <= 0.0:
        raise ValueError('topology_sample_spacing_m must be positive')
    if not 1 <= config.collision_cost_threshold <= 255:
        raise ValueError('anchor collision threshold must be in [1, 255]')
    if len(config.footprint) < 3:
        raise ValueError('anchor footprint must contain at least three points')


def _metadata(costmap: Any):
    metadata = costmap.metadata
    return (
        int(metadata.size_x),
        int(metadata.size_y),
        float(metadata.resolution),
        metadata.origin,
    )


def _cost_at(costmap: Any, x: float, y: float) -> int:
    width, height, resolution, origin = _metadata(costmap)
    if width <= 0 or height <= 0 or resolution <= 0.0:
        return 255
    q = origin.orientation
    origin_yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    dx = x - origin.position.x
    dy = y - origin.position.y
    local_x = dx * math.cos(origin_yaw) + dy * math.sin(origin_yaw)
    local_y = -dx * math.sin(origin_yaw) + dy * math.cos(origin_yaw)
    mx = int(math.floor(local_x / resolution))
    my = int(math.floor(local_y / resolution))
    if mx < 0 or my < 0 or mx >= width or my >= height:
        return 255
    index = my * width + mx
    return int(costmap.data[index]) if index < len(costmap.data) else 255


def _inside_convex_polygon(
    x: float, y: float, polygon: Sequence[Point2D]
) -> bool:
    reference = 0.0
    for index, first in enumerate(polygon):
        second = polygon[(index + 1) % len(polygon)]
        cross = (
            (second[0] - first[0]) * (y - first[1])
            - (second[1] - first[1]) * (x - first[0])
        )
        if abs(cross) <= 1e-9:
            continue
        if reference == 0.0:
            reference = cross
        elif reference * cross < 0.0:
            return False
    return True


def _padded_footprint(
    footprint: Sequence[Point2D], padding_m: float
) -> Tuple[Point2D, ...]:
    return tuple(
        (
            x + math.copysign(padding_m, x) if abs(x) > 1e-9 else x,
            y + math.copysign(padding_m, y) if abs(y) > 1e-9 else y,
        )
        for x, y in footprint
    )


def footprint_peak_cost(
    costmap: Any,
    x: float,
    y: float,
    yaw: float,
    footprint: Sequence[Point2D],
    padding_m: float,
) -> int:
    """Return the largest cost touched by a filled, padded footprint."""

    _width, _height, resolution, _origin = _metadata(costmap)
    spacing = max(0.05, resolution)
    padded = _padded_footprint(footprint, padding_m)
    min_x = min(point[0] for point in padded)
    max_x = max(point[0] for point in padded)
    min_y = min(point[1] for point in padded)
    max_y = max(point[1] for point in padded)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    peak = 0
    local_x = min_x
    while local_x <= max_x + 1e-9:
        local_y = min_y
        while local_y <= max_y + 1e-9:
            if _inside_convex_polygon(local_x, local_y, padded):
                world_x = x + local_x * cos_yaw - local_y * sin_yaw
                world_y = y + local_x * sin_yaw + local_y * cos_yaw
                peak = max(peak, _cost_at(costmap, world_x, world_y))
            local_y += spacing
        local_x += spacing
    return peak


def anchor_pose_is_safe(
    costmap: Any,
    x: float,
    y: float,
    yaw: float,
    config: AnchorRouteConfig,
) -> bool:
    """Require clearance at an anchor and immediately before/after it."""

    approach_offsets = (
        -config.approach_clearance_m,
        0.0,
        config.approach_clearance_m,
    )
    for offset in approach_offsets:
        pose_x = x + offset * math.cos(yaw)
        pose_y = y + offset * math.sin(yaw)
        if footprint_peak_cost(
            costmap,
            pose_x,
            pose_y,
            yaw,
            config.footprint,
            config.min_clearance_m,
        ) >= config.collision_cost_threshold:
            return False
    return True


def _path_samples(path: Any, sample_spacing_m: float):
    """Densify topology segments so anchors cannot skip a long clear aisle."""

    samples = []
    distance = 0.0
    for pose_stamped in path.poses:
        x = float(pose_stamped.pose.position.x)
        y = float(pose_stamped.pose.position.y)
        if not samples:
            samples.append((distance, x, y))
            continue
        previous_x = samples[-1][1]
        previous_y = samples[-1][2]
        segment_length = math.hypot(x - previous_x, y - previous_y)
        if segment_length <= 1e-4:
            continue
        steps = max(1, int(math.ceil(segment_length / sample_spacing_m)))
        for step_index in range(1, steps + 1):
            ratio = float(step_index) / float(steps)
            samples.append((
                distance + ratio * segment_length,
                previous_x + ratio * (x - previous_x),
                previous_y + ratio * (y - previous_y),
            ))
        distance += segment_length
    return samples


def _sample_yaw(samples, index: int) -> float:
    low = max(0, index - 1)
    high = min(len(samples) - 1, index + 1)
    return math.atan2(samples[high][2] - samples[low][2],
                      samples[high][1] - samples[low][1])


def select_route_anchors(
    path: Any,
    costmap: Any,
    final_goal: PoseTarget,
    config: AnchorRouteConfig,
    spacing_scale: float = 1.0,
) -> List[PoseTarget]:
    """Select safe stopping poses and append the original final goal."""

    validate_anchor_config(config)
    samples = _path_samples(path, config.topology_sample_spacing_m)
    if len(samples) < 2:
        return []
    total_length = samples[-1][0]
    preferred = min(
        config.max_spacing_m,
        max(config.min_spacing_m, config.preferred_spacing_m * spacing_scale),
    )
    desired_distances = []
    distance = preferred
    while distance < total_length - config.min_spacing_m:
        desired_distances.append(distance)
        distance += preferred

    # A topology grid corner is a routing hint, not automatically a safe
    # stopping point. Forcing every sampled 20-degree change into the queue
    # turns harmless coarse-grid jitter into repeated stop-pivot-go segments.
    if config.force_corner_anchors:
        for index in range(1, len(samples) - 1):
            incoming = math.atan2(
                samples[index][2] - samples[index - 1][2],
                samples[index][1] - samples[index - 1][1],
            )
            outgoing = math.atan2(
                samples[index + 1][2] - samples[index][2],
                samples[index + 1][1] - samples[index][1],
            )
            change = abs(math.atan2(math.sin(outgoing - incoming),
                                    math.cos(outgoing - incoming)))
            if change >= config.corner_angle_rad:
                desired_distances.append(samples[index][0])

    anchors = []
    selected_distances = []
    search_radius = max(0.25, 0.5 * preferred)
    for desired in sorted(desired_distances):
        if (
            selected_distances
            and desired - selected_distances[-1] < config.min_spacing_m
        ):
            continue
        previous_distance = (
            selected_distances[-1] if selected_distances else 0.0
        )
        candidates = []
        for index, sample in enumerate(samples[1:-1], start=1):
            path_distance, x, y = sample
            if abs(path_distance - desired) > search_radius:
                continue
            if path_distance - previous_distance < config.min_spacing_m:
                continue
            if path_distance - previous_distance > config.max_spacing_m:
                continue
            if total_length - path_distance < config.min_spacing_m:
                continue
            yaw = _sample_yaw(samples, index)
            if not anchor_pose_is_safe(costmap, x, y, yaw, config):
                continue
            peak = footprint_peak_cost(
                costmap, x, y, yaw, config.footprint, config.min_clearance_m
            )
            candidates.append((peak, abs(path_distance - desired), index, yaw))
        if not candidates:
            continue
        _peak, _offset, index, yaw = min(candidates)
        path_distance, x, y = samples[index]
        anchors.append(PoseTarget(
            name='anchor_{:02d}'.format(len(anchors) + 1),
            x=x,
            y=y,
            yaw=yaw,
            frame_id=final_goal.frame_id,
        ))
        selected_distances.append(path_distance)
        if len(anchors) >= config.max_count - 1:
            break

    if not anchors and total_length >= 2.0 * config.min_spacing_m:
        midpoint = 0.5 * total_length
        fallback_candidates = []
        for index, (path_distance, x, y) in enumerate(samples[1:-1], start=1):
            if path_distance < config.min_spacing_m:
                continue
            if total_length - path_distance < config.min_spacing_m:
                continue
            yaw = _sample_yaw(samples, index)
            if anchor_pose_is_safe(costmap, x, y, yaw, config):
                peak = footprint_peak_cost(
                    costmap, x, y, yaw, config.footprint,
                    config.min_clearance_m,
                )
                fallback_candidates.append(
                    (peak, abs(path_distance - midpoint), index, yaw)
                )
        if fallback_candidates:
            _peak, _offset, index, yaw = min(fallback_candidates)
            _distance, x, y = samples[index]
            anchors.append(PoseTarget(
                name='anchor_01', x=x, y=y, yaw=yaw,
                frame_id=final_goal.frame_id,
            ))
            selected_distances.append(_distance)

    # Do not silently create a long segment when a preferred anchor was
    # blocked. Bridge every remaining gap under the configured hard maximum,
    # or reject this anchor route so the caller can split/replan it.
    while (
        total_length - (selected_distances[-1] if selected_distances else 0.0)
        > config.max_spacing_m
    ):
        previous_distance = (
            selected_distances[-1] if selected_distances else 0.0
        )
        target_distance = min(
            previous_distance + preferred,
            total_length - config.min_spacing_m,
        )
        bridge_candidates = []
        for index, (path_distance, x, y) in enumerate(samples[1:-1], start=1):
            gap = path_distance - previous_distance
            if gap < config.min_spacing_m or gap > config.max_spacing_m:
                continue
            if total_length - path_distance < config.min_spacing_m:
                continue
            yaw = _sample_yaw(samples, index)
            if not anchor_pose_is_safe(costmap, x, y, yaw, config):
                continue
            peak = footprint_peak_cost(
                costmap, x, y, yaw, config.footprint,
                config.min_clearance_m,
            )
            bridge_candidates.append(
                (peak, abs(path_distance - target_distance), index, yaw)
            )
        if not bridge_candidates or len(anchors) >= config.max_count - 1:
            return []
        _peak, _offset, index, yaw = min(bridge_candidates)
        path_distance, x, y = samples[index]
        anchors.append(PoseTarget(
            name='anchor_{:02d}'.format(len(anchors) + 1),
            x=x,
            y=y,
            yaw=yaw,
            frame_id=final_goal.frame_id,
        ))
        selected_distances.append(path_distance)

    anchors.append(PoseTarget(
        name=final_goal.name,
        x=final_goal.x,
        y=final_goal.y,
        yaw=final_goal.yaw,
        frame_id=final_goal.frame_id,
    ))
    return anchors
