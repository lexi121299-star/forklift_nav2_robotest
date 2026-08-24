"""Geometry for an A-star aligned, straight pallet approach."""

import math
from dataclasses import dataclass
from typing import Tuple

from .route_model import PoseTarget, RelativeMoveTarget, RouteDefinition


class PalletApproachError(ValueError):
    """Raised when pallet approach geometry is unsafe or invalid."""


@dataclass(frozen=True)
class PalletApproachConfig:
    """Distances and motion limits for a pallet approach."""

    standoff_distance_m: float = 1.80
    final_approach_distance_m: float = 1.00
    alignment_runup_distance_m: float = 0.60
    final_approach_speed_mps: float = 0.10
    final_approach_timeout_sec: float = 20.0
    max_start_position_error_m: float = 0.35
    max_start_heading_error_rad: float = 0.20
    forks_on_negative_x: bool = True
    arrow_points_outward: bool = True


@dataclass(frozen=True)
class PalletApproachGeometry:
    """Computed map poses used to align and approach a pallet."""

    alignment: PoseTarget
    pre_approach: PoseTarget
    stop: PoseTarget
    final_motion: RelativeMoveTarget


def _require_positive(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise PalletApproachError('{} must be a positive finite value'.format(name))
    return result


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def pallet_exemption_active_for_distance(
    *,
    currently_active: bool,
    distance_m: float,
    activation_distance_m: float,
    deactivation_distance_m: float,
) -> bool:
    """Apply distance hysteresis to the selected-pallet exemption."""

    if currently_active:
        return float(distance_m) < float(deactivation_distance_m)
    return float(distance_m) <= float(activation_distance_m)


def build_pallet_approach(
    *,
    pallet_x: float,
    pallet_y: float,
    pallet_yaw: float,
    frame_id: str,
    config: PalletApproachConfig,
) -> PalletApproachGeometry:
    """Build two A-star alignment targets followed by a straight final motion.

    The RViz arrow points from the pallet toward the free aisle by default. For
    a rear-fork vehicle, the vehicle first drives outward to align ``+X`` with
    the arrow and then reverses toward the pallet.
    """

    x = float(pallet_x)
    y = float(pallet_y)
    yaw = float(pallet_yaw)
    if not all(math.isfinite(value) for value in (x, y, yaw)):
        raise PalletApproachError('pallet pose must contain finite values')
    if not frame_id:
        raise PalletApproachError('pallet frame_id must not be empty')

    standoff = _require_positive(
        config.standoff_distance_m, 'standoff_distance_m'
    )
    approach_distance = _require_positive(
        config.final_approach_distance_m, 'final_approach_distance_m'
    )
    runup = _require_positive(
        config.alignment_runup_distance_m, 'alignment_runup_distance_m'
    )
    speed = _require_positive(
        config.final_approach_speed_mps, 'final_approach_speed_mps'
    )
    timeout = _require_positive(
        config.final_approach_timeout_sec, 'final_approach_timeout_sec'
    )
    position_error = _require_positive(
        config.max_start_position_error_m, 'max_start_position_error_m'
    )
    heading_error = _require_positive(
        config.max_start_heading_error_rad, 'max_start_heading_error_rad'
    )
    if runup >= approach_distance:
        raise PalletApproachError(
            'alignment_runup_distance_m must be smaller than '
            'final_approach_distance_m'
        )

    outward_yaw = yaw if config.arrow_points_outward else yaw + math.pi
    outward_yaw = _normalize_angle(outward_yaw)
    outward_x = math.cos(outward_yaw)
    outward_y = math.sin(outward_yaw)

    stop_distance = standoff
    pre_distance = standoff + approach_distance
    if config.forks_on_negative_x:
        vehicle_yaw = outward_yaw
        final_distance = -approach_distance
        alignment_distance = pre_distance - runup
    else:
        vehicle_yaw = _normalize_angle(outward_yaw + math.pi)
        final_distance = approach_distance
        alignment_distance = pre_distance + runup

    def target(name: str, distance: float) -> PoseTarget:
        return PoseTarget(
            name=name,
            x=x + distance * outward_x,
            y=y + distance * outward_y,
            yaw=vehicle_yaw,
            frame_id=frame_id,
        )

    alignment = target('pallet_alignment', alignment_distance)
    pre_approach = target('pallet_pre_approach', pre_distance)
    stop = target('pallet_stop', stop_distance)
    return PalletApproachGeometry(
        alignment=alignment,
        pre_approach=pre_approach,
        stop=stop,
        final_motion=RelativeMoveTarget(
            name='pallet_final_approach',
            distance_m=final_distance,
            max_speed_mps=speed,
            timeout_sec=timeout,
            expected_start_x=pre_approach.x,
            expected_start_y=pre_approach.y,
            expected_start_yaw=pre_approach.yaw,
            frame_id=pre_approach.frame_id,
            max_start_position_error_m=position_error,
            max_start_heading_error_rad=heading_error,
            pallet_exemption_x=x,
            pallet_exemption_y=y,
            pallet_exemption_yaw=outward_yaw,
        ),
    )


def build_pallet_approach_route(
    geometry: PalletApproachGeometry,
) -> RouteDefinition:
    """Convert computed geometry into an executable mixed-motion route."""

    return RouteDefinition(
        name='rviz_pallet_approach',
        loop=False,
        targets=(
            geometry.alignment,
            geometry.pre_approach,
            geometry.final_motion,
        ),
    )


def geometry_pose_targets(
    geometry: PalletApproachGeometry,
) -> Tuple[PoseTarget, PoseTarget, PoseTarget]:
    """Return visualization poses in alignment, pre-approach, stop order."""

    return geometry.alignment, geometry.pre_approach, geometry.stop
