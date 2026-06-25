"""Pure configuration and route geometry helpers for the task manager."""

import math
import warnings
from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import yaml


class RouteConfigError(ValueError):
    """Raised when station or route configuration is invalid."""


@dataclass(frozen=True)
class PoseTarget:
    """One sparse NavigateToPose target."""

    name: str
    x: float
    y: float
    yaw: float
    frame_id: str = 'map'


@dataclass(frozen=True)
class RouteDefinition:
    """Validated route data ready for sequential execution."""

    name: str
    loop: bool
    targets: Tuple[PoseTarget, ...]


def _finite_float(value: Any, field: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise RouteConfigError('{} must be a number'.format(field)) from exc
    if not math.isfinite(result):
        raise RouteConfigError('{} must be finite'.format(field))
    return result


def derive_segment_poses(
    waypoints: Sequence[Sequence[Any]],
    *,
    eps: float = 1e-3,
) -> List[Tuple[float, float, float]]:
    """Derive sparse target yaws without interpolating between waypoints.

    Missing yaws point toward the next waypoint. The final missing yaw follows
    the direction of the final segment. Adjacent points closer than ``eps`` are
    removed with a warning.
    """

    eps_value = _finite_float(eps, 'eps')
    if eps_value <= 0.0:
        raise RouteConfigError('eps must be greater than zero')
    if isinstance(waypoints, (str, bytes)) or not isinstance(waypoints, Sequence):
        raise RouteConfigError('waypoints must be a sequence')
    if not waypoints:
        raise RouteConfigError('route must contain at least one waypoint')

    parsed: List[Tuple[float, float, Optional[float]]] = []
    for index, waypoint in enumerate(waypoints):
        if (
            isinstance(waypoint, (str, bytes))
            or not isinstance(waypoint, Sequence)
            or len(waypoint) not in (2, 3)
        ):
            raise RouteConfigError(
                'waypoint {} must contain [x, y] or [x, y, yaw]'.format(index)
            )
        x = _finite_float(waypoint[0], 'waypoint {} x'.format(index))
        y = _finite_float(waypoint[1], 'waypoint {} y'.format(index))
        yaw = (
            _finite_float(waypoint[2], 'waypoint {} yaw'.format(index))
            if len(waypoint) == 3 else None
        )

        if parsed and math.hypot(x - parsed[-1][0], y - parsed[-1][1]) < eps_value:
            warnings.warn(
                'dropping duplicate waypoint {} at ({}, {})'.format(index, x, y),
                UserWarning,
            )
            continue
        parsed.append((x, y, yaw))

    if not parsed:
        raise RouteConfigError('route has no waypoint after duplicate filtering')
    if len(parsed) == 1:
        x, y, yaw = parsed[0]
        if yaw is None:
            raise RouteConfigError('single-point route must provide yaw')
        return [(x, y, yaw)]

    poses: List[Tuple[float, float, float]] = []
    for index, (x, y, explicit_yaw) in enumerate(parsed):
        if explicit_yaw is not None:
            yaw = explicit_yaw
        elif index < len(parsed) - 1:
            next_x, next_y, _ = parsed[index + 1]
            yaw = math.atan2(next_y - y, next_x - x)
        else:
            prev_x, prev_y, _ = parsed[index - 1]
            yaw = math.atan2(y - prev_y, x - prev_x)
        poses.append((x, y, yaw))
    return poses


def _load_yaml_mapping(path: str, kind: str) -> Mapping[str, Any]:
    try:
        with open(path, 'r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise RouteConfigError('failed to load {} {}: {}'.format(kind, path, exc)) from exc
    if data is None:
        return {}
    if not isinstance(data, Mapping):
        raise RouteConfigError('{} config must be a mapping'.format(kind))
    nested = data.get(kind)
    if nested is not None:
        if not isinstance(nested, Mapping):
            raise RouteConfigError('{} must be a mapping'.format(kind))
        return nested
    return data


def load_stations(path: str) -> Dict[str, PoseTarget]:
    """Load and validate named station poses."""

    raw_stations = _load_yaml_mapping(path, 'stations')
    stations: Dict[str, PoseTarget] = {}
    for name, raw_station in raw_stations.items():
        if not isinstance(name, str) or not name:
            raise RouteConfigError('station names must be non-empty strings')
        if not isinstance(raw_station, Mapping):
            raise RouteConfigError('station {} must be a mapping'.format(name))
        for field in ('x', 'y', 'yaw'):
            if field not in raw_station:
                raise RouteConfigError('station {} is missing {}'.format(name, field))
        frame_id = raw_station.get('frame_id', 'map')
        if not isinstance(frame_id, str) or not frame_id:
            raise RouteConfigError('station {} frame_id must be a string'.format(name))
        stations[name] = PoseTarget(
            name=name,
            x=_finite_float(raw_station['x'], 'station {} x'.format(name)),
            y=_finite_float(raw_station['y'], 'station {} y'.format(name)),
            yaw=_finite_float(raw_station['yaw'], 'station {} yaw'.format(name)),
            frame_id=frame_id,
        )
    return stations


def load_routes(
    path: str,
    stations: Mapping[str, PoseTarget],
) -> Dict[str, RouteDefinition]:
    """Load routes, resolving named segments or sparse waypoints."""

    raw_routes = _load_yaml_mapping(path, 'routes')
    routes: Dict[str, RouteDefinition] = {}
    for route_name, raw_route in raw_routes.items():
        if not isinstance(route_name, str) or not route_name:
            raise RouteConfigError('route names must be non-empty strings')
        if not isinstance(raw_route, Mapping):
            raise RouteConfigError('route {} must be a mapping'.format(route_name))

        has_segments = 'segments' in raw_route
        has_waypoints = 'waypoints' in raw_route
        if has_segments == has_waypoints:
            raise RouteConfigError(
                'route {} must define exactly one of segments or waypoints'.format(
                    route_name
                )
            )
        loop = raw_route.get('loop', False)
        if not isinstance(loop, bool):
            raise RouteConfigError('route {} loop must be boolean'.format(route_name))

        targets: List[PoseTarget] = []
        if has_waypoints:
            poses = derive_segment_poses(raw_route['waypoints'])
            frame_id = raw_route.get('frame_id', 'map')
            if not isinstance(frame_id, str) or not frame_id:
                raise RouteConfigError(
                    'route {} frame_id must be a string'.format(route_name)
                )
            targets = [
                PoseTarget(
                    name='waypoint_{}'.format(index),
                    x=x,
                    y=y,
                    yaw=yaw,
                    frame_id=frame_id,
                )
                for index, (x, y, yaw) in enumerate(poses)
            ]
        else:
            raw_segments = raw_route['segments']
            if (
                isinstance(raw_segments, (str, bytes))
                or not isinstance(raw_segments, Sequence)
                or not raw_segments
            ):
                raise RouteConfigError(
                    'route {} segments must be a non-empty sequence'.format(route_name)
                )
            for index, segment in enumerate(raw_segments):
                if not isinstance(segment, Mapping):
                    raise RouteConfigError(
                        'route {} segment {} must be a mapping'.format(route_name, index)
                    )
                station_name = segment.get('to')
                if station_name not in stations:
                    raise RouteConfigError(
                        'route {} segment {} references unknown station {}'.format(
                            route_name, index, station_name
                        )
                    )
                station = stations[station_name]
                segment_type = segment.get('type', 'drive')
                if not isinstance(segment_type, str) or not segment_type:
                    raise RouteConfigError(
                        'route {} segment {} type must be a string'.format(
                            route_name, index
                        )
                    )
                yaw = (
                    _finite_float(
                        segment['yaw'],
                        'route {} segment {} yaw'.format(route_name, index),
                    )
                    if 'yaw' in segment else station.yaw
                )
                targets.append(
                    PoseTarget(
                        name='{}:{}'.format(segment_type, station_name),
                        x=station.x,
                        y=station.y,
                        yaw=yaw,
                        frame_id=station.frame_id,
                    )
                )

        routes[route_name] = RouteDefinition(
            name=route_name,
            loop=loop,
            targets=tuple(targets),
        )
    return routes
