from __future__ import annotations

import ast
import math
import copy
import threading
import time
from dataclasses import dataclass
from typing import Any, List, Optional, Sequence, Tuple

import rclpy
from forklift_msgs.msg import ForkliftControlCommand, ForkliftFaultState, ForkliftVehicleState
from forklift_msgs.srv import SetEmergencyStop
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

try:
    from nav2_msgs.msg import Costmap as Nav2Costmap
except ImportError:
    Nav2Costmap = None

Point2D = Tuple[float, float]
Pose2D = Tuple[float, float, float]


class ObstacleReleaseState:
    """Require a fresh, sustained clear sweep before resuming a blocked move."""

    def __init__(self, clear_sec=0.5, steering_change_rad=0.15):
        self.clear_sec = clear_sec
        self.steering_change_rad = steering_change_rad
        self.active = False
        self.speed_floor = 0.0
        self.signature = None
        self.interrupt()

    def interrupt(self):
        self.clear_since = None
        self.first_scan = None
        self.first_costmap = None
        self.ready = False

    def commit_release(self):
        if self.ready:
            self.active = False
            self.speed_floor = 0.0
            self.interrupt()

    def prepare(self, travel_direction, steering):
        signature = (travel_direction, steering)
        if self.signature is None or travel_direction != self.signature[0] or abs(
                steering - self.signature[1]) > self.steering_change_rad:
            # A genuinely different maneuver must be checked in its own sweep,
            # not forced to clear the old forward corridor before reversing.
            self.signature = signature
            self.speed_floor = 0.0
            self.interrupt()

    def blocked(self, speed):
        self.active = True
        self.speed_floor = max(self.speed_floor, speed)
        self.interrupt()

    def clear(self, now, scan_token, costmap_token):
        if not self.active:
            return True
        if self.clear_since is None or now < self.clear_since:
            self.clear_since = now
            self.first_scan = scan_token
            self.first_costmap = costmap_token
        fresh_scan = scan_token is None or scan_token != self.first_scan
        fresh_costmap = costmap_token is None or costmap_token != self.first_costmap
        if now - self.clear_since < self.clear_sec or not fresh_scan or not fresh_costmap:
            return False
        self.ready = True
        return True


@dataclass(frozen=True)
class PalletExemptionZone:
    """Small oriented rectangle containing the selected pallet face."""

    x: float
    y: float
    yaw: float
    half_length: float
    half_width: float


def positive(value: float, fallback: float) -> float:
    return value if value > 0.0 else fallback


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def spatial_sweep_time_step(
    speed_mps: float,
    sample_spacing_m: float,
    horizon_sec: float,
) -> float:
    """Choose a prediction step by traveled distance, not a fixed timer rate."""
    speed = abs(float(speed_mps))
    horizon = positive(float(horizon_sec), 0.1)
    if speed <= 1e-6:
        return horizon
    return max(0.01, min(horizon, positive(sample_spacing_m, 0.05) / speed))


def dynamic_stopping_distance(
    speed_mps: float,
    reaction_time_sec: float,
    brake_deceleration_mps2: float,
    clearance_m: float,
) -> float:
    """Return the required base-link travel distance before an obstacle."""

    speed = max(0.0, float(speed_mps))
    reaction = max(0.0, float(reaction_time_sec))
    deceleration = positive(float(brake_deceleration_mps2), 1.0)
    clearance = max(0.0, float(clearance_m))
    return speed * reaction + speed * speed / (2.0 * deceleration) + clearance


def source_age_sec(stamp, now_sec: float) -> float:
    """Missing/future source timestamps must not refresh an old command."""
    source = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    age = now_sec - source
    if source <= 0.0 or not math.isfinite(age) or age < -0.1:
        return math.inf
    return max(0.0, age)


def pivot_braking_poses(
    footprint, commanded_rate, measured_rate, reaction_sec,
    deceleration_radps2, margin_rad, axle_offset, spacing_m,
):
    """Sweep both residual and requested rotation, sampled at the outer corner."""
    if not all(math.isfinite(v) for v in (commanded_rate, measured_rate)):
        raise ValueError('pivot angular velocity unavailable')
    radius = max(math.hypot(x - axle_offset, y) for x, y in footprint)
    angle_step = min(0.03, positive(spacing_m, 0.05) / max(radius, 0.01))
    poses = [(0.0, 0.0, 0.0)]
    for sign in (-1.0, 1.0):
        rate = max(0.0, sign * commanded_rate, sign * measured_rate)
        if rate <= 1e-6:
            continue
        angle = min(2.0 * math.pi, dynamic_stopping_distance(
            rate, reaction_sec, deceleration_radps2, margin_rad))
        count = max(1, int(math.ceil(angle / angle_step)))
        for i in range(1, count + 1):
            yaw = sign * angle * i / count
            poses.append((axle_offset * (1.0 - math.cos(yaw)),
                          -axle_offset * math.sin(yaw), yaw))
    return poses


def scan_stop_reason(
    enabled: bool,
    age_sec: float,
    has_scan: bool,
    range_max_m: float,
    timeout_sec: float,
    required_range_m: float,
) -> str:
    if not enabled:
        return ''
    if not has_scan:
        return 'scan missing'
    if age_sec > timeout_sec:
        return 'scan timeout'
    if not math.isfinite(range_max_m) or range_max_m < required_range_m:
        return (
            f'scan range_max {range_max_m:.2f} m below required '
            f'{required_range_m:.2f} m'
        )
    return ''


def direction(command: ForkliftControlCommand) -> int:
    if command.forward and not command.reverse:
        return 1
    if command.reverse and not command.forward:
        return -1
    return 0


def is_steering_only_command(command: ForkliftControlCommand) -> bool:
    """Return whether a command can only actuate steering at zero traction.

    Fine motion must center the steering after a pivot before it may start a
    straight move.  The vehicle controller requires an enabled command for
    this, but there must be no travel direction, drive RPM, or hydraulic
    output.  Keeping this predicate deliberately narrow prevents it from
    becoming a general bypass for invalid motion commands.
    """

    zero_outputs = (
        command.velocity_mps,
        command.drive_rpm,
        command.pump_rpm,
        command.lift_valve_ma,
        command.lower_valve_ma,
        command.side_shift_left_valve_ma,
        command.side_shift_right_valve_ma,
        command.tilt_forward_valve_ma,
        command.tilt_backward_valve_ma,
    )
    return (
        command.enable
        and not command.brake
        and direction(command) == 0
        and not command.horn
        and all(math.isfinite(value) and abs(value) <= 1e-6 for value in zero_outputs)
    )


def stop_command(stamp=None) -> ForkliftControlCommand:
    command = ForkliftControlCommand()
    if stamp is not None:
        command.header.stamp = stamp
    command.enable = False
    command.brake = True
    command.forward = False
    command.reverse = False
    command.velocity_mps = 0.0
    command.drive_rpm = 0.0
    command.steering_angle_rad = 0.0
    command.steering_angle_deg = 0.0
    return command


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def parse_footprint(value) -> List[Point2D]:
    if isinstance(value, str):
        value = ast.literal_eval(value)
    if not isinstance(value, Sequence) or len(value) < 3:
        raise ValueError('footprint must contain at least three [x, y] points')

    footprint: List[Point2D] = []
    for point in value:
        if not isinstance(point, Sequence) or len(point) != 2:
            raise ValueError('footprint points must be [x, y] pairs')
        footprint.append((float(point[0]), float(point[1])))
    return footprint


def costmap_metadata(costmap: Any) -> Tuple[int, int, float, Any]:
    if hasattr(costmap, 'info'):
        return (
            int(costmap.info.width),
            int(costmap.info.height),
            float(costmap.info.resolution),
            costmap.info.origin,
        )
    metadata = costmap.metadata
    return (
        int(metadata.size_x),
        int(metadata.size_y),
        float(metadata.resolution),
        metadata.origin,
    )


def costmap_error(costmap: Any) -> str:
    if isinstance(costmap, PreparedCostmap):
        return costmap.error
    width, height, resolution, origin = costmap_metadata(costmap)
    if width <= 0 or height <= 0:
        return 'empty dimensions'
    if not math.isfinite(resolution) or resolution <= 0.0:
        return 'invalid resolution'
    if not all(math.isfinite(v) for v in (
            origin.position.x, origin.position.y, origin.orientation.x,
            origin.orientation.y, origin.orientation.z, origin.orientation.w)):
        return 'invalid origin'
    expected_cells = width * height
    if len(costmap.data) < expected_cells:
        return f'truncated data {len(costmap.data)}/{expected_cells}'
    return ''


class PreparedCostmap:
    """Read-only lookup context for one pinned costmap message, never a copy."""

    def __init__(self, message):
        self.error = costmap_error(message)
        self.width, self.height, self.resolution, origin = costmap_metadata(message)
        self.origin_x, self.origin_y = origin.position.x, origin.position.y
        self.yaw = yaw_from_quaternion(origin.orientation) if not self.error else 0.0
        self.cos_yaw, self.sin_yaw = math.cos(self.yaw), math.sin(self.yaw)
        self.data = message.data
        self.message = message

    def cell(self, x, y):
        dx, dy = x - self.origin_x, y - self.origin_y
        mx = math.floor((dx * self.cos_yaw + dy * self.sin_yaw) / self.resolution)
        my = math.floor((-dx * self.sin_yaw + dy * self.cos_yaw) / self.resolution)
        if mx < 0 or my < 0 or mx >= self.width or my >= self.height:
            return None
        return mx, my


def obstacle_stop_reason(reason):
    # Coverage/TF/compute failures stop motion but are not obstacle evidence.
    return reason.startswith((
        'footprint collision: cost ', 'footprint collision: unknown',
        'footprint reverse escape', 'scan footprint sweep collision',
        'scan reverse escape'))


def costmap_stop_reason(
    monitor_enabled: bool,
    age_sec: float,
    has_costmap: bool,
    error: str,
    timeout_sec: float,
) -> str:
    if not monitor_enabled:
        return ''
    if error:
        return f'costmap invalid: {error}'
    if not has_costmap:
        if age_sec > timeout_sec:
            return 'costmap missing'
        return ''
    if age_sec > timeout_sec:
        return 'costmap timeout'
    return ''


def transform_point(point: Point2D, pose: Pose2D) -> Point2D:
    cos_yaw = math.cos(pose[2])
    sin_yaw = math.sin(pose[2])
    return (
        pose[0] + point[0] * cos_yaw - point[1] * sin_yaw,
        pose[1] + point[0] * sin_yaw + point[1] * cos_yaw,
    )


def point_in_pallet_exemption(
    point: Point2D,
    zone: Optional[PalletExemptionZone],
) -> bool:
    """Return whether a world point is inside the active pallet rectangle."""

    if zone is None:
        return False
    dx = point[0] - zone.x
    dy = point[1] - zone.y
    local_x = dx * math.cos(zone.yaw) + dy * math.sin(zone.yaw)
    local_y = -dx * math.sin(zone.yaw) + dy * math.cos(zone.yaw)
    return (
        abs(local_x) <= zone.half_length
        and abs(local_y) <= zone.half_width
    )


def world_to_map(costmap: Any, x: float, y: float) -> Optional[Tuple[int, int]]:
    if isinstance(costmap, PreparedCostmap):
        return costmap.cell(x, y)
    width, height, resolution, origin = costmap_metadata(costmap)
    yaw = yaw_from_quaternion(origin.orientation)
    dx = x - origin.position.x
    dy = y - origin.position.y
    map_x = (dx * math.cos(yaw) + dy * math.sin(yaw)) / resolution
    map_y = (-dx * math.sin(yaw) + dy * math.cos(yaw)) / resolution
    mx = int(math.floor(map_x))
    my = int(math.floor(map_y))
    if mx < 0 or my < 0 or mx >= width or my >= height:
        return None
    return mx, my


def cost_at_world(costmap: Any, x: float, y: float) -> Optional[int]:
    cell = world_to_map(costmap, x, y)
    if cell is None:
        return None
    mx, my = cell
    width = (costmap.width if isinstance(costmap, PreparedCostmap)
             else costmap_metadata(costmap)[0])
    return int(costmap.data[my * width + mx])


def sampled_segment_points(start: Point2D, end: Point2D, spacing: float) -> List[Point2D]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    steps = max(1, int(math.ceil(length / positive(spacing, 0.05))))
    return [
        (start[0] + dx * i / steps, start[1] + dy * i / steps)
        for i in range(steps + 1)
    ]


def point_to_segment_distance(point: Point2D, start: Point2D, end: Point2D) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-12:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    ratio = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared
    ratio = clamp(ratio, 0.0, 1.0)
    nearest = (start[0] + ratio * dx, start[1] + ratio * dy)
    return math.hypot(point[0] - nearest[0], point[1] - nearest[1])


def point_in_polygon_with_padding(
    point: Point2D,
    polygon: Sequence[Point2D],
    padding_m: float,
) -> bool:
    """Return whether a point is inside an oriented footprint or touches it."""

    inside = False
    for index, start in enumerate(polygon):
        end = polygon[(index + 1) % len(polygon)]
        if point_to_segment_distance(point, start, end) <= padding_m:
            return True
        crosses = (start[1] > point[1]) != (end[1] > point[1])
        if crosses:
            intersection_x = (
                (end[0] - start[0]) * (point[1] - start[1]) /
                (end[1] - start[1]) + start[0]
            )
            if point[0] < intersection_x:
                inside = not inside
    return inside


def laser_scan_points_in_base(
    scan: LaserScan,
    scan_to_base: Pose2D,
) -> List[Point2D]:
    """Convert finite LaserScan ranges into base-frame points."""

    points: List[Point2D] = []
    angle = float(scan.angle_min)
    for distance in scan.ranges:
        range_m = float(distance)
        if math.isfinite(range_m) and scan.range_min <= range_m <= scan.range_max:
            scan_point = (range_m * math.cos(angle), range_m * math.sin(angle))
            points.append(transform_point(scan_point, scan_to_base))
        angle += float(scan.angle_increment)
    return points


def angle_in_scan_fov(
    base_angle: float,
    scan: LaserScan,
    scan_to_base_yaw: float,
) -> bool:
    """Check that the motion centreline lies inside the reported scan sector."""

    scan_angle = math.atan2(
        math.sin(base_angle - scan_to_base_yaw),
        math.cos(base_angle - scan_to_base_yaw),
    )
    lower = float(scan.angle_min) - 1e-6
    upper = float(scan.angle_max) + 1e-6
    return any(lower <= candidate <= upper for candidate in (
        scan_angle,
        scan_angle - 2.0 * math.pi,
        scan_angle + 2.0 * math.pi,
    ))


def scan_sweep_collision(
    scan_points: Sequence[Point2D],
    footprint: Sequence[Point2D],
    command: ForkliftControlCommand,
    wheel_base: float,
    pivot_turn_radius: float,
    rear_axle_x_offset: float,
    stopping_distance_m: float,
    sample_spacing_m: float,
    pivot_steering_angle_rad: float,
    collision_padding_m: float,
    pallet_exemption: Optional[PalletExemptionZone] = None,
    allow_reverse_escape: bool = False,
    reverse_escape_max_speed_mps: float = 0.15,
    reverse_escape_max_steering_angle_rad: float = 0.05,
    reverse_escape_obstacle_min_x_m: float = 0.0,
    prediction_poses=None,
) -> Tuple[bool, str]:
    travel_direction = direction(command)
    speed = abs(float(command.velocity_mps))
    if travel_direction == 0 or speed <= 1e-6:
        return False, 'scan sweep clear'

    horizon_sec = stopping_distance_m / speed
    time_step = spatial_sweep_time_step(
        speed,
        sample_spacing_m,
        horizon_sec,
    )
    poses = prediction_poses if prediction_poses is not None else predicted_poses_for_command(
        (0.0, 0.0, 0.0),
        command,
        wheel_base,
        pivot_turn_radius,
        rear_axle_x_offset,
        horizon_sec,
        time_step,
        pivot_steering_angle_rad,
    )

    # Points beyond every swept footprint cannot collide. Keep the exact
    # polygon/padding test for all remaining points.
    radius = max(math.hypot(x, y) for x, y in footprint) + collision_padding_m
    bound = radius + max(math.hypot(x, y) for x, y, _ in poses)
    scan_points = [p for p in scan_points if p[0] ** 2 + p[1] ** 2 <= bound ** 2]

    def colliding_indices(pose: Pose2D):
        world_footprint = [transform_point(point, pose) for point in footprint]
        collisions = set()
        for index, point in enumerate(scan_points):
            if point_in_pallet_exemption(point, pallet_exemption):
                continue
            if point_in_polygon_with_padding(point, world_footprint, collision_padding_m):
                collisions.add(index)
        return collisions

    initial_collisions = colliding_indices(poses[0])
    escape_candidate = (
        allow_reverse_escape
        and travel_direction < 0
        and speed <= reverse_escape_max_speed_mps + 1e-9
        and abs(float(command.steering_angle_rad))
        <= reverse_escape_max_steering_angle_rad + 1e-9
        and bool(initial_collisions)
        and all(
            scan_points[index][0] >= reverse_escape_obstacle_min_x_m
            for index in initial_collisions
        )
    )
    if escape_candidate:
        previous_count = len(initial_collisions)
        for pose in poses[1:]:
            collisions = colliding_indices(pose)
            if not collisions.issubset(initial_collisions):
                return True, 'scan reverse escape would hit a new obstacle'
            if len(collisions) > previous_count:
                return True, 'scan reverse escape overlap is increasing'
            previous_count = len(collisions)
        if previous_count == 0:
            return False, 'scan reverse escape clear'
        return True, 'scan reverse escape does not clear current overlap'

    if initial_collisions:
        return True, 'scan footprint sweep collision'
    for pose in poses[1:]:
        if colliding_indices(pose):
            return True, 'scan footprint sweep collision'
    return False, 'scan sweep clear'


def footprint_collision_at_pose(
    costmap: Any,
    footprint: Sequence[Point2D],
    pose: Pose2D,
    sample_spacing: float,
    cost_threshold: int,
    unknown_is_collision: bool,
    pallet_exemption: Optional[PalletExemptionZone] = None,
) -> Tuple[bool, str]:
    error = costmap_error(costmap)
    if error:
        return True, f'costmap invalid: {error}'

    if not isinstance(costmap, PreparedCostmap):
        costmap = PreparedCostmap(costmap)
    cos_yaw, sin_yaw = math.cos(pose[2]), math.sin(pose[2])
    world_points = [(pose[0] + x * cos_yaw - y * sin_yaw,
                     pose[1] + x * sin_yaw + y * cos_yaw) for x, y in footprint]
    max_cost = 0
    for index, start in enumerate(world_points):
        end = world_points[(index + 1) % len(world_points)]
        for x, y in sampled_segment_points(start, end, sample_spacing):
            cost = cost_at_world(costmap, x, y)
            if cost is None:
                return True, (
                    'costmap coverage insufficient: point=({:.3f},{:.3f}) '
                    'sweep_pose=({:.3f},{:.3f},{:.3f})'.format(x, y, *pose))
            if cost < 0:
                if unknown_is_collision:
                    return True, 'footprint collision: unknown costmap cell'
                continue
            max_cost = max(max_cost, cost)
            if cost >= cost_threshold:
                if point_in_pallet_exemption((x, y), pallet_exemption):
                    continue
                return True, f'footprint collision: cost {cost} >= {cost_threshold}'
    return False, f'footprint clear: max cost {max_cost}'


def predicted_poses_for_command(
    initial_pose: Pose2D,
    command: ForkliftControlCommand,
    wheel_base: float,
    pivot_turn_radius: float,
    rear_axle_x_offset: float,
    horizon_sec: float,
    time_step_sec: float,
    pivot_steering_angle_rad: float,
) -> List[Pose2D]:
    poses = [initial_pose]
    travel_direction = direction(command)
    if travel_direction == 0 or command.brake or not command.enable:
        return poses

    signed_velocity = travel_direction * abs(float(command.velocity_mps))
    if abs(signed_velocity) <= 1e-6:
        return poses

    step = positive(time_step_sec, 0.1)
    steps = max(1, int(math.ceil(positive(horizon_sec, step) / step)))
    x, y, yaw = initial_pose
    steering = float(command.steering_angle_rad)
    for _ in range(steps):
        if abs(steering) >= abs(pivot_steering_angle_rad) - 1e-3:
            yaw_rate = math.copysign(
                abs(signed_velocity) / positive(pivot_turn_radius, 0.6),
                steering,
            )
            rear_x = x + rear_axle_x_offset * math.cos(yaw)
            rear_y = y + rear_axle_x_offset * math.sin(yaw)
            yaw += yaw_rate * step
            x = rear_x - rear_axle_x_offset * math.cos(yaw)
            y = rear_y - rear_axle_x_offset * math.sin(yaw)
        else:
            yaw_rate = signed_velocity * math.tan(steering) / positive(wheel_base, 1.4)
            x += signed_velocity * math.cos(yaw) * step
            y += signed_velocity * math.sin(yaw) * step
            yaw += yaw_rate * step
        poses.append((x, y, yaw))
    return poses


def footprint_sweep_collision(
    costmap: Any,
    footprint: Sequence[Point2D],
    initial_pose: Pose2D,
    command: ForkliftControlCommand,
    wheel_base: float,
    pivot_turn_radius: float,
    rear_axle_x_offset: float,
    horizon_sec: float,
    time_step_sec: float,
    pivot_steering_angle_rad: float,
    sample_spacing: float,
    cost_threshold: int,
    unknown_is_collision: bool,
    pallet_exemption: Optional[PalletExemptionZone] = None,
    allow_initial_collision_escape: bool = False,
    prediction_poses=None,
) -> Tuple[bool, str]:
    if not isinstance(costmap, PreparedCostmap):
        costmap = PreparedCostmap(costmap)
    poses = prediction_poses if prediction_poses is not None else predicted_poses_for_command(
        initial_pose,
        command,
        wheel_base,
        pivot_turn_radius,
        rear_axle_x_offset,
        horizon_sec,
        time_step_sec,
        pivot_steering_angle_rad,
    )
    initial_collision, initial_reason = footprint_collision_at_pose(
        costmap,
        footprint,
        poses[0],
        sample_spacing,
        cost_threshold,
        unknown_is_collision,
        pallet_exemption,
    )
    if initial_collision and not allow_initial_collision_escape:
        return True, initial_reason
    if initial_collision and not initial_reason.startswith('footprint collision: cost '):
        return True, initial_reason

    cleared_initial_overlap = not initial_collision
    for pose in poses[1:]:
        collision, reason = footprint_collision_at_pose(
            costmap,
            footprint,
            pose,
            sample_spacing,
            cost_threshold,
            unknown_is_collision,
            pallet_exemption,
        )
        if collision:
            if (allow_initial_collision_escape and not cleared_initial_overlap
                    and reason.startswith('footprint collision: cost ')):
                continue
            return True, reason
        cleared_initial_overlap = True
    if initial_collision and not cleared_initial_overlap:
        return True, 'footprint reverse escape does not clear current overlap'
    if initial_collision:
        return False, 'footprint reverse escape clear'
    return False, 'footprint sweep clear'


def apply_drive_envelope(
    command: ForkliftControlCommand,
    max_drive_rpm: float,
    accel_time_sec: float,
    decel_time_sec: float,
) -> ForkliftControlCommand:
    """Enforce the manufacturer drive envelope on an outgoing motion command.

    The real Curtis 0x203 frame drives the motor from ``drive_rpm`` (0..4000),
    not ``velocity_mps``, so the gate must cap ``drive_rpm`` itself or its speed
    limit is bypassed on the vehicle. Direction is carried by the forward/reverse
    bits, so only the magnitude is clamped. ``accel_time_sec`` / ``decel_time_sec``
    are filled with the manufacturer-recommended ramp when the upstream command
    leaves them unset (<= 0), and otherwise left as the explicit upstream value.
    """
    command.drive_rpm = clamp(abs(command.drive_rpm), 0.0, max_drive_rpm)
    if command.accel_time_sec <= 0.0:
        command.accel_time_sec = accel_time_sec
    if command.decel_time_sec <= 0.0:
        command.decel_time_sec = decel_time_sec
    return command


def clamp_control_command(
    command: ForkliftControlCommand,
    max_forward_velocity_mps: float,
    max_reverse_velocity_mps: float,
    max_steering_angle_rad: float,
    max_drive_rpm: float,
    accel_time_sec: float,
    decel_time_sec: float,
) -> ForkliftControlCommand:
    gated = ForkliftControlCommand()
    gated.header = copy.deepcopy(command.header)
    gated.enable = command.enable
    gated.brake = command.brake
    gated.forward = command.forward
    gated.reverse = command.reverse
    gated.accel_time_sec = command.accel_time_sec
    gated.decel_time_sec = command.decel_time_sec
    gated.pump_rpm = command.pump_rpm
    gated.lift_valve_ma = command.lift_valve_ma
    gated.lower_valve_ma = command.lower_valve_ma
    gated.side_shift_left_valve_ma = command.side_shift_left_valve_ma
    gated.side_shift_right_valve_ma = command.side_shift_right_valve_ma
    gated.tilt_forward_valve_ma = command.tilt_forward_valve_ma
    gated.tilt_backward_valve_ma = command.tilt_backward_valve_ma
    gated.horn = command.horn
    gated.light = command.light

    travel_direction = direction(command)
    speed_limit = max_forward_velocity_mps if travel_direction >= 0 else max_reverse_velocity_mps
    speed = clamp(abs(command.velocity_mps), 0.0, speed_limit)
    gated.velocity_mps = speed
    gated.steering_angle_rad = clamp(
        command.steering_angle_rad,
        -max_steering_angle_rad,
        max_steering_angle_rad,
    )
    gated.steering_angle_deg = math.degrees(gated.steering_angle_rad)
    gated.drive_rpm = command.drive_rpm
    apply_drive_envelope(gated, max_drive_rpm, accel_time_sec, decel_time_sec)
    return gated


def cap_control_command_speed(
    command: ForkliftControlCommand,
    speed_limit_mps: float,
) -> bool:
    """Scale velocity and motor RPM together when scan freshness is degraded."""

    speed_limit = max(0.0, float(speed_limit_mps))
    requested_speed = abs(float(command.velocity_mps))
    if requested_speed <= speed_limit + 1e-9:
        return False
    scale = speed_limit / requested_speed if requested_speed > 1e-9 else 0.0
    command.velocity_mps = speed_limit
    command.drive_rpm = abs(float(command.drive_rpm)) * scale
    return True


def recovery_command_from_twist(
    twist: Twist,
    max_recovery_velocity_mps: float,
    max_recovery_angular_velocity_radps: float,
    wheel_base: float,
    pivot_turn_radius: float,
    pivot_steering_angle_rad: float,
    allow_recovery_backoff: bool,
    allow_recovery_pivot: bool,
    stamp=None,
) -> Tuple[ForkliftControlCommand, str]:
    linear = clamp(
        twist.linear.x,
        -max_recovery_velocity_mps,
        max_recovery_velocity_mps,
    )
    angular = clamp(
        twist.angular.z,
        -max_recovery_angular_velocity_radps,
        max_recovery_angular_velocity_radps,
    )

    if abs(linear) <= 1e-6 and abs(angular) <= 1e-6:
        return stop_command(stamp), 'recovery wait'

    if abs(linear) > 1e-6:
        if linear < 0.0 and not allow_recovery_backoff:
            return stop_command(stamp), 'recovery backoff disabled'
        command = ForkliftControlCommand()
        if stamp is not None:
            command.header.stamp = stamp
        command.enable = True
        command.brake = False
        command.forward = linear > 0.0
        command.reverse = linear < 0.0
        command.velocity_mps = abs(linear)
        if abs(angular) > 1e-6:
            steering = math.atan2(angular * wheel_base, linear)
            command.steering_angle_rad = clamp(
                steering,
                -pivot_steering_angle_rad,
                pivot_steering_angle_rad,
            )
            command.steering_angle_deg = math.degrees(command.steering_angle_rad)
        return command, 'recovery backoff' if linear < 0.0 else 'recovery forward'

    if not allow_recovery_pivot:
        return stop_command(stamp), 'recovery pivot disabled'

    command = ForkliftControlCommand()
    if stamp is not None:
        command.header.stamp = stamp
    command.enable = True
    command.brake = False
    command.forward = True
    command.reverse = False
    command.velocity_mps = min(
        max_recovery_velocity_mps,
        abs(angular) * positive(pivot_turn_radius, 0.6),
    )
    command.steering_angle_rad = (
        pivot_steering_angle_rad if angular > 0.0 else -pivot_steering_angle_rad
    )
    command.steering_angle_deg = math.degrees(command.steering_angle_rad)
    return command, 'recovery pivot'


class SafetyCommandGate(Node):
    """Gate all motion commands before they reach the vehicle interface."""

    def __init__(self) -> None:
        super().__init__('safety_command_gate')

        self.declare_parameter('enabled', True)
        self.declare_parameter('raw_command_topic', '/forklift/control_cmd_raw')
        self.declare_parameter('gated_command_topic', '/forklift/control_cmd')
        self.declare_parameter('recovery_twist_topic', '/cmd_vel')
        self.declare_parameter('vehicle_state_topic', '/forklift/vehicle_state')
        self.declare_parameter('fault_state_topic', '/forklift/fault_state')
        self.declare_parameter('localization_topic', '/odom')
        self.declare_parameter('localization_message_type', 'odometry')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('costmap_topic', '/local_costmap/costmap_raw')
        self.declare_parameter('costmap_message_type', 'costmap_raw')
        self.declare_parameter('status_topic', '/forklift/safety_gate/status')
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('recovery_timeout_sec', 0.5)
        self.declare_parameter('vehicle_state_timeout_sec', 0.5)
        self.declare_parameter('fault_state_timeout_sec', 0.5)
        self.declare_parameter('localization_timeout_sec', 0.5)
        self.declare_parameter('costmap_timeout_sec', 0.5)
        self.declare_parameter('require_vehicle_state', False)
        self.declare_parameter('require_fault_state', False)
        self.declare_parameter('require_localization', False)
        self.declare_parameter('costmap_monitor_enabled', True)
        self.declare_parameter('collision_check_enabled', True)
        self.declare_parameter(
            'footprint',
            '[[1.709, 0.610], [1.709, -0.610], [-1.590, -0.610], [-1.590, 0.610]]',
        )
        self.declare_parameter('footprint_sample_spacing', 0.05)
        self.declare_parameter('footprint_collision_cost_threshold', 253)
        self.declare_parameter('unknown_is_collision', True)
        self.declare_parameter('collision_check_horizon_sec', 1.0)
        self.declare_parameter('collision_check_time_step_sec', 0.1)
        self.declare_parameter('dynamic_stop_reaction_time_sec', 0.9)
        self.declare_parameter('dynamic_stop_brake_deceleration_mps2', 1.5)
        self.declare_parameter('dynamic_stop_clearance_m', 0.5)
        self.declare_parameter('pivot_brake_deceleration_radps2', 0.15)
        self.declare_parameter('pivot_stop_margin_rad', 0.05)
        self.declare_parameter('collision_compute_budget_sec', 0.15)
        self.declare_parameter('scan_protection_enabled', True)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('scan_timeout_sec', 0.7)
        self.declare_parameter('scan_high_speed_freshness_timeout_sec', 0.25)
        self.declare_parameter('scan_degraded_max_speed_mps', 1.0)
        self.declare_parameter('scan_fresh_recovery_duration_sec', 1.0)
        self.declare_parameter('obstacle_release_clear_sec', 0.5)
        self.declare_parameter('obstacle_release_steering_change_rad', 0.15)
        self.declare_parameter('scan_required_range_m', 8.0)
        self.declare_parameter('scan_collision_sample_spacing_m', 0.05)
        self.declare_parameter('scan_collision_padding_m', 0.05)
        self.declare_parameter('scan_require_motion_fov_coverage', True)
        self.declare_parameter('allow_reverse_collision_escape', True)
        self.declare_parameter('reverse_collision_escape_max_speed_mps', 0.15)
        self.declare_parameter(
            'reverse_collision_escape_max_steering_angle_rad', 0.05
        )
        self.declare_parameter('reverse_collision_escape_obstacle_min_x_m', 0.0)
        self.declare_parameter('pallet_exemption_enabled', True)
        self.declare_parameter(
            'pallet_exemption_pose_topic',
            '/forklift/pallet_approach/exemption_pose',
        )
        self.declare_parameter(
            'pallet_exemption_active_topic',
            '/forklift/pallet_approach/exemption_active',
        )
        self.declare_parameter('pallet_exemption_timeout_sec', 0.5)
        self.declare_parameter('pallet_exemption_length_m', 0.50)
        self.declare_parameter('pallet_exemption_width_m', 1.30)
        self.declare_parameter('pallet_exemption_reverse_only', True)
        self.declare_parameter('pallet_exemption_cost_threshold', 254)
        self.declare_parameter('emergency_stop_active', False)
        self.declare_parameter('allow_recovery_twist', True)
        self.declare_parameter('allow_recovery_backoff', True)
        self.declare_parameter('allow_recovery_pivot', True)
        self.declare_parameter('max_forward_velocity_mps', 0.45)
        self.declare_parameter('max_reverse_velocity_mps', 0.15)
        self.declare_parameter('max_recovery_velocity_mps', 0.10)
        self.declare_parameter('max_recovery_angular_velocity_radps', 0.30)
        self.declare_parameter('max_steering_angle_rad', math.pi / 2.0)
        self.declare_parameter('max_drive_rpm', 2485.0)
        self.declare_parameter('drive_accel_time_sec', 5.0)
        self.declare_parameter('drive_decel_time_sec', 3.0)
        self.declare_parameter('wheel_base', 1.4)
        self.declare_parameter('pivot_turn_radius', 0.6)
        self.declare_parameter('rear_axle_x_offset', 0.0)
        self.declare_parameter('pivot_steering_angle_rad', math.pi / 2.0)
        self.declare_parameter('control_rate_hz', 20.0)

        self._enabled = bool(self.get_parameter('enabled').value)
        self._raw_command_topic = str(self.get_parameter('raw_command_topic').value)
        self._gated_command_topic = str(self.get_parameter('gated_command_topic').value)
        self._recovery_twist_topic = str(self.get_parameter('recovery_twist_topic').value)
        self._vehicle_state_topic = str(self.get_parameter('vehicle_state_topic').value)
        self._fault_state_topic = str(self.get_parameter('fault_state_topic').value)
        self._localization_topic = str(self.get_parameter('localization_topic').value)
        self._localization_message_type = str(
            self.get_parameter('localization_message_type').value).lower()
        self._base_frame_id = str(self.get_parameter('base_frame_id').value)
        self._costmap_topic = str(self.get_parameter('costmap_topic').value)
        self._costmap_message_type = str(
            self.get_parameter('costmap_message_type').value).lower()
        self._status_topic = str(self.get_parameter('status_topic').value)
        self._command_timeout_sec = self._positive_param('command_timeout_sec', 0.5)
        self._recovery_timeout_sec = self._positive_param('recovery_timeout_sec', 0.5)
        self._vehicle_state_timeout_sec = self._positive_param('vehicle_state_timeout_sec', 0.5)
        self._fault_state_timeout_sec = self._positive_param('fault_state_timeout_sec', 0.5)
        self._localization_timeout_sec = self._positive_param('localization_timeout_sec', 0.5)
        self._costmap_timeout_sec = self._positive_param('costmap_timeout_sec', 0.5)
        self._require_vehicle_state = bool(self.get_parameter('require_vehicle_state').value)
        self._require_fault_state = bool(self.get_parameter('require_fault_state').value)
        self._require_localization = bool(self.get_parameter('require_localization').value)
        self._costmap_monitor_enabled = bool(
            self.get_parameter('costmap_monitor_enabled').value)
        self._collision_check_enabled = bool(
            self.get_parameter('collision_check_enabled').value)
        try:
            self._footprint = parse_footprint(self.get_parameter('footprint').value)
        except (SyntaxError, ValueError, TypeError) as exc:
            self.get_logger().error(f'Invalid footprint parameter: {exc}')
            self._footprint = parse_footprint(
                '[[1.709, 0.610], [1.709, -0.610], [-1.590, -0.610], [-1.590, 0.610]]'
            )
        self._footprint_sample_spacing = self._positive_param('footprint_sample_spacing', 0.05)
        self._footprint_collision_cost_threshold = int(
            self.get_parameter('footprint_collision_cost_threshold').value)
        self._unknown_is_collision = bool(self.get_parameter('unknown_is_collision').value)
        self._collision_check_horizon_sec = self._positive_param(
            'collision_check_horizon_sec',
            1.0,
        )
        self._collision_check_time_step_sec = self._positive_param(
            'collision_check_time_step_sec',
            0.1,
        )
        self._dynamic_stop_reaction_time_sec = max(
            0.0,
            float(self.get_parameter('dynamic_stop_reaction_time_sec').value),
        )
        self._dynamic_stop_brake_deceleration_mps2 = self._positive_param(
            'dynamic_stop_brake_deceleration_mps2',
            1.5,
        )
        self._dynamic_stop_clearance_m = max(
            0.0,
            float(self.get_parameter('dynamic_stop_clearance_m').value),
        )
        self._pivot_brake_deceleration = self._positive_param(
            'pivot_brake_deceleration_radps2', 0.15)
        self._pivot_stop_margin = max(
            0.0, float(self.get_parameter('pivot_stop_margin_rad').value))
        self._collision_compute_budget = self._positive_param(
            'collision_compute_budget_sec', 0.15)
        self._scan_protection_enabled = bool(
            self.get_parameter('scan_protection_enabled').value)
        self._scan_topic = str(self.get_parameter('scan_topic').value)
        self._scan_timeout_sec = self._positive_param('scan_timeout_sec', 0.7)
        self._scan_high_speed_freshness_timeout_sec = min(
            self._scan_timeout_sec,
            self._positive_param('scan_high_speed_freshness_timeout_sec', 0.25),
        )
        self._scan_degraded_max_speed_mps = self._positive_param(
            'scan_degraded_max_speed_mps', 1.0)
        self._scan_fresh_recovery_duration_sec = max(
            0.0,
            float(self.get_parameter('scan_fresh_recovery_duration_sec').value),
        )
        self._scan_required_range_m = self._positive_param('scan_required_range_m', 8.0)
        self._scan_collision_sample_spacing_m = self._positive_param(
            'scan_collision_sample_spacing_m', 0.05)
        self._scan_collision_padding_m = max(
            0.0,
            float(self.get_parameter('scan_collision_padding_m').value),
        )
        self._scan_require_motion_fov_coverage = bool(
            self.get_parameter('scan_require_motion_fov_coverage').value)
        self._allow_reverse_collision_escape = bool(
            self.get_parameter('allow_reverse_collision_escape').value
        )
        self._reverse_collision_escape_max_speed_mps = self._positive_param(
            'reverse_collision_escape_max_speed_mps', 0.15
        )
        self._reverse_collision_escape_max_steering_angle_rad = max(
            0.0,
            float(
                self.get_parameter(
                    'reverse_collision_escape_max_steering_angle_rad'
                ).value
            ),
        )
        self._reverse_collision_escape_obstacle_min_x_m = float(
            self.get_parameter(
                'reverse_collision_escape_obstacle_min_x_m'
            ).value
        )
        self._pallet_exemption_enabled = bool(
            self.get_parameter('pallet_exemption_enabled').value
        )
        self._pallet_exemption_pose_topic = str(
            self.get_parameter('pallet_exemption_pose_topic').value
        )
        self._pallet_exemption_active_topic = str(
            self.get_parameter('pallet_exemption_active_topic').value
        )
        self._pallet_exemption_timeout_sec = self._positive_param(
            'pallet_exemption_timeout_sec', 0.5
        )
        self._pallet_exemption_half_length = 0.5 * self._positive_param(
            'pallet_exemption_length_m', 0.50
        )
        self._pallet_exemption_half_width = 0.5 * self._positive_param(
            'pallet_exemption_width_m', 1.30
        )
        self._pallet_exemption_reverse_only = bool(
            self.get_parameter('pallet_exemption_reverse_only').value
        )
        self._pallet_exemption_cost_threshold = int(
            self.get_parameter('pallet_exemption_cost_threshold').value
        )
        self._emergency_stop = bool(self.get_parameter('emergency_stop_active').value)
        self._allow_recovery_twist = bool(self.get_parameter('allow_recovery_twist').value)
        self._allow_recovery_backoff = bool(self.get_parameter('allow_recovery_backoff').value)
        self._allow_recovery_pivot = bool(self.get_parameter('allow_recovery_pivot').value)
        self._max_forward_velocity_mps = self._positive_param('max_forward_velocity_mps', 0.45)
        self._max_reverse_velocity_mps = self._positive_param('max_reverse_velocity_mps', 0.15)
        self._max_recovery_velocity_mps = self._positive_param('max_recovery_velocity_mps', 0.10)
        self._max_recovery_angular_velocity_radps = self._positive_param(
            'max_recovery_angular_velocity_radps',
            0.30,
        )
        self._max_steering_angle_rad = self._positive_param(
            'max_steering_angle_rad',
            math.pi / 2.0,
        )
        self._max_drive_rpm = self._positive_param('max_drive_rpm', 2485.0)
        self._drive_accel_time_sec = self._positive_param('drive_accel_time_sec', 5.0)
        self._drive_decel_time_sec = self._positive_param('drive_decel_time_sec', 3.0)
        self._wheel_base = self._positive_param('wheel_base', 1.4)
        self._pivot_turn_radius = self._positive_param('pivot_turn_radius', 0.6)
        self._rear_axle_x_offset = float(
            self.get_parameter('rear_axle_x_offset').value
        )
        self._pivot_steering_angle_rad = min(
            self._positive_param('pivot_steering_angle_rad', math.pi / 2.0),
            self._max_steering_angle_rad,
        )
        control_rate_hz = self._positive_param('control_rate_hz', 20.0)

        self._last_raw_command: Optional[ForkliftControlCommand] = None
        self._last_raw_command_time = self.get_clock().now()
        self._last_recovery_twist: Optional[Twist] = None
        self._last_recovery_twist_time = self.get_clock().now()
        self._last_vehicle_state: Optional[ForkliftVehicleState] = None
        self._last_vehicle_state_time = self.get_clock().now()
        self._last_fault_state: Optional[ForkliftFaultState] = None
        self._last_fault_state_time = self.get_clock().now()
        self._last_localization_time = self.get_clock().now()
        self._last_pose: Optional[Pose2D] = None
        self._last_localization = None
        self._last_yaw_rate = None
        self._yaw_rate_stamp = None
        self._last_costmap: Optional[Any] = None
        self._last_costmap_time = self.get_clock().now()
        self._last_costmap_error = ''
        self._last_costmap_interval_sec = 0.0
        self._last_scan: Optional[LaserScan] = None
        self._last_scan_time = self.get_clock().now()
        self._scan_speed_degraded = False
        self._scan_fresh_recovery_start = None
        self._cached_scan: Optional[LaserScan] = None
        self._cached_scan_points: List[Point2D] = []
        self._cached_scan_to_base_yaw = 0.0
        self._reverse_escape_validated_by_scan = False
        self._last_reason = ''
        self._pallet_exemption_active = False
        self._last_pallet_exemption_time = self.get_clock().now()
        self._pallet_exemption_pose: Optional[PoseStamped] = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._command_lock = threading.RLock()
        self._stop_generation = 0
        self._checked_command_stamp = None
        self._checked_scan = None
        self._checked_costmap = None
        self._obstacle_release = ObstacleReleaseState(
            self._positive_param('obstacle_release_clear_sec', 0.5),
            self._positive_param('obstacle_release_steering_change_rad', 0.15))
        self._raw_callback_group = MutuallyExclusiveCallbackGroup()
        self._scan_callback_group = MutuallyExclusiveCallbackGroup()
        self._scan_lock = threading.RLock()
        self._costmap_callback_group = MutuallyExclusiveCallbackGroup()
        self._costmap_lock = threading.RLock()

        self._command_pub = self.create_publisher(
            ForkliftControlCommand,
            self._gated_command_topic,
            1,
        )
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self.create_subscription(
            ForkliftControlCommand,
            self._raw_command_topic,
            self._on_raw_command,
            1,
            callback_group=self._raw_callback_group,
        )
        if self._recovery_twist_topic:
            self.create_subscription(
                Twist,
                self._recovery_twist_topic,
                self._on_recovery_twist,
                10,
            )
        if self._vehicle_state_topic:
            self.create_subscription(
                ForkliftVehicleState,
                self._vehicle_state_topic,
                self._on_vehicle_state,
                10,
            )
        if self._fault_state_topic:
            self.create_subscription(
                ForkliftFaultState,
                self._fault_state_topic,
                self._on_fault_state,
                10,
            )
        if self._localization_topic:
            if self._localization_message_type in {
                'pose_with_covariance_stamped', 'amcl_pose'
            }:
                self.create_subscription(
                    PoseWithCovarianceStamped,
                    self._localization_topic,
                    self._on_localization,
                    10,
                )
            else:
                self.create_subscription(
                    Odometry, self._localization_topic, self._on_localization, 10)
        if self._costmap_topic:
            if self._costmap_message_type in {'costmap_raw', 'nav2_costmap'}:
                if Nav2Costmap is None:
                    self.get_logger().error(
                        'costmap_message_type requires nav2_msgs/Costmap, but nav2_msgs is not '
                        'available; no costmap subscription was created.'
                    )
                else:
                    self.create_subscription(
                        Nav2Costmap,
                        self._costmap_topic,
                        self._on_costmap,
                        1,
                        callback_group=self._costmap_callback_group,
                    )
            else:
                self.create_subscription(
                    OccupancyGrid,
                    self._costmap_topic,
                    self._on_costmap,
                    1,
                    callback_group=self._costmap_callback_group,
                )
        if self._scan_protection_enabled and self._scan_topic:
            self.create_subscription(
                LaserScan, self._scan_topic, self._on_scan, 1,
                callback_group=self._scan_callback_group)
        if self._pallet_exemption_enabled:
            self.create_subscription(
                PoseStamped,
                self._pallet_exemption_pose_topic,
                self._on_pallet_exemption_pose,
                10,
            )
            self.create_subscription(
                Bool,
                self._pallet_exemption_active_topic,
                self._on_pallet_exemption_active,
                10,
            )
        self.create_service(
            SetEmergencyStop,
            '/forklift_safety/set_emergency_stop',
            self._on_set_emergency_stop,
        )
        self.create_timer(1.0 / control_rate_hz, self._on_timer)
        self.get_logger().info(
            f'safety_command_gate ready: {self._raw_command_topic} -> '
            f'{self._gated_command_topic}, recovery={self._recovery_twist_topic or "disabled"}, '
            f'costmap={self._costmap_topic or "disabled"}, '
            f'scan={self._scan_topic if self._scan_protection_enabled else "disabled"}, '
            f'scan_timeout={self._scan_timeout_sec:.3f}s '
            f'high_speed_freshness={self._scan_high_speed_freshness_timeout_sec:.3f}s '
            f'degraded_max_speed={self._scan_degraded_max_speed_mps:.3f}m/s '
            f'collision_budget={self._collision_compute_budget:.3f}s '
            'scan_callback=independent latest-only '
            f'costmap_callback=independent latest-only costmap_timeout={self._costmap_timeout_sec:.3f}s '
            f'obstacle_release_clear={self._obstacle_release.clear_sec:.3f}s'
        )

    def _positive_param(self, name: str, fallback: float) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            self.get_logger().warning(
                f'Parameter {name} must be positive; using fallback {fallback}.'
            )
            return fallback
        return value

    def _on_raw_command(self, msg: ForkliftControlCommand) -> None:
        with self._command_lock:
            self._last_raw_command = msg
            self._last_raw_command_time = self.get_clock().now()
            expired = source_age_sec(
                msg.header.stamp, self._last_raw_command_time.nanoseconds / 1e9
            ) > self._command_timeout_sec
            if not msg.enable or msg.brake or expired:
                # This callback runs independently of expensive collision work.
                # A completed older check may never overwrite this stop.
                self._stop_generation += 1
                command = stop_command(self._last_raw_command_time.to_msg())
                if not expired and not self._stop_reason_from_health():
                    # Fork actions intentionally command hydraulics with the
                    # traction brake applied. Preserve that existing contract.
                    if msg.enable and msg.brake:
                        command = copy.deepcopy(msg)
                        command.header.stamp = self._last_raw_command_time.to_msg()
                        command.forward = command.reverse = False
                        command.velocity_mps = command.drive_rpm = 0.0
                    command.steering_angle_rad = clamp(
                        msg.steering_angle_rad,
                        -self._max_steering_angle_rad, self._max_steering_angle_rad)
                    command.steering_angle_deg = math.degrees(command.steering_angle_rad)
                    command.enable = msg.enable
                self._command_pub.publish(command)

    def _on_recovery_twist(self, msg: Twist) -> None:
        self._last_recovery_twist = msg
        self._last_recovery_twist_time = self.get_clock().now()

    def _on_vehicle_state(self, msg: ForkliftVehicleState) -> None:
        self._last_vehicle_state = msg
        self._last_vehicle_state_time = self.get_clock().now()

    def _on_fault_state(self, msg: ForkliftFaultState) -> None:
        self._last_fault_state = msg
        self._last_fault_state_time = self.get_clock().now()

    def _on_localization(self, msg) -> None:
        self._last_localization = msg
        self._last_localization_time = self.get_clock().now()
        pose = msg.pose.pose
        self._last_pose = (
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )
        self._last_yaw_rate = (
            float(msg.twist.twist.angular.z) if hasattr(msg, 'twist') else None)
        self._yaw_rate_stamp = copy.deepcopy(msg.header.stamp)

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        now = self.get_clock().now()
        error = costmap_error(msg)
        with self._costmap_lock:
            self._last_costmap_interval_sec = (
                (now - self._last_costmap_time).nanoseconds / 1e9
                if self._last_costmap is not None else 0.0)
            self._last_costmap_time = now
            self._last_costmap_error = error
            self._last_costmap = msg

    def _costmap_snapshot(self):
        with self._costmap_lock:
            return (self._last_costmap, self._last_costmap_time,
                    self._last_costmap_error, self._last_costmap_interval_sec)

    def _costmap_snapshot_stop_reason(self, snapshot, now, phase):
        costmap, received, error, interval = snapshot
        age = (now - received).nanoseconds / 1e9
        reason = costmap_stop_reason(
            self._costmap_monitor_enabled, age, costmap is not None,
            error, self._costmap_timeout_sec)
        if reason == 'costmap timeout':
            # Some Foxy Costmap publishers leave header/metadata stamps zero.
            # Report source age, but do not use a missing stamp as freshness.
            source = source_age_sec(costmap.header.stamp, now.nanoseconds / 1e9)
            self.get_logger().warning(
                'Costmap timeout detail: phase={} receive_age={:.3f}s '
                'last_interval={:.3f}s source_age={:.3f}s timeout={:.3f}s'.format(
                    phase, age, interval, source, self._costmap_timeout_sec),
                throttle_duration_sec=2.0)
        return reason

    def _on_scan(self, msg: LaserScan) -> None:
        now = self.get_clock().now()
        with self._scan_lock:
            if source_age_sec(msg.header.stamp, now.nanoseconds / 1e9) > self._scan_high_speed_freshness_timeout_sec:
                self._scan_speed_degraded = True
                self._scan_fresh_recovery_start = None
            elif self._last_scan is not None:
                interval_sec = (now - self._last_scan_time).nanoseconds / 1e9
                if interval_sec > self._scan_high_speed_freshness_timeout_sec:
                    self._scan_speed_degraded = True
                    self._scan_fresh_recovery_start = None
                elif self._scan_speed_degraded:
                    if self._scan_fresh_recovery_start is None:
                        self._scan_fresh_recovery_start = now
                    elif (
                        now - self._scan_fresh_recovery_start
                    ).nanoseconds / 1e9 >= self._scan_fresh_recovery_duration_sec:
                        self._scan_speed_degraded = False
                        self._scan_fresh_recovery_start = None
            self._last_scan = msg
            self._last_scan_time = now
        # Projection caches belong exclusively to the collision timer. The
        # message-identity key invalidates them without racing this callback.

    def _scan_snapshot(self):
        with self._scan_lock:
            return self._last_scan, self._last_scan_time

    def _scan_snapshot_stop_reason(self, snapshot, now, phase):
        scan, received = snapshot
        receive_age = (now - received).nanoseconds / 1e9
        source_age = source_age_sec(scan.header.stamp, now.nanoseconds / 1e9) if scan is not None else math.inf
        reason = scan_stop_reason(
            self._scan_protection_enabled, max(receive_age, source_age),
            scan is not None, float(scan.range_max) if scan is not None else 0.0,
            self._scan_timeout_sec, self._scan_required_range_m)
        if reason == 'scan timeout':
            stamp = scan.header.stamp
            self.get_logger().warning(
                'Scan timeout detail: phase={} receive_age={:.3f}s source_age={:.3f}s '
                'timeout={:.3f}s source_stamp={}.{:09d}; '
                'receive_age is callback age, not DDS transport age'.format(
                    phase, receive_age, source_age, self._scan_timeout_sec,
                    stamp.sec, stamp.nanosec), throttle_duration_sec=2.0)
        return reason

    def _scan_speed_is_degraded(self):
        with self._scan_lock:
            now = self.get_clock().now()
            age = (now - self._last_scan_time).nanoseconds / 1e9
            if self._last_scan is not None:
                age = max(age, source_age_sec(
                    self._last_scan.header.stamp, now.nanoseconds / 1e9))
            if self._scan_protection_enabled and age > self._scan_high_speed_freshness_timeout_sec:
                self._scan_speed_degraded = True
                self._scan_fresh_recovery_start = None
            return self._scan_protection_enabled and self._scan_speed_degraded

    def _on_pallet_exemption_pose(self, msg: PoseStamped) -> None:
        self._pallet_exemption_pose = msg

    def _on_pallet_exemption_active(self, msg: Bool) -> None:
        self._pallet_exemption_active = bool(msg.data)
        self._last_pallet_exemption_time = self.get_clock().now()

    def _on_set_emergency_stop(
        self,
        request: SetEmergencyStop.Request,
        response: SetEmergencyStop.Response,
    ) -> SetEmergencyStop.Response:
        self._emergency_stop = bool(request.emergency_stop)
        response.success = True
        response.message = (
            'safety gate emergency stop enabled'
            if self._emergency_stop else
            'safety gate emergency stop cleared'
        )
        self.get_logger().warning(response.message)
        return response

    def _on_timer(self) -> None:
        started = time.monotonic()
        self._checked_scan = None
        self._checked_costmap = None
        with self._command_lock:
            generation = self._stop_generation
        command, reason = self._latest_safe_command()
        elapsed = time.monotonic() - started
        with self._command_lock:
            if generation != self._stop_generation:
                if getattr(self, '_obstacle_release', None) is not None:
                    self._obstacle_release.interrupt()
                return
            now = self.get_clock().now()
            source_age = source_age_sec(
                self._checked_command_stamp, now.nanoseconds / 1e9
            ) if self._checked_command_stamp is not None else 0.0
            if command.enable and not command.brake:
                if source_age > self._command_timeout_sec:
                    command, reason = stop_command(now.to_msg()), 'command source timeout'
                elif elapsed > self._collision_compute_budget:
                    command, reason = stop_command(now.to_msg()), 'collision check deadline exceeded'
                if command.enable and not command.brake and self._checked_costmap is not None:
                    map_reason = self._costmap_snapshot_stop_reason(
                        self._checked_costmap, now, 'after_collision_check')
                    if not map_reason:
                        map_reason = self._costmap_snapshot_stop_reason(
                            self._costmap_snapshot(), now, 'latest_before_publish')
                    if map_reason:
                        command, reason = stop_command(now.to_msg()), map_reason
                if command.enable and not command.brake and self._checked_scan is not None:
                    # New arrivals must not freshen an older scan that was
                    # actually used for the swept-footprint collision check.
                    scan_reason = self._scan_snapshot_stop_reason(
                        self._checked_scan, now, 'after_collision_check')
                    if not scan_reason:
                        scan_reason = self._scan_snapshot_stop_reason(
                            self._scan_snapshot(), now, 'latest_before_publish')
                    if scan_reason:
                        command, reason = stop_command(now.to_msg()), scan_reason
                    elif self._scan_speed_is_degraded() and cap_control_command_speed(
                            command, self._scan_degraded_max_speed_mps):
                        reason = ('raw command: scan freshness speed cap '
                                  f'{self._scan_degraded_max_speed_mps:.2f} m/s')
            if (getattr(self, '_obstacle_release', None) is not None
                    and reason != 'obstacle release waiting for stable clearance'
                    and (not command.enable or command.brake)):
                self._obstacle_release.interrupt()
            elif (getattr(self, '_obstacle_release', None) is not None
                    and command.enable and not command.brake):
                self._obstacle_release.commit_release()
            self._command_pub.publish(command)
        if elapsed > 0.05:
            self.get_logger().warning(
                'Safety check slow: {:.3f} s; command source age {:.3f} s'.format(
                    elapsed, source_age), throttle_duration_sec=2.0)
        self._publish_status(reason)
        self._log_reason(reason)

    def _latest_safe_command(self) -> Tuple[ForkliftControlCommand, str]:
        self._checked_command_stamp = None
        stamp = self.get_clock().now().to_msg()
        stop_reason = self._stop_reason_from_health()
        if stop_reason:
            return stop_command(stamp), stop_reason

        with self._command_lock:
            raw = self._last_raw_command
            received = self._last_raw_command_time
        raw_age = (self.get_clock().now() - received).nanoseconds / 1e9
        if raw is not None:
            self._checked_command_stamp = copy.deepcopy(raw.header.stamp)
            if source_age_sec(raw.header.stamp, self.get_clock().now().nanoseconds / 1e9) > self._command_timeout_sec:
                return stop_command(stamp), 'command source timeout'
        if raw is not None and raw_age <= self._command_timeout_sec:
            command = clamp_control_command(
                raw,
                self._max_forward_velocity_mps,
                self._max_reverse_velocity_mps,
                self._max_steering_angle_rad,
                self._max_drive_rpm,
                self._drive_accel_time_sec,
                self._drive_decel_time_sec,
            )
            command.header.stamp = stamp
            if not self._enabled:
                return command, 'bypass'
            if not command.enable or command.brake:
                return command, 'raw stop'
            if direction(command) == 0:
                if is_steering_only_command(command):
                    return command, 'steering center'
                return stop_command(stamp), 'invalid direction'
            scan_speed_capped = self._scan_speed_is_degraded() and cap_control_command_speed(
                command, self._scan_degraded_max_speed_mps)
            collision_reason = self._collision_stop_reason(command)
            if collision_reason:
                stopped = stop_command(stamp)
                # Ordinary obstacle pauses preserve a fresh measured angle;
                # health failures still use the independent hard stop above.
                state = self._last_vehicle_state
                if (state is not None and math.isfinite(state.steering_angle_rad)
                        and (self.get_clock().now() - self._last_vehicle_state_time).nanoseconds / 1e9
                        <= self._vehicle_state_timeout_sec
                        and ('collision' in collision_reason or 'footprint' in collision_reason
                             or collision_reason.startswith('costmap coverage insufficient')
                             or collision_reason == 'obstacle release waiting for stable clearance')):
                    stopped.steering_angle_rad = clamp(
                        state.steering_angle_rad,
                        -self._max_steering_angle_rad, self._max_steering_angle_rad)
                    stopped.steering_angle_deg = math.degrees(stopped.steering_angle_rad)
                return stopped, collision_reason
            if scan_speed_capped:
                return command, (
                    'raw command: scan freshness speed cap '
                    f'{self._scan_degraded_max_speed_mps:.2f} m/s'
                )
            return command, 'raw command'

        if self._allow_recovery_twist and self._last_recovery_twist is not None:
            recovery_age = (
                self.get_clock().now() - self._last_recovery_twist_time
            ).nanoseconds / 1e9
            if recovery_age <= self._recovery_timeout_sec:
                command, reason = recovery_command_from_twist(
                    self._last_recovery_twist,
                    self._max_recovery_velocity_mps,
                    self._max_recovery_angular_velocity_radps,
                    self._wheel_base,
                    self._pivot_turn_radius,
                    self._pivot_steering_angle_rad,
                    self._allow_recovery_backoff,
                    self._allow_recovery_pivot,
                    stamp,
                )
                if command.enable and not command.brake and direction(command) != 0:
                    apply_drive_envelope(
                        command,
                        self._max_drive_rpm,
                        self._drive_accel_time_sec,
                        self._drive_decel_time_sec,
                    )
                collision_reason = self._collision_stop_reason(command)
                if collision_reason:
                    return stop_command(stamp), collision_reason
                return command, reason

        if self._last_raw_command is None:
            return stop_command(stamp), 'waiting for first command'
        return stop_command(stamp), 'command timeout'

    def _stop_reason_from_health(self) -> str:
        now = self.get_clock().now()
        if self._emergency_stop:
            return 'emergency stop'

        vehicle_state = self._last_vehicle_state
        if vehicle_state is None:
            if self._require_vehicle_state:
                return 'vehicle state missing'
        else:
            age = (now - self._last_vehicle_state_time).nanoseconds / 1e9
            if self._require_vehicle_state and age > self._vehicle_state_timeout_sec:
                return 'vehicle state timeout'
            if vehicle_state.emergency_stopped or vehicle_state.soft_emergency_stop:
                return 'vehicle emergency stop'
            if self._require_vehicle_state and vehicle_state.parking_brake:
                return 'parking brake'
            if not vehicle_state.interlock and self._require_vehicle_state:
                return 'vehicle interlock open'

        fault_state = self._last_fault_state
        if fault_state is None:
            if self._require_fault_state:
                return 'fault state missing'
        else:
            age = (now - self._last_fault_state_time).nanoseconds / 1e9
            if self._require_fault_state and age > self._fault_state_timeout_sec:
                return 'fault state timeout'
            if fault_state.has_fault:
                return 'vehicle fault'

        if self._require_localization:
            age = (now - self._last_localization_time).nanoseconds / 1e9
            if age > self._localization_timeout_sec:
                return 'localization timeout'

        if self._costmap_monitor_enabled:
            reason = self._costmap_snapshot_stop_reason(
                self._costmap_snapshot(), now, 'before_collision_check')
            if reason:
                return reason

        return ''

    def _collision_stop_reason(self, command: ForkliftControlCommand) -> str:
        if not self._collision_check_enabled:
            return ''
        release = self._obstacle_release
        release.prepare(direction(command), float(command.steering_angle_rad))
        probe = copy.deepcopy(command)
        # Use the same protected speed in the costmap and scan geometry. The
        # output command is unchanged; this only prevents shrinking a blocked
        # sweep as the vehicle brakes or the MPC restarts its acceleration ramp.
        probe.velocity_mps = self._protected_speed(command)
        reason = self._collision_check_reason(probe)
        if reason:
            if obstacle_stop_reason(reason):
                release.blocked(self._protected_speed(command))
            else:
                release.interrupt()
            return reason
        scan_token = None
        if self._scan_protection_enabled and self._checked_scan is not None:
            stamp = self._checked_scan[0].header.stamp
            scan_token = (stamp.sec, stamp.nanosec)
        costmap_token = (self._checked_costmap[1].nanoseconds
                         if self._checked_costmap is not None
                         and self._checked_costmap[0] is not None else None)
        if not release.clear(self.get_clock().now().nanoseconds / 1e9,
                             scan_token, costmap_token):
            return 'obstacle release waiting for stable clearance'
        return ''

    def _collision_check_reason(self, command: ForkliftControlCommand) -> str:
        started = time.monotonic()
        self._collision_timings = {}
        reason = ''
        try:
            reason = self._collision_check_impl(command)
            return reason
        finally:
            elapsed = time.monotonic() - started
            if elapsed >= 0.05:
                self.get_logger().warning(
                    'Collision timing: total={:.4f}s scan={:.4f}s pose_tf={:.4f}s '
                    'costmap={:.4f}s result={}'.format(
                        elapsed, self._collision_timings.get('scan', 0.0),
                        self._collision_timings.get('pose_tf', 0.0),
                        self._collision_timings.get('costmap', 0.0), reason),
                    throttle_duration_sec=2.0)

    def _pose_in_costmap(self, frame):
        msg = self._last_localization
        if msg is None:
            raise ValueError('costmap pose missing')
        source = msg.header.frame_id
        child = getattr(msg, 'child_frame_id', self._base_frame_id)
        if not frame or not source or not child:
            raise ValueError('costmap pose frame missing')
        age = source_age_sec(msg.header.stamp, self.get_clock().now().nanoseconds / 1e9)
        if age > self._localization_timeout_sec:
            raise ValueError('costmap pose source stale')
        p = msg.pose.pose
        pose = (float(p.position.x), float(p.position.y), yaw_from_quaternion(p.orientation))
        if not all(math.isfinite(v) for v in pose):
            raise ValueError('costmap pose invalid')

        def lookup(target, origin):
            try:
                tf = self._tf_buffer.lookup_transform(
                    target, origin, Time.from_msg(msg.header.stamp)).transform
            except Exception as exc:
                raise ValueError('costmap pose transform unavailable: {} <- {}'.format(
                    target, origin)) from exc
            result = (tf.translation.x, tf.translation.y, yaw_from_quaternion(tf.rotation))
            if not all(math.isfinite(v) for v in result):
                raise ValueError('costmap pose transform invalid')
            return result

        # Odometry may describe base_footprint rather than the footprint's base_link.
        # Use the pose's source timestamp, never the older rolling-grid timestamp.
        if child != self._base_frame_id:
            offset = lookup(child, self._base_frame_id)
            pose = (*transform_point(offset[:2], pose), pose[2] + offset[2])
        if source != frame:
            offset = lookup(frame, source)
            pose = (*transform_point(pose[:2], offset), pose[2] + offset[2])
        return pose

    def _collision_check_impl(self, command: ForkliftControlCommand) -> str:
        if not self._collision_check_enabled:
            return ''
        if not command.enable or command.brake or direction(command) == 0:
            return ''
        try:
            pivot_poses = self._pivot_prediction(command)
        except ValueError as exc:
            return str(exc)
        started = time.monotonic()
        scan_reason = self._scan_collision_stop_reason(command)
        self._collision_timings['scan'] = time.monotonic() - started
        if scan_reason:
            return scan_reason
        snapshot = self._costmap_snapshot()
        self._checked_costmap = snapshot
        costmap = snapshot[0]
        reason = self._costmap_snapshot_stop_reason(
            snapshot, self.get_clock().now(), 'collision_check')
        if reason:
            return reason
        if costmap is None:
            return ''
        started = time.monotonic()
        try:
            pose = self._pose_in_costmap(costmap.header.frame_id)
        except ValueError as exc:
            return str(exc)
        finally:
            self._collision_timings['pose_tf'] = time.monotonic() - started

        pallet_exemption = self._active_pallet_exemption(
            command,
            str(getattr(costmap.header, 'frame_id', '')),
        )
        cost_threshold = self._footprint_collision_cost_threshold
        if pallet_exemption is not None:
            cost_threshold = max(
                cost_threshold,
                self._pallet_exemption_cost_threshold,
            )

        collision_horizon_sec = self._dynamic_collision_horizon_sec(command)
        collision_time_step_sec = spatial_sweep_time_step(
            self._protected_speed(command),
            self._scan_collision_sample_spacing_m,
            collision_horizon_sec,
        )
        started = time.monotonic()
        collision, reason = footprint_sweep_collision(
            costmap,
            self._footprint,
            pose,
            command,
            self._wheel_base,
            self._pivot_turn_radius,
            self._rear_axle_x_offset,
            collision_horizon_sec,
            collision_time_step_sec,
            self._pivot_steering_angle_rad,
            self._footprint_sample_spacing,
            cost_threshold,
            self._unknown_is_collision,
            pallet_exemption,
            self._reverse_escape_validated_by_scan,
            prediction_poses=(
                [(*transform_point((x, y), pose), yaw + pose[2])
                 for x, y, yaw in pivot_poses] if pivot_poses is not None else None),
        )
        self._collision_timings['costmap'] = time.monotonic() - started
        if reason.startswith('costmap coverage insufficient'):
            width, height, resolution, origin = costmap_metadata(costmap)
            now = self.get_clock().now()
            self.get_logger().warning(
                '{}; frame={} size={}x{} resolution={:.3f} '
                'origin=({:.3f},{:.3f},{:.3f}) pose=({:.3f},{:.3f},{:.3f}) '
                'receive_age={:.3f}s source_age={:.3f}s '
                'pose_frame={} pose_age={:.3f}s speed={:.3f} horizon={:.3f}s'.format(
                    reason, costmap.header.frame_id, width, height, resolution,
                    origin.position.x, origin.position.y, yaw_from_quaternion(origin.orientation),
                    *pose, (now - snapshot[1]).nanoseconds / 1e9,
                    source_age_sec(costmap.header.stamp, now.nanoseconds / 1e9),
                    self._last_localization.header.frame_id,
                    source_age_sec(self._last_localization.header.stamp, now.nanoseconds / 1e9),
                    self._protected_speed(command), collision_horizon_sec),
                throttle_duration_sec=2.0)
        return reason if collision else ''

    def _pivot_prediction(self, command):
        if abs(command.steering_angle_rad) < self._pivot_steering_angle_rad - 1e-3:
            return None
        if (self._last_yaw_rate is None or self._yaw_rate_stamp is None
                or not math.isfinite(self._last_yaw_rate)
                or source_age_sec(self._yaw_rate_stamp, self.get_clock().now().nanoseconds / 1e9)
                > self._localization_timeout_sec):
            raise ValueError('pivot angular feedback stale or unavailable')
        rate = math.copysign(
            self._protected_speed(command) / self._pivot_turn_radius,
            command.steering_angle_rad)
        poses = pivot_braking_poses(
            self._footprint, rate, self._last_yaw_rate,
            self._dynamic_stop_reaction_time_sec, self._pivot_brake_deceleration,
            self._pivot_stop_margin, self._rear_axle_x_offset,
            self._scan_collision_sample_spacing_m)
        self.get_logger().info(
            'Pivot safety sweep: command_w={:.3f} measured_w={:.3f} '
            'angle={:.3f} rad samples={}'.format(
                rate, self._last_yaw_rate, max(abs(p[2]) for p in poses), len(poses)),
            throttle_duration_sec=2.0)
        return poses

    def _dynamic_collision_horizon_sec(
        self,
        command: ForkliftControlCommand,
    ) -> float:
        speed = self._protected_speed(command)
        if speed <= 1e-6:
            return self._collision_check_horizon_sec
        braking_travel_m = (
            speed * self._dynamic_stop_reaction_time_sec +
            speed * speed / (2.0 * self._dynamic_stop_brake_deceleration_mps2)
        )
        return max(
            self._collision_check_time_step_sec,
            braking_travel_m / speed,
        )

    def _protected_speed(self, command: ForkliftControlCommand) -> float:
        vehicle_speed = 0.0
        if self._last_vehicle_state is not None:
            vehicle_speed = abs(float(self._last_vehicle_state.velocity_mps))
        return max(vehicle_speed, abs(float(command.velocity_mps)),
                   self._obstacle_release.speed_floor if self._obstacle_release.active else 0.0)

    def _scan_collision_stop_reason(self, command: ForkliftControlCommand) -> str:
        self._reverse_escape_validated_by_scan = False
        snapshot = self._scan_snapshot()
        scan, _received = snapshot
        self._checked_scan = snapshot if self._scan_protection_enabled else None
        now = self.get_clock().now()
        reason = self._scan_snapshot_stop_reason(snapshot, now, 'before_collision_check')
        if reason:
            return reason
        if scan is None:
            return ''

        if self._cached_scan is scan:
            scan_to_base_yaw = self._cached_scan_to_base_yaw
            scan_points = self._cached_scan_points
        else:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._base_frame_id,
                    scan.header.frame_id,
                    Time.from_msg(scan.header.stamp),
                )
            except Exception:
                return 'scan transform unavailable'

            scan_to_base_yaw = yaw_from_quaternion(transform.transform.rotation)
            scan_points = laser_scan_points_in_base(
                scan,
                (
                    float(transform.transform.translation.x),
                    float(transform.transform.translation.y),
                    scan_to_base_yaw,
                ),
            )
            self._cached_scan = scan
            self._cached_scan_points = scan_points
            self._cached_scan_to_base_yaw = scan_to_base_yaw
        travel_angle = 0.0 if direction(command) > 0 else math.pi
        if self._scan_require_motion_fov_coverage and not angle_in_scan_fov(
            travel_angle,
            scan,
            scan_to_base_yaw,
        ):
            return 'scan blind motion direction'

        pallet_exemption = self._active_pallet_exemption(command, self._base_frame_id)
        collision, reason = scan_sweep_collision(
            scan_points,
            self._footprint,
            command,
            self._wheel_base,
            self._pivot_turn_radius,
            self._rear_axle_x_offset,
            dynamic_stopping_distance(
                self._protected_speed(command),
                self._dynamic_stop_reaction_time_sec,
                self._dynamic_stop_brake_deceleration_mps2,
                self._dynamic_stop_clearance_m,
            ),
            self._scan_collision_sample_spacing_m,
            self._pivot_steering_angle_rad,
            self._scan_collision_padding_m,
            pallet_exemption,
            self._allow_reverse_collision_escape,
            self._reverse_collision_escape_max_speed_mps,
            self._reverse_collision_escape_max_steering_angle_rad,
            self._reverse_collision_escape_obstacle_min_x_m,
            prediction_poses=self._pivot_prediction(command),
        )
        self._reverse_escape_validated_by_scan = (
            not collision and reason == 'scan reverse escape clear'
        )
        return reason if collision else ''

    def _active_pallet_exemption(
        self,
        command: ForkliftControlCommand,
        output_frame: str,
    ) -> Optional[PalletExemptionZone]:
        if not self._pallet_exemption_enabled:
            return None
        if not self._pallet_exemption_active:
            return None
        if self._pallet_exemption_reverse_only and direction(command) != -1:
            return None
        age = (
            self.get_clock().now() - self._last_pallet_exemption_time
        ).nanoseconds / 1e9
        if age > self._pallet_exemption_timeout_sec:
            return None
        target = self._pallet_exemption_pose
        if target is None or not output_frame:
            return None
        source_frame = str(target.header.frame_id)
        if not source_frame:
            return None

        target_yaw = yaw_from_quaternion(target.pose.orientation)
        center_x = float(target.pose.position.x)
        center_y = float(target.pose.position.y)
        if source_frame != output_frame:
            try:
                transform = self._tf_buffer.lookup_transform(
                    output_frame,
                    source_frame,
                    Time(),
                )
            except Exception:
                return None
            transform_yaw = yaw_from_quaternion(transform.transform.rotation)
            center_x, center_y = transform_point(
                (center_x, center_y),
                (
                    float(transform.transform.translation.x),
                    float(transform.transform.translation.y),
                    transform_yaw,
                ),
            )
            target_yaw += transform_yaw

        return PalletExemptionZone(
            x=center_x,
            y=center_y,
            yaw=target_yaw,
            half_length=self._pallet_exemption_half_length,
            half_width=self._pallet_exemption_half_width,
        )

    def _publish_status(self, reason: str) -> None:
        status = String()
        status.data = reason
        self._status_pub.publish(status)

    def _log_reason(self, reason: str) -> None:
        if reason == self._last_reason:
            return
        self._last_reason = reason
        if reason in {
            'emergency stop',
            'vehicle emergency stop',
            'vehicle fault',
            'invalid direction',
            'localization timeout',
            'costmap missing',
            'costmap timeout',
            'collision pose missing',
        }:
            self.get_logger().warning(f'Safety gate stopping: {reason}.')
        elif reason.startswith('costmap invalid') or reason.startswith('footprint collision'):
            self.get_logger().warning(f'Safety gate stopping: {reason}.')
        elif reason in {'raw command', 'recovery backoff', 'recovery pivot', 'recovery forward'}:
            self.get_logger().info(f'Safety gate passing: {reason}.')
        elif reason:
            self.get_logger().info(f'Safety gate state: {reason}.')


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SafetyCommandGate()
    # Keep stops and both sensor streams runnable during collision computation.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
