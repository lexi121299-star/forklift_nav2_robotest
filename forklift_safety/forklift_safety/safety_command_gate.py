from __future__ import annotations

import ast
import math
from dataclasses import dataclass
from typing import Any, List, Optional, Sequence, Tuple

import rclpy
from forklift_msgs.msg import ForkliftControlCommand, ForkliftFaultState, ForkliftVehicleState
from forklift_msgs.srv import SetEmergencyStop
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
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
    width, height, resolution, _origin = costmap_metadata(costmap)
    if width <= 0 or height <= 0:
        return 'empty dimensions'
    if resolution <= 0.0:
        return 'invalid resolution'
    expected_cells = width * height
    if len(costmap.data) < expected_cells:
        return f'truncated data {len(costmap.data)}/{expected_cells}'
    return ''


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
    width, _height, _resolution, _origin = costmap_metadata(costmap)
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
) -> Tuple[bool, str]:
    travel_direction = direction(command)
    speed = abs(float(command.velocity_mps))
    if travel_direction == 0 or speed <= 1e-6:
        return False, 'scan sweep clear'

    step_distance = positive(sample_spacing_m, 0.05)
    time_step = max(0.01, min(0.1, step_distance / speed))
    horizon_sec = stopping_distance_m / speed
    for pose in predicted_poses_for_command(
        (0.0, 0.0, 0.0),
        command,
        wheel_base,
        pivot_turn_radius,
        rear_axle_x_offset,
        horizon_sec,
        time_step,
        pivot_steering_angle_rad,
    ):
        world_footprint = [transform_point(point, pose) for point in footprint]
        for point in scan_points:
            if point_in_pallet_exemption(point, pallet_exemption):
                continue
            if point_in_polygon_with_padding(point, world_footprint, collision_padding_m):
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

    world_points = [transform_point(point, pose) for point in footprint]
    max_cost = 0
    for index, start in enumerate(world_points):
        end = world_points[(index + 1) % len(world_points)]
        for x, y in sampled_segment_points(start, end, sample_spacing):
            cost = cost_at_world(costmap, x, y)
            if cost is None:
                return True, 'footprint collision: out of costmap'
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
) -> Tuple[bool, str]:
    for pose in predicted_poses_for_command(
        initial_pose,
        command,
        wheel_base,
        pivot_turn_radius,
        rear_axle_x_offset,
        horizon_sec,
        time_step_sec,
        pivot_steering_angle_rad,
    ):
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
            return True, reason
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
    gated.header = command.header
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
        self.declare_parameter('scan_protection_enabled', True)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('scan_timeout_sec', 0.4)
        self.declare_parameter('scan_required_range_m', 8.0)
        self.declare_parameter('scan_collision_sample_spacing_m', 0.05)
        self.declare_parameter('scan_collision_padding_m', 0.05)
        self.declare_parameter('scan_require_motion_fov_coverage', True)
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
        self._scan_protection_enabled = bool(
            self.get_parameter('scan_protection_enabled').value)
        self._scan_topic = str(self.get_parameter('scan_topic').value)
        self._scan_timeout_sec = self._positive_param('scan_timeout_sec', 0.4)
        self._scan_required_range_m = self._positive_param('scan_required_range_m', 8.0)
        self._scan_collision_sample_spacing_m = self._positive_param(
            'scan_collision_sample_spacing_m', 0.05)
        self._scan_collision_padding_m = max(
            0.0,
            float(self.get_parameter('scan_collision_padding_m').value),
        )
        self._scan_require_motion_fov_coverage = bool(
            self.get_parameter('scan_require_motion_fov_coverage').value)
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
        self._last_costmap: Optional[Any] = None
        self._last_costmap_time = self.get_clock().now()
        self._last_costmap_error = ''
        self._last_scan: Optional[LaserScan] = None
        self._last_scan_time = self.get_clock().now()
        self._last_reason = ''
        self._pallet_exemption_active = False
        self._last_pallet_exemption_time = self.get_clock().now()
        self._pallet_exemption_pose: Optional[PoseStamped] = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._command_pub = self.create_publisher(
            ForkliftControlCommand,
            self._gated_command_topic,
            10,
        )
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self.create_subscription(
            ForkliftControlCommand,
            self._raw_command_topic,
            self._on_raw_command,
            10,
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
                        10,
                    )
            else:
                self.create_subscription(
                    OccupancyGrid,
                    self._costmap_topic,
                    self._on_costmap,
                    10,
                )
        if self._scan_protection_enabled and self._scan_topic:
            self.create_subscription(LaserScan, self._scan_topic, self._on_scan, 10)
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
            f'scan={self._scan_topic if self._scan_protection_enabled else "disabled"}'
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
        self._last_raw_command = msg
        self._last_raw_command_time = self.get_clock().now()

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
        self._last_localization_time = self.get_clock().now()
        pose = msg.pose.pose
        self._last_pose = (
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        self._last_costmap_time = self.get_clock().now()
        self._last_costmap_error = costmap_error(msg)
        if not self._last_costmap_error:
            self._last_costmap = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg
        self._last_scan_time = self.get_clock().now()

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
        command, reason = self._latest_safe_command()
        self._command_pub.publish(command)
        self._publish_status(reason)
        self._log_reason(reason)

    def _latest_safe_command(self) -> Tuple[ForkliftControlCommand, str]:
        stamp = self.get_clock().now().to_msg()
        stop_reason = self._stop_reason_from_health()
        if stop_reason:
            return stop_command(stamp), stop_reason

        raw_age = (self.get_clock().now() - self._last_raw_command_time).nanoseconds / 1e9
        if self._last_raw_command is not None and raw_age <= self._command_timeout_sec:
            command = clamp_control_command(
                self._last_raw_command,
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
            collision_reason = self._collision_stop_reason(command)
            if collision_reason:
                return stop_command(stamp), collision_reason
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
            age = (now - self._last_costmap_time).nanoseconds / 1e9
            reason = costmap_stop_reason(
                self._costmap_monitor_enabled,
                age,
                self._last_costmap is not None,
                self._last_costmap_error,
                self._costmap_timeout_sec,
            )
            if reason:
                return reason

        return ''

    def _collision_stop_reason(self, command: ForkliftControlCommand) -> str:
        if not self._collision_check_enabled:
            return ''
        if not command.enable or command.brake or direction(command) == 0:
            return ''
        scan_reason = self._scan_collision_stop_reason(command)
        if scan_reason:
            return scan_reason
        if self._last_costmap is None:
            return ''
        if self._last_pose is None:
            return 'collision pose missing'

        pallet_exemption = self._active_pallet_exemption(
            command,
            str(getattr(self._last_costmap.header, 'frame_id', '')),
        )
        cost_threshold = self._footprint_collision_cost_threshold
        if pallet_exemption is not None:
            cost_threshold = max(
                cost_threshold,
                self._pallet_exemption_cost_threshold,
            )

        collision, reason = footprint_sweep_collision(
            self._last_costmap,
            self._footprint,
            self._last_pose,
            command,
            self._wheel_base,
            self._pivot_turn_radius,
            self._rear_axle_x_offset,
            self._dynamic_collision_horizon_sec(command),
            self._collision_check_time_step_sec,
            self._pivot_steering_angle_rad,
            self._footprint_sample_spacing,
            cost_threshold,
            self._unknown_is_collision,
            pallet_exemption,
        )
        return reason if collision else ''

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
        return max(vehicle_speed, abs(float(command.velocity_mps)))

    def _scan_collision_stop_reason(self, command: ForkliftControlCommand) -> str:
        now = self.get_clock().now()
        scan = self._last_scan
        age_sec = (now - self._last_scan_time).nanoseconds / 1e9
        reason = scan_stop_reason(
            self._scan_protection_enabled,
            age_sec,
            scan is not None,
            float(scan.range_max) if scan is not None else 0.0,
            self._scan_timeout_sec,
            self._scan_required_range_m,
        )
        if reason:
            return reason
        if scan is None:
            return ''

        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame_id,
                scan.header.frame_id,
                Time.from_msg(scan.header.stamp),
            )
        except Exception:
            return 'scan transform unavailable'

        scan_to_base_yaw = yaw_from_quaternion(transform.transform.rotation)
        travel_angle = 0.0 if direction(command) > 0 else math.pi
        if self._scan_require_motion_fov_coverage and not angle_in_scan_fov(
            travel_angle,
            scan,
            scan_to_base_yaw,
        ):
            return 'scan blind motion direction'

        pallet_exemption = self._active_pallet_exemption(command, self._base_frame_id)
        scan_points = laser_scan_points_in_base(
            scan,
            (
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
                scan_to_base_yaw,
            ),
        )
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
