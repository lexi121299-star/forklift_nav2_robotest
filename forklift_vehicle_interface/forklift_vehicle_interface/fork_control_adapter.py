from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional, Set, Tuple

import rclpy
from forklift_msgs.action import ForkMoveTo
from forklift_msgs.msg import ForkliftControlCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState


@dataclass
class ForkPose:
    height_m: float = 0.0
    side_shift_m: float = 0.0
    tilt_rad: float = 0.0


@dataclass
class ForkTarget:
    height_m: float
    side_shift_m: float
    tilt_rad: float


@dataclass
class ForkControlConfig:
    min_height_m: float = 0.0
    max_height_m: float = 3.0
    max_abs_side_shift_m: float = 0.12
    max_abs_tilt_rad: float = 0.20

    height_tolerance_m: float = 0.01
    side_shift_tolerance_m: float = 0.005
    tilt_tolerance_rad: float = 0.01

    height_slow_zone_m: float = 0.08
    side_shift_slow_zone_m: float = 0.03
    tilt_slow_zone_rad: float = 0.04

    lift_pump_rpm: float = 1500.0
    lower_pump_rpm: float = 0.0

    lift_valve_min_ma: float = 250.0
    lift_valve_max_ma: float = 800.0
    lower_valve_min_ma: float = 180.0
    lower_valve_max_ma: float = 650.0
    side_shift_valve_min_ma: float = 180.0
    side_shift_valve_max_ma: float = 600.0
    tilt_valve_min_ma: float = 180.0
    tilt_valve_max_ma: float = 600.0

    positive_side_shift_is_right: bool = True
    positive_tilt_uses_backward_valve: bool = True


@dataclass
class ForkControlStep:
    command: ForkliftControlCommand
    phase: str
    complete: bool
    message: str = ''


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def validate_target(target: ForkTarget, config: ForkControlConfig) -> Tuple[bool, str]:
    if not math.isfinite(target.height_m):
        return False, 'target height is not finite'
    if not math.isfinite(target.side_shift_m):
        return False, 'target side shift is not finite'
    if not math.isfinite(target.tilt_rad):
        return False, 'target tilt is not finite'
    if target.height_m < config.min_height_m or target.height_m > config.max_height_m:
        return False, 'target height exceeds configured limits'
    if abs(target.side_shift_m) > config.max_abs_side_shift_m:
        return False, 'target side shift exceeds configured limits'
    if abs(target.tilt_rad) > config.max_abs_tilt_rad:
        return False, 'target tilt exceeds configured limits'
    return True, ''


def zero_command() -> ForkliftControlCommand:
    command = ForkliftControlCommand()
    command.enable = True
    command.brake = True
    command.forward = False
    command.reverse = False
    command.velocity_mps = 0.0
    command.drive_rpm = 0.0
    command.steering_angle_rad = 0.0
    command.steering_angle_deg = 0.0
    command.pump_rpm = 0.0
    command.lift_valve_ma = 0.0
    command.lower_valve_ma = 0.0
    command.side_shift_left_valve_ma = 0.0
    command.side_shift_right_valve_ma = 0.0
    command.tilt_forward_valve_ma = 0.0
    command.tilt_backward_valve_ma = 0.0
    return command


def _scaled_current(
    error_abs: float,
    slow_zone: float,
    min_ma: float,
    max_ma: float,
) -> float:
    if max_ma <= 0.0:
        return 0.0
    if slow_zone <= 0.0:
        return max_ma
    ratio = clamp(error_abs / slow_zone, 0.0, 1.0)
    return clamp(min_ma + (max_ma - min_ma) * ratio, 0.0, max_ma)


def build_fork_control_step(
    target: ForkTarget,
    pose: ForkPose,
    config: ForkControlConfig,
) -> ForkControlStep:
    ok, message = validate_target(target, config)
    if not ok:
        return ForkControlStep(zero_command(), 'INVALID_TARGET', False, message)

    command = zero_command()
    height_error = target.height_m - pose.height_m
    if abs(height_error) > config.height_tolerance_m:
        if height_error > 0.0:
            command.pump_rpm = config.lift_pump_rpm
            command.lift_valve_ma = _scaled_current(
                abs(height_error),
                config.height_slow_zone_m,
                config.lift_valve_min_ma,
                config.lift_valve_max_ma,
            )
            return ForkControlStep(command, 'RAISE_HEIGHT', False)

        command.pump_rpm = config.lower_pump_rpm
        command.lower_valve_ma = _scaled_current(
            abs(height_error),
            config.height_slow_zone_m,
            config.lower_valve_min_ma,
            config.lower_valve_max_ma,
        )
        return ForkControlStep(command, 'LOWER_HEIGHT', False)

    side_error = target.side_shift_m - pose.side_shift_m
    if abs(side_error) > config.side_shift_tolerance_m:
        valve_ma = _scaled_current(
            abs(side_error),
            config.side_shift_slow_zone_m,
            config.side_shift_valve_min_ma,
            config.side_shift_valve_max_ma,
        )
        use_right = side_error > 0.0
        if not config.positive_side_shift_is_right:
            use_right = not use_right
        if use_right:
            command.side_shift_right_valve_ma = valve_ma
            return ForkControlStep(command, 'SIDE_SHIFT_RIGHT', False)
        command.side_shift_left_valve_ma = valve_ma
        return ForkControlStep(command, 'SIDE_SHIFT_LEFT', False)

    tilt_error = target.tilt_rad - pose.tilt_rad
    if abs(tilt_error) > config.tilt_tolerance_rad:
        valve_ma = _scaled_current(
            abs(tilt_error),
            config.tilt_slow_zone_rad,
            config.tilt_valve_min_ma,
            config.tilt_valve_max_ma,
        )
        use_backward = tilt_error > 0.0
        if not config.positive_tilt_uses_backward_valve:
            use_backward = not use_backward
        if use_backward:
            command.tilt_backward_valve_ma = valve_ma
            return ForkControlStep(command, 'TILT_BACKWARD', False)
        command.tilt_forward_valve_ma = valve_ma
        return ForkControlStep(command, 'TILT_FORWARD', False)

    return ForkControlStep(command, 'HOLD', True, 'fork target reached')


class ForkControlAdapter(Node):
    """Expose ForkMoveTo action and publish forklift hydraulic control commands."""

    def __init__(self) -> None:
        super().__init__('fork_control_adapter')
        self.declare_parameter('action_name', '/forklift/fork/move_to')
        self.declare_parameter('command_topic', '/forklift/control_cmd_raw')
        self.declare_parameter('fork_state_topic', '/forklift/fork/joint_state')
        self.declare_parameter('command_rate_hz', 20.0)
        self.declare_parameter('feedback_timeout_sec', 0.5)
        self.declare_parameter('default_action_timeout_sec', 8.0)
        self.declare_parameter('height_joint_name', 'fork_height_m')
        self.declare_parameter('side_shift_joint_name', 'fork_side_shift_m')
        self.declare_parameter('tilt_joint_name', 'fork_tilt_rad')
        self.declare_parameter('require_height_feedback', True)
        self.declare_parameter('require_side_shift_feedback', False)
        self.declare_parameter('require_tilt_feedback', False)

        self._config = self._read_config()
        self._command_rate_hz = self._positive_param('command_rate_hz', 20.0)
        self._feedback_timeout_sec = self._positive_param('feedback_timeout_sec', 0.5)
        self._default_action_timeout_sec = self._positive_param(
            'default_action_timeout_sec', 8.0
        )
        self._height_joint_name = str(self.get_parameter('height_joint_name').value)
        self._side_shift_joint_name = str(
            self.get_parameter('side_shift_joint_name').value
        )
        self._tilt_joint_name = str(self.get_parameter('tilt_joint_name').value)
        self._require_height_feedback = bool(
            self.get_parameter('require_height_feedback').value
        )
        self._require_side_shift_feedback = bool(
            self.get_parameter('require_side_shift_feedback').value
        )
        self._require_tilt_feedback = bool(
            self.get_parameter('require_tilt_feedback').value
        )

        self._pose = ForkPose()
        self._pose_fields: Set[str] = set()
        self._last_pose_time = self.get_clock().now()
        self._lock = threading.Lock()
        self._active_goal = False
        self._active_goal_lock = threading.Lock()

        command_topic = str(self.get_parameter('command_topic').value)
        fork_state_topic = str(self.get_parameter('fork_state_topic').value)
        action_name = str(self.get_parameter('action_name').value)
        self._command_pub = self.create_publisher(ForkliftControlCommand, command_topic, 10)
        self.create_subscription(JointState, fork_state_topic, self._on_joint_state, 10)
        self._action_server = ActionServer(
            self,
            ForkMoveTo,
            action_name,
            execute_callback=self._execute_move_to,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.get_logger().info(
            'fork_control_adapter ready: action={} command_topic={} state_topic={}'.format(
                action_name, command_topic, fork_state_topic
            )
        )

    def _read_config(self) -> ForkControlConfig:
        config = ForkControlConfig()
        for name, value in config.__dict__.items():
            self.declare_parameter(name, value)
        return ForkControlConfig(
            min_height_m=float(self.get_parameter('min_height_m').value),
            max_height_m=float(self.get_parameter('max_height_m').value),
            max_abs_side_shift_m=float(
                self.get_parameter('max_abs_side_shift_m').value
            ),
            max_abs_tilt_rad=float(self.get_parameter('max_abs_tilt_rad').value),
            height_tolerance_m=float(self.get_parameter('height_tolerance_m').value),
            side_shift_tolerance_m=float(
                self.get_parameter('side_shift_tolerance_m').value
            ),
            tilt_tolerance_rad=float(self.get_parameter('tilt_tolerance_rad').value),
            height_slow_zone_m=float(self.get_parameter('height_slow_zone_m').value),
            side_shift_slow_zone_m=float(
                self.get_parameter('side_shift_slow_zone_m').value
            ),
            tilt_slow_zone_rad=float(self.get_parameter('tilt_slow_zone_rad').value),
            lift_pump_rpm=float(self.get_parameter('lift_pump_rpm').value),
            lower_pump_rpm=float(self.get_parameter('lower_pump_rpm').value),
            lift_valve_min_ma=float(self.get_parameter('lift_valve_min_ma').value),
            lift_valve_max_ma=float(self.get_parameter('lift_valve_max_ma').value),
            lower_valve_min_ma=float(self.get_parameter('lower_valve_min_ma').value),
            lower_valve_max_ma=float(self.get_parameter('lower_valve_max_ma').value),
            side_shift_valve_min_ma=float(
                self.get_parameter('side_shift_valve_min_ma').value
            ),
            side_shift_valve_max_ma=float(
                self.get_parameter('side_shift_valve_max_ma').value
            ),
            tilt_valve_min_ma=float(self.get_parameter('tilt_valve_min_ma').value),
            tilt_valve_max_ma=float(self.get_parameter('tilt_valve_max_ma').value),
            positive_side_shift_is_right=bool(
                self.get_parameter('positive_side_shift_is_right').value
            ),
            positive_tilt_uses_backward_valve=bool(
                self.get_parameter('positive_tilt_uses_backward_valve').value
            ),
        )

    def _positive_param(self, name: str, fallback: float) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            self.get_logger().warning(
                'Parameter {} must be positive; using fallback {}.'.format(name, fallback)
            )
            return fallback
        return value

    def _goal_callback(self, goal_request) -> GoalResponse:
        target = ForkTarget(
            height_m=float(goal_request.target_height_m),
            side_shift_m=float(goal_request.side_shift_m),
            tilt_rad=float(goal_request.tilt_rad),
        )
        ok, message = validate_target(target, self._config)
        if not ok:
            self.get_logger().warning('Rejecting ForkMoveTo goal: {}'.format(message))
            return GoalResponse.REJECT
        with self._active_goal_lock:
            if self._active_goal:
                self.get_logger().warning('Rejecting ForkMoveTo goal: another goal is active')
                return GoalResponse.REJECT
            self._active_goal = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        self._publish_command(zero_command())
        return CancelResponse.ACCEPT

    def _on_joint_state(self, msg: JointState) -> None:
        with self._lock:
            names = list(msg.name)
            positions = list(msg.position)
            if self._height_joint_name in names:
                index = names.index(self._height_joint_name)
                if index < len(positions):
                    self._pose.height_m = float(positions[index])
                    self._pose_fields.add('height')
            if self._side_shift_joint_name in names:
                index = names.index(self._side_shift_joint_name)
                if index < len(positions):
                    self._pose.side_shift_m = float(positions[index])
                    self._pose_fields.add('side_shift')
            if self._tilt_joint_name in names:
                index = names.index(self._tilt_joint_name)
                if index < len(positions):
                    self._pose.tilt_rad = float(positions[index])
                    self._pose_fields.add('tilt')
            self._last_pose_time = self.get_clock().now()

    def _execute_move_to(self, goal_handle):
        try:
            return self._run_move_to(goal_handle)
        finally:
            with self._active_goal_lock:
                self._active_goal = False

    def _run_move_to(self, goal_handle):
        target = ForkTarget(
            height_m=float(goal_handle.request.target_height_m),
            side_shift_m=float(goal_handle.request.side_shift_m),
            tilt_rad=float(goal_handle.request.tilt_rad),
        )
        start_time = time.monotonic()
        result = ForkMoveTo.Result()
        sleep_sec = 1.0 / self._command_rate_hz

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._publish_command(zero_command())
                goal_handle.canceled()
                result.success = False
                result.message = 'fork move canceled'
                result.final_height_m = self._latest_pose()[0].height_m
                return result

            if time.monotonic() - start_time > self._default_action_timeout_sec:
                self._publish_command(zero_command())
                pose, _fields, _age = self._latest_pose()
                goal_handle.abort()
                result.success = False
                result.message = 'fork move timeout'
                result.final_height_m = pose.height_m
                return result

            ready, message = self._feedback_ready(target)
            pose, _fields, _age = self._latest_pose()
            if not ready:
                self._publish_command(zero_command())
                goal_handle.abort()
                result.success = False
                result.message = message
                result.final_height_m = pose.height_m
                return result

            step = build_fork_control_step(target, pose, self._config)
            self._publish_command(step.command)
            feedback = ForkMoveTo.Feedback()
            feedback.current_height_m = pose.height_m
            feedback.phase = step.phase
            goal_handle.publish_feedback(feedback)

            if step.message and step.phase == 'INVALID_TARGET':
                self._publish_command(zero_command())
                goal_handle.abort()
                result.success = False
                result.message = step.message
                result.final_height_m = pose.height_m
                return result

            if step.complete:
                self._publish_command(zero_command())
                goal_handle.succeed()
                result.success = True
                result.message = step.message
                result.final_height_m = pose.height_m
                return result

            time.sleep(sleep_sec)

        self._publish_command(zero_command())
        goal_handle.abort()
        result.success = False
        result.message = 'rclpy shutdown'
        result.final_height_m = self._latest_pose()[0].height_m
        return result

    def _latest_pose(self) -> Tuple[ForkPose, Set[str], float]:
        with self._lock:
            pose = ForkPose(
                height_m=self._pose.height_m,
                side_shift_m=self._pose.side_shift_m,
                tilt_rad=self._pose.tilt_rad,
            )
            fields = set(self._pose_fields)
            age = (self.get_clock().now() - self._last_pose_time).nanoseconds / 1e9
        return pose, fields, age

    def _feedback_ready(self, target: ForkTarget) -> Tuple[bool, str]:
        _pose, fields, age = self._latest_pose()
        if age > self._feedback_timeout_sec:
            return False, 'fork feedback timeout'
        if self._require_height_feedback and 'height' not in fields:
            return False, 'fork height feedback missing'
        side_needed = abs(target.side_shift_m) > self._config.side_shift_tolerance_m
        if (self._require_side_shift_feedback or side_needed) and 'side_shift' not in fields:
            return False, 'fork side shift feedback missing'
        tilt_needed = abs(target.tilt_rad) > self._config.tilt_tolerance_rad
        if (self._require_tilt_feedback or tilt_needed) and 'tilt' not in fields:
            return False, 'fork tilt feedback missing'
        return True, ''

    def _publish_command(self, command: ForkliftControlCommand) -> None:
        command.header.stamp = self.get_clock().now().to_msg()
        self._command_pub.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ForkControlAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
