"""Low-speed straight relative-motion action for final pallet approach."""

import math
import threading
import time
from typing import Optional, Tuple

import rclpy
from forklift_msgs.action import MoveRelative
from forklift_msgs.msg import ForkliftControlCommand, ForkliftVehicleState
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


def yaw_from_quaternion(q) -> float:
    """Return planar yaw from a quaternion."""

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def shortest_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""

    return math.atan2(math.sin(angle), math.cos(angle))


def motion_command(
    velocity_mps: float,
    accel_time_sec: float,
    decel_time_sec: float,
) -> ForkliftControlCommand:
    """Build a straight command suitable for the upstream safety gate."""

    command = ForkliftControlCommand()
    command.enable = abs(velocity_mps) > 1e-6
    command.brake = not command.enable
    command.forward = velocity_mps > 0.0
    command.reverse = velocity_mps < 0.0
    command.velocity_mps = float(velocity_mps)
    command.steering_angle_rad = 0.0
    command.steering_angle_deg = 0.0
    command.accel_time_sec = float(accel_time_sec)
    command.decel_time_sec = float(decel_time_sec)
    return command


class FineMotionAdapter(Node):
    """Execute short straight motions using odometry feedback."""

    def __init__(self) -> None:
        super().__init__('fine_motion_adapter')
        self.declare_parameter('action_name', '/forklift/fine_motion/move_relative')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('vehicle_state_topic', '/forklift/vehicle_state')
        self.declare_parameter('command_topic', '/forklift/control_cmd_raw')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('vehicle_state_timeout_sec', 0.5)
        self.declare_parameter('progress_timeout_sec', 3.0)
        self.declare_parameter('default_goal_timeout_sec', 20.0)
        self.declare_parameter('max_speed_mps', 0.15)
        self.declare_parameter('distance_tolerance_m', 0.02)
        self.declare_parameter('progress_epsilon_m', 0.005)
        self.declare_parameter('max_lateral_deviation_m', 0.10)
        self.declare_parameter('max_heading_change_rad', 0.15)
        self.declare_parameter('max_start_steering_angle_rad', 0.10)
        self.declare_parameter('steering_center_timeout_sec', 3.0)
        self.declare_parameter('accel_time_sec', 1.0)
        self.declare_parameter('decel_time_sec', 1.0)

        self._control_rate_hz = self._positive('control_rate_hz')
        self._odom_timeout_sec = self._positive('odom_timeout_sec')
        self._vehicle_state_timeout_sec = self._positive(
            'vehicle_state_timeout_sec'
        )
        self._progress_timeout_sec = self._positive('progress_timeout_sec')
        self._default_goal_timeout_sec = self._positive(
            'default_goal_timeout_sec'
        )
        self._max_speed_mps = self._positive('max_speed_mps')
        self._distance_tolerance_m = self._positive('distance_tolerance_m')
        self._progress_epsilon_m = self._positive('progress_epsilon_m')
        self._max_lateral_deviation_m = self._positive(
            'max_lateral_deviation_m'
        )
        self._max_heading_change_rad = self._positive(
            'max_heading_change_rad'
        )
        self._max_start_steering_angle_rad = self._positive(
            'max_start_steering_angle_rad'
        )
        self._steering_center_timeout_sec = self._positive(
            'steering_center_timeout_sec'
        )
        self._accel_time_sec = self._positive('accel_time_sec')
        self._decel_time_sec = self._positive('decel_time_sec')

        self._lock = threading.Lock()
        self._active_goal = False
        self._latest_odom: Optional[Tuple[float, float, float, float]] = None
        self._latest_vehicle_state: Optional[Tuple[ForkliftVehicleState, float]] = None
        callback_group = ReentrantCallbackGroup()

        command_topic = str(self.get_parameter('command_topic').value)
        self._command_pub = self.create_publisher(
            ForkliftControlCommand, command_topic, 10
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._on_odom,
            10,
            callback_group=callback_group,
        )
        self.create_subscription(
            ForkliftVehicleState,
            str(self.get_parameter('vehicle_state_topic').value),
            self._on_vehicle_state,
            10,
            callback_group=callback_group,
        )
        action_name = str(self.get_parameter('action_name').value)
        self._action_server = ActionServer(
            self,
            MoveRelative,
            action_name,
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )
        self.get_logger().info(
            'fine_motion_adapter ready: action={} command_topic={}'.format(
                action_name, command_topic
            )
        )

    def _positive(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError('{} must be a positive finite value'.format(name))
        return value

    def _goal_callback(self, request: MoveRelative.Goal) -> GoalResponse:
        distance = float(request.distance_m)
        speed = float(request.max_speed_mps)
        if not math.isfinite(distance) or abs(distance) <= self._distance_tolerance_m:
            self.get_logger().warning('Rejecting MoveRelative: invalid distance')
            return GoalResponse.REJECT
        if not math.isfinite(speed) or speed <= 0.0:
            self.get_logger().warning('Rejecting MoveRelative: invalid speed')
            return GoalResponse.REJECT
        with self._lock:
            if self._active_goal:
                self.get_logger().warning('Rejecting MoveRelative: goal already active')
                return GoalResponse.REJECT
            self._active_goal = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        self._publish_stop()
        return CancelResponse.ACCEPT

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        with self._lock:
            self._latest_odom = (
                float(pose.position.x),
                float(pose.position.y),
                yaw_from_quaternion(pose.orientation),
                time.monotonic(),
            )

    def _on_vehicle_state(self, msg: ForkliftVehicleState) -> None:
        with self._lock:
            self._latest_vehicle_state = (msg, time.monotonic())

    def _snapshot(self):
        with self._lock:
            odom = self._latest_odom
            vehicle_state = self._latest_vehicle_state
        return odom, vehicle_state

    def _execute(self, goal_handle):
        try:
            return self._run(goal_handle)
        finally:
            self._publish_stop()
            with self._lock:
                self._active_goal = False

    def _run(self, goal_handle):
        result = MoveRelative.Result()
        distance = float(goal_handle.request.distance_m)
        direction = 1.0 if distance > 0.0 else -1.0
        target_distance = abs(distance)
        speed = min(abs(float(goal_handle.request.max_speed_mps)), self._max_speed_mps)
        start_time = time.monotonic()

        odom, vehicle_state = self._snapshot()
        readiness_error = self._readiness_error(
            odom, vehicle_state, start_time, require_centered=False
        )
        if readiness_error:
            return self._abort(goal_handle, result, readiness_error)
        center_deadline = start_time + self._steering_center_timeout_sec
        while abs(vehicle_state[0].steering_angle_rad) > (
            self._max_start_steering_angle_rad
        ):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'relative motion canceled'
                return result
            self._publish_stop()
            time.sleep(1.0 / self._control_rate_hz)
            now = time.monotonic()
            odom, vehicle_state = self._snapshot()
            readiness_error = self._readiness_error(
                odom, vehicle_state, now, require_centered=False
            )
            if readiness_error:
                return self._abort(goal_handle, result, readiness_error)
            if now >= center_deadline:
                return self._abort(
                    goal_handle,
                    result,
                    'steering did not center before relative motion',
                )

        start_time = time.monotonic()
        odom, _vehicle_state = self._snapshot()
        start_x, start_y, start_yaw, _stamp = odom

        last_progress = 0.0
        last_progress_time = start_time
        sleep_sec = 1.0 / self._control_rate_hz
        while rclpy.ok():
            now = time.monotonic()
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'relative motion canceled'
                return result
            if now - start_time > self._default_goal_timeout_sec:
                return self._abort(goal_handle, result, 'relative motion timeout')

            odom, vehicle_state = self._snapshot()
            readiness_error = self._readiness_error(odom, vehicle_state, now)
            if readiness_error:
                return self._abort(goal_handle, result, readiness_error)

            x, y, yaw, _stamp = odom
            dx = x - start_x
            dy = y - start_y
            longitudinal = dx * math.cos(start_yaw) + dy * math.sin(start_yaw)
            lateral = -dx * math.sin(start_yaw) + dy * math.cos(start_yaw)
            progress = direction * longitudinal
            remaining = max(0.0, target_distance - progress)

            if abs(lateral) > self._max_lateral_deviation_m:
                return self._abort(
                    goal_handle, result, 'relative motion lateral deviation exceeded'
                )
            if abs(shortest_angle(yaw - start_yaw)) > self._max_heading_change_rad:
                return self._abort(
                    goal_handle, result, 'relative motion heading deviation exceeded'
                )
            if progress < -self._distance_tolerance_m:
                return self._abort(
                    goal_handle, result, 'relative motion moved in wrong direction'
                )
            if remaining <= self._distance_tolerance_m:
                goal_handle.succeed()
                result.success = True
                result.message = 'relative motion completed'
                return result

            if progress >= last_progress + self._progress_epsilon_m:
                last_progress = progress
                last_progress_time = now
            elif now - last_progress_time > self._progress_timeout_sec:
                return self._abort(
                    goal_handle, result, 'relative motion made no progress'
                )

            command = motion_command(
                direction * speed,
                self._accel_time_sec,
                self._decel_time_sec,
            )
            command.header.stamp = self.get_clock().now().to_msg()
            self._command_pub.publish(command)
            feedback = MoveRelative.Feedback()
            feedback.remaining_distance_m = remaining
            goal_handle.publish_feedback(feedback)
            time.sleep(sleep_sec)

        return self._abort(goal_handle, result, 'rclpy shutdown')

    def _readiness_error(
        self,
        odom,
        vehicle_state,
        now: float,
        require_centered: bool = True,
    ) -> str:
        if odom is None:
            return 'odometry missing'
        if now - odom[3] > self._odom_timeout_sec:
            return 'odometry timeout'
        if vehicle_state is None:
            return 'vehicle state missing'
        state, state_stamp = vehicle_state
        if now - state_stamp > self._vehicle_state_timeout_sec:
            return 'vehicle state timeout'
        if not state.auto_mode:
            return 'vehicle is not in auto mode'
        if state.emergency_stopped or state.soft_emergency_stop:
            return 'vehicle emergency stop'
        if (
            require_centered
            and abs(state.steering_angle_rad) > self._max_start_steering_angle_rad
        ):
            return 'steering is not centered for straight relative motion'
        return ''

    @staticmethod
    def _abort(goal_handle, result, message: str):
        goal_handle.abort()
        result.success = False
        result.message = message
        return result

    def _publish_stop(self) -> None:
        command = motion_command(0.0, self._accel_time_sec, self._decel_time_sec)
        command.header.stamp = self.get_clock().now().to_msg()
        self._command_pub.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FineMotionAdapter()
    executor = MultiThreadedExecutor(num_threads=3)
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
