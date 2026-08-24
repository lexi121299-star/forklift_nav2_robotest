"""ROS 2 node exposing the minimal P7.1 forklift task interface."""

import math
from typing import Callable, List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from forklift_msgs.action import (
    DetectPalletOffset,
    ExecutePalletPickup,
    ExecuteRoute,
    ForkMoveTo,
    MoveRelative,
)
from forklift_msgs.msg import TaskStatus
from forklift_msgs.srv import GoToStation
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from .pallet_approach import (
    PalletApproachConfig,
    PalletApproachError,
    build_pallet_approach,
    build_pallet_approach_route,
    geometry_pose_targets,
    pallet_exemption_active_for_distance,
)
from .route_model import (
    PoseTarget,
    RelativeMoveTarget,
    RouteConfigError,
    RouteDefinition,
)
from .route_model import load_routes, load_stations
from .pallet_model import PalletConfigError, load_pallet_pickup_config
from .pallet_pickup_state_machine import PalletPickupStateMachine
from .state_machine import FAILED, IDLE, PAUSED, RECOVERING, RUNNING, SUCCEEDED
from .state_machine import TaskStateMachine


class Nav2Navigator:
    """Small adapter around the Nav2 NavigateToPose action client."""

    def __init__(self, node: Node, action_name: str) -> None:
        self._node = node
        self._client = ActionClient(node, NavigateToPose, action_name)
        self._goal_handle = None
        self._request_token = 0

    def send_goal(
        self,
        target: PoseTarget,
        done_callback: Callable[[bool, str], None],
    ) -> None:
        self._request_token += 1
        request_token = self._request_token
        if not self._client.wait_for_server(timeout_sec=0.0):
            done_callback(False, 'NavigateToPose action server unavailable')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = target.frame_id
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = target.x
        goal.pose.pose.position.y = target.y
        goal.pose.pose.orientation.z = math.sin(target.yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(target.yaw * 0.5)

        send_future = self._client.send_goal_async(goal)

        def goal_response(future) -> None:
            try:
                goal_handle = future.result()
            except Exception as exc:
                done_callback(False, 'NavigateToPose send failed: {}'.format(exc))
                return
            if not goal_handle.accepted:
                done_callback(False, 'NavigateToPose goal rejected')
                return
            if request_token != self._request_token:
                goal_handle.cancel_goal_async()
                return
            self._goal_handle = goal_handle
            result_future = goal_handle.get_result_async()

            def result_response(result_done) -> None:
                try:
                    wrapped_result = result_done.result()
                except Exception as exc:
                    done_callback(False, 'NavigateToPose result failed: {}'.format(exc))
                    return
                self._goal_handle = None
                success = wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
                message = (
                    'NavigateToPose succeeded'
                    if success else
                    'NavigateToPose ended with status {}'.format(wrapped_result.status)
                )
                done_callback(success, message)

            result_future.add_done_callback(result_response)

        send_future.add_done_callback(goal_response)

    def cancel_goal(self) -> None:
        self._request_token += 1
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None


class PickupDeviceActions:
    """Action-client adapter for fork, perception, and low-speed motion devices."""

    def __init__(
        self,
        node: Node,
        fork_action_name: str,
        detect_action_name: str,
        move_relative_action_name: str,
    ) -> None:
        self._node = node
        self._fork_client = ActionClient(node, ForkMoveTo, fork_action_name)
        self._detect_client = ActionClient(node, DetectPalletOffset, detect_action_name)
        self._move_relative_client = ActionClient(
            node, MoveRelative, move_relative_action_name
        )
        self._goal_handles = []
        self._timers = []
        self._request_token = 0

    def check_ready(self):
        return True, 'ready'

    def move_fork(
        self,
        *,
        target_height_m: float,
        side_shift_m: float,
        tilt_rad: float,
        timeout_sec: float,
        done_callback,
        feedback_callback=None,
    ) -> None:
        goal = ForkMoveTo.Goal()
        goal.target_height_m = float(target_height_m)
        goal.side_shift_m = float(side_shift_m)
        goal.tilt_rad = float(tilt_rad)

        def feedback(response) -> None:
            if feedback_callback is not None:
                feedback_callback(response.feedback.current_height_m)

        self._send_goal(
            self._fork_client,
            goal,
            'ForkMoveTo',
            timeout_sec,
            done_callback,
            feedback_callback=feedback,
        )

    def detect_offset(
        self,
        *,
        slot_id: str,
        scan_height_m: float,
        timeout_sec: float,
        done_callback,
    ) -> None:
        goal = DetectPalletOffset.Goal()
        goal.slot_id = slot_id
        goal.scan_height_m = float(scan_height_m)
        self._send_goal(
            self._detect_client,
            goal,
            'DetectPalletOffset',
            timeout_sec,
            done_callback,
        )

    def move_relative(
        self,
        *,
        distance_m: float,
        max_speed_mps: float,
        timeout_sec: float,
        done_callback,
    ) -> None:
        goal = MoveRelative.Goal()
        goal.distance_m = float(distance_m)
        goal.max_speed_mps = float(max_speed_mps)
        self._send_goal(
            self._move_relative_client,
            goal,
            'MoveRelative',
            timeout_sec,
            done_callback,
        )

    def cancel_all(self) -> None:
        self._request_token += 1
        for goal_handle in self._goal_handles:
            goal_handle.cancel_goal_async()
        self._goal_handles = []
        for timer in self._timers:
            self._node.destroy_timer(timer)
        self._timers = []

    def _send_goal(
        self,
        client,
        goal,
        action_label: str,
        timeout_sec: float,
        done_callback,
        feedback_callback=None,
    ) -> None:
        self._request_token += 1
        request_token = self._request_token
        if not client.wait_for_server(timeout_sec=0.0):
            done_callback(False, '{} action server unavailable'.format(action_label))
            return

        state = {'done': False, 'goal_handle': None, 'timer': None}

        def cleanup() -> None:
            timer = state['timer']
            if timer is not None:
                self._node.destroy_timer(timer)
                if timer in self._timers:
                    self._timers.remove(timer)
                state['timer'] = None
            goal_handle = state['goal_handle']
            if goal_handle in self._goal_handles:
                self._goal_handles.remove(goal_handle)

        def finish(success: bool, message: str, result=None) -> None:
            if state['done']:
                return
            state['done'] = True
            cleanup()
            if request_token != self._request_token:
                return
            done_callback(success, message, result)

        def timeout() -> None:
            goal_handle = state['goal_handle']
            if goal_handle is not None:
                goal_handle.cancel_goal_async()
            finish(False, '{} timed out'.format(action_label))

        if timeout_sec > 0.0:
            state['timer'] = self._node.create_timer(float(timeout_sec), timeout)
            self._timers.append(state['timer'])

        send_future = client.send_goal_async(
            goal,
            feedback_callback=feedback_callback,
        )

        def goal_response(future) -> None:
            try:
                goal_handle = future.result()
            except Exception as exc:
                finish(False, '{} send failed: {}'.format(action_label, exc))
                return
            if request_token != self._request_token:
                goal_handle.cancel_goal_async()
                return
            if not goal_handle.accepted:
                finish(False, '{} goal rejected'.format(action_label))
                return
            state['goal_handle'] = goal_handle
            self._goal_handles.append(goal_handle)
            result_future = goal_handle.get_result_async()

            def result_response(result_done) -> None:
                try:
                    wrapped_result = result_done.result()
                except Exception as exc:
                    finish(False, '{} result failed: {}'.format(action_label, exc))
                    return
                success = wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
                result = wrapped_result.result
                message = getattr(
                    result,
                    'message',
                    '{} succeeded'.format(action_label)
                    if success else '{} failed'.format(action_label),
                )
                action_success = success and bool(getattr(result, 'success', success))
                finish(action_success, message, result)

            result_future.add_done_callback(result_response)

        send_future.add_done_callback(goal_response)


class TaskMotionAdapter:
    """Dispatch route segments to Nav2 or the low-speed motion adapter."""

    def __init__(
        self,
        node: Node,
        navigator: Nav2Navigator,
        devices: PickupDeviceActions,
        base_frame_id: str,
        pallet_exemption_callback: Callable[[Optional[PoseTarget]], None],
    ) -> None:
        self._node = node
        self._navigator = navigator
        self._devices = devices
        self._base_frame_id = base_frame_id
        self._pallet_exemption_callback = pallet_exemption_callback
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

    def send_goal(self, target, done_callback) -> None:
        if isinstance(target, RelativeMoveTarget):
            ready, message = self._relative_start_is_valid(target)
            if not ready:
                done_callback(False, message)
                return
            exemption_pose = self._pallet_exemption_pose(target)
            self._pallet_exemption_callback(exemption_pose)

            def relative_done(success, message, _result) -> None:
                self._pallet_exemption_callback(None)
                done_callback(success, message)

            self._devices.move_relative(
                distance_m=target.distance_m,
                max_speed_mps=target.max_speed_mps,
                timeout_sec=target.timeout_sec,
                done_callback=relative_done,
            )
            return
        self._navigator.send_goal(target, done_callback)

    def cancel_goal(self) -> None:
        self._navigator.cancel_goal()
        self._devices.cancel_all()
        self._pallet_exemption_callback(None)

    def planar_distance_to(self, target: PoseTarget) -> float:
        transform = self._tf_buffer.lookup_transform(
            target.frame_id,
            self._base_frame_id,
            Time(),
        )
        translation = transform.transform.translation
        return math.hypot(
            translation.x - target.x,
            translation.y - target.y,
        )

    @staticmethod
    def _pallet_exemption_pose(
        target: RelativeMoveTarget,
    ) -> Optional[PoseTarget]:
        if (
            target.pallet_exemption_x is None
            or target.pallet_exemption_y is None
            or target.pallet_exemption_yaw is None
        ):
            return None
        return PoseTarget(
            name='pallet_exemption',
            x=target.pallet_exemption_x,
            y=target.pallet_exemption_y,
            yaw=target.pallet_exemption_yaw,
            frame_id=target.frame_id,
        )

    def _relative_start_is_valid(self, target: RelativeMoveTarget):
        if (
            target.expected_start_x is None
            or target.expected_start_y is None
            or target.expected_start_yaw is None
        ):
            return True, ''
        try:
            transform = self._tf_buffer.lookup_transform(
                target.frame_id,
                self._base_frame_id,
                Time(),
            )
        except Exception as exc:
            return False, 'relative motion start TF unavailable: {}'.format(exc)

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        position_error = math.hypot(
            translation.x - target.expected_start_x,
            translation.y - target.expected_start_y,
        )
        heading_error = abs(math.atan2(
            math.sin(yaw - target.expected_start_yaw),
            math.cos(yaw - target.expected_start_yaw),
        ))
        if position_error > target.max_start_position_error_m:
            return False, (
                'relative motion start position error {:.3f} m exceeds {:.3f} m'
            ).format(position_error, target.max_start_position_error_m)
        if heading_error > target.max_start_heading_error_rad:
            return False, (
                'relative motion start heading error {:.3f} rad exceeds {:.3f} rad'
            ).format(heading_error, target.max_start_heading_error_rad)
        return True, ''


class ForkliftTaskManager(Node):
    """Headless route sequencer that delegates every motion segment to Nav2."""

    def __init__(self) -> None:
        super().__init__('forklift_task_manager')

        package_share = get_package_share_directory('forklift_task_manager')
        self.declare_parameter(
            'stations_file', package_share + '/config/stations.yaml'
        )
        self.declare_parameter(
            'routes_file', package_share + '/config/routes.yaml'
        )
        self.declare_parameter('max_retries', 1)
        self.declare_parameter('navigate_to_pose_action', 'navigate_to_pose')
        self.declare_parameter(
            'pallet_slots_file', package_share + '/config/pallet_slots.yaml'
        )
        self.declare_parameter('fork_move_to_action', '/forklift/fork/move_to')
        self.declare_parameter(
            'detect_pallet_offset_action',
            '/forklift/perception/detect_pallet_offset',
        )
        self.declare_parameter(
            'move_relative_action',
            '/forklift/fine_motion/move_relative',
        )
        self.declare_parameter('enforce_pallet_approach_station', True)
        self.declare_parameter('rviz_goal_mode', 'navigation')
        self.declare_parameter('pallet_standoff_distance_m', 1.80)
        self.declare_parameter('pallet_final_approach_distance_m', 1.00)
        self.declare_parameter('pallet_alignment_runup_distance_m', 0.60)
        self.declare_parameter('pallet_final_approach_speed_mps', 0.10)
        self.declare_parameter('pallet_final_approach_timeout_sec', 20.0)
        self.declare_parameter('pallet_max_start_position_error_m', 0.35)
        self.declare_parameter('pallet_max_start_heading_error_rad', 0.20)
        self.declare_parameter('pallet_forks_on_negative_x', True)
        self.declare_parameter('pallet_arrow_points_outward', True)
        self.declare_parameter('pallet_base_frame_id', 'base_link')
        self.declare_parameter('pallet_exemption_activation_distance_m', 4.0)
        self.declare_parameter('pallet_exemption_deactivation_distance_m', 4.5)

        stations_file = self.get_parameter('stations_file').value
        routes_file = self.get_parameter('routes_file').value
        max_retries = int(self.get_parameter('max_retries').value)
        action_name = self.get_parameter('navigate_to_pose_action').value
        pallet_slots_file = self.get_parameter('pallet_slots_file').value
        fork_action_name = self.get_parameter('fork_move_to_action').value
        detect_action_name = self.get_parameter('detect_pallet_offset_action').value
        move_relative_action_name = self.get_parameter('move_relative_action').value
        self._enforce_pallet_approach_station = bool(
            self.get_parameter('enforce_pallet_approach_station').value
        )
        self._rviz_goal_mode = str(
            self.get_parameter('rviz_goal_mode').value
        ).strip().lower()
        if self._rviz_goal_mode not in {'navigation', 'pallet_approach'}:
            raise ValueError(
                'rviz_goal_mode must be navigation or pallet_approach'
            )
        self._pallet_exemption_activation_distance_m = float(
            self.get_parameter('pallet_exemption_activation_distance_m').value
        )
        self._pallet_exemption_deactivation_distance_m = float(
            self.get_parameter('pallet_exemption_deactivation_distance_m').value
        )
        if (
            not math.isfinite(self._pallet_exemption_activation_distance_m)
            or self._pallet_exemption_activation_distance_m <= 0.0
        ):
            raise ValueError(
                'pallet_exemption_activation_distance_m must be positive'
            )
        if (
            not math.isfinite(self._pallet_exemption_deactivation_distance_m)
            or self._pallet_exemption_deactivation_distance_m
            <= self._pallet_exemption_activation_distance_m
        ):
            raise ValueError(
                'pallet_exemption_deactivation_distance_m must be greater than '
                'pallet_exemption_activation_distance_m'
            )
        self._pallet_approach_config = PalletApproachConfig(
            standoff_distance_m=float(
                self.get_parameter('pallet_standoff_distance_m').value
            ),
            final_approach_distance_m=float(
                self.get_parameter('pallet_final_approach_distance_m').value
            ),
            alignment_runup_distance_m=float(
                self.get_parameter('pallet_alignment_runup_distance_m').value
            ),
            final_approach_speed_mps=float(
                self.get_parameter('pallet_final_approach_speed_mps').value
            ),
            final_approach_timeout_sec=float(
                self.get_parameter('pallet_final_approach_timeout_sec').value
            ),
            max_start_position_error_m=float(
                self.get_parameter('pallet_max_start_position_error_m').value
            ),
            max_start_heading_error_rad=float(
                self.get_parameter('pallet_max_start_heading_error_rad').value
            ),
            forks_on_negative_x=bool(
                self.get_parameter('pallet_forks_on_negative_x').value
            ),
            arrow_points_outward=bool(
                self.get_parameter('pallet_arrow_points_outward').value
            ),
        )

        try:
            self._stations = load_stations(stations_file)
            self._routes = load_routes(routes_file, self._stations)
        except RouteConfigError as exc:
            self.get_logger().fatal('Task configuration is invalid: {}'.format(exc))
            raise
        try:
            self._pallet_config = load_pallet_pickup_config(pallet_slots_file)
        except PalletConfigError as exc:
            self.get_logger().fatal(
                'Pallet pickup configuration is invalid: {}'.format(exc)
            )
            raise

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_pub = self.create_publisher(
            TaskStatus, '/forklift/task_status', status_qos
        )
        self._approach_pose_pubs = {
            name: self.create_publisher(
                PoseStamped,
                '/forklift/pallet_approach/{}_pose'.format(name),
                status_qos,
            )
            for name in ('alignment', 'pre_approach', 'stop')
        }
        self._pallet_exemption_pose_pub = self.create_publisher(
            PoseStamped,
            '/forklift/pallet_approach/exemption_pose',
            status_qos,
        )
        self._pallet_exemption_active_pub = self.create_publisher(
            Bool,
            '/forklift/pallet_approach/exemption_active',
            status_qos,
        )
        self._pallet_exemption_target: Optional[PoseTarget] = None
        self._pallet_exemption_active = False

        self._pickup_devices = PickupDeviceActions(
            self,
            fork_action_name,
            detect_action_name,
            move_relative_action_name,
        )
        self._nav2_navigator = Nav2Navigator(self, action_name)
        self._navigator = TaskMotionAdapter(
            self,
            self._nav2_navigator,
            self._pickup_devices,
            str(self.get_parameter('pallet_base_frame_id').value),
            self._set_pallet_exemption,
        )
        self._machine = TaskStateMachine(
            self._navigator,
            max_retries=max_retries,
            status_callback=self._publish_navigation_status,
        )
        self._pickup_machine = PalletPickupStateMachine(
            self._pickup_devices,
            self._pallet_config.defaults,
            status_callback=self._publish_pickup_status,
        )
        self._last_completed_station = ''

        self._action_cb_group = ReentrantCallbackGroup()
        self._execute_route_server = ActionServer(
            self,
            ExecuteRoute,
            'execute_route',
            execute_callback=self._execute_route,
            goal_callback=self._route_goal,
            cancel_callback=self._route_cancel,
            callback_group=self._action_cb_group,
        )
        self._execute_pallet_pickup_server = ActionServer(
            self,
            ExecutePalletPickup,
            'execute_pallet_pickup',
            execute_callback=self._execute_pallet_pickup,
            goal_callback=self._pallet_pickup_goal,
            cancel_callback=self._pallet_pickup_cancel,
            callback_group=self._action_cb_group,
        )
        self.create_service(GoToStation, 'go_to_station', self._go_to_station)
        self.create_service(Trigger, 'pause', self._pause)
        self.create_service(Trigger, 'resume', self._resume)
        self.create_service(Trigger, 'cancel', self._cancel)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_pose, 10)
        self.create_subscription(
            String, '/forklift/safety_gate/status', self._safety_status, 10
        )
        self.create_timer(0.1, self._update_pallet_exemption)
        self._publish_pallet_exemption()
        self._publish_navigation_status(self._machine)
        self.get_logger().info(
            'Loaded {} stations, {} routes, and {} pallet slots; RViz goal mode={}.'.format(
                len(self._stations), len(self._routes), len(self._pallet_config.slots),
                self._rviz_goal_mode,
            )
        )

    def _route_goal(self, goal_request: ExecuteRoute.Goal) -> GoalResponse:
        if goal_request.route_name not in self._routes:
            self.get_logger().warning(
                'Rejecting unknown route {}.'.format(goal_request.route_name)
            )
            return GoalResponse.REJECT
        if self._machine.state in {RUNNING, PAUSED, RECOVERING}:
            self.get_logger().warning('Rejecting route while another task is active.')
            return GoalResponse.REJECT
        if self._pickup_machine.state in {RUNNING, PAUSED}:
            self.get_logger().warning(
                'Rejecting route while pallet pickup is active.'
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _route_cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _pallet_pickup_goal(
        self,
        goal_request: ExecutePalletPickup.Goal,
    ) -> GoalResponse:
        slot = self._pallet_config.slots.get(goal_request.slot_id)
        if slot is None:
            self.get_logger().warning(
                'Rejecting unknown pallet slot {}.'.format(goal_request.slot_id)
            )
            return GoalResponse.REJECT
        if self._machine.state in {RUNNING, PAUSED, RECOVERING}:
            self.get_logger().warning(
                'Rejecting pallet pickup while navigation task is active.'
            )
            return GoalResponse.REJECT
        if self._pickup_machine.state in {RUNNING, PAUSED}:
            self.get_logger().warning(
                'Rejecting pallet pickup while another pickup task is active.'
            )
            return GoalResponse.REJECT
        if (
            self._enforce_pallet_approach_station
            and self._last_completed_station != slot.approach_station
        ):
            self.get_logger().warning(
                'Rejecting pallet pickup for {}; last completed station is {}, '
                'expected {}.'.format(
                    goal_request.slot_id,
                    self._last_completed_station or '<none>',
                    slot.approach_station,
                )
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _pallet_pickup_cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_route(self, goal_handle):
        route = self._routes[goal_handle.request.route_name]
        accepted, message = self._machine.start(
            route, loop=goal_handle.request.loop
        )
        result = ExecuteRoute.Result()
        if not accepted:
            result.success = False
            result.message = message
            goal_handle.abort()
            return result

        rate = self.create_rate(10.0)
        try:
            while self._machine.state in {RUNNING, PAUSED, RECOVERING}:
                if goal_handle.is_cancel_requested:
                    self._machine.cancel('execute_route action canceled')
                    result.success = False
                    result.message = 'execute_route action canceled'
                    goal_handle.canceled()
                    return result
                goal_handle.publish_feedback(self._route_feedback())
                rate.sleep()
        finally:
            self.destroy_rate(rate)

        result.success = self._machine.state == SUCCEEDED
        result.message = self._machine.reason
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _execute_pallet_pickup(self, goal_handle):
        slot = self._pallet_config.slots[goal_handle.request.slot_id]
        accepted, message = self._pickup_machine.start(slot)
        result = ExecutePalletPickup.Result()
        if not accepted:
            result.success = False
            result.message = message
            goal_handle.abort()
            return result

        rate = self.create_rate(10.0)
        try:
            while self._pickup_machine.state in {RUNNING, PAUSED}:
                if goal_handle.is_cancel_requested:
                    self._pickup_machine.cancel('execute_pallet_pickup action canceled')
                    result.success = False
                    result.message = 'execute_pallet_pickup action canceled'
                    goal_handle.canceled()
                    return result
                goal_handle.publish_feedback(self._pallet_pickup_feedback())
                rate.sleep()
        finally:
            self.destroy_rate(rate)

        result.success = self._pickup_machine.state == SUCCEEDED
        result.message = self._pickup_machine.reason
        result.offset_x_m = self._pickup_machine.offset_x_m
        result.offset_y_m = self._pickup_machine.offset_y_m
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _route_feedback(self) -> ExecuteRoute.Feedback:
        feedback = ExecuteRoute.Feedback()
        feedback.current_segment = self._machine.current_segment
        feedback.segment_index = self._machine.segment_index
        feedback.segment_count = self._machine.segment_count
        if self._machine.state == SUCCEEDED:
            feedback.progress = 1.0
        elif self._machine.segment_count > 0 and self._machine.segment_index >= 0:
            feedback.progress = float(
                self._machine.segment_index
            ) / float(self._machine.segment_count)
        else:
            feedback.progress = 0.0
        return feedback

    def _pallet_pickup_feedback(self) -> ExecutePalletPickup.Feedback:
        feedback = ExecutePalletPickup.Feedback()
        feedback.phase = self._pickup_machine.phase
        feedback.progress = float(self._pickup_machine.progress)
        feedback.current_height_m = self._pickup_machine.current_height_m
        return feedback

    def _go_to_station(
        self,
        request: GoToStation.Request,
        response: GoToStation.Response,
    ) -> GoToStation.Response:
        if self._pickup_machine.state in {RUNNING, PAUSED}:
            response.success = False
            response.message = 'pallet pickup task is active'
            return response
        station = self._stations.get(request.station)
        if station is None:
            response.success = False
            response.message = 'unknown station {}'.format(request.station)
            return response
        route = RouteDefinition(
            name='station:{}'.format(request.station),
            loop=False,
            targets=(station,),
        )
        response.success, response.message = self._machine.start(route, loop=False)
        return response

    def _pause(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self._machine.state in {RUNNING, RECOVERING}:
            response.success, response.message = self._machine.pause()
            return response
        if self._pickup_machine.state == RUNNING:
            response.success, response.message = self._pickup_machine.pause()
            return response
        response.success = False
        response.message = 'task is not running'
        return response

    def _resume(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self._machine.state == PAUSED:
            response.success, response.message = self._machine.resume()
            return response
        if self._pickup_machine.state == PAUSED:
            response.success, response.message = self._pickup_machine.resume()
            return response
        response.success = False
        response.message = 'task is not paused'
        return response

    def _cancel(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self._machine.state in {RUNNING, PAUSED, RECOVERING}:
            response.success, response.message = self._machine.cancel()
            return response
        if self._pickup_machine.state in {RUNNING, PAUSED}:
            response.success, response.message = self._pickup_machine.cancel()
            return response
        response.success = False
        response.message = 'no active task'
        return response

    def _goal_pose(self, pose: PoseStamped) -> None:
        if self._pickup_machine.state in {RUNNING, PAUSED}:
            self.get_logger().warning(
                'Ignoring /goal_pose while pallet pickup is active; cancel it first.'
            )
            return
        if self._machine.state in {RUNNING, PAUSED, RECOVERING}:
            self.get_logger().warning(
                'Ignoring /goal_pose while another task is active; cancel it first.'
            )
            return
        orientation = pose.pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        if self._rviz_goal_mode == 'pallet_approach':
            self._start_rviz_pallet_approach(pose, yaw)
            return

        target = PoseTarget(
            name='goal_pose',
            x=float(pose.pose.position.x),
            y=float(pose.pose.position.y),
            yaw=yaw,
            frame_id=pose.header.frame_id or 'map',
        )
        route = RouteDefinition(
            name='goal_pose',
            loop=False,
            targets=(target,),
        )
        accepted, message = self._machine.start(route, loop=False)
        if not accepted:
            self.get_logger().warning(message)

    def _start_rviz_pallet_approach(
        self,
        pallet_pose: PoseStamped,
        pallet_yaw: float,
    ) -> None:
        try:
            geometry = build_pallet_approach(
                pallet_x=float(pallet_pose.pose.position.x),
                pallet_y=float(pallet_pose.pose.position.y),
                pallet_yaw=pallet_yaw,
                frame_id=pallet_pose.header.frame_id or 'map',
                config=self._pallet_approach_config,
            )
        except PalletApproachError as exc:
            self.get_logger().error(
                'Rejecting RViz pallet goal: {}'.format(exc)
            )
            return

        for name, target in zip(
            ('alignment', 'pre_approach', 'stop'),
            geometry_pose_targets(geometry),
        ):
            self._publish_approach_pose(name, target)

        route = build_pallet_approach_route(geometry)
        route_exemption = TaskMotionAdapter._pallet_exemption_pose(
            geometry.final_motion
        )
        self._arm_pallet_exemption(route_exemption)
        accepted, message = self._machine.start(route, loop=False)
        if not accepted:
            self._clear_pallet_exemption()
            self.get_logger().warning(message)
            return
        self.get_logger().info(
            'Accepted pallet goal ({:.3f}, {:.3f}, yaw {:.3f}); '
            'stop=({:.3f}, {:.3f}), final_motion={:.3f} m at {:.3f} m/s.'.format(
                pallet_pose.pose.position.x,
                pallet_pose.pose.position.y,
                pallet_yaw,
                geometry.stop.x,
                geometry.stop.y,
                geometry.final_motion.distance_m,
                geometry.final_motion.max_speed_mps,
            )
        )

    def _publish_approach_pose(self, name: str, target: PoseTarget) -> None:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = target.frame_id
        pose.pose.position.x = target.x
        pose.pose.position.y = target.y
        pose.pose.orientation.z = math.sin(target.yaw * 0.5)
        pose.pose.orientation.w = math.cos(target.yaw * 0.5)
        self._approach_pose_pubs[name].publish(pose)

    def _set_pallet_exemption(self, target: Optional[PoseTarget]) -> None:
        # The final relative segment re-publishes the same pallet pose. A None
        # callback means that segment ended; proximity, not segment completion,
        # owns the lifetime of the exemption.
        if target is not None:
            self._arm_pallet_exemption(target)

    def _arm_pallet_exemption(
        self,
        target: Optional[PoseTarget],
    ) -> None:
        self._pallet_exemption_target = target
        self._pallet_exemption_active = False
        self._update_pallet_exemption()

    def _clear_pallet_exemption(self) -> None:
        self._pallet_exemption_target = None
        self._pallet_exemption_active = False
        self._publish_pallet_exemption()

    def _update_pallet_exemption(self) -> None:
        target = self._pallet_exemption_target
        if target is None:
            self._publish_pallet_exemption()
            return
        try:
            distance = self._navigator.planar_distance_to(target)
        except Exception as exc:
            self.get_logger().warning(
                'Pallet exemption distance TF unavailable: {}'.format(exc),
                throttle_duration_sec=2.0,
            )
            self._publish_pallet_exemption()
            return

        was_active = self._pallet_exemption_active
        self._pallet_exemption_active = pallet_exemption_active_for_distance(
            currently_active=was_active,
            distance_m=distance,
            activation_distance_m=self._pallet_exemption_activation_distance_m,
            deactivation_distance_m=self._pallet_exemption_deactivation_distance_m,
        )
        if self._pallet_exemption_active and not was_active:
            self.get_logger().info(
                'Pallet scan exemption enabled at {:.3f} m from target.'.format(
                    distance
                )
            )
        elif was_active and not self._pallet_exemption_active:
            self.get_logger().info(
                'Pallet scan exemption disabled after leaving target area at '
                '{:.3f} m.'.format(distance)
            )
            self._clear_pallet_exemption()
            return
        self._publish_pallet_exemption()

    def _publish_pallet_exemption(self) -> None:
        target = self._pallet_exemption_target
        if target is not None:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = target.frame_id
            pose.pose.position.x = target.x
            pose.pose.position.y = target.y
            pose.pose.orientation.z = math.sin(target.yaw * 0.5)
            pose.pose.orientation.w = math.cos(target.yaw * 0.5)
            self._pallet_exemption_pose_pub.publish(pose)
        active = Bool()
        active.data = target is not None and self._pallet_exemption_active
        self._pallet_exemption_active_pub.publish(active)

    def _safety_status(self, status: String) -> None:
        if self._machine.observe_safety_status(status.data):
            self.get_logger().warning(
                'Task paused by safety gate: {}.'.format(status.data)
            )
        if self._pickup_machine.observe_safety_status(status.data):
            self.get_logger().warning(
                'Pallet pickup paused by safety gate: {}.'.format(status.data)
            )

    def _publish_navigation_status(self, machine: TaskStateMachine) -> None:
        if machine.state == SUCCEEDED and machine.current_segment:
            self._last_completed_station = self._station_name_from_segment(
                machine.current_segment
            )
        if (
            machine.state in {SUCCEEDED, FAILED, IDLE}
            and self._pallet_exemption_target is not None
            and not self._pallet_exemption_active
        ):
            self._clear_pallet_exemption()
        status = TaskStatus()
        status.state = machine.state
        status.active_route = machine.active_route
        status.current_segment = machine.current_segment
        status.segment_index = machine.segment_index
        status.segment_count = machine.segment_count
        status.reason = machine.reason
        self._status_pub.publish(status)

    def _publish_pickup_status(self, machine: PalletPickupStateMachine) -> None:
        status = TaskStatus()
        status.state = machine.state
        status.active_route = (
            'pickup:{}'.format(machine.active_slot) if machine.active_slot else ''
        )
        status.current_segment = machine.phase
        status.segment_index = machine.phase_index
        status.segment_count = machine.phase_count
        status.reason = machine.reason
        self._status_pub.publish(status)

    @staticmethod
    def _station_name_from_segment(segment_name: str) -> str:
        return segment_name.rsplit(':', 1)[-1]


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = ForkliftTaskManager()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
