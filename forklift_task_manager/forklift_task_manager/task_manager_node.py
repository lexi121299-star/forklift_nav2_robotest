"""ROS 2 node exposing the minimal P7.1 forklift task interface."""

import ast
import math
import time
import traceback
from typing import Callable, List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from forklift_msgs.action import (
    DetectPalletOffset,
    ExecutePalletPickup,
    ExecuteRoute,
    ForkMoveTo,
    MoveRelative,
    PivotRelative,
)
from forklift_msgs.msg import ForkliftVehicleState, TaskStatus
from forklift_msgs.srv import GoToStation
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from .pallet_approach import (
    PalletApproachConfig,
    PalletApproachError,
    build_pallet_approach_route,
    build_selected_pallet_approach,
    pallet_exemption_active_for_distance,
)
from .anchor_route import (
    AnchorRouteConfig,
    select_route_anchors,
    validate_anchor_config,
)
from .pallet_maneuver import (
    PalletManeuverConfig,
    circular_turn_is_clear,
    select_clear_candidates,
    validate_maneuver_config,
)
from .route_model import (
    PoseTarget,
    PivotTarget,
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
        self._path_client = ActionClient(
            node, ComputePathToPose, 'compute_path_to_pose'
        )
        self._follow_path_client = ActionClient(node, FollowPath, 'follow_path')
        self._cancel_client = node.create_client(
            CancelGoal, '{}/_action/cancel_goal'.format(action_name.rstrip('/'))
        )
        self._goal_handle = None
        self._request_token = 0
        self._path_goal_handle = None
        self._path_request_token = 0
        self._path_timeout_timer = None
        self._follow_path_goal_handle = None
        self._motion_sends = set()
        self._motion_handles = {}
        self._motion_wait_timer = None

    def _clear_motion_wait(self):
        timer = self._motion_wait_timer
        self._motion_wait_timer = None
        if timer is not None:
            timer.cancel()
            self._node.destroy_timer(timer)

    def _cancel_motion_handles(self):
        for handle, _result in list(self._motion_handles.values()):
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self._node.get_logger().warning('Motion cancel failed: {}'.format(exc))

    def _send_motion_when_idle(self, client, goal, token, response, complete):
        """A cancel ACK is not a terminal result. Never preempt a plan-once BT."""
        self._clear_motion_wait()

        def dispatch():
            if token != self._request_token:
                return
            try:
                future = client.send_goal_async(goal)
            except Exception as exc:
                complete(False, 'Motion goal send failed: {}'.format(exc))
                return
            self._motion_sends.add(future)

            def accepted(done):
                try:
                    handle = done.result()
                    result = None
                    if handle is not None and handle.accepted:
                        key = id(handle)
                        # An accepted goal remains a barrier even if the result
                        # request itself fails: its server may still be moving.
                        self._motion_handles[key] = (handle, None)
                        result = handle.get_result_async()
                        self._motion_handles[key] = (handle, result)

                        def terminal(finished):
                            try:
                                wrapped = finished.result()
                                if wrapped.status not in (
                                    GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                                    GoalStatus.STATUS_ABORTED,
                                ):
                                    raise RuntimeError('non-terminal action result {}'.format(wrapped.status))
                            except Exception as exc:
                                self._node.get_logger().error(
                                    'Motion terminal state unknown; blocking replacement: {}'.format(exc))
                                return
                            self._motion_handles.pop(key, None)

                        result.add_done_callback(terminal)
                        if token != self._request_token:
                            handle.cancel_goal_async()
                    response(done, result)
                except Exception as exc:
                    complete(False, 'Motion goal response failed: {}'.format(exc))
                finally:
                    self._motion_sends.discard(done)

            future.add_done_callback(accepted)

        if not self._motion_sends and not self._motion_handles:
            dispatch()
            return
        self._cancel_motion_handles()
        deadline = time.monotonic() + 5.0

        def wait_for_terminal():
            if token != self._request_token:
                return
            if not self._motion_sends and not self._motion_handles:
                self._clear_motion_wait()
                dispatch()
            elif time.monotonic() >= deadline:
                self._clear_motion_wait()
                complete(False, 'Previous motion did not reach terminal state within 5 s; replacement not sent')

        self._node.get_logger().info('Waiting for previous motion terminal result before sending replacement goal.')
        self._motion_wait_timer = self._node.create_timer(0.05, wait_for_terminal)

    @staticmethod
    def _pose_from_target(target: PoseTarget, stamp: PoseStamped) -> None:
        stamp.header.frame_id = target.frame_id
        stamp.pose.position.x = target.x
        stamp.pose.position.y = target.y
        stamp.pose.orientation.z = math.sin(target.yaw * 0.5)
        stamp.pose.orientation.w = math.cos(target.yaw * 0.5)

    def send_goal(
        self,
        target: PoseTarget,
        done_callback: Callable[[bool, str], None],
    ) -> None:
        self._request_token += 1
        request_token = self._request_token

        def complete(success: bool, message: str) -> None:
            """Contain orchestration callback failures inside this ROS node.

            ComputePathToPose already has this boundary. NavigateToPose did
            not, so an exception while an anchor retried or advanced escaped a
            rclpy future callback and terminated the complete Task Manager.
            """

            if request_token != self._request_token:
                return
            try:
                done_callback(success, message)
            except Exception:
                self._node.get_logger().error(
                    'Unhandled NavigateToPose callback exception for {}:\n{}'.format(
                        target.name, traceback.format_exc()
                    )
                )

        if not self._client.wait_for_server(timeout_sec=0.0):
            complete(False, 'NavigateToPose action server unavailable')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        self._pose_from_target(target, goal.pose)

        def goal_response(future, result_future=None) -> None:
            try:
                goal_handle = future.result()
            except Exception as exc:
                complete(False, 'NavigateToPose send failed: {}'.format(exc))
                return
            if not goal_handle.accepted:
                complete(False, 'NavigateToPose goal rejected')
                return
            if request_token != self._request_token:
                goal_handle.cancel_goal_async()
                return
            self._goal_handle = goal_handle
            if result_future is None:
                result_future = goal_handle.get_result_async()

            def result_response(result_done) -> None:
                if request_token != self._request_token:
                    return
                try:
                    wrapped_result = result_done.result()
                except Exception as exc:
                    complete(False, 'NavigateToPose result failed: {}'.format(exc))
                    return
                self._goal_handle = None
                success = wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
                message = (
                    'NavigateToPose succeeded'
                    if success else
                    'NavigateToPose ended with status {}'.format(wrapped_result.status)
                )
                complete(success, message)

            result_future.add_done_callback(result_response)

        self._send_motion_when_idle(
            self._client, goal, request_token, goal_response, complete)

    def compute_path(
        self,
        target: PoseTarget,
        done_callback: Callable[[bool, str], None],
        timeout_sec: float = 12.0,
        planner_id: str = '',
    ) -> None:
        """Ask Nav2 whether the current pose can reach a pallet runup point."""

        def discard_path(success, message, _path) -> None:
            done_callback(success, message)

        self.compute_path_result(
            target,
            discard_path,
            timeout_sec=timeout_sec,
            planner_id=planner_id,
        )

    def send_follow_path(
        self,
        path,
        done_callback: Callable[[bool, str], None],
        controller_id: str = '',
    ) -> None:
        """Execute one already-validated route without stopping at anchors."""

        self._request_token += 1
        request_token = self._request_token

        def complete(success: bool, message: str) -> None:
            if request_token != self._request_token:
                return
            try:
                done_callback(success, message)
            except Exception:
                self._node.get_logger().error(
                    'Unhandled FollowPath callback exception:\n{}'.format(
                        traceback.format_exc()
                    )
                )

        if path is None or len(path.poses) < 2:
            complete(False, 'FollowPath requires at least two path poses')
            return
        if not self._follow_path_client.wait_for_server(timeout_sec=0.0):
            complete(False, 'FollowPath action server unavailable')
            return

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = controller_id
        def goal_response(future, result_future=None) -> None:
            try:
                goal_handle = future.result()
            except Exception as exc:
                complete(False, 'FollowPath send failed: {}'.format(exc))
                return
            if goal_handle is None or not goal_handle.accepted:
                complete(False, 'FollowPath goal rejected')
                return
            if request_token != self._request_token:
                goal_handle.cancel_goal_async()
                return
            self._follow_path_goal_handle = goal_handle
            if result_future is None:
                result_future = goal_handle.get_result_async()

            def result_response(result_done) -> None:
                if request_token != self._request_token:
                    return
                try:
                    wrapped_result = result_done.result()
                except Exception as exc:
                    complete(False, 'FollowPath result failed: {}'.format(exc))
                    return
                self._follow_path_goal_handle = None
                success = wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
                complete(
                    success,
                    'FollowPath succeeded' if success else
                    'FollowPath ended with status {}'.format(wrapped_result.status),
                )

            result_future.add_done_callback(result_response)

        self._send_motion_when_idle(
            self._follow_path_client, goal, request_token, goal_response, complete)

    def compute_path_result(
        self,
        target: PoseTarget,
        done_callback,
        timeout_sec: float = 12.0,
        planner_id: str = '',
    ) -> None:
        """Compute a path and return the actual path to an orchestration caller."""

        self.cancel_path_goal()
        request_token = self._path_request_token
        completed = {'value': False}

        def complete(success: bool, message: str, path=None) -> None:
            """Keep exceptions in a caller callback out of the ROS executor."""

            if completed['value'] or request_token != self._path_request_token:
                return
            completed['value'] = True
            self._path_goal_handle = None
            self._cancel_path_timeout_timer()
            try:
                done_callback(success, message, path)
            except Exception:
                self._node.get_logger().error(
                    'Unhandled pallet ComputePathToPose callback exception:\n{}'.format(
                        traceback.format_exc()
                    )
                )

        if not self._path_client.wait_for_server(timeout_sec=0.0):
            complete(False, 'ComputePathToPose action server unavailable')
            return
        try:
            goal = ComputePathToPose.Goal()
            # Foxy names the target pose ``pose``; later Nav2 releases use
            # ``goal``. Keep the task manager deployable on both interfaces.
            target_pose = getattr(goal, 'goal', None)
            if target_pose is None:
                target_pose = getattr(goal, 'pose', None)
            if target_pose is None:
                raise AttributeError(
                    'ComputePathToPose.Goal has neither goal nor pose field'
                )
            target_pose.header.stamp = self._node.get_clock().now().to_msg()
            self._pose_from_target(target, target_pose)
            if hasattr(goal, 'use_start'):
                goal.use_start = False
            if hasattr(goal, 'planner_id'):
                goal.planner_id = planner_id
            send_future = self._path_client.send_goal_async(goal)
        except Exception as exc:
            complete(False, 'ComputePathToPose request creation failed: {}'.format(exc))
            return

        def goal_response(future) -> None:
            try:
                goal_handle = future.result()
                if goal_handle is None or not goal_handle.accepted:
                    complete(False, 'ComputePathToPose goal rejected')
                    return
                if request_token != self._path_request_token:
                    goal_handle.cancel_goal_async()
                    return
                self._path_goal_handle = goal_handle
                result_future = goal_handle.get_result_async()
            except Exception as exc:
                complete(False, 'ComputePathToPose send failed: {}'.format(exc))
                return

            def result_response(result_done) -> None:
                try:
                    wrapped_result = result_done.result()
                    result = getattr(wrapped_result, 'result', None)
                    path = getattr(result, 'path', None)
                    path_points = len(path.poses) if path is not None else 0
                    status = getattr(wrapped_result, 'status', None)
                except Exception as exc:
                    complete(
                        False, 'ComputePathToPose result failed: {}'.format(exc)
                    )
                    return
                if (
                    status == GoalStatus.STATUS_SUCCEEDED
                    and path_points >= 2
                ):
                    complete(True, 'ComputePathToPose succeeded', path)
                    return
                complete(
                    False,
                    'ComputePathToPose ended with status {} path_points={}'.format(
                        status, path_points
                    ),
                )

            result_future.add_done_callback(result_response)

        send_future.add_done_callback(goal_response)
        timeout_sec = max(0.1, float(timeout_sec))

        def path_timeout() -> None:
            if completed['value'] or request_token != self._path_request_token:
                return
            if self._path_goal_handle is not None:
                self._path_goal_handle.cancel_goal_async()
            complete(
                False,
                'ComputePathToPose timed out after {:.1f} s'.format(timeout_sec),
            )

        self._path_timeout_timer = self._node.create_timer(
            timeout_sec, path_timeout
        )

    def _cancel_path_timeout_timer(self) -> None:
        timer = self._path_timeout_timer
        self._path_timeout_timer = None
        if timer is not None:
            timer.cancel()
            self._node.destroy_timer(timer)

    def cancel_path_goal(self) -> None:
        self._path_request_token += 1
        self._cancel_path_timeout_timer()
        if self._path_goal_handle is not None:
            self._path_goal_handle.cancel_goal_async()
            self._path_goal_handle = None

    def cancel_goal(self) -> None:
        self._request_token += 1
        self._clear_motion_wait()
        self._cancel_motion_handles()
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        if self._follow_path_goal_handle is not None:
            self._follow_path_goal_handle.cancel_goal_async()
            self._follow_path_goal_handle = None

    def cancel_all_goals(self) -> None:
        """Cancel goals from other clients before a pallet task takes ownership."""

        self._request_token += 1
        self._clear_motion_wait()
        self._cancel_motion_handles()
        self.cancel_path_goal()
        self._goal_handle = None
        self._follow_path_goal_handle = None
        if self._cancel_client.wait_for_service(timeout_sec=0.0):
            self._cancel_client.call_async(CancelGoal.Request())


class HierarchicalNav2Navigator:
    """Retry a failed final goal through safe A* topology anchors."""

    def __init__(
        self,
        node: Node,
        navigator: Nav2Navigator,
        costmap_provider,
        config: AnchorRouteConfig,
        topology_planner_id: str,
        compute_timeout_sec: float,
        state_callback=None,
        continuous_topology_planner_id: str = '',
        continuous_path_enabled: bool = False,
    ) -> None:
        validate_anchor_config(config)
        self._node = node
        self._navigator = navigator
        self._costmap_provider = costmap_provider
        self._config = config
        self._topology_planner_id = str(topology_planner_id)
        self._continuous_topology_planner_id = str(
            continuous_topology_planner_id
        )
        self._continuous_path_enabled = bool(continuous_path_enabled) and bool(
            self._continuous_topology_planner_id
        )
        self._compute_timeout_sec = max(0.1, float(compute_timeout_sec))
        self._state_callback = state_callback
        self._token = 0
        self._anchor_queue = []
        self._anchor_index = 0
        self._segment_retries = 0
        self._split_depth = 0
        self._original_goal = None
        self._done_callback = None
        self._anchor_mode = False
        self._completed_anchor_count = 0
        self._anchor_arrival_checker = None
        self._anchor_arrival_timer = None
        self._anchor_transition_timer = None
        # A NavigateToPose result and a timer can arrive after the next anchor
        # has already started.  This generation distinguishes those old events
        # from the active anchor segment.
        self._anchor_segment_token = 0

    def _publish_state(self, anchor_index: int) -> None:
        if self._state_callback is not None:
            self._state_callback(int(anchor_index))

    def set_anchor_arrival_checker(self, checker) -> None:
        """Install the TF/vehicle-state based guard owned by TaskMotionAdapter."""

        self._anchor_arrival_checker = checker

    def send_goal(self, target: PoseTarget, done_callback) -> None:
        self.cancel_goal()
        self._token += 1
        token = self._token
        self._original_goal = target
        self._done_callback = done_callback
        self._anchor_mode = False
        self._completed_anchor_count = 0
        self._publish_state(-1)
        self._node.get_logger().info(
            'Hierarchical navigation trying complete trajectory to {}.'.format(
                target.name
            )
        )

        def direct_done(success: bool, message: str) -> None:
            if token != self._token:
                return
            if success:
                self._finish(True, message)
                return
            self._node.get_logger().warning(
                'Complete trajectory failed: {}; requesting {} topology.'.format(
                    message, self._topology_planner_id
                )
            )
            self._anchor_mode = True
            self._split_depth = 0
            if self._continuous_path_enabled:
                self._plan_continuous_topology_route(token, message)
            else:
                self._plan_anchor_route(token, message)

        self._navigator.send_goal(target, direct_done)

    def cancel_goal(self) -> None:
        self._token += 1
        self._anchor_segment_token += 1
        self._clear_anchor_arrival_timer()
        self._clear_anchor_transition_timer()
        self._navigator.cancel_goal()
        self._navigator.cancel_path_goal()
        self._anchor_queue = []
        self._original_goal = None
        self._done_callback = None
        self._anchor_mode = False
        self._completed_anchor_count = 0
        self._publish_state(-1)

    def cancel_all_goals(self) -> None:
        self.cancel_goal()
        self._navigator.cancel_all_goals()

    @property
    def anchor_mode(self) -> bool:
        return self._anchor_mode

    @property
    def anchor_index(self) -> int:
        return self._anchor_index

    def _finish(self, success: bool, message: str) -> None:
        if success and self._original_goal is not None and self._anchor_arrival_checker is not None:
            try:
                accepted, distance, reason = self._anchor_arrival_checker(
                    self._original_goal, self._config.arrival_tolerance_m, math.inf)
                if not accepted:
                    success = False
                    message = 'Final goal verification failed at {:.3f} m: {}'.format(distance, reason)
            except Exception as exc:
                success = False
                message = 'Final goal verification unavailable: {}'.format(exc)
        self._anchor_segment_token += 1
        self._clear_anchor_arrival_timer()
        self._clear_anchor_transition_timer()
        callback = self._done_callback
        self._done_callback = None
        self._anchor_queue = []
        self._original_goal = None
        self._anchor_mode = False
        self._publish_state(-1)
        if callback is not None:
            try:
                callback(success, message)
            except Exception:
                self._node.get_logger().error(
                    'Unhandled hierarchical navigation completion callback:\n{}'.format(
                        traceback.format_exc()
                    )
                )

    def _plan_anchor_route(self, token: int, previous_failure: str) -> None:
        if token != self._token or self._original_goal is None:
            return
        costmap = self._costmap_provider()
        if costmap is None:
            self._finish(
                False,
                '{}; anchor fallback unavailable: global costmap missing'.format(
                    previous_failure
                ),
            )
            return

        def topology_done(success, message, path) -> None:
            if token != self._token or self._original_goal is None:
                return
            if not success or path is None:
                self._finish(
                    False,
                    '{}; topology planning failed: {}'.format(
                        previous_failure, message
                    ),
                )
                return
            spacing_scale = max(0.01, 0.5 ** self._split_depth)
            try:
                anchors = select_route_anchors(
                    path,
                    costmap,
                    self._original_goal,
                    self._config,
                    spacing_scale=spacing_scale,
                )
            except (TypeError, ValueError) as exc:
                self._finish(False, 'anchor selection failed: {}'.format(exc))
                return
            if not anchors:
                self._finish(False, 'topology path contains no safe anchor')
                return
            if (
                len(anchors) == 1
                and self._split_depth > 0
            ):
                self._finish(
                    False,
                    'trajectory failed and topology path is too short to split safely',
                )
                return
            self._anchor_queue = anchors
            self._anchor_index = 0
            self._segment_retries = 0
            self._node.get_logger().warning(
                'Anchor fallback armed: anchors={} split_depth={} original_goal={}.'.format(
                    len(anchors), self._split_depth, self._original_goal.name
                )
            )
            self._send_current_anchor(token)

        self._navigator.compute_path_result(
            self._original_goal,
            topology_done,
            timeout_sec=self._compute_timeout_sec,
            planner_id=self._topology_planner_id,
        )

    def _plan_continuous_topology_route(
        self, token: int, previous_failure: str
    ) -> None:
        """Use one prevalidated segmented route before stop-and-go fallback.

        The continuous topology planner does the same coarse global A* search
        as ``TopologyOnly`` and then emits a single footprint-validated path
        with explicit pivots only where geometry requires them.  It therefore
        adds no per-anchor full-map planning pass.
        """

        if token != self._token or self._original_goal is None:
            return

        def continuous_plan_done(success, message, path) -> None:
            if token != self._token or self._original_goal is None:
                return
            if success and path is not None:
                # A stale planner binary may accept the new planner ID but
                # still return TopologyOnly anchors. Never execute that graph
                # as a sampled, collision-validated vehicle trajectory.
                previous = None
                for pose in path.poses:
                    p = pose.pose.position
                    if not math.isfinite(p.x) or not math.isfinite(p.y):
                        success, message = False, 'non-finite continuous route point'
                        break
                    if previous is not None and math.hypot(
                        p.x - previous.x, p.y - previous.y
                    ) > 0.11:
                        success, message = False, (
                            'sparse topology route returned by {}; check plugin '
                            'build and topology_emit_segmented_path=true'
                        ).format(self._continuous_topology_planner_id)
                        break
                    previous = p
                if len(path.poses) < 2:
                    success, message = False, 'empty continuous route'
            if not success or path is None:
                self._node.get_logger().warning(
                    'Continuous topology route unavailable: {}; using '
                    'stop-and-go anchors.'.format(message)
                )
                self._plan_anchor_route(token, previous_failure)
                return
            self._node.get_logger().info(
                'Executing continuous topology route: poses={} planner={}.'.format(
                    len(path.poses), self._continuous_topology_planner_id
                )
            )

            def continuous_route_done(route_success, route_message) -> None:
                if token != self._token:
                    return
                if route_success:
                    self._finish(True, 'continuous topology route completed')
                    return
                self._node.get_logger().warning(
                    'Continuous topology FollowPath failed: {}; falling back '
                    'to stop-and-go anchors.'.format(route_message)
                )
                self._plan_anchor_route(
                    token,
                    '{}; continuous route failed: {}'.format(
                        previous_failure, route_message
                    ),
                )

            self._navigator.send_follow_path(path, continuous_route_done)

        self._navigator.compute_path_result(
            self._original_goal,
            continuous_plan_done,
            timeout_sec=self._compute_timeout_sec,
            planner_id=self._continuous_topology_planner_id,
        )

    def _send_current_anchor(self, token: int) -> None:
        if token != self._token or not self._anchor_queue:
            return
        self._anchor_segment_token += 1
        segment_token = self._anchor_segment_token
        target = self._anchor_queue[self._anchor_index]
        is_final = self._anchor_index == len(self._anchor_queue) - 1
        self._publish_state(self._completed_anchor_count)
        self._node.get_logger().info(
            'Navigating anchor {}/{} {} at ({:.3f}, {:.3f}).'.format(
                self._anchor_index + 1,
                len(self._anchor_queue),
                'final' if is_final else target.name,
                target.x,
                target.y,
            )
        )

        segment_completed = {'value': False}

        def segment_done(success: bool, message: str) -> None:
            if (
                segment_completed['value']
                or token != self._token
                or segment_token != self._anchor_segment_token
            ):
                return
            try:
                segment_completed['value'] = True
                self._clear_anchor_arrival_timer()
                self._clear_anchor_transition_timer()
                if success:
                    self._segment_retries = 0
                    if is_final:
                        self._finish(True, 'anchor route completed')
                        return
                    self._completed_anchor_count += 1
                    if self._completed_anchor_count >= self._config.max_count:
                        self._finish(
                            False,
                            'anchor route exceeded maximum completed anchor count {}'.format(
                                self._config.max_count
                            ),
                        )
                        return
                    # Preserve the verified topology queue after a successful
                    # anchor. Rebuilding it at every stop made small topology-grid
                    # changes create a fresh set of short, zig-zag segments.
                    self._anchor_index += 1
                    self._send_current_anchor(token)
                    return
                if self._segment_retries < self._config.segment_retry_count:
                    self._segment_retries += 1
                    self._node.get_logger().warning(
                        'Anchor {} failed: {}; retry {}/{}.'.format(
                            target.name,
                            message,
                            self._segment_retries,
                            self._config.segment_retry_count,
                        )
                    )
                    self._send_current_anchor(token)
                    return
                if self._split_depth < self._config.split_max_depth:
                    self._split_depth += 1
                    self._node.get_logger().warning(
                        'Anchor segment failed after retries; replanning with split depth {}.'.format(
                            self._split_depth
                        )
                    )
                    self._plan_anchor_route(token, message)
                    return
                self._finish(
                    False,
                    'anchor {} failed after {} retries and {} split levels: {}'.format(
                        target.name,
                        self._config.segment_retry_count,
                        self._config.split_max_depth,
                        message,
                    ),
                )
            except Exception:
                self._node.get_logger().error(
                    'Unhandled anchor segment callback for {}:\n{}'.format(
                        target.name, traceback.format_exc()
                    )
                )
                self._finish(False, 'anchor {} callback failed'.format(target.name))

        self._start_anchor_arrival_guard(
            token,
            segment_token,
            target,
            is_final,
            segment_completed,
            segment_done,
        )
        self._navigator.send_goal(target, segment_done)

    def _start_anchor_arrival_guard(
        self,
        token: int,
        segment_token: int,
        target: PoseTarget,
        is_final: bool,
        segment_completed,
        segment_done,
    ) -> None:
        """Advance only a stopped intermediate anchor that Nav2 left active."""

        self._clear_anchor_arrival_timer()
        if is_final or self._anchor_arrival_checker is None:
            return
        stationary_since = {'value': None}

        def check_arrival() -> None:
            if (
                token != self._token
                or segment_token != self._anchor_segment_token
                or segment_completed['value']
            ):
                return
            try:
                accepted, distance, reason = self._anchor_arrival_checker(
                    target,
                    self._config.arrival_tolerance_m,
                    self._config.arrival_speed_mps,
                )
            except Exception as exc:
                self._node.get_logger().warning(
                    'Anchor {} arrival guard unavailable: {}'.format(
                        target.name, exc
                    ),
                    throttle_duration_sec=2.0,
                )
                stationary_since['value'] = None
                return
            if not accepted:
                stationary_since['value'] = None
                return
            now = time.monotonic()
            if stationary_since['value'] is None:
                stationary_since['value'] = now
                return
            if now - stationary_since['value'] < self._config.arrival_settle_sec:
                return
            self._node.get_logger().warning(
                'Anchor {} accepted by arrival guard at {:.3f} m ({}).'.format(
                    target.name, distance, reason
                )
            )
            # Nav2 can remain active near an already-stopped intermediate goal.
            # Invalidate its eventual canceled result before queuing the next leg.
            # Foxy needs a short action lifecycle gap here: issuing the next goal
            # in this same callback can leave BT in preempted-without-replan state.
            self._navigator.cancel_goal()
            self._schedule_anchor_transition(
                token,
                segment_token,
                segment_completed,
                segment_done,
                'anchor accepted by arrival guard at {:.3f} m'.format(distance),
            )

        self._anchor_arrival_timer = self._node.create_timer(0.1, check_arrival)

    def _clear_anchor_arrival_timer(self) -> None:
        if self._anchor_arrival_timer is None:
            return
        timer = self._anchor_arrival_timer
        self._anchor_arrival_timer = None
        timer.cancel()
        self._node.destroy_timer(timer)

    def _schedule_anchor_transition(
        self,
        token: int,
        segment_token: int,
        segment_completed,
        segment_done,
        message: str,
    ) -> None:
        """Send the next anchor only after Nav2 has consumed the cancel event."""

        self._clear_anchor_arrival_timer()
        self._clear_anchor_transition_timer()

        def advance() -> None:
            if (
                token != self._token
                or segment_token != self._anchor_segment_token
                or segment_completed['value']
            ):
                return
            self._clear_anchor_transition_timer()
            segment_done(True, message)

        if self._config.arrival_transition_delay_sec <= 0.0:
            advance()
            return
        self._anchor_transition_timer = self._node.create_timer(
            self._config.arrival_transition_delay_sec, advance
        )

    def _clear_anchor_transition_timer(self) -> None:
        if self._anchor_transition_timer is None:
            return
        timer = self._anchor_transition_timer
        self._anchor_transition_timer = None
        timer.cancel()
        self._node.destroy_timer(timer)


class PickupDeviceActions:
    """Action-client adapter for fork, perception, and low-speed motion devices."""

    def __init__(
        self,
        node: Node,
        fork_action_name: str,
        detect_action_name: str,
        move_relative_action_name: str,
        pivot_relative_action_name: str,
    ) -> None:
        self._node = node
        self._fork_client = ActionClient(node, ForkMoveTo, fork_action_name)
        self._detect_client = ActionClient(node, DetectPalletOffset, detect_action_name)
        self._move_relative_client = ActionClient(
            node, MoveRelative, move_relative_action_name
        )
        self._pivot_relative_client = ActionClient(
            node, PivotRelative, pivot_relative_action_name
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

    def pivot_relative(
        self,
        *,
        angle_rad: float,
        max_speed_mps: float,
        timeout_sec: float,
        done_callback,
    ) -> None:
        goal = PivotRelative.Goal()
        goal.angle_rad = float(angle_rad)
        goal.max_speed_mps = float(max_speed_mps)
        self._send_goal(
            self._pivot_relative_client,
            goal,
            'PivotRelative',
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
        pivot_validation_callback,
        pivot_blocked_wait_sec: float,
        pallet_nav_arrival_tolerance_m: float,
        pallet_nav_arrival_speed_mps: float,
        pallet_nav_arrival_settle_sec: float,
        pivot_verify_timeout_sec: float = 3.0,
        pivot_yaw_tolerance_rad: float = 0.05,
        pivot_yaw_rate_tolerance_radps: float = 0.03,
        pivot_feedback_timeout_sec: float = 0.5,
        pivot_odom_topic: str = '/odom',
    ) -> None:
        self._node = node
        self._navigator = navigator
        self._devices = devices
        self._base_frame_id = base_frame_id
        self._pallet_exemption_callback = pallet_exemption_callback
        self._pivot_validation_callback = pivot_validation_callback
        self._pivot_blocked_wait_sec = max(0.0, float(pivot_blocked_wait_sec))
        self._pallet_nav_arrival_tolerance_m = max(
            0.01, float(pallet_nav_arrival_tolerance_m)
        )
        self._pallet_nav_arrival_speed_mps = max(
            0.0, float(pallet_nav_arrival_speed_mps)
        )
        self._pallet_nav_arrival_settle_sec = max(
            0.0, float(pallet_nav_arrival_settle_sec)
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)
        self._latest_vehicle_speed_mps = None
        self._latest_vehicle_state_time = 0.0
        self._arrival_timer = None
        self._pivot_verify_timer = None
        self._motion_token = 0
        self._pivot_verify_timeout_sec = pivot_verify_timeout_sec
        self._pivot_yaw_tolerance_rad = pivot_yaw_tolerance_rad
        self._pivot_yaw_rate_tolerance_radps = pivot_yaw_rate_tolerance_radps
        self._pivot_feedback_timeout_sec = pivot_feedback_timeout_sec
        for value in (pivot_verify_timeout_sec, pivot_yaw_tolerance_rad,
                      pivot_yaw_rate_tolerance_radps, pivot_feedback_timeout_sec):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError('pallet pivot verification parameters must be finite and positive')
        if pivot_verify_timeout_sec <= max(0.3, self._pallet_nav_arrival_settle_sec):
            raise ValueError('pallet pivot verification timeout must exceed settling duration')
        self._latest_pivot_odom = None
        self._latest_pivot_odom_time = 0.0
        self._latest_pivot_vehicle_state = None
        self._node.create_subscription(
            ForkliftVehicleState,
            '/forklift/vehicle_state',
            self._on_vehicle_state,
            10,
        )
        self._node.create_subscription(
            Odometry, pivot_odom_topic, self._on_pivot_odom, 1)

    def send_goal(self, target, done_callback) -> None:
        self._motion_token += 1
        token = self._motion_token
        self._clear_pivot_verification()
        if isinstance(target, PivotTarget):
            ready, message, angle = self._pivot_start_is_valid(target)
            if not ready:
                done_callback(False, message)
                return
            if abs(angle) <= self._pivot_yaw_tolerance_rad:
                self._verify_pivot_completion(target, token, done_callback)
                return

            def pivot_done(success, message, result) -> None:
                if token != self._motion_token:
                    return
                # Only this terminal odom-relative yaw failure is recoverable.
                # Timeout, cancellation and safety/feedback failures stay failed.
                odom_yaw_mismatch = (
                    result is not None and
                    message == 'pivot stopped outside yaw tolerance; reverse correction disabled'
                )
                if not success and not odom_yaw_mismatch:
                    done_callback(False, message)
                    return
                self._verify_pivot_completion(
                    target, token, done_callback,
                    odom_yaw_mismatch=odom_yaw_mismatch)

            self._devices.pivot_relative(
                angle_rad=angle,
                max_speed_mps=target.max_speed_mps,
                timeout_sec=target.timeout_sec,
                done_callback=pivot_done,
            )
            return
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
        if target.name in {'pallet_staging_runup', 'pallet_staging'}:
            self._send_pallet_navigation(target, done_callback)
            return
        self._navigator.send_goal(target, done_callback)

    def cancel_goal(self) -> None:
        self._motion_token += 1
        self._clear_pivot_verification()
        self._clear_arrival_timer()
        self._navigator.cancel_goal()
        self._devices.cancel_all()
        self._pallet_exemption_callback(None)

    def _on_vehicle_state(self, message: ForkliftVehicleState) -> None:
        self._latest_vehicle_speed_mps = abs(float(message.velocity_mps))
        self._latest_vehicle_state_time = time.monotonic()
        self._latest_pivot_vehicle_state = message

    def _on_pivot_odom(self, message: Odometry) -> None:
        self._latest_pivot_odom = message
        self._latest_pivot_odom_time = time.monotonic()

    def _pivot_stamp_is_fresh(self, stamp) -> bool:
        source_ns = int(stamp.sec) * 1000000000 + int(stamp.nanosec)
        age = (self._node.get_clock().now().nanoseconds - source_ns) / 1e9
        return source_ns > 0 and -0.1 <= age <= self._pivot_feedback_timeout_sec

    def _pivot_completion_status(self, target):
        """Verify the absolute pallet heading, not odom's accumulated rotation."""
        now = time.monotonic()
        state = self._latest_pivot_vehicle_state
        odom = self._latest_pivot_odom
        if (state is None or
                now - self._latest_vehicle_state_time > self._pivot_feedback_timeout_sec or
                not self._pivot_stamp_is_fresh(state.header.stamp)):
            return False, 'vehicle feedback stale', 0
        if (not state.enabled or not state.auto_mode or state.emergency_stopped or
                state.soft_emergency_stop or not state.interlock or state.parking_brake):
            return False, 'vehicle not ready', 0
        if (odom is None or
                now - self._latest_pivot_odom_time > self._pivot_feedback_timeout_sec or
                not self._pivot_stamp_is_fresh(odom.header.stamp)):
            return False, 'angular feedback stale', 0
        speed = float(state.velocity_mps)
        yaw_rate = float(odom.twist.twist.angular.z)
        if not math.isfinite(speed) or not math.isfinite(yaw_rate):
            return False, 'non-finite motion feedback', 0
        if (abs(speed) > self._pallet_nav_arrival_speed_mps or
                abs(yaw_rate) > self._pivot_yaw_rate_tolerance_radps):
            return False, 'vehicle not stationary: v={:.3f} w={:.3f}'.format(
                speed, yaw_rate), 0
        try:
            transform = self._tf_buffer.lookup_transform(
                target.frame_id, self._base_frame_id, Time())
        except Exception as exc:
            return False, 'map pose unavailable: {}'.format(exc), 0
        if not self._pivot_stamp_is_fresh(transform.header.stamp):
            return False, 'map pose stale', 0
        translation = transform.transform.translation
        q = transform.transform.rotation
        values = (translation.x, translation.y, q.x, q.y, q.z, q.w)
        if (not all(math.isfinite(v) for v in values) or
                abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) > 0.01):
            return False, 'invalid map pose', 0
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        heading_error = abs(math.atan2(math.sin(target.yaw-yaw), math.cos(target.yaw-yaw)))
        position_error = math.hypot(translation.x-target.x, translation.y-target.y)
        detail = 'position_error={:.3f} m heading_error={:.4f} rad'.format(
            position_error, heading_error)
        valid = (position_error <= target.max_start_position_error_m and
                 heading_error <= self._pivot_yaw_tolerance_rad)
        stamp_ns = (int(transform.header.stamp.sec)*1000000000 +
                    int(transform.header.stamp.nanosec))
        return valid, detail, stamp_ns

    def _verify_pivot_completion(
        self, target, token, done_callback, odom_yaw_mismatch=False,
    ) -> None:
        self._clear_pivot_verification()
        started = time.monotonic()
        stable_since = None
        first_stamp = None
        finished = False

        def check():
            nonlocal stable_since, first_stamp, finished
            if finished or token != self._motion_token:
                return
            valid, detail, stamp = self._pivot_completion_status(target)
            if token != self._motion_token:
                return
            now = time.monotonic()
            if not valid:
                stable_since = first_stamp = None
            elif stable_since is None:
                stable_since, first_stamp = now, stamp
            settled = (valid and stable_since is not None and stamp > first_stamp and
                       now-stable_since >= max(0.3, self._pallet_nav_arrival_settle_sec))
            if not settled and now-started < self._pivot_verify_timeout_sec:
                return
            finished = True
            self._clear_pivot_verification()
            if settled:
                message = 'pallet pivot map verification passed: ' + detail
                if odom_yaw_mismatch:
                    message += '; odom-relative yaw failure superseded by fresh stationary map pose'
                self._node.get_logger().info(message)
                done_callback(True, message)
            else:
                message = 'pallet pivot map verification failed: ' + detail
                self._node.get_logger().warning(message)
                done_callback(False, message)

        self._pivot_verify_timer = self._node.create_timer(0.05, check)
        check()

    def _clear_pivot_verification(self) -> None:
        if self._pivot_verify_timer is not None:
            timer = self._pivot_verify_timer
            self._pivot_verify_timer = None
            timer.cancel()
            self._node.destroy_timer(timer)

    def anchor_arrival_status(
        self,
        target: PoseTarget,
        tolerance_m: float,
        speed_limit_mps: float,
    ):
        """Return whether a non-final hierarchy anchor is safely stopped."""

        distance = self.planar_distance_to(target)
        if distance > tolerance_m:
            return False, distance, 'distance exceeds tolerance'
        state_age = time.monotonic() - self._latest_vehicle_state_time
        if self._latest_vehicle_speed_mps is None or state_age > 0.5:
            return False, distance, 'vehicle state is stale'
        if self._latest_vehicle_speed_mps > speed_limit_mps:
            return False, distance, 'vehicle is still moving'
        return True, distance, 'vehicle stopped inside tolerance'

    def _send_pallet_navigation(self, target: PoseTarget, done_callback) -> None:
        """Advance a pallet stage once Nav2 is close and the vehicle has stopped."""

        self._clear_arrival_timer()
        completed = False
        stationary_since = None

        def finish(success: bool, message: str) -> None:
            nonlocal completed
            if completed:
                return
            completed = True
            self._clear_arrival_timer()
            done_callback(success, message)

        def nav_done(success: bool, message: str) -> None:
            if success:
                try:
                    distance = self.planar_distance_to(target)
                except Exception as exc:
                    finish(
                        False,
                        '{} Nav2 success could not be position-verified: {}'.format(
                            target.name, exc
                        ),
                    )
                    return
                if distance > self._pallet_nav_arrival_tolerance_m:
                    finish(
                        False,
                        '{} Nav2 success rejected: {:.3f} m from target exceeds '
                        '{:.3f} m'.format(
                            target.name,
                            distance,
                            self._pallet_nav_arrival_tolerance_m,
                        ),
                    )
                    return
            finish(success, message)

        def check_arrival() -> None:
            nonlocal stationary_since
            if completed:
                return
            try:
                distance = self.planar_distance_to(target)
            except Exception:
                stationary_since = None
                return
            vehicle_state_age = time.monotonic() - self._latest_vehicle_state_time
            if (
                distance > self._pallet_nav_arrival_tolerance_m
                or self._latest_vehicle_speed_mps is None
                or vehicle_state_age > 0.5
                or self._latest_vehicle_speed_mps
                > self._pallet_nav_arrival_speed_mps
            ):
                stationary_since = None
                return
            now = time.monotonic()
            if stationary_since is None:
                stationary_since = now
                return
            if now - stationary_since < self._pallet_nav_arrival_settle_sec:
                return
            self._navigator.cancel_goal()
            finish(
                True,
                '{} accepted by pallet arrival guard at {:.3f} m'.format(
                    target.name, distance
                ),
            )

        self._arrival_timer = self._node.create_timer(0.1, check_arrival)
        self._navigator.send_goal(target, nav_done)

    def _clear_arrival_timer(self) -> None:
        if self._arrival_timer is None:
            return
        timer = self._arrival_timer
        self._arrival_timer = None
        timer.cancel()
        self._node.destroy_timer(timer)

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

    def _pivot_start_is_valid(self, target: PivotTarget):
        try:
            transform = self._tf_buffer.lookup_transform(
                target.frame_id,
                self._base_frame_id,
                Time(),
            )
        except Exception as exc:
            return False, 'pivot start TF unavailable: {}'.format(exc), 0.0
        translation = transform.transform.translation
        position_error = math.hypot(
            translation.x - target.x,
            translation.y - target.y,
        )
        if position_error > target.max_start_position_error_m:
            return False, (
                'pivot start position error {:.3f} m exceeds {:.3f} m'
            ).format(position_error, target.max_start_position_error_m), 0.0
        deadline = time.monotonic() + self._pivot_blocked_wait_sec
        while True:
            valid, message = self._pivot_validation_callback(target)
            if valid:
                break
            if time.monotonic() >= deadline:
                return False, message, 0.0
            time.sleep(0.25)
        rotation = transform.transform.rotation
        current_yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        angle = math.atan2(
            math.sin(target.yaw - current_yaw),
            math.cos(target.yaw - current_yaw),
        )
        if abs(angle) <= self._pivot_yaw_tolerance_rad:
            return True, 'pivot target yaw is already reached', 0.0
        return True, '', angle

    def point_in_frame(self, target: PoseTarget, destination_frame: str):
        if target.frame_id == destination_frame:
            return target.x, target.y
        transform = self._tf_buffer.lookup_transform(
            destination_frame,
            target.frame_id,
            Time(),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return (
            translation.x + target.x * math.cos(yaw) - target.y * math.sin(yaw),
            translation.y + target.x * math.sin(yaw) + target.y * math.cos(yaw),
        )

    def pose_in_frame(self, target: PoseTarget, destination_frame: str):
        if target.frame_id == destination_frame:
            return target.x, target.y, target.yaw
        transform = self._tf_buffer.lookup_transform(
            destination_frame,
            target.frame_id,
            Time(),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return (
            translation.x + target.x * math.cos(yaw) - target.y * math.sin(yaw),
            translation.y + target.x * math.sin(yaw) + target.y * math.cos(yaw),
            math.atan2(math.sin(yaw + target.yaw), math.cos(yaw + target.yaw)),
        )


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
        self.declare_parameter(
            'pivot_relative_action',
            '/forklift/fine_motion/pivot_relative',
        )
        self.declare_parameter('enforce_pallet_approach_station', True)
        self.declare_parameter('rviz_goal_mode', 'navigation')
        self.declare_parameter('legacy_goal_pose_enabled', False)
        self.declare_parameter(
            'navigation_goal_topic', '/forklift/navigation_goal'
        )
        self.declare_parameter('pallet_goal_topic', '/forklift/pallet_goal')
        self.declare_parameter('compute_path_timeout_sec', 12.0)
        self.declare_parameter('pallet_selection_total_timeout_sec', 20.0)
        self.declare_parameter('hierarchical_navigation_enabled', True)
        self.declare_parameter('topology_planner_id', 'TopologyOnly')
        self.declare_parameter('continuous_anchor_path_enabled', True)
        self.declare_parameter(
            'continuous_topology_planner_id', 'TopologyContinuous'
        )
        self.declare_parameter('anchor_preferred_spacing_m', 4.0)
        self.declare_parameter('anchor_min_spacing_m', 1.5)
        self.declare_parameter('anchor_max_spacing_m', 5.0)
        self.declare_parameter('anchor_min_clearance_m', 0.25)
        self.declare_parameter('anchor_approach_clearance_m', 0.50)
        self.declare_parameter('anchor_max_count', 20)
        self.declare_parameter('anchor_split_max_depth', 4)
        self.declare_parameter('anchor_segment_retry_count', 2)
        self.declare_parameter('anchor_arrival_tolerance_m', 0.35)
        self.declare_parameter('anchor_arrival_speed_mps', 0.03)
        self.declare_parameter('anchor_arrival_settle_sec', 0.30)
        self.declare_parameter('anchor_arrival_transition_delay_sec', 0.25)
        self.declare_parameter('anchor_collision_cost_threshold', 254)
        self.declare_parameter('anchor_force_corner_anchors', False)
        self.declare_parameter('anchor_topology_sample_spacing_m', 0.25)
        self.declare_parameter(
            'cancel_external_nav2_goals_on_pallet_goal', True
        )
        self.declare_parameter('pallet_standoff_distance_m', -1.0)
        self.declare_parameter('pallet_fork_tip_offset_m', 1.59)
        self.declare_parameter('pallet_stop_clearance_m', 0.30)
        self.declare_parameter('pallet_final_approach_distance_m', 1.00)
        self.declare_parameter('pallet_alignment_runup_distance_m', 0.60)
        self.declare_parameter('pallet_final_approach_speed_mps', 0.10)
        self.declare_parameter('pallet_final_approach_timeout_sec', 20.0)
        self.declare_parameter('pallet_max_start_position_error_m', 0.35)
        self.declare_parameter('pallet_max_start_heading_error_rad', 0.20)
        self.declare_parameter('pallet_forks_on_negative_x', True)
        self.declare_parameter('pallet_arrow_points_outward', True)
        self.declare_parameter('pallet_base_frame_id', 'base_link')
        self.declare_parameter('pallet_pivot_speed_mps', 0.10)
        self.declare_parameter('pallet_pivot_timeout_sec', 30.0)
        self.declare_parameter('pallet_turn_min_distance_m', 2.50)
        self.declare_parameter('pallet_turn_preferred_max_distance_m', 4.00)
        self.declare_parameter('pallet_turn_max_distance_m', 5.00)
        self.declare_parameter('pallet_turn_distance_step_m', 0.25)
        self.declare_parameter('pallet_turn_lateral_max_m', 1.50)
        self.declare_parameter('pallet_turn_lateral_step_m', 0.25)
        self.declare_parameter('pallet_turn_runup_distance_m', 0.60)
        self.declare_parameter('pallet_turn_clearance_padding_m', 0.10)
        self.declare_parameter('pallet_turn_keepout_radius_m', 1.50)
        self.declare_parameter('pallet_turn_collision_cost_threshold', 254)
        self.declare_parameter('pallet_turn_blocked_wait_sec', 5.0)
        self.declare_parameter('pallet_nav_arrival_tolerance_m', 0.35)
        self.declare_parameter('pallet_nav_arrival_speed_mps', 0.03)
        self.declare_parameter('pallet_nav_arrival_settle_sec', 0.30)
        self.declare_parameter('pallet_pivot_verify_timeout_sec', 3.0)
        self.declare_parameter('pallet_pivot_yaw_tolerance_rad', 0.05)
        self.declare_parameter('pallet_pivot_yaw_rate_tolerance_radps', 0.03)
        self.declare_parameter('pallet_pivot_feedback_timeout_sec', 0.5)
        self.declare_parameter('pallet_pivot_odom_topic', '/odom')
        self.declare_parameter(
            'pallet_vehicle_footprint',
            '[[1.709, 0.610], [1.709, -0.610], '
            '[-1.590, -0.610], [-1.590, 0.610]]',
        )
        self.declare_parameter(
            'pallet_global_costmap_topic', '/global_costmap/costmap_raw'
        )
        self.declare_parameter(
            'pallet_local_costmap_topic', '/local_costmap/costmap_raw'
        )
        self.declare_parameter('pallet_exemption_activation_distance_m', 4.0)
        self.declare_parameter('pallet_exemption_deactivation_distance_m', 4.5)
        self.declare_parameter('pallet_exemption_length_m', 1.80)
        self.declare_parameter('pallet_exemption_width_m', 1.60)
        self.declare_parameter('pallet_reachability_max_candidates', 32)
        self.declare_parameter('pallet_keepout_publish_settle_sec', 0.15)

        stations_file = self.get_parameter('stations_file').value
        routes_file = self.get_parameter('routes_file').value
        max_retries = int(self.get_parameter('max_retries').value)
        action_name = self.get_parameter('navigate_to_pose_action').value
        pallet_slots_file = self.get_parameter('pallet_slots_file').value
        fork_action_name = self.get_parameter('fork_move_to_action').value
        detect_action_name = self.get_parameter('detect_pallet_offset_action').value
        move_relative_action_name = self.get_parameter('move_relative_action').value
        pivot_relative_action_name = self.get_parameter('pivot_relative_action').value
        self._enforce_pallet_approach_station = bool(
            self.get_parameter('enforce_pallet_approach_station').value
        )
        self._rviz_goal_mode = str(
            self.get_parameter('rviz_goal_mode').value
        ).strip().lower()
        self._legacy_goal_pose_enabled = bool(
            self.get_parameter('legacy_goal_pose_enabled').value
        )
        self._compute_path_timeout_sec = max(
            0.1, float(self.get_parameter('compute_path_timeout_sec').value)
        )
        self._hierarchical_navigation_enabled = bool(
            self.get_parameter('hierarchical_navigation_enabled').value
        )
        self._topology_planner_id = str(
            self.get_parameter('topology_planner_id').value
        ).strip()
        if self._hierarchical_navigation_enabled and not self._topology_planner_id:
            raise ValueError('topology_planner_id must not be empty')
        self._continuous_anchor_path_enabled = bool(
            self.get_parameter('continuous_anchor_path_enabled').value
        )
        self._continuous_topology_planner_id = str(
            self.get_parameter('continuous_topology_planner_id').value
        ).strip()
        if (
            self._continuous_anchor_path_enabled
            and not self._continuous_topology_planner_id
        ):
            raise ValueError(
                'continuous_topology_planner_id must not be empty when '
                'continuous_anchor_path_enabled is true'
            )
        self._pallet_selection_total_timeout_sec = max(
            self._compute_path_timeout_sec,
            float(
                self.get_parameter('pallet_selection_total_timeout_sec').value
            ),
        )
        self._cancel_external_nav2_goals_on_pallet_goal = bool(
            self.get_parameter(
                'cancel_external_nav2_goals_on_pallet_goal'
            ).value
        )
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
        self._pallet_exemption_length_m = float(
            self.get_parameter('pallet_exemption_length_m').value
        )
        self._pallet_exemption_width_m = float(
            self.get_parameter('pallet_exemption_width_m').value
        )
        if (
            not math.isfinite(self._pallet_exemption_length_m)
            or self._pallet_exemption_length_m <= 0.0
            or not math.isfinite(self._pallet_exemption_width_m)
            or self._pallet_exemption_width_m <= 0.0
        ):
            raise ValueError('pallet exemption dimensions must be positive')
        self._pallet_reachability_max_candidates = int(
            self.get_parameter('pallet_reachability_max_candidates').value
        )
        if self._pallet_reachability_max_candidates <= 0:
            raise ValueError('pallet_reachability_max_candidates must be positive')
        self._pallet_keepout_publish_settle_sec = max(
            0.0,
            float(self.get_parameter('pallet_keepout_publish_settle_sec').value),
        )
        self._pallet_approach_config = PalletApproachConfig(
            standoff_distance_m=float(
                self.get_parameter('pallet_standoff_distance_m').value
            ),
            fork_tip_offset_m=float(
                self.get_parameter('pallet_fork_tip_offset_m').value
            ),
            stop_clearance_m=float(
                self.get_parameter('pallet_stop_clearance_m').value
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
            pivot_speed_mps=float(
                self.get_parameter('pallet_pivot_speed_mps').value
            ),
            pivot_timeout_sec=float(
                self.get_parameter('pallet_pivot_timeout_sec').value
            ),
        )
        try:
            footprint = tuple(
                (float(point[0]), float(point[1]))
                for point in ast.literal_eval(
                    str(self.get_parameter('pallet_vehicle_footprint').value)
                )
            )
        except (SyntaxError, TypeError, ValueError) as exc:
            raise ValueError('invalid pallet_vehicle_footprint: {}'.format(exc))
        self._pallet_maneuver_config = PalletManeuverConfig(
            fork_tip_offset_m=float(
                self.get_parameter('pallet_fork_tip_offset_m').value
            ),
            stop_clearance_m=float(
                self.get_parameter('pallet_stop_clearance_m').value
            ),
            turn_min_distance_m=float(
                self.get_parameter('pallet_turn_min_distance_m').value
            ),
            turn_preferred_max_distance_m=float(
                self.get_parameter('pallet_turn_preferred_max_distance_m').value
            ),
            turn_max_distance_m=float(
                self.get_parameter('pallet_turn_max_distance_m').value
            ),
            turn_distance_step_m=float(
                self.get_parameter('pallet_turn_distance_step_m').value
            ),
            turn_lateral_max_m=float(
                self.get_parameter('pallet_turn_lateral_max_m').value
            ),
            turn_lateral_step_m=float(
                self.get_parameter('pallet_turn_lateral_step_m').value
            ),
            turn_runup_distance_m=float(
                self.get_parameter('pallet_turn_runup_distance_m').value
            ),
            turn_clearance_padding_m=float(
                self.get_parameter('pallet_turn_clearance_padding_m').value
            ),
            pallet_turn_keepout_radius_m=float(
                self.get_parameter('pallet_turn_keepout_radius_m').value
            ),
            pallet_exemption_length_m=self._pallet_exemption_length_m,
            pallet_exemption_width_m=self._pallet_exemption_width_m,
            collision_cost_threshold=int(
                self.get_parameter('pallet_turn_collision_cost_threshold').value
            ),
            footprint=footprint,
        )
        validate_maneuver_config(self._pallet_maneuver_config)
        self._anchor_route_config = AnchorRouteConfig(
            preferred_spacing_m=float(
                self.get_parameter('anchor_preferred_spacing_m').value
            ),
            min_spacing_m=float(
                self.get_parameter('anchor_min_spacing_m').value
            ),
            max_spacing_m=float(
                self.get_parameter('anchor_max_spacing_m').value
            ),
            min_clearance_m=float(
                self.get_parameter('anchor_min_clearance_m').value
            ),
            approach_clearance_m=float(
                self.get_parameter('anchor_approach_clearance_m').value
            ),
            max_count=int(self.get_parameter('anchor_max_count').value),
            split_max_depth=int(
                self.get_parameter('anchor_split_max_depth').value
            ),
            segment_retry_count=int(
                self.get_parameter('anchor_segment_retry_count').value
            ),
            arrival_tolerance_m=float(
                self.get_parameter('anchor_arrival_tolerance_m').value
            ),
            arrival_speed_mps=float(
                self.get_parameter('anchor_arrival_speed_mps').value
            ),
            arrival_settle_sec=float(
                self.get_parameter('anchor_arrival_settle_sec').value
            ),
            arrival_transition_delay_sec=float(
                self.get_parameter('anchor_arrival_transition_delay_sec').value
            ),
            collision_cost_threshold=int(
                self.get_parameter('anchor_collision_cost_threshold').value
            ),
            force_corner_anchors=bool(
                self.get_parameter('anchor_force_corner_anchors').value
            ),
            topology_sample_spacing_m=float(
                self.get_parameter('anchor_topology_sample_spacing_m').value
            ),
            footprint=footprint,
        )
        validate_anchor_config(self._anchor_route_config)
        self._pallet_turn_blocked_wait_sec = float(
            self.get_parameter('pallet_turn_blocked_wait_sec').value
        )
        if self._pallet_turn_blocked_wait_sec < 0.0:
            raise ValueError('pallet_turn_blocked_wait_sec must not be negative')
        self._pallet_nav_arrival_tolerance_m = float(
            self.get_parameter('pallet_nav_arrival_tolerance_m').value
        )
        self._pallet_nav_arrival_speed_mps = float(
            self.get_parameter('pallet_nav_arrival_speed_mps').value
        )
        self._pallet_nav_arrival_settle_sec = float(
            self.get_parameter('pallet_nav_arrival_settle_sec').value
        )
        if self._pallet_nav_arrival_tolerance_m <= 0.0:
            raise ValueError('pallet_nav_arrival_tolerance_m must be positive')
        if self._pallet_nav_arrival_speed_mps < 0.0:
            raise ValueError('pallet_nav_arrival_speed_mps must not be negative')
        if self._pallet_nav_arrival_settle_sec < 0.0:
            raise ValueError('pallet_nav_arrival_settle_sec must not be negative')
        if self._pallet_approach_config.standoff_distance_m > 0.0:
            self.get_logger().warning(
                'pallet_standoff_distance_m is deprecated; using its explicit '
                'base_link stop distance override'
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
            for name in (
                'staging_runup', 'staging', 'alignment', 'pre_approach', 'stop'
            )
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
        self._navigation_anchor_index_pub = self.create_publisher(
            Int32,
            '/forklift/navigation_anchor_index',
            status_qos,
        )
        self._pallet_exemption_target: Optional[PoseTarget] = None
        self._pallet_exemption_active = False
        self._pallet_exemption_phase_enabled = False
        self._pallet_selection_token = 0
        self._pallet_selection_active = False
        self._pallet_selection_deadline = 0.0
        self._latest_global_costmap = None
        self._latest_local_costmap = None

        self._pickup_devices = PickupDeviceActions(
            self,
            fork_action_name,
            detect_action_name,
            move_relative_action_name,
            pivot_relative_action_name,
        )
        self._nav2_navigator = Nav2Navigator(self, action_name)
        self._hierarchical_navigator = HierarchicalNav2Navigator(
            self,
            self._nav2_navigator,
            lambda: self._latest_global_costmap,
            self._anchor_route_config,
            self._topology_planner_id,
            self._compute_path_timeout_sec,
            self._publish_navigation_anchor_index,
            self._continuous_topology_planner_id,
            self._continuous_anchor_path_enabled,
        )
        route_navigator = (
            self._hierarchical_navigator
            if self._hierarchical_navigation_enabled
            else self._nav2_navigator
        )
        self._navigator = TaskMotionAdapter(
            self,
            route_navigator,
            self._pickup_devices,
            str(self.get_parameter('pallet_base_frame_id').value),
            self._set_pallet_exemption,
            self._pivot_space_is_clear,
            self._pallet_turn_blocked_wait_sec,
            self._pallet_nav_arrival_tolerance_m,
            self._pallet_nav_arrival_speed_mps,
            self._pallet_nav_arrival_settle_sec,
            pivot_verify_timeout_sec=float(
                self.get_parameter('pallet_pivot_verify_timeout_sec').value),
            pivot_yaw_tolerance_rad=float(
                self.get_parameter('pallet_pivot_yaw_tolerance_rad').value),
            pivot_yaw_rate_tolerance_radps=float(
                self.get_parameter('pallet_pivot_yaw_rate_tolerance_radps').value),
            pivot_feedback_timeout_sec=float(
                self.get_parameter('pallet_pivot_feedback_timeout_sec').value),
            pivot_odom_topic=str(self.get_parameter('pallet_pivot_odom_topic').value),
        )
        self._hierarchical_navigator.set_anchor_arrival_checker(
            self._navigator.anchor_arrival_status
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
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('navigation_goal_topic').value),
            self._navigation_goal_pose,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('pallet_goal_topic').value),
            self._pallet_goal_pose,
            10,
        )
        if self._legacy_goal_pose_enabled:
            self.create_subscription(PoseStamped, '/goal_pose', self._goal_pose, 10)
        self.create_subscription(
            String, '/forklift/safety_gate/status', self._safety_status, 10
        )
        self.create_subscription(
            Costmap,
            str(self.get_parameter('pallet_global_costmap_topic').value),
            self._on_global_costmap,
            1,
        )
        self.create_subscription(
            Costmap,
            str(self.get_parameter('pallet_local_costmap_topic').value),
            self._on_local_costmap,
            1,
        )
        self.create_timer(0.1, self._update_pallet_exemption)
        self._publish_pallet_exemption()
        self._publish_navigation_status(self._machine)
        self.get_logger().info(
            'Loaded {} stations, {} routes, and {} pallet slots; RViz goal mode={} '
            'legacy_goal_pose={} hierarchical_navigation={} topology_planner={} '
            'continuous_anchor_path={} continuous_planner={}.'.format(
                len(self._stations), len(self._routes), len(self._pallet_config.slots),
                self._rviz_goal_mode,
                self._legacy_goal_pose_enabled,
                self._hierarchical_navigation_enabled,
                self._topology_planner_id,
                self._continuous_anchor_path_enabled,
                self._continuous_topology_planner_id,
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
        if self._pallet_selection_active:
            self._pallet_selection_token += 1
            self._pallet_selection_active = False
            self._nav2_navigator.cancel_path_goal()
            self._clear_pallet_exemption()
            response.success = True
            response.message = 'pallet candidate selection canceled'
            return response
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
        """Optional compatibility entry point for old RViz configurations."""
        if self._rviz_goal_mode == 'pallet_approach':
            self._pallet_goal_pose(pose)
        else:
            self._navigation_goal_pose(pose)

    @staticmethod
    def _pose_yaw(pose: PoseStamped) -> float:
        orientation = pose.pose.orientation
        return math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )

    def _pallet_work_active(self) -> bool:
        return self._pallet_selection_active or (
            self._machine.state in {RUNNING, PAUSED, RECOVERING}
            and self._machine.active_route == 'rviz_pallet_approach'
        )

    def _fail_pallet_selection(self, reason: str) -> None:
        self._pallet_selection_active = False
        self._nav2_navigator.cancel_path_goal()
        self._clear_pallet_exemption()
        self._machine.reject_pending('rviz_pallet_approach', reason)
        self.get_logger().error(reason)

    def _navigation_goal_pose(self, pose: PoseStamped) -> None:
        if (
            self._pickup_machine.state in {RUNNING, PAUSED}
            or self._pallet_work_active()
        ):
            self.get_logger().warning(
                'Rejecting navigation goal while a pallet task owns motion control.'
            )
            return
        if self._machine.state in {RUNNING, PAUSED, RECOVERING}:
            self._machine.cancel('replaced by a newer navigation goal')
        yaw = self._pose_yaw(pose)
        target = PoseTarget(
            name='navigation_goal',
            x=float(pose.pose.position.x),
            y=float(pose.pose.position.y),
            yaw=yaw,
            frame_id=pose.header.frame_id or 'map',
        )
        route = RouteDefinition(
            name='rviz_navigation',
            loop=False,
            targets=(target,),
        )
        accepted, message = self._machine.start(route, loop=False)
        if not accepted:
            self.get_logger().warning(message)

    def _pallet_goal_pose(self, pose: PoseStamped) -> None:
        if self._pickup_machine.state in {RUNNING, PAUSED}:
            self.get_logger().warning(
                'Rejecting pallet goal while pallet pickup is active; cancel it first.'
            )
            return
        if self._pallet_work_active():
            self.get_logger().warning(
                'Rejecting pallet goal while another pallet approach is active.'
            )
            return
        if self._machine.state in {RUNNING, PAUSED, RECOVERING}:
            self._machine.cancel('preempted by pallet approach goal')
        if self._cancel_external_nav2_goals_on_pallet_goal:
            self._nav2_navigator.cancel_all_goals()
        self._start_rviz_pallet_approach(pose, self._pose_yaw(pose))

    def _start_rviz_pallet_approach(
        self,
        pallet_pose: PoseStamped,
        pallet_yaw: float,
    ) -> None:
        if self._latest_global_costmap is None:
            self._fail_pallet_selection(
                'Rejecting pallet goal: global costmap is unavailable'
            )
            return
        frame_id = pallet_pose.header.frame_id or 'map'
        costmap_frame = self._latest_global_costmap.header.frame_id
        if costmap_frame and frame_id != costmap_frame:
            self._fail_pallet_selection(
                'Rejecting pallet goal: pallet frame {} does not match '
                'global costmap frame {}'.format(frame_id, costmap_frame)
            )
            return
        outward_yaw = (
            pallet_yaw
            if self._pallet_approach_config.arrow_points_outward
            else pallet_yaw + math.pi
        )
        outward_yaw = math.atan2(math.sin(outward_yaw), math.cos(outward_yaw))
        # Publish the selected pallet before ComputePathToPose. The global
        # planner treats this fresh pose as a keepout while exemption_active is
        # false, so navigation reaches the requested normal side without
        # cutting through a pallet that is absent from the static map.
        self._prepare_pallet_exemption(
            PoseTarget(
                name='pallet_planning_keepout',
                x=float(pallet_pose.pose.position.x),
                y=float(pallet_pose.pose.position.y),
                yaw=outward_yaw,
                frame_id=frame_id,
            )
        )
        try:
            candidates = select_clear_candidates(
                costmap=self._latest_global_costmap,
                pallet_x=float(pallet_pose.pose.position.x),
                pallet_y=float(pallet_pose.pose.position.y),
                outward_yaw=outward_yaw,
                frame_id=frame_id,
                config=self._pallet_maneuver_config,
                max_candidates=self._pallet_reachability_max_candidates,
            )
            if not candidates:
                raise PalletApproachError(
                    'no collision-free staging candidate in the 2.5-5.0 m search area'
                )
        except (PalletApproachError, ValueError) as exc:
            self._fail_pallet_selection(
                'Rejecting pallet goal: {}'.format(exc)
            )
            return

        self._pallet_selection_token += 1
        selection_token = self._pallet_selection_token
        self._pallet_selection_active = True
        self._pallet_selection_deadline = (
            time.monotonic() + self._pallet_selection_total_timeout_sec
        )

        def begin_candidate_search() -> None:
            if selection_token != self._pallet_selection_token:
                return
            self._try_reachable_pallet_candidate(
                selection_token=selection_token,
                candidates=candidates,
                candidate_index=0,
                pallet_x=float(pallet_pose.pose.position.x),
                pallet_y=float(pallet_pose.pose.position.y),
                pallet_yaw=pallet_yaw,
                frame_id=frame_id,
            )

        if self._pallet_keepout_publish_settle_sec <= 0.0:
            begin_candidate_search()
            return

        timer_holder = {}

        def delayed_candidate_search() -> None:
            timer = timer_holder.get('timer')
            if timer is not None:
                timer.cancel()
                self.destroy_timer(timer)
            begin_candidate_search()

        timer_holder['timer'] = self.create_timer(
            self._pallet_keepout_publish_settle_sec,
            delayed_candidate_search,
        )

    def _try_reachable_pallet_candidate(
        self,
        *,
        selection_token: int,
        candidates,
        candidate_index: int,
        pallet_x: float,
        pallet_y: float,
        pallet_yaw: float,
        frame_id: str,
    ) -> None:
        if selection_token != self._pallet_selection_token:
            return
        if time.monotonic() >= self._pallet_selection_deadline:
            self._fail_pallet_selection(
                'Pallet staging selection exceeded {:.1f} s'.format(
                    self._pallet_selection_total_timeout_sec)
            )
            return
        if candidate_index >= len(candidates):
            self._fail_pallet_selection(
                'No globally reachable pallet staging '
                'candidate among {} locally clear candidates'.format(
                    len(candidates)
                )
            )
            return

        selected = candidates[candidate_index]

        def path_checked(success: bool, message: str) -> None:
            try:
                if selection_token != self._pallet_selection_token:
                    return
                if success:
                    self._pallet_selection_active = False
                    self._start_selected_pallet_approach(
                        selected=selected,
                        candidate_index=candidate_index,
                        candidate_count=len(candidates),
                        pallet_x=pallet_x,
                        pallet_y=pallet_y,
                        pallet_yaw=pallet_yaw,
                        frame_id=frame_id,
                    )
                    return
                remaining = max(
                    0.1, self._pallet_selection_deadline - time.monotonic()
                )

                def topology_checked(topology_success, topology_message, _path):
                    if selection_token != self._pallet_selection_token:
                        return
                    if topology_success:
                        self._pallet_selection_active = False
                        self.get_logger().warning(
                            'Pallet staging candidate {}/{} requires anchor '
                            'fallback after continuous failure: {}'.format(
                                candidate_index + 1, len(candidates), message
                            )
                        )
                        self._start_selected_pallet_approach(
                            selected=selected,
                            candidate_index=candidate_index,
                            candidate_count=len(candidates),
                            pallet_x=pallet_x,
                            pallet_y=pallet_y,
                            pallet_yaw=pallet_yaw,
                            frame_id=frame_id,
                        )
                        return
                    self.get_logger().warning(
                        'Pallet staging candidate {}/{} has no topology path: {}'
                        .format(
                            candidate_index + 1,
                            len(candidates),
                            topology_message,
                        )
                    )
                    self._try_reachable_pallet_candidate(
                        selection_token=selection_token,
                        candidates=candidates,
                        candidate_index=candidate_index + 1,
                        pallet_x=pallet_x,
                        pallet_y=pallet_y,
                        pallet_yaw=pallet_yaw,
                        frame_id=frame_id,
                    )

                self._nav2_navigator.compute_path_result(
                    selected.staging,
                    topology_checked,
                    timeout_sec=min(self._compute_path_timeout_sec, remaining),
                    planner_id=self._topology_planner_id,
                )
            except Exception:
                self._fail_pallet_selection(
                    'Pallet staging candidate callback failed; task was not '
                    'started:\n{}'.format(traceback.format_exc())
                )

        remaining_sec = max(0.1, self._pallet_selection_deadline - time.monotonic())
        self._nav2_navigator.compute_path(
            selected.staging,
            path_checked,
            timeout_sec=min(self._compute_path_timeout_sec, remaining_sec),
        )

    def _start_selected_pallet_approach(
        self,
        *,
        selected,
        candidate_index: int,
        candidate_count: int,
        pallet_x: float,
        pallet_y: float,
        pallet_yaw: float,
        frame_id: str,
    ) -> None:
        try:
            geometry = build_selected_pallet_approach(
                pallet_x=pallet_x,
                pallet_y=pallet_y,
                pallet_yaw=pallet_yaw,
                frame_id=frame_id,
                config=self._pallet_approach_config,
                maneuver_config=self._pallet_maneuver_config,
                candidate=selected,
            )
        except (PalletApproachError, ValueError) as exc:
            self._fail_pallet_selection(
                'Rejecting globally reachable pallet candidate: {}'.format(exc)
            )
            return

        visualization_targets = {
            'staging_runup': geometry.staging_runup,
            'staging': geometry.staging,
            'alignment': geometry.alignment,
            'pre_approach': geometry.pre_approach,
            'stop': geometry.stop,
        }
        for name, target in visualization_targets.items():
            if target is None:
                continue
            self._publish_approach_pose(name, target)

        route = build_pallet_approach_route(geometry)
        route_exemption = TaskMotionAdapter._pallet_exemption_pose(
            geometry.final_motion
        )
        # The pallet remains an obstacle during staging navigation and pivot.
        # Its exemption is activated by the final straight approach only.
        self._prepare_pallet_exemption(route_exemption)
        accepted, message = self._machine.start(route, loop=False)
        if not accepted:
            self._clear_pallet_exemption()
            self.get_logger().warning(message)
            return
        self.get_logger().info(
            'Accepted pallet goal ({:.3f}, {:.3f}, yaw {:.3f}); staging='
            '({:.3f}, {:.3f}) distance={:.2f} m lateral={:.2f} m '
            'candidate={}/{} mode={}; '
            'stop=({:.3f}, {:.3f}), final_motion={:.3f} m at {:.3f} m/s.'.format(
                pallet_x,
                pallet_y,
                pallet_yaw,
                selected.staging.x,
                selected.staging.y,
                selected.distance_m,
                selected.lateral_m,
                candidate_index + 1,
                candidate_count,
                'single_pivot',
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

    def _publish_navigation_anchor_index(self, anchor_index: int) -> None:
        message = Int32()
        message.data = int(anchor_index)
        self._navigation_anchor_index_pub.publish(message)

    def _set_pallet_exemption(self, target: Optional[PoseTarget]) -> None:
        # The final relative segment re-publishes the same pallet pose. A None
        # callback means that segment ended; proximity, not segment completion,
        # owns the lifetime of the exemption.
        if target is not None:
            self._pallet_exemption_phase_enabled = True
            self._arm_pallet_exemption(target)

    def _prepare_pallet_exemption(
        self,
        target: Optional[PoseTarget],
    ) -> None:
        self._pallet_exemption_target = target
        self._pallet_exemption_phase_enabled = False
        self._pallet_exemption_active = False
        self._publish_pallet_exemption()

    def _arm_pallet_exemption(
        self,
        target: Optional[PoseTarget],
    ) -> None:
        self._pallet_exemption_target = target
        self._pallet_exemption_active = False
        self._update_pallet_exemption()

    def _clear_pallet_exemption(self) -> None:
        self._pallet_exemption_target = None
        self._pallet_exemption_phase_enabled = False
        self._pallet_exemption_active = False
        self._publish_pallet_exemption()

    def _update_pallet_exemption(self) -> None:
        target = self._pallet_exemption_target
        if target is None:
            self._publish_pallet_exemption()
            return
        if not self._pallet_exemption_phase_enabled:
            self._pallet_exemption_active = False
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

    def _on_global_costmap(self, msg: Costmap) -> None:
        self._latest_global_costmap = msg

    def _on_local_costmap(self, msg: Costmap) -> None:
        self._latest_local_costmap = msg

    def _pivot_space_is_clear(self, target: PivotTarget):
        costmap = self._latest_local_costmap
        if costmap is None:
            return False, 'local costmap unavailable for pivot revalidation'
        frame_id = costmap.header.frame_id
        if not frame_id:
            return False, 'local costmap frame is empty'
        try:
            x, y = self._navigator.point_in_frame(target, frame_id)
        except Exception as exc:
            return False, 'pivot costmap TF unavailable: {}'.format(exc)
        radius = max(
            math.hypot(px, py)
            for px, py in self._pallet_maneuver_config.footprint
        ) + self._pallet_maneuver_config.turn_clearance_padding_m
        pallet_exemption = None
        if self._pallet_exemption_active and self._pallet_exemption_target is not None:
            try:
                exemption_x, exemption_y, exemption_yaw = (
                    self._navigator.pose_in_frame(
                        self._pallet_exemption_target, frame_id
                    )
                )
                pallet_exemption = (
                    exemption_x,
                    exemption_y,
                    exemption_yaw,
                    0.5 * self._pallet_exemption_length_m,
                    0.5 * self._pallet_exemption_width_m,
                )
            except Exception as exc:
                return False, 'pivot pallet exemption TF unavailable: {}'.format(exc)
        if not circular_turn_is_clear(
            costmap,
            (x, y),
            radius,
            self._pallet_maneuver_config.collision_cost_threshold,
            pallet_exemption=pallet_exemption,
        ):
            return False, 'pivot sweep is blocked in the latest local costmap'
        return True, ''

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
