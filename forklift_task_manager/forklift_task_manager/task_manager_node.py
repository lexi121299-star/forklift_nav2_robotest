"""ROS 2 node exposing the minimal P7.1 forklift task interface."""

import math
from typing import Callable, List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from forklift_msgs.action import ExecuteRoute
from forklift_msgs.msg import TaskStatus
from forklift_msgs.srv import GoToStation
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .route_model import PoseTarget, RouteConfigError, RouteDefinition
from .route_model import load_routes, load_stations
from .state_machine import PAUSED, RECOVERING, RUNNING, SUCCEEDED
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

        stations_file = self.get_parameter('stations_file').value
        routes_file = self.get_parameter('routes_file').value
        max_retries = int(self.get_parameter('max_retries').value)
        action_name = self.get_parameter('navigate_to_pose_action').value

        try:
            self._stations = load_stations(stations_file)
            self._routes = load_routes(routes_file, self._stations)
        except RouteConfigError as exc:
            self.get_logger().fatal('Task configuration is invalid: {}'.format(exc))
            raise

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_pub = self.create_publisher(
            TaskStatus, '/forklift/task_status', status_qos
        )

        self._navigator = Nav2Navigator(self, action_name)
        self._machine = TaskStateMachine(
            self._navigator,
            max_retries=max_retries,
            status_callback=self._publish_status,
        )

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
        self.create_service(GoToStation, 'go_to_station', self._go_to_station)
        self.create_service(Trigger, 'pause', self._pause)
        self.create_service(Trigger, 'resume', self._resume)
        self.create_service(Trigger, 'cancel', self._cancel)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_pose, 10)
        self.create_subscription(
            String, '/forklift/safety_gate/status', self._safety_status, 10
        )
        self._publish_status(self._machine)
        self.get_logger().info(
            'Loaded {} stations and {} routes.'.format(
                len(self._stations), len(self._routes)
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
        return GoalResponse.ACCEPT

    def _route_cancel(self, _goal_handle) -> CancelResponse:
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

    def _go_to_station(
        self,
        request: GoToStation.Request,
        response: GoToStation.Response,
    ) -> GoToStation.Response:
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
        response.success, response.message = self._machine.pause()
        return response

    def _resume(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        response.success, response.message = self._machine.resume()
        return response

    def _cancel(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        response.success, response.message = self._machine.cancel()
        return response

    def _goal_pose(self, pose: PoseStamped) -> None:
        if self._machine.state in {RUNNING, PAUSED, RECOVERING}:
            self.get_logger().warning(
                'Ignoring /goal_pose while another task is active; cancel it first.'
            )
            return
        yaw = 2.0 * math.atan2(
            pose.pose.orientation.z, pose.pose.orientation.w
        )
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

    def _safety_status(self, status: String) -> None:
        if self._machine.observe_safety_status(status.data):
            self.get_logger().warning(
                'Task paused by safety gate: {}.'.format(status.data)
            )

    def _publish_status(self, machine: TaskStateMachine) -> None:
        status = TaskStatus()
        status.state = machine.state
        status.active_route = machine.active_route
        status.current_segment = machine.current_segment
        status.segment_index = machine.segment_index
        status.segment_count = machine.segment_count
        status.reason = machine.reason
        self._status_pub.publish(status)


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
