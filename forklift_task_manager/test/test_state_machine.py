import inspect

from forklift_task_manager.task_manager_node import ForkliftTaskManager
from forklift_task_manager.route_model import (
    PoseTarget,
    RelativeMoveTarget,
    RouteDefinition,
)
from forklift_task_manager.state_machine import (
    FAILED,
    IDLE,
    PAUSED,
    RECOVERING,
    RUNNING,
    SUCCEEDED,
    TaskStateMachine,
)


class MockActionClient:
    def __init__(self):
        self.sent = []
        self.callbacks = []
        self.cancel_count = 0

    def send_goal(self, target, callback):
        self.sent.append(target)
        self.callbacks.append(callback)

    def cancel_goal(self):
        self.cancel_count += 1

    def complete_latest(self, success=True, message=''):
        self.callbacks[-1](success, message)


def test_execute_route_callback_is_synchronous():
    assert inspect.iscoroutinefunction(ForkliftTaskManager._execute_route) is False


def make_route(loop=False):
    return RouteDefinition(
        name='test_route',
        loop=loop,
        targets=(
            PoseTarget('first', 0.0, 0.0, 0.0),
            PoseTarget('corner', 2.0, 0.0, 1.57),
        ),
    )


def test_state_machine_advances_segments_and_succeeds():
    action_client = MockActionClient()
    machine = TaskStateMachine(action_client)

    accepted, _ = machine.start(make_route())
    assert accepted is True
    assert machine.state == RUNNING
    assert [target.name for target in action_client.sent] == ['first']

    action_client.complete_latest(success=True)
    assert machine.segment_index == 1
    assert [target.name for target in action_client.sent] == ['first', 'corner']

    action_client.complete_latest(success=True)
    assert machine.state == SUCCEEDED
    assert machine.reason == 'route completed'


def test_loop_route_wraps_to_first_segment():
    action_client = MockActionClient()
    machine = TaskStateMachine(action_client)
    machine.start(make_route(loop=True))

    action_client.complete_latest(success=True)
    action_client.complete_latest(success=True)

    assert machine.state == RUNNING
    assert machine.segment_index == 0
    assert [target.name for target in action_client.sent] == [
        'first', 'corner', 'first'
    ]


def test_pause_resume_and_cancel_preserve_then_clear_current_segment():
    action_client = MockActionClient()
    machine = TaskStateMachine(action_client)
    machine.start(make_route())

    paused, _ = machine.pause()
    assert paused is True
    assert machine.state == PAUSED
    assert machine.segment_index == 0
    assert action_client.cancel_count == 1

    resumed, _ = machine.resume()
    assert resumed is True
    assert machine.state == RUNNING
    assert [target.name for target in action_client.sent] == ['first', 'first']

    canceled, _ = machine.cancel()
    assert canceled is True
    assert machine.state == IDLE
    assert machine.active_route == ''
    assert machine.segment_index == -1
    assert action_client.cancel_count == 2


def test_safety_emergency_stop_pauses_and_requires_explicit_resume():
    action_client = MockActionClient()
    machine = TaskStateMachine(action_client)
    machine.start(make_route())

    changed = machine.observe_safety_status('emergency stop')
    assert changed is True
    assert machine.state == PAUSED
    assert machine.reason == 'emergency stop'
    assert action_client.cancel_count == 1

    assert machine.observe_safety_status('raw command') is False
    assert machine.state == PAUSED

    machine.resume()
    assert machine.state == RUNNING
    assert len(action_client.sent) == 2


def test_navigation_failure_retries_then_fails():
    action_client = MockActionClient()
    states = []
    machine = TaskStateMachine(
        action_client,
        max_retries=2,
        status_callback=lambda current: states.append(current.state),
    )
    machine.start(make_route())

    action_client.complete_latest(False, 'planner failed')
    assert machine.state == RUNNING
    assert RECOVERING in states
    assert len(action_client.sent) == 2

    action_client.complete_latest(False, 'planner failed')
    assert machine.state == RUNNING
    assert len(action_client.sent) == 3

    action_client.complete_latest(False, 'planner failed')
    assert machine.state == FAILED
    assert machine.reason == 'planner failed after 2 retries'


def test_relative_motion_failure_is_not_retried():
    action_client = MockActionClient()
    route = RouteDefinition(
        name='fine_motion',
        loop=False,
        targets=(RelativeMoveTarget('reverse', -1.0, 0.1, 20.0),),
    )
    machine = TaskStateMachine(action_client, max_retries=2)

    machine.start(route)
    action_client.complete_latest(False, 'no progress')

    assert machine.state == FAILED
    assert machine.reason == 'no progress'
    assert len(action_client.sent) == 1
