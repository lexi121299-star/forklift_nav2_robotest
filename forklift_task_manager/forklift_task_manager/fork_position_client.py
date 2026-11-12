"""Translate task-manager fork targets into TC397 position topic operations."""

import math
import time
from dataclasses import dataclass
from typing import Callable, Optional

from forklift_msgs.msg import ForkPositionCommand, ForkState


@dataclass(frozen=True)
class ForkPositionConfig:
    command_topic: str = '/forklift/fork/command'
    state_topic: str = '/forklift/fork/state'
    state_timeout_sec: float = 0.5
    height_tolerance_m: float = 0.01
    side_shift_tolerance_m: float = 0.005
    tilt_tolerance_rad: float = 0.01
    height_max_speed_mps: float = 0.0
    side_shift_max_speed_mps: float = 0.0
    tilt_max_speed_radps: float = 0.0
    require_height_feedback: bool = True
    require_side_shift_feedback: bool = True
    require_tilt_feedback: bool = True


@dataclass(frozen=True)
class _AxisTarget:
    mode: int
    label: str
    target: float
    tolerance: float
    max_speed: float


class ForkPositionClient:
    """Run one absolute fork pose request through command/state topics.

    The TC397 bridge owns all motor control. This client only sends one axis
    command at a time and waits for the matching command_id in ForkState.
    """

    def __init__(self, node, config: ForkPositionConfig) -> None:
        self._node = node
        self._config = config
        self._command_pub = node.create_publisher(
            ForkPositionCommand, config.command_topic, 10
        )
        self._state_sub = node.create_subscription(
            ForkState, config.state_topic, self._on_state, 10
        )
        self._state: Optional[ForkState] = None
        self._state_received_at = 0.0
        self._operation_token = 0
        self._next_command_id = 1
        self._timer = None
        self._operation = None

    def check_ready(self):
        state, message = self._fresh_state()
        if state is None:
            return False, message
        if state.status == ForkState.STATUS_FAULT or state.fault_code != 0:
            return False, 'fork controller fault {}'.format(state.fault_code)
        required = (
            ('height', state.height_valid, self._config.require_height_feedback),
            (
                'side shift',
                state.side_shift_valid,
                self._config.require_side_shift_feedback,
            ),
            ('tilt', state.tilt_valid, self._config.require_tilt_feedback),
        )
        for name, valid, required_feedback in required:
            if required_feedback and not valid:
                return False, '{} feedback is invalid'.format(name)
        return True, 'fork position feedback is ready'

    def move_to(
        self,
        *,
        target_height_m: float,
        side_shift_m: float,
        tilt_rad: float,
        timeout_sec: float,
        done_callback: Callable,
        feedback_callback: Optional[Callable] = None,
    ) -> None:
        values = (target_height_m, side_shift_m, tilt_rad, timeout_sec)
        if not all(math.isfinite(float(value)) for value in values):
            done_callback(False, 'fork target or timeout is not finite', None)
            return
        if timeout_sec <= 0.0:
            done_callback(False, 'fork timeout must be positive', None)
            return
        ready, message = self.check_ready()
        if not ready:
            done_callback(False, message, None)
            return

        self.cancel(publish_stop=False)
        self._operation_token += 1
        token = self._operation_token
        axes = (
            _AxisTarget(
                ForkPositionCommand.MODE_HEIGHT_POSITION,
                'HEIGHT',
                float(target_height_m),
                self._config.height_tolerance_m,
                self._config.height_max_speed_mps,
            ),
            _AxisTarget(
                ForkPositionCommand.MODE_SIDE_SHIFT_POSITION,
                'SIDE_SHIFT',
                float(side_shift_m),
                self._config.side_shift_tolerance_m,
                self._config.side_shift_max_speed_mps,
            ),
            _AxisTarget(
                ForkPositionCommand.MODE_TILT_POSITION,
                'TILT',
                float(tilt_rad),
                self._config.tilt_tolerance_rad,
                self._config.tilt_max_speed_radps,
            ),
        )
        self._operation = {
            'token': token,
            'axes': axes,
            'axis_index': 0,
            'command_id': None,
            'deadline': time.monotonic() + float(timeout_sec),
            'done_callback': done_callback,
            'feedback_callback': feedback_callback,
        }
        self._timer = self._node.create_timer(0.02, self._tick)
        self._tick()

    def cancel(self, publish_stop: bool = True) -> None:
        self._operation_token += 1
        operation = self._operation
        self._operation = None
        if self._timer is not None:
            timer = self._timer
            self._timer = None
            timer.cancel()
            self._node.destroy_timer(timer)
        if publish_stop and operation is not None:
            command_id = operation.get('command_id')
            self._publish_stop(
                self._next_command_id if command_id is None else command_id
            )

    def _on_state(self, state: ForkState) -> None:
        self._state = state
        self._state_received_at = time.monotonic()
        operation = self._operation
        if operation is not None and operation['feedback_callback'] is not None:
            if state.height_valid:
                operation['feedback_callback'](float(state.height_m))

    def _fresh_state(self):
        if self._state is None:
            return None, 'fork state has not been received'
        age = time.monotonic() - self._state_received_at
        if age > self._config.state_timeout_sec:
            return None, 'fork state is stale for {:.3f} s'.format(age)
        return self._state, ''

    def _tick(self) -> None:
        operation = self._operation
        if operation is None:
            return
        if time.monotonic() >= operation['deadline']:
            self._finish(False, 'fork position operation timed out')
            return
        state, message = self._fresh_state()
        if state is None:
            self._finish(False, message)
            return
        if state.status == ForkState.STATUS_FAULT or state.fault_code != 0:
            self._finish(False, 'fork controller fault {}'.format(state.fault_code))
            return

        axis = operation['axes'][operation['axis_index']]
        if self._axis_reached(axis, state):
            self._advance_axis()
            return
        command_id = operation['command_id']
        if command_id is None:
            command_id = self._allocate_command_id()
            operation['command_id'] = command_id
            self._publish_axis(axis, command_id)
            return
        if state.active_command_id != command_id:
            return
        if state.status == ForkState.STATUS_REACHED:
            if self._axis_reached(axis, state):
                self._advance_axis()
            else:
                self._finish(
                    False,
                    '{} reported reached outside tolerance'.format(axis.label),
                )
        elif state.status == ForkState.STATUS_REJECTED:
            self._finish(False, '{} command was rejected'.format(axis.label))

    def _axis_reached(self, axis: _AxisTarget, state: ForkState) -> bool:
        if axis.mode == ForkPositionCommand.MODE_HEIGHT_POSITION:
            return (
                state.height_valid
                and abs(state.height_m - axis.target) <= axis.tolerance
            )
        if axis.mode == ForkPositionCommand.MODE_SIDE_SHIFT_POSITION:
            return (
                state.side_shift_valid
                and abs(state.side_shift_m - axis.target) <= axis.tolerance
            )
        return state.tilt_valid and abs(state.tilt_rad - axis.target) <= axis.tolerance

    def _advance_axis(self) -> None:
        operation = self._operation
        if operation is None:
            return
        operation['axis_index'] += 1
        operation['command_id'] = None
        if operation['axis_index'] >= len(operation['axes']):
            state = self._state
            result = type('ForkPositionResult', (), {})()
            result.success = True
            result.message = 'fork target reached'
            result.final_height_m = float(state.height_m) if state else 0.0
            self._finish(True, result.message, result)

    def _finish(self, success: bool, message: str, result=None) -> None:
        operation = self._operation
        if operation is None:
            return
        self.cancel(publish_stop=not success)
        operation['done_callback'](success, message, result)

    def _allocate_command_id(self) -> int:
        command_id = self._next_command_id
        self._next_command_id = 1 if command_id >= 0xFFFFFFFF else command_id + 1
        return command_id

    def _publish_axis(self, axis: _AxisTarget, command_id: int) -> None:
        command = ForkPositionCommand()
        command.header.stamp = self._node.get_clock().now().to_msg()
        command.mode = axis.mode
        command.command_id = command_id
        command.target = axis.target
        command.max_speed = axis.max_speed
        self._command_pub.publish(command)

    def _publish_stop(self, command_id: int) -> None:
        command = ForkPositionCommand()
        command.header.stamp = self._node.get_clock().now().to_msg()
        command.mode = ForkPositionCommand.MODE_STOP
        command.command_id = command_id
        command.target = 0.0
        command.max_speed = 0.0
        self._command_pub.publish(command)
