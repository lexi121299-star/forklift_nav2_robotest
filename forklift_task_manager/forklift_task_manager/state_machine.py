"""ROS-independent task state machine used by the task manager node."""

from typing import Callable, Optional

from .route_model import RouteDefinition, TaskTarget


IDLE = 'IDLE'
RUNNING = 'RUNNING'
PAUSED = 'PAUSED'
SUCCEEDED = 'SUCCEEDED'
FAILED = 'FAILED'
RECOVERING = 'RECOVERING'

_ACTIVE_STATES = {RUNNING, PAUSED, RECOVERING}
_SAFETY_PAUSE_REASONS = {
    'emergency stop',
    'vehicle emergency stop',
    'vehicle fault',
}


class TaskStateMachine:
    """Sequence sparse targets through an injected navigation adapter."""

    def __init__(
        self,
        navigator,
        max_retries: int = 1,
        status_callback: Optional[Callable[['TaskStateMachine'], None]] = None,
    ) -> None:
        if max_retries < 0:
            raise ValueError('max_retries must be non-negative')
        self.navigator = navigator
        self.max_retries = int(max_retries)
        self.status_callback = status_callback
        self.state = IDLE
        self.active_route = ''
        self.route: Optional[RouteDefinition] = None
        self.loop = False
        self.segment_index = -1
        self.reason = ''
        self._retry_count = 0
        self._dispatch_token = 0

    @property
    def segment_count(self) -> int:
        return len(self.route.targets) if self.route is not None else 0

    @property
    def current_target(self) -> Optional[TaskTarget]:
        if (
            self.route is None
            or self.segment_index < 0
            or self.segment_index >= len(self.route.targets)
        ):
            return None
        return self.route.targets[self.segment_index]

    @property
    def current_segment(self) -> str:
        target = self.current_target
        return target.name if target is not None else ''

    def start(self, route: RouteDefinition, loop: Optional[bool] = None):
        if self.state in _ACTIVE_STATES:
            return False, 'another task is active'
        if not route.targets:
            return False, 'route has no targets'
        self.route = route
        self.active_route = route.name
        self.loop = route.loop if loop is None else bool(loop)
        self.segment_index = 0
        self._retry_count = 0
        self.reason = ''
        self._set_state(RUNNING)
        self._dispatch_current()
        return True, 'route accepted'

    def pause(self, reason: str = 'paused by request'):
        if self.state not in {RUNNING, RECOVERING}:
            return False, 'task is not running'
        self._invalidate_navigation()
        self.navigator.cancel_goal()
        self.reason = reason
        self._set_state(PAUSED)
        return True, reason

    def resume(self):
        if self.state != PAUSED:
            return False, 'task is not paused'
        self.reason = ''
        self._set_state(RUNNING)
        self._dispatch_current()
        return True, 'task resumed'

    def cancel(self, reason: str = 'canceled by request'):
        if self.state not in _ACTIVE_STATES:
            return False, 'no active task'
        self._invalidate_navigation()
        self.navigator.cancel_goal()
        self.route = None
        self.active_route = ''
        self.segment_index = -1
        self._retry_count = 0
        self.reason = reason
        self._set_state(IDLE)
        return True, reason

    def observe_safety_status(self, reason: str) -> bool:
        normalized = reason.strip().lower()
        if normalized not in _SAFETY_PAUSE_REASONS:
            return False
        if self.state not in {RUNNING, RECOVERING}:
            return False
        self.pause(reason=reason.strip())
        return True

    def _dispatch_current(self) -> None:
        target = self.current_target
        if target is None:
            self._fail('current segment is unavailable')
            return
        self._dispatch_token += 1
        token = self._dispatch_token

        def done(success: bool, message: str = '') -> None:
            if token != self._dispatch_token or self.state != RUNNING:
                return
            if success:
                self._segment_succeeded()
            else:
                self._segment_failed(message or 'NavigateToPose failed')

        try:
            self.navigator.send_goal(target, done)
        except Exception as exc:  # Keep adapter failures inside task semantics.
            done(False, 'failed to send NavigateToPose goal: {}'.format(exc))

    def _segment_succeeded(self) -> None:
        self._retry_count = 0
        if self.segment_index + 1 < self.segment_count:
            self.segment_index += 1
            self.reason = ''
            self._notify()
            self._dispatch_current()
            return
        if self.loop:
            self.segment_index = 0
            self.reason = ''
            self._notify()
            self._dispatch_current()
            return
        self.reason = 'route completed'
        self._set_state(SUCCEEDED)

    def _segment_failed(self, message: str) -> None:
        target = self.current_target
        retryable = bool(getattr(target, 'retryable', True))
        if not retryable:
            self._fail(message)
            return
        if retryable and self._retry_count < self.max_retries:
            self._retry_count += 1
            self.reason = '{}; retry {}/{}'.format(
                message, self._retry_count, self.max_retries
            )
            self._set_state(RECOVERING)
            self._set_state(RUNNING)
            self._dispatch_current()
            return
        self._fail('{} after {} retries'.format(message, self.max_retries))

    def _fail(self, reason: str) -> None:
        self.reason = reason
        self._set_state(FAILED)

    def _invalidate_navigation(self) -> None:
        self._dispatch_token += 1

    def _set_state(self, state: str) -> None:
        self.state = state
        self._notify()

    def _notify(self) -> None:
        if self.status_callback is not None:
            self.status_callback(self)
