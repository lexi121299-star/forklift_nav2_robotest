"""Dispatch-level pickup workflow composed from navigation and fork phases."""

from typing import Callable, Optional

from .pallet_model import PalletSlot, PickupDefaults
from .pallet_pickup_state_machine import PalletPickupStateMachine
from .route_model import RouteDefinition
from .state_machine import FAILED, IDLE, PAUSED, RECOVERING, RUNNING, SUCCEEDED
from .state_machine import TaskStateMachine


NAVIGATE_TO_APPROACH = 'NAVIGATE_TO_APPROACH'
CANCELED = 'CANCELED'

_ACTIVE_STATES = {RUNNING, PAUSED, RECOVERING}


class PickupJobStateMachine:
    """Navigate to a slot approach station before executing pallet pickup."""

    def __init__(
        self,
        navigator,
        devices,
        defaults: PickupDefaults,
        max_retries: int,
        status_callback: Optional[Callable[['PickupJobStateMachine'], None]] = None,
    ) -> None:
        self.status_callback = status_callback
        self.state = IDLE
        self.phase = ''
        self.reason = ''
        self.slot: Optional[PalletSlot] = None
        self._canceled = False
        self._navigation = TaskStateMachine(
            navigator,
            max_retries=max_retries,
            status_callback=self._on_navigation_status,
        )
        self._pickup = PalletPickupStateMachine(
            devices,
            defaults,
            status_callback=self._on_pickup_status,
        )

    @property
    def active_slot(self) -> str:
        return self.slot.name if self.slot is not None else ''

    @property
    def current_segment(self) -> str:
        if self.phase == NAVIGATE_TO_APPROACH:
            return self._navigation.current_segment
        return self._pickup.phase

    @property
    def progress(self) -> float:
        if self.state == SUCCEEDED:
            return 1.0
        if self.phase == NAVIGATE_TO_APPROACH:
            if self._navigation.segment_count <= 0:
                return 0.0
            completed = max(0, self._navigation.segment_index)
            return 0.4 * float(completed) / float(self._navigation.segment_count)
        return 0.4 + 0.6 * self._pickup.progress

    def start(self, slot: PalletSlot, approach_target):
        if self.state in _ACTIVE_STATES:
            return False, 'another pickup job is active'
        self.slot = slot
        self.phase = NAVIGATE_TO_APPROACH
        self.reason = ''
        self._canceled = False
        self._set_state(RUNNING)
        route = RouteDefinition(
            name='pickup_approach:{}'.format(slot.name),
            loop=False,
            targets=(approach_target,),
        )
        accepted, message = self._navigation.start(route)
        if not accepted:
            self._fail('approach navigation was not accepted: {}'.format(message))
        return accepted, message

    def pause(self, reason: str = 'paused by request'):
        if self.state not in {RUNNING, RECOVERING}:
            return False, 'pickup job is not running'
        if self.phase == NAVIGATE_TO_APPROACH:
            accepted, message = self._navigation.pause(reason)
        else:
            accepted, message = self._pickup.pause(reason)
        if accepted:
            self.reason = message
            self._set_state(PAUSED)
        return accepted, message

    def resume(self):
        if self.state != PAUSED:
            return False, 'pickup job is not paused'
        self._canceled = False
        if self.phase == NAVIGATE_TO_APPROACH:
            accepted, message = self._navigation.resume()
        else:
            accepted, message = self._pickup.resume()
        if accepted:
            self.reason = ''
            self._set_state(RUNNING)
        return accepted, message

    def cancel(self, reason: str = 'pickup job canceled'):
        if self.state not in _ACTIVE_STATES:
            return False, 'no active pickup job'
        self._canceled = True
        if self.phase == NAVIGATE_TO_APPROACH:
            self._navigation.cancel(reason)
        else:
            self._pickup.cancel(reason)
        self.reason = reason
        self._set_state(CANCELED)
        return True, reason

    def observe_safety_status(self, reason: str) -> bool:
        if self.phase == NAVIGATE_TO_APPROACH:
            paused = self._navigation.observe_safety_status(reason)
        else:
            paused = self._pickup.observe_safety_status(reason)
        if paused:
            self.reason = reason
            self._set_state(PAUSED)
        return paused

    def _on_navigation_status(self, navigation: TaskStateMachine) -> None:
        if self._canceled or self.phase != NAVIGATE_TO_APPROACH:
            return
        if navigation.state == FAILED:
            self._fail('approach navigation failed: {}'.format(navigation.reason))
            return
        if navigation.state == PAUSED:
            self.reason = navigation.reason
            self._set_state(PAUSED)
            return
        if navigation.state == RECOVERING:
            self.reason = navigation.reason
            self._set_state(RECOVERING)
            return
        if navigation.state == SUCCEEDED:
            if self.slot is None:
                self._fail('pallet slot is unavailable after approach navigation')
                return
            self.phase = 'START_PICKUP'
            self.reason = ''
            self._notify()
            accepted, message = self._pickup.start(self.slot)
            if not accepted:
                self._fail('pallet pickup was not accepted: {}'.format(message))
            return
        self._notify()

    def _on_pickup_status(self, pickup: PalletPickupStateMachine) -> None:
        if self._canceled or self.phase == NAVIGATE_TO_APPROACH:
            return
        if pickup.state == FAILED:
            self._fail('pallet pickup failed: {}'.format(pickup.reason))
            return
        if pickup.state == PAUSED:
            self.reason = pickup.reason
            self._set_state(PAUSED)
            return
        if pickup.state == SUCCEEDED:
            self.reason = pickup.reason
            self._set_state(SUCCEEDED)
            return
        self.phase = pickup.phase
        self.reason = pickup.reason
        self._set_state(pickup.state)

    def _fail(self, reason: str) -> None:
        self.reason = reason
        self._set_state(FAILED)

    def _set_state(self, state: str) -> None:
        self.state = state
        self._notify()

    def _notify(self) -> None:
        if self.status_callback is not None:
            self.status_callback(self)
