"""ROS-independent state machine for two-stage pallet pickup phase two."""

from typing import Callable, Optional

from .pallet_model import PalletSlot, PickupDefaults, pickup_heights
from .state_machine import FAILED, IDLE, PAUSED, RUNNING, SUCCEEDED


WAIT_READY = 'WAIT_READY'
RAISE_TO_SCAN_HEIGHT = 'RAISE_TO_SCAN_HEIGHT'
DETECT_OFFSET = 'DETECT_OFFSET'
VALIDATE_OFFSET = 'VALIDATE_OFFSET'
LOWER_TO_PICK_HEIGHT = 'LOWER_TO_PICK_HEIGHT'
APPLY_LATERAL_OFFSET = 'APPLY_LATERAL_OFFSET'
INSERT_FORK = 'INSERT_FORK'
LIFT_CLEARANCE = 'LIFT_CLEARANCE'

PHASES = (
    WAIT_READY,
    RAISE_TO_SCAN_HEIGHT,
    DETECT_OFFSET,
    VALIDATE_OFFSET,
    LOWER_TO_PICK_HEIGHT,
    APPLY_LATERAL_OFFSET,
    INSERT_FORK,
    LIFT_CLEARANCE,
)

_ACTIVE_STATES = {RUNNING, PAUSED}
_SAFETY_PAUSE_REASONS = {
    'emergency stop',
    'vehicle emergency stop',
    'vehicle fault',
}


class PalletPickupStateMachine:
    """Sequence fork, perception, and fine-motion adapters for pallet pickup."""

    def __init__(
        self,
        devices,
        defaults: PickupDefaults,
        status_callback: Optional[Callable[['PalletPickupStateMachine'], None]] = None,
    ) -> None:
        self.devices = devices
        self.defaults = defaults
        self.status_callback = status_callback
        self.state = IDLE
        self.active_slot = ''
        self.slot: Optional[PalletSlot] = None
        self.phase = ''
        self.phase_index = -1
        self.reason = ''
        self.current_height_m = 0.0
        self.offset_x_m = 0.0
        self.offset_y_m = 0.0
        self.confidence = 0.0
        self._dispatch_token = 0

    @property
    def phase_count(self) -> int:
        return len(PHASES)

    @property
    def progress(self) -> float:
        if self.state == SUCCEEDED:
            return 1.0
        if self.phase_index < 0:
            return 0.0
        return float(self.phase_index) / float(max(1, self.phase_count))

    def start(self, slot: PalletSlot):
        if self.state in _ACTIVE_STATES:
            return False, 'another pallet pickup task is active'
        self.slot = slot
        self.active_slot = slot.name
        self.phase_index = 0
        self.phase = PHASES[self.phase_index]
        self.reason = ''
        self.offset_x_m = 0.0
        self.offset_y_m = 0.0
        self.confidence = 0.0
        self._set_state(RUNNING)
        self._dispatch_current()
        return True, 'pallet pickup accepted'

    def pause(self, reason: str = 'paused by request'):
        if self.state != RUNNING:
            return False, 'pallet pickup task is not running'
        self._invalidate_action()
        self.devices.cancel_all()
        self.reason = reason
        self._set_state(PAUSED)
        return True, reason

    def resume(self):
        if self.state != PAUSED:
            return False, 'pallet pickup task is not paused'
        self.reason = ''
        self._set_state(RUNNING)
        self._dispatch_current()
        return True, 'pallet pickup resumed'

    def cancel(self, reason: str = 'pallet pickup canceled'):
        if self.state not in _ACTIVE_STATES:
            return False, 'no active pallet pickup task'
        self._invalidate_action()
        self.devices.cancel_all()
        self.slot = None
        self.active_slot = ''
        self.phase = ''
        self.phase_index = -1
        self.reason = reason
        self._set_state(IDLE)
        return True, reason

    def observe_safety_status(self, reason: str) -> bool:
        normalized = reason.strip().lower()
        if normalized not in _SAFETY_PAUSE_REASONS:
            return False
        if self.state != RUNNING:
            return False
        self.pause(reason=reason.strip())
        return True

    def _dispatch_current(self) -> None:
        if self.slot is None:
            self._fail('pallet slot is unavailable')
            return
        self._dispatch_token += 1
        token = self._dispatch_token
        phase = self.phase

        def done(success: bool, message: str = '', result=None) -> None:
            if token != self._dispatch_token or self.state != RUNNING:
                return
            if success:
                self._phase_succeeded(result)
            else:
                self._fail(message or '{} failed'.format(phase))

        def height_feedback(current_height_m: float) -> None:
            if token != self._dispatch_token or self.state != RUNNING:
                return
            self.current_height_m = float(current_height_m)
            self._notify()

        try:
            if phase == WAIT_READY:
                ready, message = self.devices.check_ready()
                done(ready, message)
            elif phase == RAISE_TO_SCAN_HEIGHT:
                _, scan_height_m = pickup_heights(self.slot, self.defaults)
                self.devices.move_fork(
                    target_height_m=scan_height_m,
                    side_shift_m=0.0,
                    tilt_rad=self.defaults.tilt_rad,
                    timeout_sec=self.defaults.fork_timeout_sec,
                    done_callback=done,
                    feedback_callback=height_feedback,
                )
            elif phase == DETECT_OFFSET:
                _, scan_height_m = pickup_heights(self.slot, self.defaults)
                self.devices.detect_offset(
                    slot_id=self.slot.name,
                    scan_height_m=scan_height_m,
                    timeout_sec=self.defaults.detection_timeout_sec,
                    done_callback=done,
                )
            elif phase == VALIDATE_OFFSET:
                done(*self._validate_offset())
            elif phase == LOWER_TO_PICK_HEIGHT:
                pick_height_m, _ = pickup_heights(self.slot, self.defaults)
                self.devices.move_fork(
                    target_height_m=pick_height_m,
                    side_shift_m=0.0,
                    tilt_rad=self.defaults.tilt_rad,
                    timeout_sec=self.defaults.fork_timeout_sec,
                    done_callback=done,
                    feedback_callback=height_feedback,
                )
            elif phase == APPLY_LATERAL_OFFSET:
                pick_height_m, _ = pickup_heights(self.slot, self.defaults)
                self.devices.move_fork(
                    target_height_m=pick_height_m,
                    side_shift_m=self.offset_y_m,
                    tilt_rad=self.defaults.tilt_rad,
                    timeout_sec=self.defaults.fork_timeout_sec,
                    done_callback=done,
                    feedback_callback=height_feedback,
                )
            elif phase == INSERT_FORK:
                self.devices.move_relative(
                    distance_m=self.offset_x_m + self.defaults.fork_insert_depth_m,
                    max_speed_mps=self.defaults.fine_motion_speed_mps,
                    timeout_sec=self.defaults.fine_motion_timeout_sec,
                    done_callback=done,
                )
            elif phase == LIFT_CLEARANCE:
                pick_height_m, _ = pickup_heights(self.slot, self.defaults)
                self.devices.move_fork(
                    target_height_m=pick_height_m + self.defaults.pallet_clearance_m,
                    side_shift_m=self.offset_y_m,
                    tilt_rad=self.defaults.tilt_rad,
                    timeout_sec=self.defaults.fork_timeout_sec,
                    done_callback=done,
                    feedback_callback=height_feedback,
                )
            else:
                self._fail('unknown pallet pickup phase {}'.format(phase))
        except Exception as exc:  # Keep adapter failures inside task semantics.
            done(False, '{} failed to dispatch: {}'.format(phase, exc))

    def _phase_succeeded(self, result=None) -> None:
        if self.phase == DETECT_OFFSET and result is not None:
            self.offset_x_m = float(getattr(result, 'offset_x_m', 0.0))
            self.offset_y_m = float(getattr(result, 'offset_y_m', 0.0))
            self.confidence = float(getattr(result, 'confidence', 0.0))
        if result is not None and hasattr(result, 'final_height_m'):
            self.current_height_m = float(getattr(result, 'final_height_m'))

        if self.phase_index + 1 < self.phase_count:
            self.phase_index += 1
            self.phase = PHASES[self.phase_index]
            self.reason = ''
            self._notify()
            self._dispatch_current()
            return
        self.reason = 'pallet pickup completed'
        self._set_state(SUCCEEDED)

    def _validate_offset(self):
        if self.confidence < self.defaults.min_detection_confidence:
            return False, 'pallet detection confidence too low'
        if abs(self.offset_x_m) > self.defaults.max_offset_x_m:
            return False, 'pallet offset_x exceeds limit'
        if abs(self.offset_y_m) > self.defaults.max_offset_y_m:
            return False, 'pallet offset_y exceeds limit'
        return True, ''

    def _fail(self, reason: str) -> None:
        self.reason = reason
        self._set_state(FAILED)

    def _invalidate_action(self) -> None:
        self._dispatch_token += 1

    def _set_state(self, state: str) -> None:
        self.state = state
        self._notify()

    def _notify(self) -> None:
        if self.status_callback is not None:
            self.status_callback(self)
