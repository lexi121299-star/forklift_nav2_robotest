from forklift_task_manager.pallet_model import PalletSlot, PickupDefaults
from forklift_task_manager.pickup_job_state_machine import (
    CANCELED,
    NAVIGATE_TO_APPROACH,
    PickupJobStateMachine,
)
from forklift_task_manager.route_model import PoseTarget
from forklift_task_manager.state_machine import FAILED, RUNNING, SUCCEEDED


def _defaults():
    return PickupDefaults(
        level_pitch_m=0.55,
        scan_level_offset=3,
        fork_insert_depth_m=0.80,
        pallet_clearance_m=0.08,
        max_offset_x_m=0.20,
        max_offset_y_m=0.08,
        min_detection_confidence=0.75,
        fork_timeout_sec=8.0,
        detection_timeout_sec=3.0,
        fine_motion_timeout_sec=6.0,
        fine_motion_speed_mps=0.08,
    )


def _slot():
    return PalletSlot('slot_001', 'slot_001_approach', 1, 0.55)


class _Navigator:
    def __init__(self):
        self.callbacks = []
        self.cancel_count = 0

    def send_goal(self, target, done_callback):
        self.callbacks.append((target, done_callback))

    def cancel_goal(self):
        self.cancel_count += 1


class _Devices:
    def __init__(self):
        self.calls = []
        self.callbacks = []
        self.cancel_count = 0

    @staticmethod
    def check_ready():
        return True, 'ready'

    def move_fork(self, **kwargs):
        self.calls.append(('fork', kwargs))
        self.callbacks.append(kwargs['done_callback'])

    def detect_offset(self, **kwargs):
        self.calls.append(('detect', kwargs))
        self.callbacks.append(kwargs['done_callback'])

    def move_relative(self, **kwargs):
        self.calls.append(('move_relative', kwargs))
        self.callbacks.append(kwargs['done_callback'])

    def cancel_all(self):
        self.cancel_count += 1

    def complete_latest(self, success=True, message='', result=None):
        self.callbacks[-1](success, message, result)


def test_pickup_job_navigates_before_fork_motion():
    navigator = _Navigator()
    devices = _Devices()
    machine = PickupJobStateMachine(navigator, devices, _defaults(), 0)
    approach = PoseTarget('slot_001_approach', 1.0, 2.0, 0.0)

    accepted, _ = machine.start(_slot(), approach)

    assert accepted is True
    assert machine.state == RUNNING
    assert machine.phase == NAVIGATE_TO_APPROACH
    assert not devices.calls

    navigator.callbacks[-1][1](True, 'arrived')

    assert machine.phase == 'RAISE_TO_SCAN_HEIGHT'
    assert devices.calls[-1][0] == 'fork'


def test_pickup_job_never_starts_fork_motion_after_approach_failure():
    navigator = _Navigator()
    devices = _Devices()
    machine = PickupJobStateMachine(navigator, devices, _defaults(), 0)

    machine.start(_slot(), PoseTarget('slot_001_approach', 1.0, 2.0, 0.0))
    navigator.callbacks[-1][1](False, 'no path')

    assert machine.state == FAILED
    assert 'approach navigation failed' in machine.reason
    assert not devices.calls


def test_pickup_job_cancel_stops_active_navigation():
    navigator = _Navigator()
    devices = _Devices()
    machine = PickupJobStateMachine(navigator, devices, _defaults(), 0)

    machine.start(_slot(), PoseTarget('slot_001_approach', 1.0, 2.0, 0.0))
    canceled, _ = machine.cancel()

    assert canceled is True
    assert machine.state == CANCELED
    assert navigator.cancel_count == 1


def test_pickup_job_runs_navigation_then_every_pickup_phase():
    """Exercise the dispatch workflow without Nav2, radar, or fork hardware."""
    navigator = _Navigator()
    devices = _Devices()
    machine = PickupJobStateMachine(navigator, devices, _defaults(), 0)

    accepted, _ = machine.start(
        _slot(), PoseTarget('slot_001_approach', 1.0, 2.0, 0.0)
    )
    assert accepted is True
    assert machine.phase == NAVIGATE_TO_APPROACH

    navigator.callbacks[-1][1](True, 'arrived')
    assert machine.phase == 'RAISE_TO_SCAN_HEIGHT'
    assert devices.calls[-1][0] == 'fork'
    assert abs(devices.calls[-1][1]['target_height_m'] - 2.20) < 1e-9

    devices.complete_latest(
        True,
        'scan height reached',
        type('Result', (), {'final_height_m': 2.20})(),
    )
    assert machine.phase == 'DETECT_OFFSET'
    assert devices.calls[-1][0] == 'detect'

    devices.complete_latest(
        True,
        'pallet detected',
        type(
            'Detection',
            (),
            {'offset_x_m': 0.06, 'offset_y_m': -0.03, 'confidence': 0.90},
        )(),
    )
    assert machine.phase == 'LOWER_TO_PICK_HEIGHT'
    assert abs(devices.calls[-1][1]['target_height_m'] - 0.55) < 1e-9

    devices.complete_latest(
        True,
        'pick height reached',
        type('Result', (), {'final_height_m': 0.55})(),
    )
    assert machine.phase == 'APPLY_LATERAL_OFFSET'
    assert abs(devices.calls[-1][1]['side_shift_m'] + 0.03) < 1e-9

    devices.complete_latest(
        True,
        'side shift reached',
        type('Result', (), {'final_height_m': 0.55})(),
    )
    assert machine.phase == 'INSERT_FORK'
    assert abs(devices.calls[-1][1]['distance_m'] - 0.86) < 1e-9

    devices.complete_latest(True, 'fork inserted')
    assert machine.phase == 'LIFT_CLEARANCE'
    assert abs(devices.calls[-1][1]['target_height_m'] - 0.63) < 1e-9

    devices.complete_latest(
        True,
        'pallet clear',
        type('Result', (), {'final_height_m': 0.63})(),
    )
    assert machine.state == SUCCEEDED
    assert machine.progress == 1.0


def test_pickup_job_cancellation_stops_active_fork_phase():
    navigator = _Navigator()
    devices = _Devices()
    machine = PickupJobStateMachine(navigator, devices, _defaults(), 0)

    machine.start(_slot(), PoseTarget('slot_001_approach', 1.0, 2.0, 0.0))
    navigator.callbacks[-1][1](True, 'arrived')
    canceled, _ = machine.cancel('operator canceled at scan height')

    assert canceled is True
    assert machine.state == CANCELED
    assert devices.cancel_count == 1
