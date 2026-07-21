from types import SimpleNamespace

import pytest

from forklift_task_manager.pallet_model import PalletSlot, PickupDefaults
from forklift_task_manager.pallet_pickup_state_machine import (
    APPLY_LATERAL_OFFSET,
    DETECT_OFFSET,
    INSERT_FORK,
    LIFT_CLEARANCE,
    LOWER_TO_PICK_HEIGHT,
    RAISE_TO_SCAN_HEIGHT,
    PalletPickupStateMachine,
)
from forklift_task_manager.state_machine import FAILED, PAUSED, RUNNING, SUCCEEDED


def defaults():
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


def slot():
    return PalletSlot(
        name='slot_001',
        approach_station='slot_001_approach',
        level_index=1,
        pick_height_m=0.55,
    )


class MockPickupDevices:
    def __init__(self):
        self.calls = []
        self.callbacks = []
        self.cancel_count = 0
        self.ready = True

    def check_ready(self):
        return self.ready, 'ready' if self.ready else 'not ready'

    def move_fork(
        self,
        *,
        target_height_m,
        side_shift_m,
        tilt_rad,
        timeout_sec,
        done_callback,
        feedback_callback=None,
    ):
        self.calls.append(
            ('fork', target_height_m, side_shift_m, tilt_rad, timeout_sec)
        )
        self.callbacks.append(done_callback)
        if feedback_callback is not None:
            feedback_callback(target_height_m)

    def detect_offset(self, *, slot_id, scan_height_m, timeout_sec, done_callback):
        self.calls.append(('detect', slot_id, scan_height_m, timeout_sec))
        self.callbacks.append(done_callback)

    def move_relative(
        self,
        *,
        distance_m,
        max_speed_mps,
        timeout_sec,
        done_callback,
    ):
        self.calls.append(('move_relative', distance_m, max_speed_mps, timeout_sec))
        self.callbacks.append(done_callback)

    def cancel_all(self):
        self.cancel_count += 1

    def complete_latest(self, success=True, message='', result=None):
        self.callbacks[-1](success, message, result)


def complete_fork(devices, height):
    devices.complete_latest(
        True,
        'fork done',
        SimpleNamespace(success=True, message='fork done', final_height_m=height),
    )


def complete_detection(devices, x=0.05, y=-0.02, confidence=0.9):
    devices.complete_latest(
        True,
        'detected',
        SimpleNamespace(
            success=True,
            message='detected',
            offset_x_m=x,
            offset_y_m=y,
            confidence=confidence,
        ),
    )


def test_pallet_pickup_runs_full_two_stage_pick_sequence():
    devices = MockPickupDevices()
    machine = PalletPickupStateMachine(devices, defaults())

    accepted, _ = machine.start(slot())

    assert accepted is True
    assert machine.state == RUNNING
    assert machine.phase == RAISE_TO_SCAN_HEIGHT
    assert devices.calls[-1][0] == 'fork'
    assert devices.calls[-1][1:] == pytest.approx((2.20, 0.0, 0.0, 8.0))

    complete_fork(devices, 2.20)
    assert machine.phase == DETECT_OFFSET
    assert devices.calls[-1][0:2] == ('detect', 'slot_001')
    assert devices.calls[-1][2:] == pytest.approx((2.20, 3.0))

    complete_detection(devices, x=0.06, y=-0.03)
    assert machine.phase == LOWER_TO_PICK_HEIGHT
    assert devices.calls[-1][0] == 'fork'
    assert devices.calls[-1][1:] == pytest.approx((0.55, 0.0, 0.0, 8.0))

    complete_fork(devices, 0.55)
    assert machine.phase == APPLY_LATERAL_OFFSET
    assert devices.calls[-1][0] == 'fork'
    assert devices.calls[-1][1:] == pytest.approx((0.55, -0.03, 0.0, 8.0))

    complete_fork(devices, 0.55)
    assert machine.phase == INSERT_FORK
    assert devices.calls[-1][0] == 'move_relative'
    assert devices.calls[-1][1:] == pytest.approx((0.86, 0.08, 6.0))

    devices.complete_latest(True, 'inserted', SimpleNamespace(success=True))
    assert machine.phase == LIFT_CLEARANCE
    assert devices.calls[-1][0] == 'fork'
    assert devices.calls[-1][1:] == pytest.approx((0.63, -0.03, 0.0, 8.0))

    complete_fork(devices, 0.63)
    assert machine.state == SUCCEEDED
    assert machine.reason == 'pallet pickup completed'
    assert machine.offset_x_m == pytest.approx(0.06)
    assert machine.offset_y_m == pytest.approx(-0.03)


def test_pallet_pickup_fails_on_low_detection_confidence():
    devices = MockPickupDevices()
    machine = PalletPickupStateMachine(devices, defaults())
    machine.start(slot())

    complete_fork(devices, 2.20)
    complete_detection(devices, confidence=0.2)

    assert machine.state == FAILED
    assert machine.reason == 'pallet detection confidence too low'
    assert devices.calls[-1][0] == 'detect'


def test_pallet_pickup_fails_on_offset_limit():
    devices = MockPickupDevices()
    machine = PalletPickupStateMachine(devices, defaults())
    machine.start(slot())

    complete_fork(devices, 2.20)
    complete_detection(devices, x=0.30)

    assert machine.state == FAILED
    assert machine.reason == 'pallet offset_x exceeds limit'


def test_pallet_pickup_pause_resume_and_cancel():
    devices = MockPickupDevices()
    machine = PalletPickupStateMachine(devices, defaults())
    machine.start(slot())

    paused, _ = machine.pause()
    assert paused is True
    assert machine.state == PAUSED
    assert devices.cancel_count == 1

    resumed, _ = machine.resume()
    assert resumed is True
    assert machine.state == RUNNING
    assert len(devices.calls) == 2

    canceled, _ = machine.cancel()
    assert canceled is True
    assert devices.cancel_count == 2
