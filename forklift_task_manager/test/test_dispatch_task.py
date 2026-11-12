from forklift_task_manager.dispatch_task import (
    DispatchTaskContext,
    STATUS_CANCELED,
    STATUS_FAILED,
    STATUS_RUNNING,
    STATUS_SUCCEEDED,
    TYPE_NAVIGATE,
    TYPE_PICKUP,
)


def test_navigation_context_tracks_machine_status_and_progress():
    context = DispatchTaskContext()
    context.begin('job-42', TYPE_NAVIGATE)

    context.update_from_navigation('RUNNING', 'anchor:2', 0.4, '')

    assert context.task_id == 'job-42'
    assert context.status == STATUS_RUNNING
    assert context.phase == 'anchor:2'
    assert context.progress == 0.4

    context.update_from_navigation('SUCCEEDED', 'goal', 1.0, 'route completed')
    assert context.status == STATUS_SUCCEEDED
    assert context.reason == 'route completed'


def test_canceled_navigation_is_not_replaced_by_machine_idle():
    context = DispatchTaskContext()
    context.begin('job-42', TYPE_NAVIGATE)
    context.cancel('replaced by newer task')

    context.update_from_navigation('IDLE', '', 0.0, 'replaced by newer task')

    assert context.status == STATUS_CANCELED
    assert context.reason == 'replaced by newer task'


def test_pickup_context_tracks_composite_workflow_to_success():
    context = DispatchTaskContext()
    context.begin('pickup-17', TYPE_PICKUP, 'slot_017')

    context.update_from_pickup(
        'RUNNING', 'NAVIGATE_TO_APPROACH', 0.20, ''
    )
    assert context.status == STATUS_RUNNING
    assert context.phase == 'NAVIGATE_TO_APPROACH'
    assert context.slot_id == 'slot_017'

    context.update_from_pickup('RUNNING', 'INSERT_FORK', 0.82, '')
    assert context.status == STATUS_RUNNING
    assert context.phase == 'INSERT_FORK'
    assert context.progress == 0.82

    context.update_from_pickup(
        'SUCCEEDED', 'LIFT_CLEARANCE', 1.0, 'pallet pickup completed'
    )
    assert context.status == STATUS_SUCCEEDED
    assert context.reason == 'pallet pickup completed'


def test_pickup_context_keeps_explicit_cancellation_and_failure_reason():
    context = DispatchTaskContext()
    context.begin('pickup-18', TYPE_PICKUP, 'slot_018')

    context.update_from_pickup(
        'CANCELED', 'INSERT_FORK', 0.82, 'operator canceled'
    )
    assert context.status == STATUS_CANCELED
    assert context.phase == 'INSERT_FORK'

    context.begin('pickup-19', TYPE_PICKUP, 'slot_019')
    context.update_from_pickup(
        'FAILED', 'DETECT_OFFSET', 0.55, 'pallet detection confidence too low'
    )
    assert context.status == STATUS_FAILED
    assert context.reason == 'pallet detection confidence too low'
