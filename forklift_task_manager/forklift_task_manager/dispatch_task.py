"""Dispatch-task state shared by the ROS action and status publisher."""

from dataclasses import dataclass


TYPE_NONE = 0
TYPE_NAVIGATE = 1
TYPE_PICKUP = 2
TYPE_DROPOFF = 3

STATUS_IDLE = 0
STATUS_ACCEPTED = 1
STATUS_RUNNING = 2
STATUS_PAUSED = 3
STATUS_SUCCEEDED = 4
STATUS_FAILED = 5
STATUS_CANCELED = 6
STATUS_REJECTED = 7


MACHINE_STATE_TO_DISPATCH_STATUS = {
    'IDLE': STATUS_IDLE,
    'RUNNING': STATUS_RUNNING,
    'RECOVERING': STATUS_RUNNING,
    'PAUSED': STATUS_PAUSED,
    'SUCCEEDED': STATUS_SUCCEEDED,
    'FAILED': STATUS_FAILED,
}


@dataclass
class DispatchTaskContext:
    """The one dispatch task currently represented by task-status output."""

    task_id: str = ''
    task_type: int = TYPE_NONE
    slot_id: str = ''
    status: int = STATUS_IDLE
    phase: str = ''
    progress: float = 0.0
    reason: str = ''

    @property
    def active(self) -> bool:
        return self.status in {STATUS_ACCEPTED, STATUS_RUNNING, STATUS_PAUSED}

    def begin(self, task_id: str, task_type: int, slot_id: str = '') -> None:
        self.task_id = task_id
        self.task_type = task_type
        self.slot_id = slot_id
        self.status = STATUS_ACCEPTED
        self.phase = 'ACCEPTED'
        self.progress = 0.0
        self.reason = ''

    def reject(self, task_id: str, task_type: int, reason: str) -> None:
        self.task_id = task_id
        self.task_type = task_type
        self.slot_id = ''
        self.status = STATUS_REJECTED
        self.phase = 'REJECTED'
        self.progress = 0.0
        self.reason = reason

    def cancel(self, reason: str) -> None:
        self.status = STATUS_CANCELED
        self.phase = 'CANCELED'
        self.reason = reason

    def update_from_navigation(
        self,
        machine_state: str,
        phase: str,
        progress: float,
        reason: str,
    ) -> None:
        if self.task_type != TYPE_NAVIGATE:
            return
        if machine_state == 'IDLE' and self.status == STATUS_CANCELED:
            return
        self.status = MACHINE_STATE_TO_DISPATCH_STATUS[machine_state]
        self.phase = phase or machine_state
        self.progress = max(0.0, min(1.0, float(progress)))
        self.reason = reason

    def update_from_pickup(
        self,
        machine_state: str,
        phase: str,
        progress: float,
        reason: str,
    ) -> None:
        if self.task_type != TYPE_PICKUP:
            return
        if machine_state == 'CANCELED':
            self.status = STATUS_CANCELED
        else:
            self.status = MACHINE_STATE_TO_DISPATCH_STATUS[machine_state]
        self.phase = phase or machine_state
        self.progress = max(0.0, min(1.0, float(progress)))
        self.reason = reason
