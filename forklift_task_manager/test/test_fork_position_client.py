from types import SimpleNamespace

from builtin_interfaces.msg import Time
from forklift_msgs.msg import ForkPositionCommand, ForkState

from forklift_task_manager.fork_position_client import (
    ForkPositionClient,
    ForkPositionConfig,
)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Timer:
    def __init__(self, callback):
        self.callback = callback
        self.canceled = False

    def cancel(self):
        self.canceled = True


class _Node:
    def __init__(self):
        self.publisher = _Publisher()
        self.timers = []

    def create_publisher(self, *_args):
        return self.publisher

    def create_subscription(self, *_args):
        return object()

    def create_timer(self, _period, callback):
        timer = _Timer(callback)
        self.timers.append(timer)
        return timer

    @staticmethod
    def destroy_timer(timer):
        timer.cancel()

    @staticmethod
    def get_clock():
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))


def _state(
    *,
    command_id=0,
    status=ForkState.STATUS_IDLE,
    height=0.0,
    side_shift=0.0,
    tilt=0.0,
):
    message = ForkState()
    message.active_command_id = command_id
    message.status = status
    message.height_m = height
    message.side_shift_m = side_shift
    message.tilt_rad = tilt
    message.height_valid = True
    message.side_shift_valid = True
    message.tilt_valid = True
    return message


def test_position_client_sequences_axes_and_completes():
    node = _Node()
    client = ForkPositionClient(node, ForkPositionConfig())
    client._on_state(_state())
    completed = []

    client.move_to(
        target_height_m=1.0,
        side_shift_m=0.0,
        tilt_rad=0.0,
        timeout_sec=5.0,
        done_callback=lambda *args: completed.append(args),
    )

    assert len(node.publisher.messages) == 1
    command = node.publisher.messages[-1]
    assert command.mode == ForkPositionCommand.MODE_HEIGHT_POSITION
    assert command.command_id == 1
    assert command.target == 1.0

    client._on_state(
        _state(
            command_id=1,
            status=ForkState.STATUS_REACHED,
            height=1.0,
        )
    )
    client._tick()
    client._tick()
    client._tick()

    assert completed
    assert completed[-1][0] is True
    assert completed[-1][2].final_height_m == 1.0


def test_position_client_cancel_publishes_stop_for_active_command():
    node = _Node()
    client = ForkPositionClient(node, ForkPositionConfig())
    client._on_state(_state())
    client.move_to(
        target_height_m=1.0,
        side_shift_m=0.0,
        tilt_rad=0.0,
        timeout_sec=5.0,
        done_callback=lambda *_args: None,
    )

    client.cancel()

    command = node.publisher.messages[-1]
    assert command.mode == ForkPositionCommand.MODE_STOP
    assert command.command_id == 1
