from concurrent.futures import Future
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time

from forklift_task_manager.route_model import PoseTarget
from forklift_task_manager.task_manager_node import Nav2Navigator


class Node:
    def __init__(self):
        self.timers = []

    def get_clock(self):
        return SimpleNamespace(now=lambda: Time(seconds=10))

    def get_logger(self):
        return SimpleNamespace(info=lambda *_: None, warning=lambda *_: None, error=lambda *_: None)

    def create_timer(self, period, callback):
        timer = SimpleNamespace(callback=callback, canceled=False)
        timer.cancel = lambda: setattr(timer, 'canceled', True)
        self.timers.append(timer)
        return timer

    def destroy_timer(self, timer):
        timer.cancel()

    def tick(self):
        for timer in list(self.timers):
            if not timer.canceled:
                timer.callback()


class Client:
    def __init__(self):
        self.sent = []

    def wait_for_server(self, timeout_sec):
        return True

    def send_goal_async(self, goal):
        future = Future()
        self.sent.append((goal, future))
        return future


class Handle:
    accepted = True

    def __init__(self):
        self.result = Future()
        self.cancels = 0

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancels += 1
        future = Future()
        future.set_result(SimpleNamespace(return_code=0))
        return future


def navigator():
    nav = Nav2Navigator.__new__(Nav2Navigator)
    nav._node = Node()
    nav._client = Client()
    nav._follow_path_client = Client()
    nav._goal_handle = None
    nav._follow_path_goal_handle = None
    nav._request_token = 0
    nav._motion_sends = set()
    nav._motion_handles = {}
    nav._motion_wait_timer = None
    return nav


def test_cancel_ack_does_not_allow_new_goal_until_old_terminal_result():
    nav = navigator()
    done = []
    nav.send_goal(PoseTarget('old', 0., 35.95, 0.), lambda *args: done.append(args))
    handle = Handle()
    nav._client.sent[0][1].set_result(handle)
    nav.cancel_goal()
    nav.send_goal(PoseTarget('new', 0., 30.66, 0.), lambda *args: done.append(args))
    nav._node.tick()
    assert handle.cancels > 0
    assert len(nav._client.sent) == 1
    handle.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    nav._node.tick()
    assert not done
    assert len(nav._client.sent) == 2
    assert nav._client.sent[1][0].pose.pose.position.y == 30.66


def test_cancel_before_goal_acceptance_and_latest_goal_wins():
    nav = navigator()
    nav.send_goal(PoseTarget('a', 1., 0., 0.), lambda *_: None)
    nav.cancel_goal()
    nav.send_goal(PoseTarget('b', 2., 0., 0.), lambda *_: None)
    nav.cancel_goal()
    nav.send_goal(PoseTarget('c', 3., 0., 0.), lambda *_: None)
    old = Handle()
    nav._client.sent[0][1].set_result(old)
    nav._node.tick()
    assert len(nav._client.sent) == 1
    assert old.cancels > 0
    old.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    nav._node.tick()
    assert len(nav._client.sent) == 2
    assert nav._client.sent[-1][0].pose.pose.position.x == 3.


def test_follow_path_and_navigation_cannot_overlap():
    nav = navigator()
    path = Path()
    path.poses = [PoseStamped(), PoseStamped()]
    nav.send_follow_path(path, lambda *_: None)
    old = Handle()
    nav._follow_path_client.sent[0][1].set_result(old)
    nav.cancel_goal()
    nav.send_goal(PoseTarget('new', 5., 0., 0.), lambda *_: None)
    nav._node.tick()
    assert not nav._client.sent
    old.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    nav._node.tick()
    assert len(nav._client.sent) == 1


def test_cancel_timeout_fails_request_without_sending_or_exiting(monkeypatch):
    nav = navigator()
    now = [10.]
    monkeypatch.setattr('forklift_task_manager.task_manager_node.time.monotonic', lambda: now[0])
    nav.send_goal(PoseTarget('old', 1., 0., 0.), lambda *_: None)
    old = Handle()
    nav._client.sent[0][1].set_result(old)
    nav.cancel_goal()
    results = []
    nav.send_goal(PoseTarget('new', 2., 0., 0.), lambda *args: results.append(args))
    now[0] = 16.
    nav._node.tick()
    assert results and not results[0][0]
    assert len(nav._client.sent) == 1
    old.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    nav.send_goal(PoseTarget('next', 3., 0., 0.), lambda *_: None)
    assert len(nav._client.sent) == 2
