from types import SimpleNamespace

from forklift_task_manager.anchor_route import AnchorRouteConfig
from forklift_task_manager.route_model import PoseTarget
from forklift_task_manager.task_manager_node import HierarchicalNav2Navigator


class FakeLogger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass

    def error(self, _message):
        pass


class FakeNode:
    def __init__(self):
        self.timers = []

    def get_logger(self):
        return FakeLogger()

    def create_timer(self, _period, callback):
        timer = SimpleNamespace(callback=callback, canceled=False)
        timer.cancel = lambda: setattr(timer, 'canceled', True)
        self.timers.append(timer)
        return timer

    def destroy_timer(self, _timer):
        pass


class FakeNavigator:
    def __init__(self, path):
        self.path = path
        self.goals = []
        self.goal_callbacks = []
        self.follow_paths = []
        self.follow_callbacks = []
        self.compute_planner_ids = []
        self.cancel_goal_count = 0
        self.cancel_path_count = 0

    def send_goal(self, target, callback):
        self.goals.append(target)
        self.goal_callbacks.append(callback)

    def send_follow_path(self, path, callback):
        self.follow_paths.append(path)
        self.follow_callbacks.append(callback)

    def compute_path_result(self, _target, callback, timeout_sec, planner_id):
        assert timeout_sec > 0.0
        self.compute_planner_ids.append(planner_id)
        callback(True, 'topology ready', self.path)

    def cancel_goal(self):
        self.cancel_goal_count += 1

    def cancel_path_goal(self):
        self.cancel_path_count += 1

    def cancel_all_goals(self):
        pass


def clear_costmap():
    size = 300
    resolution = 0.1
    origin = SimpleNamespace(
        position=SimpleNamespace(x=-10.0, y=-10.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    return SimpleNamespace(
        metadata=SimpleNamespace(
            size_x=size, size_y=size, resolution=resolution, origin=origin
        ),
        data=[0] * (size * size),
    )


def straight_path(length=10, spacing=1.0):
    poses = []
    for i in range(int(round(length / spacing)) + 1):
        x = i * spacing
        poses.append(SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(x=float(x), y=0.0))
        ))
    return SimpleNamespace(poses=poses)


def make_hierarchy(raw):
    return HierarchicalNav2Navigator(
        FakeNode(),
        raw,
        clear_costmap,
        AnchorRouteConfig(footprint=((0.3, 0.2), (0.3, -0.2),
                                     (-0.3, -0.2), (-0.3, 0.2))),
        'TopologyOnly',
        5.0,
    )


def test_success_at_old_anchor_is_not_success_at_replacement_goal():
    raw = FakeNavigator(straight_path(12))
    hierarchy = make_hierarchy(raw)
    hierarchy.set_anchor_arrival_checker(lambda *_: (False, 5.3, 'not at requested goal'))
    result = []
    hierarchy.send_goal(PoseTarget('new', 12., 0., 0.), lambda *args: result.append(args))
    raw.goal_callbacks[-1](True, 'Nav2 succeeded at old anchor')
    assert result and result[0][0] is False
    assert 'Final goal verification failed' in result[0][1]


def test_stopped_intermediate_anchor_advances_when_nav2_remains_active():
    raw = FakeNavigator(straight_path(12))
    node = FakeNode()
    hierarchy = HierarchicalNav2Navigator(
        node,
        raw,
        clear_costmap,
        AnchorRouteConfig(
            arrival_settle_sec=0.0,
            arrival_transition_delay_sec=0.0,
            footprint=((0.3, 0.2), (0.3, -0.2),
                       (-0.3, -0.2), (-0.3, 0.2)),
        ),
        'TopologyOnly',
        5.0,
    )
    hierarchy.set_anchor_arrival_checker(
        lambda _target, _tolerance, _speed: (True, 0.20, 'vehicle stopped')
    )

    hierarchy.send_goal(PoseTarget('goal', 12.0, 0.0, 0.0), lambda *_: None)
    raw.goal_callbacks.pop(0)(False, 'continuous failed')
    assert raw.goals[-1].name == 'anchor_01'

    node.timers[-1].callback()
    node.timers[-1].callback()

    assert raw.cancel_goal_count == 2
    assert raw.goals[-1].name == 'anchor_02'


def test_anchor_arrival_waits_for_cancel_transition_before_next_goal():
    raw = FakeNavigator(straight_path(12))
    node = FakeNode()
    hierarchy = HierarchicalNav2Navigator(
        node,
        raw,
        clear_costmap,
        AnchorRouteConfig(
            arrival_settle_sec=0.0,
            arrival_transition_delay_sec=0.25,
            footprint=((0.3, 0.2), (0.3, -0.2),
                       (-0.3, -0.2), (-0.3, 0.2)),
        ),
        'TopologyOnly',
        5.0,
    )
    hierarchy.set_anchor_arrival_checker(
        lambda _target, _tolerance, _speed: (True, 0.20, 'vehicle stopped')
    )

    hierarchy.send_goal(PoseTarget('goal', 12.0, 0.0, 0.0), lambda *_: None)
    raw.goal_callbacks.pop(0)(False, 'continuous failed')
    guard_timer = node.timers[-1]
    guard_timer.callback()
    guard_timer.callback()

    assert raw.cancel_goal_count == 2
    assert raw.goals[-1].name == 'anchor_01'
    transition_timer = node.timers[-1]
    assert transition_timer is not guard_timer
    transition_timer.callback()

    assert raw.goals[-1].name == 'anchor_02'


def test_stale_anchor_guard_cannot_cancel_the_next_anchor_guard():
    raw = FakeNavigator(straight_path(12))
    node = FakeNode()
    hierarchy = HierarchicalNav2Navigator(
        node,
        raw,
        clear_costmap,
        AnchorRouteConfig(
            arrival_settle_sec=0.0,
            arrival_transition_delay_sec=0.0,
            footprint=((0.3, 0.2), (0.3, -0.2),
                       (-0.3, -0.2), (-0.3, 0.2)),
        ),
        'TopologyOnly',
        5.0,
    )
    hierarchy.set_anchor_arrival_checker(
        lambda _target, _tolerance, _speed: (True, 0.20, 'vehicle stopped')
    )

    hierarchy.send_goal(PoseTarget('goal', 12.0, 0.0, 0.0), lambda *_: None)
    raw.goal_callbacks.pop(0)(False, 'continuous failed')
    first_guard = node.timers[-1]
    first_guard.callback()
    first_guard.callback()

    assert raw.goals[-1].name == 'anchor_02'
    second_guard = node.timers[-1]
    assert second_guard is not first_guard
    assert second_guard.canceled is False

    # A queued callback from anchor_01 must not destroy anchor_02's timer.
    first_guard.callback()
    assert second_guard.canceled is False


def test_complete_trajectory_success_does_not_request_topology():
    raw = FakeNavigator(straight_path())
    hierarchy = make_hierarchy(raw)
    results = []

    hierarchy.send_goal(
        PoseTarget('goal', 10.0, 0.0, 0.0),
        lambda success, message: results.append((success, message)),
    )
    raw.goal_callbacks.pop(0)(True, 'complete')

    assert raw.compute_planner_ids == []
    assert results == [(True, 'complete')]


def test_driveable_topology_route_uses_one_follow_path_without_anchor_stops():
    raw = FakeNavigator(straight_path(12, spacing=0.05))
    node = FakeNode()
    hierarchy = HierarchicalNav2Navigator(
        node,
        raw,
        clear_costmap,
        AnchorRouteConfig(footprint=((0.3, 0.2), (0.3, -0.2),
                                     (-0.3, -0.2), (-0.3, 0.2))),
        'TopologyOnly',
        5.0,
        continuous_topology_planner_id='TopologyContinuous',
        continuous_path_enabled=True,
    )
    results = []

    hierarchy.send_goal(
        PoseTarget('goal', 12.0, 0.0, 0.0),
        lambda success, message: results.append((success, message)),
    )
    raw.goal_callbacks.pop(0)(False, 'complete trajectory failed')

    assert raw.compute_planner_ids == ['TopologyContinuous']
    assert len(raw.follow_paths) == 1
    assert raw.goals[-1].name == 'goal'

    raw.follow_callbacks.pop(0)(True, 'continuous path reached goal')
    assert results == [(True, 'continuous topology route completed')]


def test_stale_continuous_planner_cannot_send_sparse_topology_to_controller():
    raw = FakeNavigator(straight_path(12))
    hierarchy = HierarchicalNav2Navigator(
        FakeNode(), raw, clear_costmap,
        AnchorRouteConfig(footprint=((0.3, 0.2), (0.3, -0.2),
                                    (-0.3, -0.2), (-0.3, 0.2))),
        'TopologyOnly', 5.0,
        continuous_topology_planner_id='TopologyContinuous',
        continuous_path_enabled=True,
    )
    hierarchy.send_goal(PoseTarget('goal', 12.0, 0.0, 0.0), lambda *_: None)
    raw.goal_callbacks.pop(0)(False, 'complete trajectory failed')
    assert raw.follow_paths == []
    assert raw.compute_planner_ids == ['TopologyContinuous', 'TopologyOnly']
    assert raw.goals[-1].name == 'anchor_01'


def test_failed_complete_trajectory_uses_topology_and_first_safe_anchor():
    raw = FakeNavigator(straight_path())
    hierarchy = make_hierarchy(raw)

    hierarchy.send_goal(PoseTarget('goal', 10.0, 0.0, 0.0), lambda *_: None)
    raw.goal_callbacks.pop(0)(False, 'continuous failed')

    assert raw.compute_planner_ids == ['TopologyOnly']
    assert len(raw.goals) == 2
    assert raw.goals[-1].name == 'anchor_01'
    assert raw.goals[-1].x == 4.0


def test_new_goal_cancels_old_anchor_route_and_replaces_it():
    raw = FakeNavigator(straight_path())
    hierarchy = make_hierarchy(raw)
    old_results = []

    hierarchy.send_goal(
        PoseTarget('old', 10.0, 0.0, 0.0),
        lambda success, message: old_results.append((success, message)),
    )
    raw.goal_callbacks.pop(0)(False, 'continuous failed')
    old_anchor_callback = raw.goal_callbacks.pop(0)
    hierarchy.send_goal(PoseTarget('new', 8.0, 0.0, 0.0), lambda *_: None)
    old_anchor_callback(True, 'stale success')

    assert hierarchy.anchor_mode is False
    assert raw.goals[-1].name == 'new'
    assert old_results == []


def test_successful_anchor_advances_existing_queue_without_replanning():
    raw = FakeNavigator(straight_path(12))
    hierarchy = make_hierarchy(raw)

    hierarchy.send_goal(PoseTarget('goal', 12.0, 0.0, 0.0), lambda *_: None)
    raw.goal_callbacks.pop(0)(False, 'continuous failed')
    assert raw.goals[-1].name == 'anchor_01'

    raw.goal_callbacks.pop(0)(True, 'first anchor reached')

    assert raw.compute_planner_ids == ['TopologyOnly']
    assert raw.goals[-1].name == 'anchor_02'
