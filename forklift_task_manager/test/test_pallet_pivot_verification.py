"""Pallet map alignment must be verified before the final relative move."""

import math
from types import SimpleNamespace

import pytest
from forklift_msgs.msg import ForkliftVehicleState
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.time import Time

from forklift_task_manager.route_model import PivotTarget, RelativeMoveTarget, RouteDefinition
from forklift_task_manager.state_machine import TaskStateMachine, SUCCEEDED
from forklift_task_manager.task_manager_node import TaskMotionAdapter


YAW_FAILURE = 'pivot stopped outside yaw tolerance; reverse correction disabled'


@pytest.fixture
def rig(monkeypatch):
    now = [100.0]
    monkeypatch.setattr('forklift_task_manager.task_manager_node.time.monotonic', lambda: now[0])
    timers, logs, pivots, moves, exemptions = [], [], [], [], []

    def timer(period, callback):
        item = SimpleNamespace(callback=callback, canceled=False)
        item.cancel = lambda: setattr(item, 'canceled', True)
        timers.append(item)
        return item

    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=now[0])),
        get_logger=lambda: SimpleNamespace(info=logs.append, warning=logs.append),
        create_timer=timer, destroy_timer=lambda item: item.cancel())
    adapter = TaskMotionAdapter.__new__(TaskMotionAdapter)
    adapter._node = node
    adapter._motion_token = 0
    adapter._pivot_verify_timer = adapter._arrival_timer = None
    adapter._base_frame_id = 'base_link'
    adapter._pivot_verify_timeout_sec = 3.0
    adapter._pivot_yaw_tolerance_rad = 0.05
    adapter._pivot_yaw_rate_tolerance_radps = 0.03
    adapter._pivot_feedback_timeout_sec = 0.5
    adapter._pallet_nav_arrival_speed_mps = 0.03
    adapter._pallet_nav_arrival_settle_sec = 0.3
    adapter._pivot_start_is_valid = lambda target: (True, '', -1.88)
    adapter._navigator = SimpleNamespace(cancel_goal=lambda: None)
    adapter._devices = SimpleNamespace(
        pivot_relative=lambda **kwargs: pivots.append(kwargs),
        move_relative=lambda **kwargs: moves.append(kwargs), cancel_all=lambda: None)
    adapter._pallet_exemption_callback = exemptions.append
    pose = TransformStamped()
    state = ForkliftVehicleState()
    state.enabled = state.auto_mode = state.interlock = True
    odom = Odometry()
    target = PivotTarget('pallet_pivot_to_target', -8.51784667, 49.46803616,
                         math.radians(-69.1128), 0.12, 60.0)
    # Recorded 133517 final map pose. Odom relative yaw disagreed by about 4 deg.
    pose.transform.translation.x = -8.276
    pose.transform.translation.y = 49.712
    yaw = math.radians(-69.45)
    pose.transform.rotation.z = math.sin(yaw/2)
    pose.transform.rotation.w = math.cos(yaw/2)
    adapter._tf_buffer = SimpleNamespace(lookup_transform=lambda *args: pose)

    def refresh():
        for msg in (pose, state, odom):
            msg.header.stamp = Time(seconds=now[0]).to_msg()
        adapter._on_vehicle_state(state)
        adapter._on_pivot_odom(odom)

    def tick(dt=0.1, fresh=True):
        now[0] += dt
        if fresh:
            refresh()
        for item in list(timers):
            if not item.canceled:
                item.callback()

    refresh()
    return SimpleNamespace(adapter=adapter, target=target, now=now, tick=tick,
                           refresh=refresh, pose=pose, state=state, odom=odom,
                           pivots=pivots, moves=moves, exemptions=exemptions, logs=logs)


def start_verification(rig, success=False, message=YAW_FAILURE):
    done = []
    rig.adapter.send_goal(rig.target, lambda *args: done.append(args))
    rig.pivots[-1]['done_callback'](success, message, SimpleNamespace(success=success))
    return done


def test_recorded_map_pose_recovers_odom_yaw_failure_and_runs_final_approach(rig):
    target = rig.target
    final = RelativeMoveTarget(
        'pallet_final_approach', -1.610, 0.10, 60.0,
        expected_start_x=target.x, expected_start_y=target.y,
        expected_start_yaw=target.yaw,
        pallet_exemption_x=-9.765992, pallet_exemption_y=52.737919,
        pallet_exemption_yaw=target.yaw)
    machine = TaskStateMachine(rig.adapter, max_retries=0)
    machine.start(RouteDefinition('pallet', False, (target, final)))
    assert len(rig.pivots) == 1
    rig.pivots[0]['done_callback'](False, YAW_FAILURE, SimpleNamespace(success=False))
    assert not rig.moves and not rig.exemptions
    for _ in range(5):
        rig.tick()
    assert len(rig.pivots) == 1  # Never request a reverse correction.
    assert len(rig.moves) == 1
    assert rig.moves[0]['distance_m'] == -1.610
    assert rig.moves[0]['max_speed_mps'] == 0.10
    assert rig.exemptions[-1] is not None
    rig.moves[0]['done_callback'](True, 'relative motion completed', None)
    assert machine.state == SUCCEEDED
    assert rig.exemptions[-1] is None


@pytest.mark.parametrize('message', [
    'pivot motion timeout', 'pivot motion canceled', 'odometry timeout',
    'vehicle emergency stop', 'pivot motion made no progress',
])
def test_other_failures_are_never_overridden(rig, message):
    done = start_verification(rig, message=message)
    assert done == [(False, message)]
    assert rig.adapter._pivot_verify_timer is None


@pytest.mark.parametrize('fault', ['heading', 'position', 'yaw_rate', 'velocity',
                                 'emergency', 'interlock', 'parking_brake',
                                 'invalid_pose', 'invalid_rate'])
def test_even_action_success_requires_valid_stationary_map_pose(rig, fault):
    if fault == 'heading':
        rig.pose.transform.rotation.z = 0.0
        rig.pose.transform.rotation.w = 1.0
    elif fault == 'position':
        rig.pose.transform.translation.x += 1.0
    elif fault == 'yaw_rate':
        rig.odom.twist.twist.angular.z = 0.10
    elif fault == 'velocity':
        rig.state.velocity_mps = 0.10
    elif fault == 'emergency':
        rig.state.emergency_stopped = True
    elif fault == 'interlock':
        rig.state.interlock = False
    elif fault == 'parking_brake':
        rig.state.parking_brake = True
    elif fault == 'invalid_pose':
        rig.pose.transform.translation.x = math.nan
    else:
        rig.odom.twist.twist.angular.z = math.nan
    done = start_verification(rig, success=True, message='pivot motion completed')
    rig.tick(3.1)
    assert len(done) == 1 and not done[0][0]
    assert not rig.moves and not rig.exemptions


@pytest.mark.parametrize('source', ['pose', 'odom', 'state'])
def test_stale_source_headers_cannot_complete_pivot(rig, source):
    getattr(rig, source).header.stamp = Time(seconds=90).to_msg()
    ok, reason, _ = rig.adapter._pivot_completion_status(rig.target)
    assert not ok and 'stale' in reason


def test_one_good_sample_or_unchanged_tf_cannot_complete_pivot(rig):
    done = start_verification(rig)
    rig.tick(0.4, fresh=False)
    assert not done  # No new map pose despite being within the freshness timeout.
    rig.odom.twist.twist.angular.z = 0.10
    rig.tick()
    rig.odom.twist.twist.angular.z = 0.0
    rig.tick()
    rig.tick(0.2)
    assert not done
    rig.tick(0.2)
    assert done[0][0]


def test_cancel_invalidates_verification_and_late_action_result(rig):
    done = start_verification(rig)
    old_callback = rig.pivots[0]['done_callback']
    rig.adapter.cancel_goal()
    rig.tick(1.0)
    old_callback(False, YAW_FAILURE, SimpleNamespace(success=False))
    assert not done and not rig.moves
    assert rig.adapter._pivot_verify_timer is None


def test_already_aligned_still_requires_stationary_verification(rig):
    rig.adapter._pivot_start_is_valid = lambda target: (True, '', 0.0)
    rig.state.velocity_mps = 0.4
    done = []
    rig.adapter.send_goal(rig.target, lambda *args: done.append(args))
    rig.tick(0.4)
    assert not done and not rig.pivots
    rig.state.velocity_mps = 0.0
    for _ in range(5):
        rig.tick()
    assert done[0][0]
