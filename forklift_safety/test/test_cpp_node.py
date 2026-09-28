"""Real native-node IO tests in an isolated test namespace, never vehicle topics."""
import math
import copy
import os
import subprocess
import tempfile
import time
import uuid

import pytest
import rclpy
from forklift_msgs.msg import ForkliftControlCommand, ForkliftVehicleState
from forklift_msgs.srv import SetEmergencyStop
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool


@pytest.fixture
def native(request):
    executable = os.environ.get('SAFETY_CPP_NODE')
    if not executable:
        pytest.skip('native test executable is supplied by CMake')
    rclpy.init()
    node = rclpy.create_node('gate_test_' + uuid.uuid4().hex[:8])
    prefix = '/gate_test_' + uuid.uuid4().hex[:8]
    args = [executable, '--ros-args', '-p', 'use_sim_time:=false']
    scan_only = getattr(request, 'param', '') == 'scan_only'
    if scan_only:
        args += ['-p', 'costmap_monitor_enabled:=false', '-p', 'costmap_collision_check_enabled:=false']
    for key, value in dict(raw_command_topic=prefix+'/raw', gated_command_topic=prefix+'/out',
        status_topic=prefix+'/status', scan_topic=prefix+'/scan', costmap_topic=prefix+'/grid',
        localization_topic=prefix+'/odom', vehicle_state_topic=prefix+'/vehicle',
        recovery_twist_topic=prefix+'/twist', fault_state_topic=prefix+'/fault',
        pallet_exemption_pose_topic=prefix+'/target', pallet_exemption_active_topic=prefix+'/active').items():
        args += ['-p', key+':='+value]
    output = tempfile.TemporaryFile(mode='w+')
    process = subprocess.Popen(args, stdout=output, stderr=output)
    publishers = {name: node.create_publisher(kind, prefix+'/'+name, 1) for name,kind in
        [('raw',ForkliftControlCommand),('scan',LaserScan),('grid',Costmap),
         ('odom',Odometry),('vehicle',ForkliftVehicleState),('target',PoseStamped),('active',Bool)]}
    commands, reasons = [], []
    subscriptions = [node.create_subscription(ForkliftControlCommand,prefix+'/out',commands.append,100),
                     node.create_subscription(String,prefix+'/status',lambda m: reasons.append(m.data),100)]

    def cycle(duration, obstacle=False, stale=False, stop=False, pallet=False, rear_range=1.8):
        end = time.monotonic()+duration
        while time.monotonic()<end:
            stamp=node.get_clock().now().to_msg()
            msg=Odometry(); msg.header.stamp=copy.deepcopy(stamp); msg.header.frame_id='odom'
            msg.child_frame_id='base_link'; msg.pose.pose.orientation.w=1.
            if stale:
                msg.header.stamp.sec-=2
            publishers['odom'].publish(msg)
            scan=LaserScan(); scan.header.stamp=stamp; scan.header.frame_id='base_link'
            scan.angle_min=-math.pi; scan.angle_max=math.pi; scan.angle_increment=2*math.pi/720
            scan.range_min=.01; scan.range_max=8.; scan.ranges=[float('inf')]*721
            if obstacle:
                scan.ranges[360]=1.
            if pallet:
                scan.ranges[0]=rear_range
                target=PoseStamped(); target.header.stamp=stamp; target.header.frame_id='base_link'
                target.pose.position.x=-1.8; target.pose.orientation.w=1.
                publishers['target'].publish(target)
                active=Bool(); active.data=True; publishers['active'].publish(active)
            publishers['scan'].publish(scan)
            grid=Costmap(); grid.header.frame_id='odom'; grid.header.stamp=stamp
            grid.metadata.size_x=grid.metadata.size_y=360; grid.metadata.resolution=.05
            grid.metadata.origin.position.x=grid.metadata.origin.position.y=-9.
            grid.metadata.origin.orientation.w=1.; grid.data=[0]*129600
            if not scan_only:
                publishers['grid'].publish(grid)
            state=ForkliftVehicleState(); state.enabled=state.auto_mode=state.interlock=True
            state.header.stamp=stamp; publishers['vehicle'].publish(state)
            cmd=ForkliftControlCommand(); cmd.header.stamp=stamp
            cmd.enable=not stop; cmd.brake=stop; cmd.forward=not stop and not pallet
            cmd.reverse=not stop and pallet
            cmd.velocity_mps=0. if stop else (.1 if pallet else .3); cmd.drive_rpm=0. if stop else 100.
            publishers['raw'].publish(cmd)
            rclpy.spin_once(node,timeout_sec=.01)
            time.sleep(.02)
        for _ in range(10):
            rclpy.spin_once(node,timeout_sec=.005)
    try:
        cycle(1.5)
        assert process.poll() is None
        yield node,cycle,commands,reasons,publishers
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait()
        if process.returncode not in (0,-15):
            output.seek(0); print(output.read())
        output.close(); subscriptions.clear(); node.destroy_node(); rclpy.shutdown()


def test_latest_odom_remains_fresh_and_obstacle_still_stops(native):
    _,cycle,commands,reasons,_=native
    commands.clear(); reasons.clear(); cycle(1.)
    assert any(c.enable and not c.brake and c.velocity_mps>0 for c in commands)
    assert 'costmap pose source stale' not in reasons
    commands.clear(); reasons.clear(); cycle(.7,obstacle=True)
    assert 'scan footprint sweep collision' in reasons
    assert commands[-1].brake and commands[-1].drive_rpm==0
    commands.clear(); reasons.clear(); cycle(1.2)
    assert any(c.enable and not c.brake for c in commands[-5:])


def test_raw_stop_cannot_be_overwritten_and_emergency_is_immediate(native):
    node,cycle,commands,reasons,_=native
    commands.clear(); cycle(.5,stop=True)
    assert all(c.brake and c.drive_rpm==0 for c in commands[-5:])
    client=node.create_client(SetEmergencyStop,'/forklift_safety/set_emergency_stop')
    assert client.wait_for_service(timeout_sec=2.)
    request=SetEmergencyStop.Request(); request.emergency_stop=True
    future=client.call_async(request)
    rclpy.spin_until_future_complete(node,future,timeout_sec=2.)
    assert future.result().success
    commands.clear(); cycle(.6)
    assert 'emergency stop' in reasons
    assert all(c.brake and c.drive_rpm==0 for c in commands[-5:])


def test_stale_odom_is_not_accepted_as_fresh(native):
    _,cycle,commands,reasons,_=native
    commands.clear(); reasons.clear(); cycle(1.2,stale=True)
    assert 'costmap pose source stale' in reasons
    assert commands[-1].brake
    cycle(.5)
    assert any(c.enable and not c.brake for c in commands[-5:])


@pytest.mark.parametrize('native', ['scan_only'], indirect=True)
def test_scan_only_needs_no_costmap_but_keeps_obstacle_stop(native):
    node,cycle,commands,reasons,publishers=native
    commands.clear(); reasons.clear(); cycle(.7)
    assert any(c.enable and not c.brake for c in commands)
    assert not any('costmap' in reason for reason in reasons)
    # Even an invalid, subsequently stale map must not affect scan-only release.
    grid=Costmap(); grid.header.frame_id='odom'; grid.header.stamp=node.get_clock().now().to_msg()
    publishers['grid'].publish(grid)
    cycle(.6,obstacle=True)
    assert 'scan footprint sweep collision' in reasons
    assert commands[-1].brake
    cycle(1.2)
    assert any(c.enable and not c.brake for c in commands[-5:])


def test_stop_interrupts_expensive_sweep_without_motion_after_stop(native):
    node,cycle,commands,_,publishers=native
    cycle(.5)
    scan=LaserScan(); scan.header.stamp=node.get_clock().now().to_msg()
    scan.header.frame_id='base_link'; scan.angle_min=0.; scan.angle_max=math.pi
    scan.angle_increment=math.pi/100000; scan.range_min=.01; scan.range_max=8.
    scan.ranges=[3.]*100001
    publishers['scan'].publish(scan)
    time.sleep(.04)
    cmd=ForkliftControlCommand(); cmd.header.stamp=node.get_clock().now().to_msg(); cmd.brake=True
    commands.clear(); start=time.monotonic(); publishers['raw'].publish(cmd)
    while time.monotonic()-start<.3 and not any(c.brake for c in commands):
        rclpy.spin_once(node,timeout_sec=.01)
    assert any(c.brake for c in commands)
    stop_index=next(i for i,c in enumerate(commands) if c.brake)
    end=time.monotonic()+.4
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.01)
    assert all(c.brake and c.drive_rpm==0 for c in commands[stop_index:])


def test_pallet_rectangle_does_not_exempt_other_obstacles(native):
    _,cycle,commands,reasons,_=native
    commands.clear(); reasons.clear(); cycle(1.,pallet=True)
    assert any(c.reverse and c.enable and not c.brake for c in commands[-5:])
    cycle(.8,pallet=True,rear_range=2.2)
    assert 'scan footprint sweep collision' in reasons
    assert commands[-1].brake
