"""Compare the native kernel with the retained Python reference, without ROS IO."""
import json
import math
import os
import random
import subprocess

import pytest
from nav_msgs.msg import OccupancyGrid
from forklift_safety import safety_command_gate as reference


def test_native_geometry_matches_reference():
    executable = os.environ.get('SAFETY_PARITY_RUNNER')
    if not executable:
        pytest.skip('native test executable is supplied by CMake')
    rng = random.Random(924)
    queries, expected = [], []
    footprint = [(1.709, .610), (1.709, -.610), (-1.590, -.610), (-1.590, .610)]
    for i in range(240):
        reverse = bool(i % 2)
        speed = .1 if i % 3 == 0 else rng.uniform(.1, 2.)
        steering = rng.uniform(-.4, .4) if i % 4 else 0.
        pivot = i % 11 == 0
        if pivot:
            steering = math.copysign(math.pi / 2, rng.uniform(-1, 1))
            speed = .1
        cmd = reference.ForkliftControlCommand()
        cmd.enable, cmd.forward, cmd.reverse = True, not reverse, reverse
        cmd.velocity_mps, cmd.steering_angle_rad = speed, steering
        distance = reference.dynamic_stopping_distance(speed, .9, 1.5, .5)
        horizon = distance / speed
        q = dict(direction=-1 if reverse else 1, speed=speed, steering=steering,
                 footprint=footprint, horizon=horizon, escape=True)
        poses = None
        if pivot:
            q['measured_yaw_rate'] = rng.uniform(-.2, .2)
            poses = reference.pivot_braking_poses(footprint, math.copysign(speed/.6, steering),
                                                q['measured_yaw_rate'], .9, .15, .05, 0., .05)
        zone = None
        if i % 5 == 0:
            q['zone'] = [2., 0., .3, .9, .8]
            zone = reference.PalletExemptionZone(*q['zone'])
        if i < 120:
            q['points'] = [(rng.uniform(-5, 5), rng.uniform(-5, 5)) for _ in range(50)]
            hit, _ = reference.scan_sweep_collision(q['points'], footprint, cmd, 1.4, .6, 0.,
                distance, .05, math.pi/2, .05, zone, True, prediction_poses=poses)
        else:
            grid = OccupancyGrid()
            grid.info.width = grid.info.height = 100
            grid.info.resolution = .1
            grid.info.origin.position.x = grid.info.origin.position.y = -5.
            grid.info.origin.orientation.w = 1.
            grid.data = [rng.choice([0]*199 + [100, -1]) for _ in range(10000)]
            q['threshold'] = 100
            q['grid'] = dict(width=100, height=100, resolution=float(grid.info.resolution),
                             origin=[-5., -5., 0.], data=list(grid.data))
            q['pose'] = [0., 0., 0.]
            hit, _ = reference.footprint_sweep_collision(grid, footprint, (0.,0.,0.), cmd,
                1.4, .6, 0., horizon, reference.spatial_sweep_time_step(speed,.05,horizon),
                math.pi/2, .05, 100, True, zone, True, prediction_poses=poses)
        queries.append(json.dumps(q))
        expected.append(hit)
    run = subprocess.run([executable], input='\n'.join(queries)+'\n', text=True,
                         capture_output=True, check=True, timeout=60)
    actual = [line.startswith('1 ') for line in run.stdout.splitlines()]
    assert len(actual) == len(expected)
    assert actual == expected
