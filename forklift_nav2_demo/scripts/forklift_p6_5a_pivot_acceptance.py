#!/usr/bin/python3

"""Dedicated P6.5a acceptance entry point.

The implementation is shared with forklift_ab_acceptance so command accounting,
Gazebo reset behavior, and Nav2 result handling stay identical across the
forward/reverse/dynamic regression gates.
"""

import sys

import forklift_ab_acceptance as ab_acceptance


PIVOT_SCENARIOS = {
    name: ab_acceptance.SCENARIOS[name]
    for name in (
        "pivot_90_left_in_place",
        "pivot_90_right_in_place",
        "pivot_90_then_forward_ab",
        "pivot_blocked_stop",
        "l_shaped_corridor_ab",
        "sparse_90_turn_ab",
    )
}


def main():
    ab_acceptance.SCENARIOS = PIVOT_SCENARIOS
    if not any("scenario:=" in argument for argument in sys.argv):
        sys.argv.extend([
            "--ros-args",
            "-p",
            "scenario:=pivot_90_left_in_place",
        ])
    ab_acceptance.main()


if __name__ == "__main__":
    main()
