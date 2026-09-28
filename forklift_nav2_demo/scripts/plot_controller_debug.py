#!/usr/bin/env python3
"""Export and plot /forklift/controller_debug from a ROS 2 bag."""

import argparse
import csv
import os


FIELDS = (
    'time_sec',
    'route_token',
    'anchor_index',
    'segment_state',
    'planning_mode',
    'frenet_s_m',
    'lateral_error_m',
    'heading_error_rad',
    'velocity_reference_mps',
    'velocity_command_mps',
    'velocity_measured_mps',
    'steering_reference_rad',
    'steering_command_rad',
    'steering_feedback_rad',
    'curvature_inv_m',
    'tracking_preview_m',
    'profile_preview_m',
    'active_velocity_limit_mps',
    'velocity_limit_reason',
    'reverse_motion',
    'pivot_motion',
)


def read_debug_messages(bag_path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path, storage_id='sqlite3'
    )
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = {
        entry.name: entry.type for entry in reader.get_all_topics_and_types()
    }
    topic = '/forklift/controller_debug'
    if topic not in topic_types:
        raise RuntimeError('{} is not present in {}'.format(topic, bag_path))
    message_type = get_message(topic_types[topic])
    rows = []
    first_time = None
    while reader.has_next():
        name, data, timestamp = reader.read_next()
        if name != topic:
            continue
        if first_time is None:
            first_time = timestamp
        message = deserialize_message(data, message_type)
        rows.append({
            'time_sec': (timestamp - first_time) * 1e-9,
            'route_token': message.route_token,
            'anchor_index': message.anchor_index,
            'segment_state': message.segment_state,
            'planning_mode': message.planning_mode,
            'frenet_s_m': message.frenet_s_m,
            'lateral_error_m': message.lateral_error_m,
            'heading_error_rad': message.heading_error_rad,
            'velocity_reference_mps': message.velocity_reference_mps,
            'velocity_command_mps': message.velocity_command_mps,
            'velocity_measured_mps': message.velocity_measured_mps,
            'steering_reference_rad': message.steering_reference_rad,
            'steering_command_rad': message.steering_command_rad,
            'steering_feedback_rad': message.steering_feedback_rad,
            'curvature_inv_m': message.curvature_inv_m,
            'tracking_preview_m': message.tracking_preview_m,
            'profile_preview_m': message.profile_preview_m,
            'active_velocity_limit_mps': message.active_velocity_limit_mps,
            'velocity_limit_reason': message.velocity_limit_reason,
            'reverse_motion': message.reverse_motion,
            'pivot_motion': message.pivot_motion,
        })
    return rows


def write_csv(rows, output_path):
    with open(output_path, 'w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def write_plot(rows, output_path):
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError(
            'matplotlib is required for PNG output; the CSV was still written'
        ) from exc
    time = [row['time_sec'] for row in rows]
    figure, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    axes[0].plot(
        time, [row['lateral_error_m'] for row in rows], label='lateral m'
    )
    axes[0].plot(
        time, [row['heading_error_rad'] for row in rows], label='heading rad'
    )
    axes[0].legend()
    axes[0].grid(True)
    velocity_keys = (
        'velocity_reference_mps',
        'velocity_command_mps',
        'velocity_measured_mps',
    )
    for key in velocity_keys:
        axes[1].plot(time, [row[key] for row in rows], label=key)
    axes[1].legend()
    axes[1].grid(True)
    steering_keys = (
        'steering_reference_rad',
        'steering_command_rad',
        'steering_feedback_rad',
    )
    for key in steering_keys:
        axes[2].plot(time, [row[key] for row in rows], label=key)
    previous_boundary = None
    for index, row in enumerate(rows):
        boundary = (
            row['route_token'], row['anchor_index'], row['segment_state']
        )
        if index > 0 and boundary != previous_boundary:
            for axis in axes:
                axis.axvline(time[index], color='0.6', linewidth=0.7)
        previous_boundary = boundary
    axes[2].legend()
    axes[2].grid(True)
    axes[2].set_xlabel('time [s]')
    figure.tight_layout()
    figure.savefig(output_path, dpi=150)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', help='ROS 2 bag directory')
    parser.add_argument('--output-prefix', default='controller_debug')
    args = parser.parse_args()
    rows = read_debug_messages(args.bag)
    if not rows:
        raise RuntimeError('controller debug topic contains no messages')
    prefix = os.path.abspath(args.output_prefix)
    write_csv(rows, prefix + '.csv')
    write_plot(rows, prefix + '.png')
    print('wrote {}.csv and {}.png ({} samples)'.format(
        prefix, prefix, len(rows)))


if __name__ == '__main__':
    main()
