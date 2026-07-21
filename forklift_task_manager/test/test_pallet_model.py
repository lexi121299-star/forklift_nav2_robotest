import pytest
import yaml

from forklift_task_manager.pallet_model import (
    PalletConfigError,
    load_pallet_pickup_config,
    pickup_heights,
)


def write_yaml(path, data):
    path.write_text(yaml.safe_dump(data), encoding='utf-8')
    return str(path)


def valid_config():
    return {
        'slots': {
            'slot_001': {
                'approach_station': 'slot_001_approach',
                'level_index': 1,
                'pick_height_m': 0.55,
            }
        },
        'pickup_defaults': {
            'level_pitch_m': 0.55,
            'scan_level_offset': 3,
            'fork_insert_depth_m': 0.80,
            'pallet_clearance_m': 0.08,
            'max_offset_x_m': 0.20,
            'max_offset_y_m': 0.08,
            'min_detection_confidence': 0.75,
            'fork_timeout_sec': 8.0,
            'detection_timeout_sec': 3.0,
            'fine_motion_timeout_sec': 6.0,
            'fine_motion_speed_mps': 0.08,
        },
    }


def test_load_pallet_pickup_config_and_scan_height(tmp_path):
    config = load_pallet_pickup_config(
        write_yaml(tmp_path / 'pallet_slots.yaml', valid_config())
    )

    slot = config.slots['slot_001']
    pick_height_m, scan_height_m = pickup_heights(slot, config.defaults)

    assert slot.approach_station == 'slot_001_approach'
    assert pick_height_m == pytest.approx(0.55)
    assert scan_height_m == pytest.approx(2.20)


def test_load_pallet_pickup_config_rejects_bad_confidence(tmp_path):
    data = valid_config()
    data['pickup_defaults']['min_detection_confidence'] = 1.2

    with pytest.raises(PalletConfigError, match='confidence'):
        load_pallet_pickup_config(write_yaml(tmp_path / 'bad.yaml', data))


def test_load_pallet_pickup_config_rejects_bad_slot(tmp_path):
    data = valid_config()
    del data['slots']['slot_001']['approach_station']

    with pytest.raises(PalletConfigError, match='approach_station'):
        load_pallet_pickup_config(write_yaml(tmp_path / 'bad.yaml', data))
