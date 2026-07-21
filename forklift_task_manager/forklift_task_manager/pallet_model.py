"""Pallet slot configuration helpers for two-stage pickup tasks."""

import math
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Tuple

import yaml


class PalletConfigError(ValueError):
    """Raised when pallet slot configuration is invalid."""


@dataclass(frozen=True)
class PalletSlot:
    """One configured rack slot that can be picked from an approach station."""

    name: str
    approach_station: str
    level_index: int
    pick_height_m: float


@dataclass(frozen=True)
class PickupDefaults:
    """Validated pickup parameters shared by slot pickup tasks."""

    level_pitch_m: float
    scan_level_offset: int
    fork_insert_depth_m: float
    pallet_clearance_m: float
    max_offset_x_m: float
    max_offset_y_m: float
    min_detection_confidence: float
    fork_timeout_sec: float
    detection_timeout_sec: float
    fine_motion_timeout_sec: float
    fine_motion_speed_mps: float
    tilt_rad: float = 0.0


@dataclass(frozen=True)
class PalletPickupConfig:
    """All configured pallet slots plus common pickup defaults."""

    slots: Dict[str, PalletSlot]
    defaults: PickupDefaults


def _finite_float(value: Any, field: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise PalletConfigError('{} must be a number'.format(field)) from exc
    if not math.isfinite(result):
        raise PalletConfigError('{} must be finite'.format(field))
    return result


def _positive_float(value: Any, field: str) -> float:
    result = _finite_float(value, field)
    if result <= 0.0:
        raise PalletConfigError('{} must be greater than zero'.format(field))
    return result


def _non_negative_float(value: Any, field: str) -> float:
    result = _finite_float(value, field)
    if result < 0.0:
        raise PalletConfigError('{} must be non-negative'.format(field))
    return result


def _int_value(value: Any, field: str) -> int:
    if isinstance(value, bool):
        raise PalletConfigError('{} must be an integer'.format(field))
    try:
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise PalletConfigError('{} must be an integer'.format(field)) from exc
    if float(result) != float(value):
        raise PalletConfigError('{} must be an integer'.format(field))
    return result


def _required_mapping(data: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = data.get(key)
    if not isinstance(value, Mapping):
        raise PalletConfigError('{} must be a mapping'.format(key))
    return value


def load_pallet_pickup_config(path: str) -> PalletPickupConfig:
    """Load and validate pallet slot pickup configuration."""

    try:
        with open(path, 'r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise PalletConfigError(
            'failed to load pallet slots {}: {}'.format(path, exc)
        ) from exc
    if not isinstance(data, Mapping):
        raise PalletConfigError('pallet slot config must be a mapping')

    raw_slots = _required_mapping(data, 'slots')
    raw_defaults = _required_mapping(data, 'pickup_defaults')

    slots: Dict[str, PalletSlot] = {}
    for slot_name, raw_slot in raw_slots.items():
        if not isinstance(slot_name, str) or not slot_name:
            raise PalletConfigError('slot names must be non-empty strings')
        if not isinstance(raw_slot, Mapping):
            raise PalletConfigError('slot {} must be a mapping'.format(slot_name))
        approach_station = raw_slot.get('approach_station')
        if not isinstance(approach_station, str) or not approach_station:
            raise PalletConfigError(
                'slot {} approach_station must be a string'.format(slot_name)
            )
        level_index = _int_value(
            raw_slot.get('level_index'), 'slot {} level_index'.format(slot_name)
        )
        if level_index < 0:
            raise PalletConfigError(
                'slot {} level_index must be non-negative'.format(slot_name)
            )
        slots[slot_name] = PalletSlot(
            name=slot_name,
            approach_station=approach_station,
            level_index=level_index,
            pick_height_m=_positive_float(
                raw_slot.get('pick_height_m'),
                'slot {} pick_height_m'.format(slot_name),
            ),
        )

    defaults = PickupDefaults(
        level_pitch_m=_positive_float(
            raw_defaults.get('level_pitch_m'), 'pickup_defaults level_pitch_m'
        ),
        scan_level_offset=_int_value(
            raw_defaults.get('scan_level_offset'),
            'pickup_defaults scan_level_offset',
        ),
        fork_insert_depth_m=_positive_float(
            raw_defaults.get('fork_insert_depth_m'),
            'pickup_defaults fork_insert_depth_m',
        ),
        pallet_clearance_m=_positive_float(
            raw_defaults.get('pallet_clearance_m'),
            'pickup_defaults pallet_clearance_m',
        ),
        max_offset_x_m=_non_negative_float(
            raw_defaults.get('max_offset_x_m'), 'pickup_defaults max_offset_x_m'
        ),
        max_offset_y_m=_non_negative_float(
            raw_defaults.get('max_offset_y_m'), 'pickup_defaults max_offset_y_m'
        ),
        min_detection_confidence=_non_negative_float(
            raw_defaults.get('min_detection_confidence'),
            'pickup_defaults min_detection_confidence',
        ),
        fork_timeout_sec=_positive_float(
            raw_defaults.get('fork_timeout_sec'), 'pickup_defaults fork_timeout_sec'
        ),
        detection_timeout_sec=_positive_float(
            raw_defaults.get('detection_timeout_sec'),
            'pickup_defaults detection_timeout_sec',
        ),
        fine_motion_timeout_sec=_positive_float(
            raw_defaults.get('fine_motion_timeout_sec'),
            'pickup_defaults fine_motion_timeout_sec',
        ),
        fine_motion_speed_mps=_positive_float(
            raw_defaults.get('fine_motion_speed_mps'),
            'pickup_defaults fine_motion_speed_mps',
        ),
        tilt_rad=_finite_float(raw_defaults.get('tilt_rad', 0.0), 'pickup_defaults tilt_rad'),
    )
    if defaults.scan_level_offset < 0:
        raise PalletConfigError('pickup_defaults scan_level_offset must be non-negative')
    if defaults.min_detection_confidence > 1.0:
        raise PalletConfigError(
            'pickup_defaults min_detection_confidence must be <= 1.0'
        )
    return PalletPickupConfig(slots=slots, defaults=defaults)


def pickup_heights(slot: PalletSlot, defaults: PickupDefaults) -> Tuple[float, float]:
    """Return (pick_height_m, scan_height_m) for a pallet slot."""

    pick_height_m = slot.pick_height_m
    scan_height_m = (
        pick_height_m + float(defaults.scan_level_offset) * defaults.level_pitch_m
    )
    return pick_height_m, scan_height_m
