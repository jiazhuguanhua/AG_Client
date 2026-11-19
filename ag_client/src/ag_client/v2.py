"""Minimal ROS-independent JSON contract for the air/ground V2 topics."""
import math

from .protocol import ProtocolError, loads_json, point

COMMAND = '/air_ground/v2/command'
STATE = '/air_ground/v2/state'
UAV_STATE = '/air_ground/v2/uav_state'
INTENT = '/air_ground/v2/uav_intent'
OPERATIONS = {
    'SET_TARGET': {'target'},
    'INSERT_MVP': {'mvp_id', 'point'},
    'DEL_MVP': {'mvp_id'},
    'UPDATE_MVP': {'mvp_id', 'point'},
    'PAUSE': set(),
    'RESUME': set(),
    'STOP': set(),
}
MAX_SEQ = (1 << 53) - 1


def _id(value, name):
    if not isinstance(value, str) or not value.strip() or len(value) > 128:
        raise ProtocolError(f'{name} must be a nonempty string of at most 128 characters')
    return value


def coordinates(value):
    if not isinstance(value, dict) or set(value) != {'lat', 'lon'}:
        raise ProtocolError('point must contain lat and lon')
    canonical = point(value['lat'], value['lon'])
    return {'lat': canonical['latitude'], 'lon': canonical['longitude']}


def number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def decode(raw):
    if len(raw.encode('utf-8')) > 16384:
        raise ProtocolError('message exceeds 16 KiB')
    value = loads_json(raw)
    if not isinstance(value, dict):
        raise ProtocolError('message must be a JSON object')
    return value


def validate(value, wire=True):
    op = value.get('op')
    if not isinstance(op, str) or op not in OPERATIONS:
        raise ProtocolError('unknown operation')
    base = {'op', 'task_id'}
    if wire:
        base |= {'version', 'seq', 'sent_at'}
        if type(value.get('version')) is not int or value['version'] != 2:
            raise ProtocolError('version must be 2')
        seq, stamp = value.get('seq'), value.get('sent_at')
        if type(seq) is not int or not 1 <= seq <= MAX_SEQ:
            raise ProtocolError('seq must be a positive JSON-safe integer')
        if not number(stamp) or stamp <= 0:
            raise ProtocolError('sent_at must be a positive timestamp')
    required = base | OPERATIONS[op]
    supported = required | ({'before_id'} if op == 'INSERT_MVP' else set())
    if not required <= value.keys() <= supported:
        raise ProtocolError('missing or unexpected fields for ' + op)

    out = dict(value)
    for key in ('task_id', 'mvp_id', 'before_id'):
        if key in out:
            out[key] = _id(out[key], key)
    for key in ('target', 'point'):
        if key in out:
            out[key] = coordinates(out[key])
    return out


def vehicle_position(lat=None, lon=None, stamp=None):
    return dict(lat=lat, lon=lon, measured_at=stamp, valid=lat is not None)


def snapshot(vehicle_id, now, position, *, task_id=None, phase=None, reason=None,
             active=None, mvps=None, target=None, last_command=None):
    return dict(version=2, updated_at=now, vehicle_id=vehicle_id,
                vehicle_position=position, task_id=task_id, phase=phase, reason=reason,
                active_point=active, remaining_mvps=list(mvps or []), target=target,
                last_command=last_command)
