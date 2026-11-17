"""Interpret the vehicle's worker health and task-owned arrival evidence.

These are software observations, not independent proof of physical arrival.
"""
import copy
import math

from .protocol import ProtocolError


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def execution_problem(state, ros_now, max_age=2.):
    """Gate route updates, never PAUSE/CANCEL, on current worker observations.

    Passing does not assert localization or Mux readiness: the vehicle performs
    those checks. A fresh publication alone does not imply a healthy worker.
    """
    if not _finite(max_age) or max_age <= 0:
        raise ProtocolError('execution_max_age must be finite and positive')
    stamp = state.get('execution_updated_stamp')
    if not _finite(ros_now) or not _finite(stamp) or stamp <= 0 or not 0 <= ros_now-stamp <= max_age:
        return 'Execution snapshot stale, missing or in the future'
    if state.get('datum') != 'WGS84' or state.get('physical_reference') != 'base_link_horizontal':
        return 'Vehicle datum/physical reference does not match WGS84 base_link_horizontal'
    if state.get('startup_checked') is not True or state.get('runtime_phase') == 'WAITING_NAVIGATION':
        return 'Vehicle has not confirmed startup navigation status'
    if state.get('cancellation_pending') is True or state.get('phase') == 'CANCELING':
        return 'Vehicle is still waiting for Action cancellation'
    return None


def _completion_problem(result, task_id):
    if not isinstance(result, dict):
        return 'Final arrival evidence unavailable'
    target = result.get('target')
    if not isinstance(target, dict) or target.get('task_id') != task_id:
        return 'Arrival evidence belongs to another or unknown task'
    action, stop, geo = (result.get(key) for key in ('move_base', 'stop', 'geographic'))
    if not all(isinstance(value, dict) for value in (action, stop, geo)):
        return 'Action, stop or geographic evidence unavailable'
    if type(action.get('state')) is not int or action['state'] != 3:
        return 'Final Action did not report SUCCEEDED'
    if stop.get('confirmed') is not True or geo.get('confirmed') is not True or geo.get('qualified') is not True:
        return 'Stopping and qualified geographic review are not both confirmed'
    residual, tolerance = geo.get('horizontal_residual_m'), geo.get('tolerance_m')
    if not all(_finite(value) and value >= 0 for value in (residual, tolerance)) or residual > tolerance:
        return 'Geographic residual or tolerance is invalid/out of range'
    window = geo.get('stopped_window')
    if not isinstance(window, dict):
        return 'Geographic stopped window unavailable'
    start, end, stamp = window.get('start_stamp'), window.get('end_stamp'), geo.get('stamp')
    if not all(_finite(value) for value in (start, end, stamp)) or not start <= stamp <= end:
        return 'Qualified RTK observation is outside the stopped window'
    if stop.get('window_start_stamp') != start or stop.get('window_end_stamp') != end:
        return 'Stop and geographic windows do not match'
    return None


def task_report(state, task_id):
    """Report only the requested task; never infer success from stale arrival.

    A previous task can still be queried through last_task_result while a new
    task runs. The caller must obtain a fresh state before using this function.
    """
    if not isinstance(task_id, str) or not task_id.strip():
        raise ProtocolError('task_id must be a nonempty string')
    record = state.get('last_task_result')
    result = None
    if isinstance(record, dict) and record.get('task_id') == task_id:
        status = record.get('status', 'UNKNOWN')
        reason = record.get('reason', '')
        result = record.get('result')
        if status == 'COMPLETED':
            problem = _completion_problem(result, task_id)
            return dict(task_id=task_id, status='COMPLETION_UNCONFIRMED' if problem else 'COMPLETED',
                        verified_complete=problem is None, reason=problem or reason,
                        result=copy.deepcopy(result))
    elif state.get('task_id') == task_id:
        status, reason = state.get('phase', 'UNKNOWN'), state.get('reason', '')
        if status == 'COMPLETED':
            status, reason = 'COMPLETION_UNCONFIRMED', 'Matching last_task_result unavailable'
    else:
        status, reason = 'UNKNOWN', 'Task is absent from the latest snapshot; history is not retained here'
    return dict(task_id=task_id, status=status, verified_complete=False, reason=reason,
                result=copy.deepcopy(result))
