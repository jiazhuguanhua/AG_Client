import copy
import unittest

from ag_uav_client.observation import execution_problem, task_report
from ag_uav_client.protocol import ProtocolError


class ObservationTests(unittest.TestCase):
    def healthy(self):
        return dict(execution_updated_stamp=100., datum='WGS84',
                    physical_reference='base_link_horizontal', startup_checked=True,
                    cancellation_pending=False, phase='READY', runtime_phase='READY')

    def completed(self):
        result = dict(target=dict(task_id='task'), move_base=dict(state=3),
                      stop=dict(confirmed=True, window_start_stamp=90., window_end_stamp=91.),
                      geographic=dict(confirmed=True, qualified=True, horizontal_residual_m=.1,
                                      tolerance_m=.35, stamp=90.5,
                                      stopped_window=dict(start_stamp=90., end_stamp=91.)))
        return dict(task_id='task', phase='COMPLETED', last_task_result=dict(
            task_id='task', status='COMPLETED', reason='', result=result))

    def test_execution_health(self):
        self.assertIsNone(execution_problem(self.healthy(), 100.1))

    def test_execution_stamp_is_distinct_from_feedback_stamp(self):
        for stamp in (None, 0, 1., 101., float('nan'), True):
            state = self.healthy()
            state.update(updated_stamp=100., execution_updated_stamp=stamp)
            self.assertIsNotNone(execution_problem(state, 100.))

    def test_health_requires_startup_reference_and_cancellation_barrier(self):
        for key, value in [('datum', 'GCJ02'), ('physical_reference', 'antenna'),
                           ('startup_checked', False), ('runtime_phase', 'WAITING_NAVIGATION'),
                           ('cancellation_pending', True), ('phase', 'CANCELING')]:
            state = self.healthy()
            state[key] = value
            self.assertIsNotNone(execution_problem(state, 100.1))

    def test_bad_health_configuration(self):
        for value in (0, -1, float('inf'), True):
            with self.assertRaises(ProtocolError):
                execution_problem(self.healthy(), 100., value)

    def test_complete_requires_three_checks(self):
        report = task_report(self.completed(), 'task')
        self.assertEqual(report['status'], 'COMPLETED')
        self.assertTrue(report['verified_complete'])

    def test_action_success_alone_is_not_completion(self):
        for group, key, bad in [('move_base', 'state', 4), ('move_base', 'state', True),
                                ('stop', 'confirmed', False), ('geographic', 'confirmed', False),
                                ('geographic', 'qualified', False)]:
            state = self.completed()
            state['last_task_result']['result'][group][key] = bad
            report = task_report(state, 'task')
            self.assertEqual(report['status'], 'COMPLETION_UNCONFIRMED')
            self.assertFalse(report['verified_complete'])

    def test_review_numeric_bounds(self):
        for key, bad in [('horizontal_residual_m', .36), ('horizontal_residual_m', -1),
                         ('horizontal_residual_m', float('nan')), ('tolerance_m', True),
                         ('tolerance_m', float('inf')), ('stamp', 92.)]:
            state = self.completed()
            state['last_task_result']['result']['geographic'][key] = bad
            self.assertFalse(task_report(state, 'task')['verified_complete'])

    def test_windows_must_match(self):
        state = self.completed()
        state['last_task_result']['result']['stop']['window_end_stamp'] = 92.
        self.assertFalse(task_report(state, 'task')['verified_complete'])

    def test_different_task_evidence_is_not_reused(self):
        state = self.completed()
        state.update(task_id='new-task', phase='EXECUTING', arrival=state['last_task_result']['result'])
        self.assertEqual(task_report(state, 'new-task')['status'], 'EXECUTING')
        self.assertFalse(task_report(state, 'new-task')['verified_complete'])
        self.assertTrue(task_report(state, 'task')['verified_complete'])
        state['last_task_result']['result']['target']['task_id'] = 'wrong'
        self.assertFalse(task_report(state, 'task')['verified_complete'])

    def test_missing_result_does_not_confirm(self):
        for record in (None, dict(task_id='task', status='COMPLETED', result=None)):
            state = dict(task_id='task', phase='COMPLETED', last_task_result=record)
            self.assertEqual(task_report(state, 'task')['status'], 'COMPLETION_UNCONFIRMED')

    def test_statuses_and_snapshot_copy(self):
        state = self.completed()
        before = copy.deepcopy(state)
        task_report(state, 'task')['result']['stop']['confirmed'] = False
        self.assertEqual(state, before)
        self.assertEqual(task_report(state, 'absent')['status'], 'UNKNOWN')
        state['last_task_result'].update(status='REPLACED', reason='new task')
        self.assertEqual(task_report(state, 'task')['status'], 'REPLACED')
        for phase in ('BLOCKED', 'PAUSED', 'PAUSING', 'CANCELED', 'CANCELING'):
            self.assertEqual(task_report(dict(task_id='task', phase=phase), 'task')['status'], phase)

    def test_task_identity_is_required(self):
        for value in (None, '', True):
            with self.assertRaises(ProtocolError):
                task_report({}, value)


if __name__ == '__main__':
    unittest.main()
