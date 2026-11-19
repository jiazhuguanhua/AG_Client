import unittest

from ag_ugv_task.route import Route

A = {'lat': 31.0, 'lon': 118.0}
B = {'lat': 31.1, 'lon': 118.1}
C = {'lat': 31.2, 'lon': 118.2}


class RouteTest(unittest.TestCase):
    def setUp(self):
        self.route = Route()
        self.seq = 0

    def command(self, op, task='run', **fields):
        self.seq += 1
        result = self.route.handle(dict(version=2, seq=self.seq, sent_at=1,
                                        task_id=task, op=op, **fields))
        return result['accepted']

    def test_update_target_leaves_active_mvp_and_pending_order(self):
        self.assertFalse(self.command('INSERT_MVP', mvp_id='p1', point=B))
        self.assertTrue(self.command('SET_TARGET', target=A))
        self.assertTrue(self.command('INSERT_MVP', mvp_id='p1', point=B))
        self.assertTrue(self.command('INSERT_MVP', mvp_id='p2', point=C))
        current = self.route.activate()
        self.assertTrue(self.command('SET_TARGET', target=C))
        self.assertEqual(current['token'], self.route.active['token'])
        self.assertTrue(self.route.settle(current['token'], True))
        self.assertEqual(self.route.next_point()['mvp_id'], 'p2')
        self.assertFalse(self.command('UPDATE_MVP', mvp_id='p1', point=A))
        self.assertFalse(self.command('INSERT_MVP', mvp_id='p1', point=A))

    def test_insert_before_update_and_delete_invalidate_only_changed_current(self):
        self.assertTrue(self.command('SET_TARGET', target=A))
        target = self.route.activate()
        self.assertTrue(self.command('INSERT_MVP', mvp_id='p1', point=B))
        self.assertFalse(self.route.settle(target['token'], True))
        first = self.route.activate()
        self.assertFalse(self.command('INSERT_MVP', mvp_id='p2', before_id='missing', point=C))
        self.assertEqual(self.route.active['token'], first['token'])
        self.assertTrue(self.command('INSERT_MVP', mvp_id='p2', before_id='p1', point=C))
        self.assertIsNone(self.route.active)
        second = self.route.activate()
        self.assertEqual(second['public']['mvp_id'], 'p2')
        self.assertTrue(self.command('UPDATE_MVP', mvp_id='p2', point=A))
        self.assertIsNone(self.route.active)
        self.route.activate()
        self.assertTrue(self.command('DEL_MVP', mvp_id='p2'))
        self.assertEqual(self.route.next_point()['mvp_id'], 'p1')

    def test_pause_edits_stop_and_task_ids(self):
        self.assertTrue(self.command('SET_TARGET', target=A))
        self.route.activate()
        self.assertTrue(self.command('PAUSE'))
        self.assertTrue(self.command('SET_TARGET', target=B))
        self.assertTrue(self.command('INSERT_MVP', mvp_id='p1', point=C))
        self.assertFalse(self.command('RESUME'))
        self.route.mark_stopped()
        self.assertEqual(self.route.phase, 'PAUSED')
        self.assertTrue(self.command('RESUME'))
        self.assertEqual(self.route.activate()['public']['mvp_id'], 'p1')
        self.assertFalse(self.command('STOP', task='other'))
        self.assertTrue(self.command('STOP'))
        self.route.block('waiting for parking evidence')
        self.assertFalse(self.command('RESUME'))
        self.route.mark_stopped()
        self.assertIsNone(self.route.target)
        self.assertFalse(self.command('SET_TARGET', target=A))
        self.assertTrue(self.command('SET_TARGET', task='next', target=A))

    def test_validation_is_atomic_and_sequences_do_not_replay(self):
        self.assertTrue(self.command('SET_TARGET', target=A))
        self.seq += 1
        rejected = dict(version=2, seq=self.seq, sent_at=1, task_id='run', op='SET_TARGET', target=B)
        def reject(*_):
            raise ValueError('no localization')
        self.assertFalse(self.route.handle(rejected, reject)['accepted'])
        self.assertEqual(self.route.target, A)
        self.assertFalse(self.route.handle(rejected)['accepted'])
        self.assertFalse(self.command('SET_TARGET', target={'lat': 91, 'lon': 0}))


if __name__ == '__main__':
    unittest.main()
