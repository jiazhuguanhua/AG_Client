"""V2 route state, implementing the car TaskExecutor's core interface."""
import copy
import uuid

from ag_client.v2 import validate

EDIT = {'SET_TARGET', 'INSERT_MVP', 'DEL_MVP', 'UPDATE_MVP'}
ENDED = {'IDLE', 'COMPLETED', 'CANCELED'}
MAX_MVPS = 64


class Route:
    def __init__(self):
        self.task_id = self.target = self.active = None
        self.mvps, self.used_ids, self.ended_ids = [], set(), set()
        self.last_sequence = 0
        self.phase, self.reason, self.revision = 'IDLE', '', 0

    def snapshot(self):
        return dict(task_id=self.task_id, phase=self.phase,
                    paused=self.phase in ('PAUSING', 'PAUSED', 'BLOCKED'),
                    active=self.active, reason=self.reason)

    def next_point(self):
        if self.mvps:
            return dict(kind='MVP', **self.mvps[0])
        if self.target is not None:
            return dict(kind='TARGET', mvp_id=None, **self.target)
        return None

    @staticmethod
    def backend_point(point):
        return dict(latitude=point['lat'], longitude=point['lon'],
                    altitude_ellipsoid_m=None, yaw=None)

    def handle(self, command, validate_route=None):
        try:
            command = validate(command)
            if command['seq'] <= self.last_sequence:
                raise ValueError('STALE_SEQUENCE')
            self.last_sequence = command['seq']
            if (command['op'] == 'PAUSE' and command['task_id'] == self.task_id
                    and self.phase in ('PAUSING', 'PAUSED')):
                return dict(accepted=True, reason=None, duplicate=True)
            self.apply(command, validate_route)
        except ValueError as error:
            return dict(accepted=False, reason=str(error))
        return dict(accepted=True, reason=None)

    def apply(self, command, validate_route):
        op, task = command['op'], command['task_id']
        new_task = self.phase in ENDED
        if new_task:
            if op != 'SET_TARGET' or task in self.ended_ids:
                raise ValueError('NO_MATCHING_ACTIVE_TASK')
        elif self.task_id != task:
            raise ValueError('TASK_ID_MISMATCH')
        if self.phase == 'CANCELING':
            raise ValueError('STOP_IN_PROGRESS')

        if op in EDIT:
            self.edit(command, new_task, validate_route)
        elif op == 'PAUSE':
            if self.phase not in ('PAUSED', 'PAUSING'):
                self.active = None
                self.phase, self.reason = 'PAUSING', ''
        elif op == 'RESUME':
            if self.phase not in ('PAUSED', 'BLOCKED'):
                raise ValueError('TASK_NOT_PAUSED_OR_BLOCKED')
            if validate_route:
                self.validate_route(validate_route, self.target, self.mvps)
            self.phase, self.reason = 'READY', ''
        elif op == 'STOP':
            self.active = None
            self.phase, self.reason = 'CANCELING', ''

    def edit(self, command, new_task, validate_route):
        op, mvp_id = command['op'], command.get('mvp_id')
        old_next = self.next_point()
        target = command.get('target', self.target)
        mvps = [] if new_task else list(self.mvps)
        used = set() if new_task else set(self.used_ids)
        ids = [item['mvp_id'] for item in mvps]
        if op == 'INSERT_MVP':
            if mvp_id in used:
                raise ValueError('MVP_ID_ALREADY_USED')
            if len(mvps) == MAX_MVPS:
                raise ValueError('MVP_LIMIT_REACHED')
            before = command.get('before_id')
            if before is not None and before not in ids:
                raise ValueError('BEFORE_MVP_NOT_PENDING')
            mvps.insert(len(mvps) if before is None else ids.index(before),
                        dict(mvp_id=mvp_id, **command['point']))
            used.add(mvp_id)
        elif op in ('DEL_MVP', 'UPDATE_MVP'):
            if mvp_id not in ids:
                raise ValueError('MVP_NOT_PENDING')
            if op == 'DEL_MVP':
                mvps.pop(ids.index(mvp_id))
            else:
                mvps[ids.index(mvp_id)] = dict(mvp_id=mvp_id, **command['point'])

        if validate_route:
            self.validate_route(validate_route, target, mvps)
        self.task_id, self.target, self.mvps, self.used_ids = command['task_id'], target, mvps, used
        self.revision += 1
        if new_task or (self.phase == 'EXECUTING' and old_next != self.next_point()):
            self.active = None
            self.phase, self.reason = 'READY', ''

    def validate_route(self, validate_route, target, mvps):
        validate_route(self.backend_point(target), [self.backend_point(p) for p in mvps])

    def activate(self):
        if self.phase != 'READY':
            return None
        current = self.next_point()
        self.active = dict(token=str(uuid.uuid4()), task_id=self.task_id, revision=self.revision,
                           kind='via' if current['kind'] == 'MVP' else 'final',
                           point=self.backend_point(current), public=current)
        self.phase = 'EXECUTING'
        return copy.deepcopy(self.active)

    def settle(self, token, success, reason='', result=None):
        if self.active is None or self.active['token'] != token or self.phase != 'EXECUTING':
            return False
        kind = self.active['kind']
        self.active = None
        if not success:
            self.block(reason or 'navigation failed')
        elif kind == 'via':
            self.mvps.pop(0)
            self.phase = 'READY'
        else:
            self.phase = 'COMPLETED'
            self.ended_ids.add(self.task_id)
        return True

    def mark_stopped(self):
        if self.phase == 'PAUSING':
            self.phase = 'PAUSED'
        elif self.phase == 'CANCELING':
            self.phase = 'CANCELED'
            self.target, self.mvps = None, []
            self.ended_ids.add(self.task_id)

    def block(self, reason):
        if self.phase not in ENDED:
            self.active = None
            # Keep STOP terminal: a failed stop observation must not make it resumable.
            if self.phase != 'CANCELING':
                self.phase = 'BLOCKED'
            self.reason = str(reason)
