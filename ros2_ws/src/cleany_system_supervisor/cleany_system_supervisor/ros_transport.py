"""Nonblocking ROS Action clients; terminal results are distinct from acceptance."""
from threading import RLock

from action_msgs.msg import GoalStatus
from cleany_interfaces.action import RecoverComponent, StopMission
from rclpy.action import ActionClient


class ActionCalls:
    def __init__(self, node, action_type, group):
        self.node, self.action_type, self.group = node, action_type, group
        self.clients, self.calls = {}, {}
        self.lock = RLock()

    def send(self, key, endpoint, goal):
        with self.lock:
            client = self.clients.get(endpoint)
            if client is None:
                client = ActionClient(self.node, self.action_type, endpoint, callback_group=self.group)
                self.clients[endpoint] = client
            if not client.server_is_ready():
                raise RuntimeError(f'{endpoint} action unavailable')
            call = {'status': 'RUNNING', 'handle': None, 'cancel': False}
            self.calls[key] = call
            future = client.send_goal_async(goal)
            future.add_done_callback(lambda completed: self._accepted(call, completed))

    def _accepted(self, call, future):
        with self.lock:
            try:
                handle = future.result()
                if not handle.accepted:
                    call['status'] = 'FAILED'
                    return
                call['handle'], call['status'] = handle, 'ACCEPTED'
                handle.get_result_async().add_done_callback(lambda done: self._result(call, done))
                if call['cancel']:
                    handle.cancel_goal_async()
            except Exception:
                # Unknown remote outcome must time out and remain quarantined.
                call['status'] = 'UNKNOWN'

    def _result(self, call, future):
        with self.lock:
            try:
                response = future.result()
                succeeded = response.status == GoalStatus.STATUS_SUCCEEDED
                success = getattr(response.result, 'success', getattr(response.result, 'stopped', False))
                call['status'] = ('SUCCEEDED' if succeeded and success else
                                  'CANCELED' if response.status == GoalStatus.STATUS_CANCELED else 'FAILED')
            except Exception:
                call['status'] = 'UNKNOWN'

    def poll(self, key):
        with self.lock:
            return self.calls.get(key, {}).get('status', 'FAILED')

    def cancel(self, key):
        with self.lock:
            call = self.calls[key]
            call['cancel'] = True
            if call['handle']:
                call['handle'].cancel_goal_async()


class RecoveryPort:
    def __init__(self, node, group):
        self.calls = ActionCalls(node, RecoverComponent, group)

    @staticmethod
    def _key(job):
        return (job.job_id, job.attempt)

    def start(self, job):
        goal = RecoverComponent.Goal(job_id=job.job_id, fault_code=job.fault_code, source=job.source)
        self.calls.send(self._key(job), job.policy['action'], goal)

    def poll(self, job):
        return self.calls.poll(self._key(job))

    def cancel(self, job):
        self.calls.cancel(self._key(job))


class MissionStopPort:
    def __init__(self, node, endpoint, group):
        self.calls, self.endpoint = ActionCalls(node, StopMission, group), endpoint

    def request(self, lease, epoch):
        goal = StopMission.Goal(epoch=epoch, mission_id=lease.mission_id,
                                lease_id=lease.lease_id, reason=lease.reason)
        self.calls.send(lease.lease_id, self.endpoint, goal)

    def poll(self, key):
        result = self.calls.poll(key)
        return 'STOPPED' if result == 'SUCCEEDED' else result
