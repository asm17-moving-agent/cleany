from sensor_msgs.msg import JointState
from cleany_skill_executor.core.grasp_selection import REQUIRED_JOINT_NAMES

class Future:
    def __init__(self, value):
        self.value = value

    def done(self):
        return True

    def result(self):
        return self.value

    def cancel(self):
        pass


class ServiceClient:
    def __init__(self, response):
        self.response = response
        self.requests = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request)
        return Future(self.response)


class GoalHandle:
    accepted = True

    def __init__(self, result):
        self.result = result

    def get_result_async(self):
        return Future(self.result)


class PlanClient:
    def __init__(self, wrapped):
        self.wrapped = wrapped
        self.goals = []

    def wait_for_server(self, timeout_sec):
        return True

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return Future(GoalHandle(self.wrapped))


def current_state():
    state = JointState()
    state.name = list(REQUIRED_JOINT_NAMES)
    state.position = [float(index) for index in range(len(state.name))]
    state.velocity = [0.0] * len(state.name)
    return state
