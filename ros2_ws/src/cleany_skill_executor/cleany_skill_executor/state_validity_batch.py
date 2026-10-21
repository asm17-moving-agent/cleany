"""Bounded MoveIt state checks; an unavailable extension uses the native service."""
from collections.abc import Callable, Sequence
from typing import Any

from cleany_interfaces.srv import CheckStateValidityBatch
from moveit_msgs.msg import RobotState
from cleany_skill_executor.core.physical_grasp_selection import InfrastructureError


class StateValidityBatch:
    def __init__(self, node: Any, *, batch_size: int = 32, client: Any = None) -> None:
        if type(batch_size) is not int or not 1 <= batch_size <= 64:
            raise ValueError('State validity batch size must be 1..64')
        self.batch_size = batch_size
        self.client = client
        if client is None and hasattr(node, 'create_client'):
            self.client = node.create_client(CheckStateValidityBatch, '/check_state_validity_batch')

    def check(self, states: Sequence[RobotState], call: Callable, *, group_name: str = ''
              ) -> CheckStateValidityBatch.Response | None:
        if self.client is None or not self.client.service_is_ready():
            return None
        if not 1 <= len(states) <= self.batch_size:
            raise ValueError('Invalid state validity batch length')
        request = CheckStateValidityBatch.Request(robot_states=list(states), group_name=group_name)
        response = call(self.client, request)
        if response is None or not response.success:
            raise InfrastructureError('MoveIt batch validity failed: ' +
                                      (response.message if response is not None else 'empty response'))
        index = response.first_invalid_index
        flags = list(response.valid)
        if index == -1:
            well_formed = len(flags) == len(states) and all(flags) and not response.contacts
        else:
            well_formed = (0 <= index < len(states) and len(flags) == index + 1
                           and all(flags[:index]) and flags[index] is False)
        if not well_formed:
            raise InfrastructureError('Malformed MoveIt batch validity response')
        return response
