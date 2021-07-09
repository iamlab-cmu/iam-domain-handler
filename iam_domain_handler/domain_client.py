from .state_client import StateClient
from .robot_client import RobotClient


class DomainClient:

    def __init__(self):
        self._state_client = StateClient()
        self._robot_client = RobotClient()

    @property
    def state(self):
        return self._state_client.get_state()

    def run_skill(self, skill_name, skill_param):
        return self._robot_client.run_skill(skill_name, skill_param)

    def get_skill_status(self, skill_id):
        return self._robot_client.get_skill_status(skill_id)
