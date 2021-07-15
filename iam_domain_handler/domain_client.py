from iam_domain_handler.human_client import HumanClient
from .state_client import StateClient
from .robot_client import RobotClient
from .human_client import HumanClient

class DomainClient:

    def __init__(self):
        self._state_client = StateClient()
        self._robot_client = RobotClient()
        self._human_client = HumanClient()

    @property
    def state(self):
        return self._state_client.get_state()

    def run_skill(self, skill_name, skill_param=''):
        return self._robot_client.run_skill(skill_name, skill_param)

    def get_skill_status(self, skill_id):
        return self._robot_client.get_skill_status(skill_id)
<<<<<<< HEAD
    
    def run_query(self, query_name, query_param):
        return self._human_client.run_query(query_name, query_param)
    
    def get_query_status(self, query_id):
        return self._human_client.get_query_status(query_id)
=======

    def run_query(self,query_name, query_param=''):
        print('GOTHERE2')
        return self._human_client.run_query(query_name, query_param)
        
    def get_skill_status(self, query_id):
        return self._human_client.get_query_status(query_id)
>>>>>>> d42c9ffb4f13fc3e3460f3cd1fa4703fff66ca62
