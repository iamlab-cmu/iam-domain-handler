import time

from iam_domain_handler.human_client import HumanClient
from .state_client import StateClient
from .robot_client import RobotClient
from .human_client import HumanClient
from .memory_client import MemoryClient

class DomainClient:

    def __init__(self):
        self._state_client = StateClient()
        self._robot_client = RobotClient()
        self._human_client = HumanClient()
        self._memory_client = MemoryClient()

    @property
    def state(self):
        return self._state_client.get_state()

    def get_skill_traj(self, skill_name, skill_param):
        return self._robot_client.get_skill_traj(skill_name, skill_param)
    
    def run_skill(self, skill_name, skill_param):
        return self._robot_client.run_skill(skill_name, skill_param)
 
    def get_skill_status(self, skill_id):
        return self._robot_client.get_skill_status(skill_id)

    def stop_skill(self, skill_id):
        return self._robot_client.stop_skill(skill_id)

    def cancel_skill(self, skill_id):
        return self._robot_client.cancel_skill(skill_id)
    
    def run_query(self, query_name, query_param):
        return self._human_client.run_query(query_name, query_param)

    def run_query_until_done(self, query_name, query_param, timeout=30):

        response = {}
        has_buttons = 'buttons' in query_param.keys()
        has_text_inputs = 'text_inputs' in query_param.keys()

        query_id = self.run_query(query_name, json.dumps(query_param))
        query_result = self.wait_until_query_done(query_id, timeout)

        query_complete = False

        while not query_complete:
            while not query_result:
                self.cancel_query(query_id)
                query_id = self.run_query(query_name, json.dumps(query_param))
                query_result = self.wait_until_query_done(query_id, timeout)

            if has_buttons:
                button_inputs = self.get_memory_objects(['buttons'])['buttons']
                if len(button_inputs.keys()) != len(query_param['buttons']):
                    query_result = False
                    continue
                else:
                    response['button_inputs'] = button_inputs
            if has_text_inputs:
                text_inputs = self.get_memory_objects(['text_inputs'])['text_inputs']
                if len(button_inputs.keys()) != len(query_param['text_inputs']):
                    query_result = False
                    continue
                else:
                    response['text_inputs'] = text_inputs
            query_complete = True

        return response

    def cancel_query(self, query_id):
        return self._human_client.cancel_query(query_id)
    
    def wait_until_skill_done(self, skill_id):
        skill_status = self.get_skill_status(skill_id)

        while skill_status != 'success':
            skill_status = self.get_skill_status(skill_id)
            time.sleep(0.1)

    def wait_until_skill_or_query_done(self, skill_id, query_id):
        skill_status = self.get_skill_status(skill_id)
        query_status = self.get_query_status(query_id)

        while skill_status != 'success' and query_status != 'success':
            skill_status = self.get_skill_status(skill_id)
            query_status = self.get_query_status(query_id)
            time.sleep(0.1)
        return (skill_status == 'success', query_status == 'success')

    def wait_until_query_done(self, query_id, timeout=30):
        query_status = self.get_query_status(query_id)
        current_time = 0

        while query_status != 'success' and current_time < timeout:
            query_status = self.get_query_status(query_id)
            time.sleep(0.1)
            current_time += 0.1

        return (query_status == 'success')

    def get_query_status(self, query_id):
        return self._human_client.get_query_status(query_id)

    def get_memory_objects(self, keys):
        return self._memory_client.get_memory_objects(keys)

    def set_memory_objects(self, objects):
        return self._memory_client.set_memory_objects(objects)

    def clear_memory(self, keys):
        return self._memory_client.clear_memory(keys)

    def clear_human_inputs(self):
        return self._memory_client.clear_memory(['buttons', 'sliders', 'text_inputs', 'bboxes', 'query_done'])
