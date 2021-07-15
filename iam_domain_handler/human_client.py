from threading import Thread
import rospy

from domain_handler_msgs.srv import RunSkill

from .skill_registry_client import SkillRegistryClient


class HumanClient:

    def __init__(self):
        self._run_query_srv_name = 'run_query'
        
        self._query_registry_client = SkillRegistryClient()
        self._run_query_srv_proxy = rospy.ServiceProxy(self._run_query_srv_name, RunSkill)
        self._runnning_query_id = None

    def run_query(self, query_params):
        if self._runnning_query_id is not None:
            assert self.get_query_status(self._runnning_query_id) in ('success', 'failure'), 'A query is currently running!'

        self._runnning_query_id = self._query_registry_client.register_skill(query_params)

        # Make this request async so we don't wait until skill finishes
        def req_run_query():
            self._run_query_srv_proxy(self._runnning_query_id)
        th = Thread(target=req_run_query)
        th.start()

        return self._runnning_query_id

    def get_query_status(self, query_id):
        return self._query_registry_client.get_skill_status(query_id)
