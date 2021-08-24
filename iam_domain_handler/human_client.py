from threading import Thread
import rospy

from domain_handler_msgs.srv import RunQuery
from .action_registry_client import ActionRegistryClient


class HumanClient:

    def __init__(self):
        self._run_query_srv_name = 'run_query'
        
        self._action_registry_client = ActionRegistryClient()

        self._run_query_srv_proxy = rospy.ServiceProxy(self._run_query_srv_name, RunQuery)

        self._running_query_id = None

    def run_query(self, query_name, query_param):
        if self._running_query_id is not None:
            assert self.get_query_status(self._running_query_id) in ('success', 'failure'), 'A query is currently running!'

        self._running_query_id = self._action_registry_client.register_action(query_name, query_param)
        # Make this request async so we don't wait until query finishes
        def req_run_query():
            self._run_query_srv_proxy(self._running_query_id)
            
        th = Thread(target=req_run_query)
        th.start()

        return self._running_query_id

    def get_query_status(self, query_id):
        return self._action_registry_client.get_action_status(query_id)
