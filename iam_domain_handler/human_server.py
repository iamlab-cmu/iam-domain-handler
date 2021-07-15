import rospy

from domain_handler_msgs.srv import RunQuery

from .state_client import StateClient
from .action_registry_client import ActionRegistryClient


class HumanServer:

    def __init__(self):
        self._state_client = StateClient()
        self._action_registry_client = ActionRegistryClient()

        self._run_query_srv = rospy.Service('run_query', RunQuery, self._run_query_srv_handler)

        rospy.loginfo('Running Human Interface Server...')
        rospy.spin()


    def _run_query_srv_handler(self, req):                
        query_info = self._action_registry_client.get_action_info(req.query_id)
        self._action_registry_client.set_action_status(req.query_id, 'running')
        
        cur_state = self._state_client.get_state()
        
        ### Process the actual query using  query_info.query_param and cur_state
        ###t_step = self._run_policy_on_robot(init_state, policy, skill_info.skill_param, skill)
        print("query params:",query_info.action_param)
        ### query_status='success'
        ### query_status='failure'
        query_status='success'
    
        self._action_registry_client.set_action_status(req.query_id, query_status)
        return query_status