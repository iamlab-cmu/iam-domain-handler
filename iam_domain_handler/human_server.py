import rospy

from domain_handler_msgs.srv import RunQuery, GetHumanInterfaceCmd
from domain_handler_msgs.msg import HumanInterfaceRequest

from .state_client import StateClient
from .action_registry_client import ActionRegistryClient
from .human_interface_msg_process import params_to_human_interface_request_msg


class HumanServer:

    def __init__(self):
        self._state_client = StateClient()
        self._action_registry_client = ActionRegistryClient()
        # self._human_interface_pub = rospy.Publisher('human_interface', HumanInterfaceRequest)
        # self._state_server_reset_pub = rospy.Publisher('reset_query_done_state', ResetState)
        self._run_query_srv = rospy.Service('run_query', RunQuery, self._run_query_srv_handler)

        rospy.loginfo('Running Human Interface Server...')
        rospy.spin()

    def _run_query_srv_handler(self, req):                
        query_info = self._action_registry_client.get_action_info(req.query_id)
        self._action_registry_client.set_action_status(req.query_id, 'running')
        hi_msg = params_to_human_interface_request_msg(query_info.skill_param)
        
        cur_state = self._state_client.get_state()

        self._human_interface_pub.publish(hi_msg)

        query_not_done = False 
        while query_not_done:
            cur_state = self._state_client.get_state()
            query_not_done = not cur_state['any_button_pushed']
    
        self._action_registry_client.set_action_status(req.query_id, query_status)
        return query_status
