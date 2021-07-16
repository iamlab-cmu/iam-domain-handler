import rospy

from domain_handler_msgs.srv import RunQuery
from web_interface_msgs.msg import Request, Confirmation

from .state_client import StateClient
from .action_registry_client import ActionRegistryClient
from .human_interface_msg_process import params_to_human_interface_request_msg
from std_msgs.msg import Int32


class HumanServer:

    def __init__(self):
        rospy.init_node('human_server', anonymous=True)
        
        self._state_client = StateClient()
        self._action_registry_client = ActionRegistryClient()
        self._human_interface_pub = rospy.Publisher('/human_interface_request', Request, queue_size=1000)
        self._state_server_reset_pub = rospy.Publisher('/reset_query_done_state', Confirmation, queue_size=10)
        self._run_query_srv = rospy.Service('run_query', RunQuery, self._run_query_srv_handler)

        rospy.loginfo('Running Human Interface Server...')
        rospy.spin()

    def _run_query_srv_handler(self, req):                
        query_info = self._action_registry_client.get_action_info(req.query_id)
        self._action_registry_client.set_action_status(req.query_id, 'running')
        hi_msg = params_to_human_interface_request_msg(query_info.action_param)
        
        cur_state = self._state_client.get_state()

        reset_msg = Confirmation()
        reset_msg.succeed = False
        self._state_server_reset_pub.publish(reset_msg)
        self._human_interface_pub.publish(hi_msg)

        query_not_done = False 
        rate = rospy.Rate(10)
        while not query_not_done:
            cur_state = self._state_client.get_state()
            if cur_state.has_prop('query_done'):
                rospy.loginfo('SUCCESS 0...')
                query_not_done = cur_state['query_done'][0] > 0
            rate.sleep()
        rospy.loginfo('SUCCESS...')
        self._action_registry_client.set_action_status(req.query_id, 'success')
        return 'success'
