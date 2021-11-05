import rospy

from domain_handler_msgs.srv import RunQuery
from web_interface_msgs.msg import Request as webrequest
from web_interface_msgs.msg import Confirmation
from bokeh_server_msgs.msg import Request as bokehrequest

from .state_client import StateClient
from .memory_client import MemoryClient
from .action_registry_client import ActionRegistryClient
from .human_interface_msg_process import params_to_web_interface_request_msg, params_to_bokeh_request_msg
from std_msgs.msg import Int32


class HumanServer:

    def __init__(self):
        rospy.init_node('human_server')
        
        self._state_client = StateClient()
        self._memory_client = MemoryClient()
        self._action_registry_client = ActionRegistryClient()
        self._human_interface_pub = rospy.Publisher('/human_interface_request', webrequest, queue_size=10)
        self._bokeh_request_pub = rospy.Publisher('/bokeh_request', bokehrequest, queue_size=10)
        self._state_server_reset_pub = rospy.Publisher('/reset_query_done_state', Confirmation, queue_size=10)
        self._run_query_srv = rospy.Service('run_query', RunQuery, self._run_query_srv_handler)

        rospy.loginfo('Running Human Interface Server...')
        rospy.spin()

    def _run_query_srv_handler(self, req):                
        query_info = self._action_registry_client.get_action_info(req.query_id)
        self._action_registry_client.set_action_status(req.query_id, 'running')
        (hi_msg, pub_bokeh_msg) = params_to_web_interface_request_msg(query_info.action_param)
        
        reset_msg = Confirmation()
        reset_msg.succeed = False
        self._state_server_reset_pub.publish(reset_msg)
        self._human_interface_pub.publish(hi_msg)
        if pub_bokeh_msg:
            rospy.sleep(1)
            bokeh_msg = params_to_bokeh_request_msg(query_info.action_param)
            self._bokeh_request_pub.publish(bokeh_msg)
        rospy.loginfo('Published query message to human interface...')

        query_not_done = False 
        rate = rospy.Rate(10)
        while not query_not_done:
            query_done_dict = self._memory_client.get_memory_objects(['query_done'])
            if query_done_dict is not None and query_done_dict['query_done'] is not None:
                query_not_done = query_done_dict['query_done'] > 0
            rate.sleep()
        rospy.loginfo('Successfully querying human interface and got query_done...')
        self._action_registry_client.set_action_status(req.query_id, 'success')
        return 'success'