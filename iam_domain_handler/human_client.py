from threading import Thread
import rospy

from domain_handler_msgs.srv import RunQuery
from .action_registry_client import ActionRegistryClient
from bokeh_server_msgs.msg import Request


class HumanClient:

    def __init__(self):
        self._run_query_srv_name = 'run_query'
        
        self._action_registry_client = ActionRegistryClient()

        self._run_query_srv_proxy = rospy.ServiceProxy(self._run_query_srv_name, RunQuery)
        self._bokeh_request_pub = rospy.Publisher('/bokeh_request', Request, queue_size=10)

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

    def cancel_query(self, query_id):
        return self._action_registry_client.set_action_status(query_id, 'success')

    def cancel_query(self, query_id):
        return self._action_registry_client.set_action_status(query_id, 'success')

    def label_image(self, image):
        req = {}
        req['display_type'] = 1
        req['image'] = image
        self.send_bokeh_request(req)

    def get_point_goals(self, image):
        req = {}
        req['display_type'] = 2
        req['image'] = image
        self.send_bokeh_request(req)

    def send_bokeh_request(self, req):
        bokeh_request_msg = Request()
        bokeh_request_msg.display_type = req['display_type']
        if req['display_type'] == 0:
            bokeh_request_msg.traj = req['traj']
        elif req['display_type'] == 1 or req['display_type'] == 2:
            bokeh_request_msg.image = req['image']
        self._bokeh_request_pub.publish(bokeh_request_msg)
