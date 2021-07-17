import rospy
from shortuuid import uuid

from domain_handler_msgs.srv import RegisterAction, SetActionStatus, GetActionInfo, DoesActionExist

from .state_client import StateClient


class ActionRegistryServer:

    def __init__(self):
        rospy.init_node('action_registry_server')

        self._action_registry = {}
        self._does_action_exist_srv = rospy.Service('does_action_exist', DoesActionExist, self._does_action_exist_srv_handler)
        self._register_action_srv = rospy.Service('register_action', RegisterAction, self._register_action_srv_handler)
        self._set_action_status_srv = rospy.Service('set_action_status', SetActionStatus, self._set_action_status_srv_handler)
        self._get_action_status_srv = rospy.Service('get_action_info', GetActionInfo, self._get_action_info_srv_handler)

        rospy.loginfo('Running Action Registry Server...')
        rospy.spin()

    def _does_action_exist_srv_handler(self, req):
        action_exists = req.action_id in self._action_registry
        rospy.loginfo(f'Got req: Does Action Exist? action_id={req.action_id} | ret: {action_exists}')
        return action_exists

    def _register_action_srv_handler(self, req):
        action_id = f'{req.action_name}_{uuid()}'
        rospy.loginfo(f'Got req: Registering action {req.action_name} with param  | action_id={action_id}')
        # {req.action_param}
        self._action_registry[action_id] = {
            'action_name': req.action_name,
            'action_param': req.action_param,
            'action_status': 'registered'
        }

        return action_id

    def _set_action_status_srv_handler(self, req):
        prev_action_status = self._action_registry[req.action_id]['action_status']
        action_name = self._action_registry[req.action_id]['action_name']
        action_param = self._action_registry[req.action_id]['action_param']
        rospy.loginfo(f'Got req: Setting action w/ id {req.action_id}, name {action_name}, param  from {prev_action_status} to {req.action_status}') # {action_param}

        self._action_registry[req.action_id]['action_status'] = req.action_status
        return prev_action_status

    def _get_action_info_srv_handler(self, req):
        action_name = self._action_registry[req.action_id]['action_name']
        action_param = self._action_registry[req.action_id]['action_param']
        action_status = self._action_registry[req.action_id]['action_status']
        # rospy.loginfo(f'Got req: Returning action info w/ id {req.action_id}, name {action_name}, param {action_param}, status {action_status}')
        
        return req.action_id, action_name, action_param, action_status
