import rospy

from domain_handler_msgs.srv import RegisterAction, SetActionStatus, GetActionInfo, DoesActionExist


class ActionRegistryClient:

    def __init__(self):
        self._does_action_exist_srv_name = 'does_action_exist'
        self._register_action_srv_name = 'register_action'
        self._set_action_status_srv_name = 'set_action_status'
        self._get_action_info_srv_name = 'get_action_info'
        
        self._does_action_exist_srv_proxy = rospy.ServiceProxy(self._does_action_exist_srv_name, DoesActionExist)
        self._register_action_srv_proxy = rospy.ServiceProxy(self._register_action_srv_name, RegisterAction)
        self._set_action_status_srv_proxy = rospy.ServiceProxy(self._set_action_status_srv_name, SetActionStatus)
        self._get_action_info_srv_proxy = rospy.ServiceProxy(self._get_action_info_srv_name, GetActionInfo)

    def does_action_exist(self, action_id):
        rospy.wait_for_service(self._does_action_exist_srv_proxy)
        return self._does_action_exist_srv_proxy(action_id).action_exists

    def register_action(self, action_name, action_param):
        rospy.wait_for_service(self._register_action_srv_name)
        return self._register_action_srv_proxy(action_name, action_param).action_id

    def set_action_status(self, action_id, action_status):
        rospy.wait_for_service(self._set_action_status_srv_name)
        self._set_action_status_srv_proxy(action_id, action_status)

    def get_action_info(self, action_id):
        rospy.wait_for_service(self._get_action_info_srv_name)
        return self._get_action_info_srv_proxy(action_id)

    def get_action_status(self, action_id):
        return self.get_action_info(action_id).action_status
