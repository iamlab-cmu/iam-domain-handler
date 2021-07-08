import rospy
from pillar_state import State

from domain_handler_msgs.srv import GetCurrentState


class StateClient:

    def __init__(self):
        self._state_srv_name = 'get_current_state'
        rospy.wait_for_service(self._state_srv_name)

        self._state_srv_proxy = rospy.ServiceProxy(self._state_srv_name, GetCurrentState)

    def get_state(self):
        return State.create_from_serialized_string(eval(self._state_srv_proxy().state_protobuf))
