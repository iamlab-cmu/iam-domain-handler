import rospy
from pillar_state import State

from domain_handler_msgs.srv import GetCurrentState


class StateServer:

    def __init__(self, sub_handlers):
        rospy.init_node('state_server', anonymous=True)

        self._state = State()

        self._subs = {}
        for topic, msg_type, handler in sub_handlers:
            self._subs[topic] = rospy.Subscriber(topic, msg_type, self._sub_cb, handler)

        self._state_srv = rospy.Service('get_current_state', GetCurrentState, self._state_srv_handler)

        rospy.loginfo('Running State Server...')
        rospy.spin()
    
    def _sub_cb(self, data, handler):
        for k, v in handler(data).items():
            self._state[k] = v

    def _state_srv_handler(self, req):
        return repr(self._state.get_serialized_string())
