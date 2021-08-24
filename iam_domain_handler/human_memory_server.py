import rospy
from .memory_client import MemoryClient

class HumanMemoryServer:

    def __init__(self, sub_handlers):
        self._memory_client = MemoryClient()

        self._subs = {}
        for topic, msg_type, handler in sub_handlers:
            self._subs[topic] = rospy.Subscriber(topic, msg_type, self._sub_cb, handler)
        
        rospy.init_node('human_memory_server')

        rospy.loginfo('Running Human Memory Server...')
        rospy.spin()

    def _sub_cb(self, data, handler):
        self._memory_client.set_memory_objects(handler(data))
