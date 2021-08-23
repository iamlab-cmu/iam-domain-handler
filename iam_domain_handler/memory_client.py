import rospy
import pickle

from domain_handler_msgs.srv import *

class MemoryClient:

    def __init__(self):
        self._get_memory_objects_srv_name = 'get_memory_objects'
        self._set_memory_objects_srv_name = 'set_memory_objects'
        self._load_memory_srv_name = 'load_memory'
        self._clear_memory_srv_name = 'clear_memory'
        self._save_memory_srv_name = 'save_memory'
        rospy.wait_for_service(self._get_memory_objects_srv_name)
        rospy.wait_for_service(self._set_memory_objects_srv_name)
        rospy.wait_for_service(self._load_memory_srv_name)
        rospy.wait_for_service(self._clear_memory_srv_name)
        rospy.wait_for_service(self._save_memory_srv_name)

        self._get_memory_objects_srv_proxy = rospy.ServiceProxy(self._get_memory_objects_srv_name, GetMemoryObjects)
        self._set_memory_objects_srv_proxy = rospy.ServiceProxy(self._set_memory_objects_srv_name, SetMemoryObjects)
        self._load_memory_srv_proxy = rospy.ServiceProxy(self._load_memory_srv_name, LoadMemory)
        self._clear_memory_srv_proxy = rospy.ServiceProxy(self._clear_memory_srv_name, ClearMemory)
        self._save_memory_srv_proxy = rospy.ServiceProxy(self._save_memory_srv_name, SaveMemory)

    def clear_memory(self):
        return self._clear_memory_srv_proxy().success

    def load_memory(self, filename):
        return self._load_memory_srv_proxy(filename).success

    def save_memory(self, filename):
        return self._save_memory_srv_proxy(filename).success

    def get_memory_objects(self, keys):
        try:
            return pickle.loads(self._get_memory_objects_srv_proxy(keys).objects)
        except:
            return None

    def set_memory_objects(self, objects):
        try:
            return self._set_memory_objects_srv_proxy(pickle.dumps(objects)).success
        except:
            return False