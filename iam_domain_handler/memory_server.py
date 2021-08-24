import rospy
import pickle

from domain_handler_msgs.srv import *

class MemoryServer:

    def __init__(self, saved_memory_file=None):
        if saved_memory_file is not None:
            try:
                self.memory = pickle.load(saved_memory_file)
            except:
                rospy.loginfo('Failed to load saved memory file:' + saved_memory_file)
                self.memory = {}
        else:
            self.memory = {}

        rospy.init_node('memory_server')
        self._get_memory_objects_srv_name = 'get_memory_objects'
        self._set_memory_objects_srv_name = 'set_memory_objects'
        self._load_memory_srv_name = 'load_memory'
        self._clear_all_memory_srv_name = 'clear_all_memory'
        self._clear_memory_srv_name = 'clear_memory'
        self._save_memory_srv_name = 'save_memory'
        self._get_memory_objects_srv = rospy.Service(self._get_memory_objects_srv_name, GetMemoryObjects, self._get_memory_objects_handler)
        self._set_memory_objects_srv = rospy.Service(self._set_memory_objects_srv_name, SetMemoryObjects, self._set_memory_objects_handler)
        self._load_memory_srv = rospy.Service(self._load_memory_srv_name, LoadMemory, self._load_memory_handler)
        self._clear_all_memory_srv = rospy.Service(self._clear_all_memory_srv_name, ClearAllMemory, self._clear_all_memory_handler)
        self._clear_memory_srv = rospy.Service(self._clear_memory_srv_name, ClearMemory, self._clear_memory_handler)
        self._save_memory_srv = rospy.Service(self._save_memory_srv_name, SaveMemory, self._save_memory_handler)
        
        rospy.loginfo('Running Memory Server...')
        rospy.spin()

    def _load_memory_handler(self, req):
        try:
            self.memory = pickle.load(req.filename)
            return LoadMemoryResponse(True)
        except:
            rospy.loginfo('Failed to load saved memory file:' + req.filename)
            return LoadMemoryResponse(False)

    def _save_memory_handler(self, req):
        try:
            pickle.dump(self.memory, req.filename)
            return SaveMemoryResponse(True)
        except:
            rospy.loginfo('Failed to save memory file:' + req.filename)
            return SaveMemoryResponse(False)

    def _clear_all_memory_handler(self, req):
        self.memory = {}
        return ClearAllMemoryResponse(True)

    def _clear_memory_handler(self, req):
        keys = pickle.loads(req.keys)
        for key in keys:
            self.memory.pop(key, None)

        return ClearMemoryResponse(True)

    def _get_memory_objects_handler(self, req):
        response_dict = {}

        keys = pickle.loads(req.keys)
        for key in keys:
            try:
                response_dict[key] = self.memory[key]
            except:
                response_dict[key] = None

        return GetMemoryObjectsResponse(pickle.dumps(response_dict))

    def _set_memory_objects_handler(self, req):
        try:
            request_dict = pickle.loads(req.objects)
            for key in request_dict.keys():
                self.memory[key] = request_dict[key]

            return SetMemoryObjectsResponse(True)
        except:
            return SetMemoryObjectsResponse(False)
