import rospy
import pickle

from domain_handler_msgs.srv import *
from iam_vision_msgs.srv import *

class VisionClient:

    def __init__(self):
        self._iam_vision_srv_name = 'iam_vision_server'
        rospy.wait_for_service(self._iam_vision_srv_name)

        self._iam_vision_srv_proxy = rospy.ServiceProxy(self._iam_vision_srv_name, IAMVision)

    def save_image(self, image_topic):
        request = IAMVisionRequest()
        request.request_type = 0
        request.image_topic_name = image_topic
        resp = self._iam_vision_srv_proxy(request)

        return (resp.request_success, resp.image_path)

    def get_image(self, image_path=None):
        request = IAMVisionRequest()
        request.request_type = 1
        if image_path is not None:
            request.image_path = image_path
        resp = self._iam_vision_srv_proxy(request)

        return (resp.request_success, resp.image_path, resp.image)
