import rospy
import pickle

from domain_handler_msgs.srv import *
from iam_vision_msgs.srv import *
from cv_bridge import CvBridge

class VisionClient:

    def __init__(self):
        self._iam_vision_srv_name = 'iam_vision_server'
        rospy.wait_for_service(self._iam_vision_srv_name)

        self.bridge = CvBridge()

        self._iam_vision_srv_proxy = rospy.ServiceProxy(self._iam_vision_srv_name, IAMVision)

    def save_camera_image(self, camera_topic):
        request = IAMVisionRequest()
        request.request_type = 0
        request.camera_topic_name = camera_topic
        resp = self._iam_vision_srv_proxy(request)

        return (resp.request_success, resp.image_path)

    def save_image(self, image_path, image):
        request = IAMVisionRequest()
        request.request_type = 2
        request.camera_topic_name = camera_topic
        request.image = self.bridge.cv2_to_imgmsg(image)
        resp = self._iam_vision_srv_proxy(request)

        return (resp.request_success, resp.image_path)

    def get_image(self, image_path=None):
        request = IAMVisionRequest()
        request.request_type = 1
        if image_path is not None:
            request.image_path = image_path
        resp = self._iam_vision_srv_proxy(request)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(resp.image)
            return (resp.request_success, resp.image_path, cv_image)
        except:
            return (False, resp.image_path, None)

        
