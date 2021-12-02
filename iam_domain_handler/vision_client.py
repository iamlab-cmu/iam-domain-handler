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

    def save_rgb_camera_image(self, camera_topic):
        request = IAMVisionRequest()
        request.request_type = 0
        request.camera_topic_name = camera_topic
        resp = self._iam_vision_srv_proxy(request)

        return (resp.request_success, resp.rgb_image_path)

    def save_depth_camera_image(self, camera_topic, depth_image_path):
        request = IAMVisionRequest()
        request.request_type = 1
        request.camera_topic_name = camera_topic
        request.depth_image_path = depth_image_path
        resp = self._iam_vision_srv_proxy(request)

        return (resp.request_success, resp.depth_image_path)

    def save_rgb_image(self, rgb_image_path, image):
        request = IAMVisionRequest()
        request.request_type = 5
        request.rgb_image_path = rgb_image_path
        request.image = self.bridge.cv2_to_imgmsg(image)
        resp = self._iam_vision_srv_proxy(request)

        return (resp.request_success, resp.rgb_image_path)

    def save_depth_image(self, depth_image_path, image):
        request = IAMVisionRequest()
        request.request_type = 6
        request.depth_image_path = depth_image_path
        request.image = self.bridge.cv2_to_imgmsg(image)
        resp = self._iam_vision_srv_proxy(request)

        return (resp.request_success, resp.depth_image_path)

    def save_image_labels(self, rgb_image_path, object_names, masks, bounding_boxes):
        request = IAMVisionRequest()
        request.request_type = 3
        request.rgb_image_path = rgb_image_path
        request.object_names = object_names
        request.masks = masks
        request.bounding_boxes = bounding_boxes
        resp = self._iam_vision_srv_proxy(request)

        return (resp.request_success, resp.rgb_image_path)

    def get_rgb_image(self, rgb_image_path=None):
        request = IAMVisionRequest()
        request.request_type = 2
        if rgb_image_path is not None:
            request.rgb_image_path = rgb_image_path
        resp = self._iam_vision_srv_proxy(request)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(resp.image)
            return (resp.request_success, resp.rgb_image_path, cv_image)
        except:
            return (False, resp.rgb_image_path, None)

    def get_depth_image(self, depth_image_path=None):
        request = IAMVisionRequest()
        request.request_type = 7
        if depth_image_path is not None:
            request.depth_image_path = depth_image_path
        resp = self._iam_vision_srv_proxy(request)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(resp.image)
            return (resp.request_success, resp.depth_image_path, cv_image)
        except:
            return (False, resp.depth_image_path, None)

    def get_goal_points(self, depth_image_path, desired_positions):
        request = IAMVisionRequest()
        request.request_type = 4
        request.depth_image_path = depth_image_path
        request.points = desired_positions
        resp = self._iam_vision_srv_proxy(request)

        return (resp.request_success, resp.points)