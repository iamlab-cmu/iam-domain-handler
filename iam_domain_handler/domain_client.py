import time
import json
import numpy as np

from iam_domain_handler.human_client import HumanClient
from .state_client import StateClient
from .robot_client import RobotClient
from .human_client import HumanClient
from .memory_client import MemoryClient
from .vision_client import VisionClient

class DomainClient:

    def __init__(self):
        self._state_client = StateClient()
        self._robot_client = RobotClient()
        self._human_client = HumanClient()
        self._memory_client = MemoryClient()
        self._vision_client = VisionClient()

    @property
    def state(self):
        return self._state_client.get_state()

    def get_skill_traj(self, skill_name, skill_param):
        return self._robot_client.get_skill_traj(skill_name, skill_param)
    
    def run_skill(self, skill_name, skill_param):
        return self._robot_client.run_skill(skill_name, skill_param)
 
    def get_skill_status(self, skill_id):
        return self._robot_client.get_skill_status(skill_id)

    def stop_skill(self, skill_id):
        return self._robot_client.stop_skill(skill_id)

    def cancel_skill(self, skill_id):
        return self._robot_client.cancel_skill(skill_id)
    
    def run_query(self, query_name, query_param):
        return self._human_client.run_query(query_name, query_param)

    def run_query_until_done(self, query_name, query_param, timeout=300):

        response = {}
        has_buttons = ('buttons' in query_param.keys())
        has_sliders = ('sliders' in query_param.keys())
        has_text_inputs = ('text_inputs' in query_param.keys())
        has_dmp_params = ('bokeh_display_type' in query_param.keys() and query_param['bokeh_display_type'] == 0)
        label_image = ('bokeh_display_type' in query_param.keys() and query_param['bokeh_display_type'] == 1)
        has_points = ('bokeh_display_type' in query_param.keys() and query_param['bokeh_display_type'] == 2)
        has_query = ('bokeh_display_type' in query_param.keys() and query_param['bokeh_display_type'] == 3)

        query_id = self.run_query(query_name, json.dumps(query_param))
        query_result = self.wait_until_query_done(query_id, timeout)

        query_complete = False

        while not query_complete:
            while not query_result:
                self.cancel_query(query_id)
                time.sleep(1)
                query_id = self.run_query(query_name, json.dumps(query_param))
                query_result = self.wait_until_query_done(query_id, timeout)

            if has_buttons:
                button_inputs = self.get_memory_objects(['buttons'])['buttons']
                for button in query_param['buttons']:
                    if button['name'] not in button_inputs.keys():
                        query_result = False
                        continue
                response['button_inputs'] = button_inputs
            if has_sliders:
                sliders = self.get_memory_objects(['sliders'])['sliders']
                for slider in query_param['sliders']:
                    if slider['name'] not in sliders.keys():
                        query_result = False
                        continue
                response['sliders'] = sliders
            if has_text_inputs:
                text_inputs = self.get_memory_objects(['text_inputs'])['text_inputs']
                for text_input in query_param['text_inputs']:
                    if text_input['name'] not in text_inputs.keys():
                        query_result = False
                        continue
                response['text_inputs'] = text_inputs
            if has_dmp_params:
                dmp_info = self.get_memory_objects(['dmp_params'])['dmp_params']
                dmp_params = {}
                quat_dmp_params = {}
                dmp_params['dmp_type'] = dmp_info.dmp_type
                dmp_params['tau'] = dmp_info.tau
                dmp_params['alpha'] = dmp_info.alpha
                dmp_params['beta'] = dmp_info.beta
                dmp_params['num_dims'] = dmp_info.num_dims
                dmp_params['num_basis'] = dmp_info.num_basis
                dmp_params['num_sensors'] = dmp_info.num_sensors
                dmp_params['mu'] = dmp_info.mu
                dmp_params['h'] = dmp_info.h
                dmp_params['phi_j'] = dmp_info.phi_j
                dmp_params['weights'] = np.array(dmp_info.weights).reshape((dmp_info.num_dims,dmp_info.num_sensors,dmp_info.num_basis)).tolist()
                if dmp_info.dmp_type == 0:
                    quat_dmp_params['tau'] = dmp_info.quat_tau
                    quat_dmp_params['alpha'] = dmp_info.quat_alpha
                    quat_dmp_params['beta'] = dmp_info.quat_beta
                    quat_dmp_params['num_dims'] = dmp_info.quat_num_dims
                    quat_dmp_params['num_basis'] = dmp_info.quat_num_basis
                    quat_dmp_params['num_sensors'] = dmp_info.quat_num_sensors
                    quat_dmp_params['mu'] = dmp_info.quat_mu
                    quat_dmp_params['h'] = dmp_info.quat_h
                    quat_dmp_params['phi_j'] = dmp_info.quat_phi_j
                    quat_dmp_params['weights'] = np.array(dmp_info.quat_weights).reshape((dmp_info.quat_num_dims,dmp_info.quat_num_sensors,dmp_info.quat_num_basis)).tolist()
                    dmp_params['quat_dmp_params'] = quat_dmp_params
                response['dmp_params'] = dmp_params
            if label_image:
                response = self.get_memory_objects(['request_next_image', 'object_names', 'masks', 'bounding_boxes'])
            if has_points:
                response = self.get_memory_objects(['object_names', 'desired_positions'])
            if has_query:
                response = self.get_memory_objects(['query_type', 'query_response', 'query_point'])
            query_complete = True

        return response

    def cancel_query(self, query_id):
        return self._human_client.cancel_query(query_id)
    
    def wait_until_skill_done(self, skill_id):
        skill_status = self.get_skill_status(skill_id)

        while skill_status != 'success':
            skill_status = self.get_skill_status(skill_id)
            time.sleep(0.1)

    def wait_until_skill_or_query_done(self, skill_id, query_id):
        skill_status = self.get_skill_status(skill_id)
        query_status = self.get_query_status(query_id)

        while skill_status != 'success' and query_status != 'success':
            skill_status = self.get_skill_status(skill_id)
            query_status = self.get_query_status(query_id)
            time.sleep(0.1)
        return (skill_status == 'success', query_status == 'success')

    def wait_until_query_done(self, query_id, timeout=30):
        query_status = self.get_query_status(query_id)
        current_time = 0

        while query_status != 'success' and current_time < timeout:
            query_status = self.get_query_status(query_id)
            time.sleep(0.1)
            current_time += 0.1

        return (query_status == 'success')

    def get_query_status(self, query_id):
        return self._human_client.get_query_status(query_id)

    def get_memory_objects(self, keys):
        return self._memory_client.get_memory_objects(keys)

    def set_memory_objects(self, objects):
        return self._memory_client.set_memory_objects(objects)

    def clear_memory(self, keys):
        return self._memory_client.clear_memory(keys)

    def clear_human_inputs(self):
        return self._memory_client.clear_memory(['buttons', 'sliders', 'text_inputs', 
                                                 'request_next_image', 'object_names', 
                                                 'masks', 'bounding_boxes', 'desired_positions', 
                                                 'query_done'])

    def save_rgb_camera_image(self, camera_topic):
        return self._vision_client.save_rgb_camera_image(camera_topic)

    def save_depth_camera_image(self, camera_topic, depth_image_path):
        return self._vision_client.save_depth_camera_image(camera_topic, depth_image_path)

    def save_rgb_image(self, image_path, image):
        return self._vision_client.save_rgb_image(image_path, image)

    def save_depth_image(self, image_path, image):
        return self._vision_client.save_depth_image(image_path, image)

    def save_image_labels(self, image_path, object_names, masks, bounding_boxes):
        return self._vision_client.save_image_labels(image_path, object_names, masks, bounding_boxes)

    def get_rgb_image(self, rgb_image_path=None):
        return self._vision_client.get_rgb_image(rgb_image_path)

    def get_depth_image(self, depth_image_path=None):
        return self._vision_client.get_depth_image(depth_image_path)

    def get_goal_points(self, depth_image_path, desired_positions):
        return self._vision_client.get_goal_points(depth_image_path, desired_positions)