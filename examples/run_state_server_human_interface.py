import rospy
import numpy as np
import quaternion as qt
from franka_interface_msgs.msg import RobotState
from geometry_msgs.msg import Pose
from domain_handler_msgs.msg import HumanInterfaceReply, HumanInterfaceConfirmation
from iam_domain_handler.state_server import StateServer
from std_msgs.msg import Int32

def pen_pose_handler(data):
    print("get pen pose")
    return {
        'frame:pen:pose/position': [data.position.x, data.position.y, data.position.z],
        'frame:pen:pose/quaternion': [data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z]
    }

def robot_state_handler(data):
    np_ee_pose = np.array(data.O_T_EE).reshape(4, 4).T
    ee_pos = np_ee_pose[:3, 3]
    ee_quat = qt.as_float_array(qt.from_rotation_matrix(np_ee_pose[:3, :3]))
    return {
        'frame:franka:ee:pose/position': ee_pos,
        'frame:franka:ee:pose/quaternion': ee_quat,
        'frame:franka:joints/torque': data.tau_J,
        'frame:franka:joints/position': data.q,
        'frame:franka:joints/position_desired': data.q_d,
        'frame:franka:joints/velocity': data.dq,
        'frame:franka:gripper/width': [data.gripper_width],
        'frame:franka:gripper/is_grasped': [data.gripper_is_grasped],
    }

def human_interface_reply_handler(data):
    '''
    buttons, sliders, text_inputs, bboxes
    '''
    # Buttons
    buttons = data.buttons
    button_clicked = None
    for button_idx, button in enumerate(buttons):
        if button.value:
            button_clicked = button_idx
    # Sliders
    slider_values = []
    for slider in data.sliders:
        slider_values += slider.value

    # Bounding boxes
    bbox_values = []
    for bbox_idx, bbox in enumerate(data.bboxes):
        bbox_values += bbox.value
    
    # text_inputs = data.text_inputs
    # for text_input in text_inputs:
    #     name = text_input.name
    # return_dict['text_inputs_value'] = [text_input.value]
    
    return {
        'clicked_button_index' : button_clicked,
        'slider_values' : slider_values,
        'bbox_values' : bbox_values,
    }


def human_interface_confirmation_handler(data):
    print("HANDLE")
    return {
        'query_done' : data.succeed,
    }

def human_server_reset_handler(data):
    return {
        'query_done' : data.succeed,
    }


def sub_cb(data, handler):
    for k, v in handler(data).items():
        print(k,v)

if __name__ == '__main__':
    sub_handlers = [
        ('/robot_state_publisher_node_1/robot_state', RobotState, robot_state_handler),
        ('/pen_pose', Pose, pen_pose_handler),
        ('/human_interface_reply', HumanInterfaceReply, human_interface_reply_handler),
        ('/human_interface_confirmation', HumanInterfaceConfirmation, human_interface_confirmation_handler),
        ('/reset_query_done_state', HumanInterfaceConfirmation, human_server_reset_handler)
    ]
    state_server = StateServer(sub_handlers)