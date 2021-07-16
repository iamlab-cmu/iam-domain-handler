import rospy
import numpy as np
import quaternion as qt
from franka_interface_msgs.msg import RobotState
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory

from domain_handler_msgs.msg import HumanInterfaceReply, Confirmation
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
    button_values = None
    for button_idx, button in enumerate(buttons):
        button_values.append(button.value)
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
        'button_values' : button_values,
        'slider_values' : slider_values,
        'bbox_values' : bbox_values,
    }


def human_interface_confirmation_handler(data):
    return {
        'query_done' : data.succeed,
    }

def human_server_reset_handler(data):
    return {
        'query_done' : data.succeed,
    }

def skill_trajectory_done_reset_handler(data):
    return {
        'skill_trajectory_done' : False,
    }

def skill_trajectory_handler(data):
    pts = []
    for pt in data.points:
        pts += list(pt.positions)
    return {
        'skill_trajectory' : pts,
        'skill_trajectory_done' : True,
    }

if __name__ == '__main__':
    human_interface_handlers = [
        ('/human_interface_reply', HumanInterfaceReply, human_interface_reply_handler),
        ('/human_interface_confirmation', Confirmation, human_interface_confirmation_handler),
        ('/reset_query_done_state', Confirmation, human_server_reset_handler),
    ]
    
    robot_server_handlers = [
        ('/set_state_trajectory', JointTrajectory, skill_trajectory_handler),
        ('/state_trajectory_done_reset', Confirmation, skill_trajectory_done_reset_handler),
    ]
    
    sub_handlers = [
        ('/robot_state_publisher_node_1/robot_state', RobotState, robot_state_handler),
        ('/pen_pose', Pose, pen_pose_handler),
    ]
    state_server = StateServer(human_interface_handlers + robot_server_handlers + sub_handlers)