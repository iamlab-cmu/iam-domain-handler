import rospy
import numpy as np
import quaternion as qt
from franka_interface_msgs.msg import RobotState
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory
from domain_handler_msgs.msg import GetTrajectory

from web_interface_msgs.msg import Reply, Confirmation
from bokeh_server_msgs.msg import Response
from iam_domain_handler.human_memory_server import HumanMemoryServer
from std_msgs.msg import Int32

def human_interface_reply_handler(data):
    '''
    buttons, sliders, text_inputs, bboxes
    '''
    # Buttons
    buttons = {}
    for button in data.buttons:
        buttons[button.name] = button.value

    # Sliders
    sliders = {}
    for slider in data.sliders:
        sliders[slider.name] = slider.value
    
    text_inputs = {}
    for text_input in data.text_inputs:
        text_inputs[text_input.name] = text_input.value
    
    return {
        'buttons' : buttons,
        'sliders' : sliders,
        'text_inputs' : text_inputs,
        'query_done' : True,
    }

def bokeh_server_response_handler(data):
    '''
    response_type, object_names, masks, dmp_params, desired_positions, bounding_boxes
    '''
    
    return {
        'bokeh_response_type' : data.response_type,
        'object_names' : data.object_names,
        'masks' : data.masks,
        'dmp_params' : data.dmp_params,
        'desired_positions' : data.desired_positions,
        'bounding_boxes' : data.bounding_boxes,
        'query_done' : True,
    }

def human_server_reset_handler(data):
    return {
        'query_done' : data.succeed,
    }

def skill_trajectory_done_reset_handler(data):
    print("skill_trajectory_done_reset_handler")
    return {
        'skill_trajectory_done' : False,
    }

def skill_trajectory_handler(data):
    print("skill_trajectory_handler")
    pts = []
    for pt in data.trajectory.points:
        pts += list(pt.positions)
    return {
        'skill_trajectory' : pts,
        'skill_trajectory_done' : True,
    }

if __name__ == '__main__':
    human_interface_handlers = [
        ('/human_interface_reply', Reply, human_interface_reply_handler),
        ('/bokeh_response', Response, bokeh_server_response_handler),
        ('/reset_query_done_state', Confirmation, human_server_reset_handler),
    ]
    
    robot_server_handlers = [
        ('/set_state_trajectory', GetTrajectory, skill_trajectory_handler),
        ('/state_trajectory_done_reset', Confirmation, skill_trajectory_done_reset_handler),
    ]
    
    human_memory_server = HumanMemoryServer(human_interface_handlers + robot_server_handlers)