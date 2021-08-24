import rospy
import numpy as np
import quaternion as qt
from franka_interface_msgs.msg import RobotState
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory
from domain_handler_msgs.msg import GetTrajectory

from web_interface_msgs.msg import Reply, Confirmation
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

    bboxes = {}
    for bbox in data.bboxes:
        bboxes[bbox.name] = bbox.value
    
    return {
        'buttons' : buttons,
        'sliders' : sliders,
        'text_inputs' : text_inputs,
        'bboxes' : bboxes,
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
        ('/reset_query_done_state', Confirmation, human_server_reset_handler),
    ]
    
    robot_server_handlers = [
        ('/set_state_trajectory', GetTrajectory, skill_trajectory_handler),
        ('/state_trajectory_done_reset', Confirmation, skill_trajectory_done_reset_handler),
    ]
    
    human_memory_server = HumanMemoryServer(human_interface_handlers + robot_server_handlers)