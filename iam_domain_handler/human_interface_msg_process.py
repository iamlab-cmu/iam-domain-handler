import json
import numpy as np
from web_interface_msgs.msg import Request as webrequest
from bokeh_server_msgs.msg import Request as bokehrequest
from web_interface_msgs.msg import Button, Slider, TextInput
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def from_dict_button_to_msg_button(button):
    b = Button()
    b.name = button['name']
    b.text = button['text']
    return b

def from_dict_slider_to_msg_slider(slider):
    s = Slider()
    s.name = slider['name']
    s.text = slider['text']
    s.min = slider['min'] 
    s.max = slider['max'] 
    return s

def from_dict_text_input_to_msg_text_input(text_input):
    t = TextInput()
    t.name = text_input['name']
    t.text = text_input['text']
    return t

def from_trajs_to_msg_trajs(trajs):
    """Compile the query parameter into JointTrajectory type to send to human interface
    for display.

    Args:
        trajs: List floats with N x 7 items, where N is the length of the trajectory

    Returns:
        JointTrajectory object
    """
    joint_names = [ 
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
        "panda_joint8",
        "panda_hand_joint",
        "panda_finger_joint1",
        "panda_finger_joint2"
    ]
    N = int(len(trajs) / 7)
    pts = []
    for i in range(N):
        pt = JointTrajectoryPoint()
        start, end = 7*i, 7*(i+1)
        pt.positions = trajs[start:end]
        pts.append(pt)

    pts_msg = JointTrajectory()
    pts_msg.points = pts
    pts_msg.joint_names = joint_names
    return pts_msg

def params_to_web_interface_request_msg(params):
    params = json.loads(params)
    hi_msg = webrequest()  
    pub_bokeh_msg = False

    if 'buttons' in params:
        hi_msg.buttons = [from_dict_button_to_msg_button(button) for button in params['buttons']] 
    if 'sliders' in params:
        hi_msg.sliders = [from_dict_slider_to_msg_slider(slider) for slider in params['sliders']]
    if 'text_inputs' in params:
        hi_msg.text_inputs = [from_dict_text_input_to_msg_text_input(text_input) for text_input in params['text_inputs']]
    if 'traj1' in params:
        hi_msg.traj1 = from_trajs_to_msg_trajs(params['traj1'])
    if 'traj2' in params:
        hi_msg.traj2 = from_trajs_to_msg_trajs(params['traj2'])
    if 'instruction_text' in params:
        hi_msg.instruction_text = params['instruction_text']
    if 'camera_topic' in params:
        hi_msg.camera_topic = params['camera_topic']
    if 'display_type' in params:
        hi_msg.display_type = params['display_type']
        if hi_msg.display_type == 3:
            pub_bokeh_msg = True
    if 'robot' in params:
        hi_msg.robot = params['robot']
    if 'robot_joint_topic' in params:
        hi_msg.robot_joint_topic = params['robot_joint_topic']
    return (hi_msg, pub_bokeh_msg)

def params_to_bokeh_request_msg(params):
    params = json.loads(params)
    bokeh_msg = bokehrequest()  
    bridge = CvBridge()  
    
    if 'bokeh_display_type' in params:
        bokeh_msg.display_type = params['bokeh_display_type']
    if 'bokeh_traj' in params:
        bokeh_msg.traj.time_since_skill_started = params['bokeh_traj']['time_since_skill_started']
        bokeh_msg.traj.num_joints = params['bokeh_traj']['num_joints']
        bokeh_msg.traj.cart_traj = params['bokeh_traj']['cart_traj']
        bokeh_msg.traj.joint_traj = params['bokeh_traj']['joint_traj']
    if 'bokeh_image' in params:
        bokeh_msg.image = bridge.cv2_to_imgmsg(np.array(params['bokeh_image'], dtype=np.uint8))

    return bokeh_msg