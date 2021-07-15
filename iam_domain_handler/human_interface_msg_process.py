
from domain_handler_msgs.msg import HumanInterfaceRequest, Button, Slider, TextInput, Bbox

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


def from_dict_bbox_to_msg_bbox(bbox):
    bb = Bbox()
    bb.name = bbox['name']
    return bb

def from_trajs_to_msg_trajs(trajs):
    return []

def params_to_human_interface_request_msg(params):
    hi_msg = HumanInterfaceRequest()   
    if 'buttons' in params:
        hi_msg.buttons = [from_dict_button_to_msg_button(button) for button in params['buttons']] 
    if 'sliders' in params:
        hi_msg.sliders = [from_dict_slider_to_msg_slider(slider) for slider in params['sliders']]
    if 'bboxes' in params:
        hi_msg.bboxes = [from_dict_bbox_to_msg_bbox(bbox) for bbox in params['bboxes']]
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
    if 'robot' in params:
        hi_msg.robot = params['robot']
    if 'robot_joint_topic' in params:
        hi_msg.robot_joint_topic = params['robot_joint_topic']
    
    return hi_msg

def human_interface_reply_handler(data):
    '''
    buttons, sliders, text_inputs, bboxes
    '''
    return_dict = dict()
    buttons = data.buttons
    for button in buttons:
        name = button.name
        return_dict[f'buttons:{name}'] = button.value
    bboxes = data.bboxes
    for bbox in bboxes:
        name = bbox.name
        return_dict[f'bboxes:{name}'] = bbox.value
    sliders = data.sliders
    for slider in sliders:
        name = slider.name
        return_dict[f'sliders:{name}'] = slider.value
    text_inputs = data.text_inputs
    for text_input in text_inputs:
        name = text_input.name
        return_dict[f'text_inputs:{name}'] = text_input.value
    return return_dict
