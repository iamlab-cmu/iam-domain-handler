import rospy 
from std_msgs.msg import Int32

from domain_handler_msgs.msg import HumanInterfaceRequest, Confirmation

def human_interface_reply_handler(data):
    '''
    buttons, sliders, text_inputs, bboxes
    '''
    return_dict = dict()
    buttons = data.buttons
    button_clicked = None
    for button_idx, button in enumerate(buttons):
        if button.value:
            button_clicked = button.name
    return_dict['clicked_button_name'] = [button_idx]
    # bboxes = data.bboxes
    # bboxes_value = []
    # for bbox in bboxes:
    #     name = bbox.name
    #     bboxes_value.append(bbox.value)
    # return_dict['bbox_value'] = bboxes_value
    # sliders = data.sliders
    # for slider in sliders:
    #     name = slider.name
    # return_dict['slider_value'] = [slider.value]
    # text_inputs = data.text_inputs
    # for text_input in text_inputs:
    #     name = text_input.name
    # return_dict['text_inputs_value'] = [text_input.value]
    return return_dict

def human_server_reset_handler(data):
    print("getting query done")
    return {
        'query_done' : [data.succeed],
    }

def sub_cb(data, handler):
    for k, v in handler(data).items():
        print(k,v)

mock_human_interface_sub = rospy.Subscriber('/mock_human_interface_publisher', Confirmation, sub_cb, human_server_reset_handler)

if __name__ == '__main__':
    rospy.init_node('human_interface_subscriber', anonymous=True)
    rospy.loginfo('Running State Server...')
    rospy.spin()