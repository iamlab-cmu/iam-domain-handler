import rospy
from std_msgs.msg import Int32

import json
from iam_domain_handler.human_interface_msg_process import params_to_human_interface_request_msg
from domain_handler_msgs.msg import HumanInterfaceRequest, HumanInterfaceConfirmation, HumanInterfaceReply


query_params = {
        'buttons' : [
            {
                'name' : 'grasp_button',
                'text' : 'Execute Grasp Skill',
                'value' : False,
            },
            {
                'name' : 'move_ee_to_pose_button',
                'text' : 'Execute Move EE to Pose Skill',
                'value' : False,
            },
            {
                'name' : 'stop_button',
                'text' : 'Click this to terminate behavior tree',
                'value' : False,
            },
        ]
    }

# def params_to_human_interface_reply_mock_msg(params):
#     params = json.loads(params)
#     hi_msg = HumanInterfaceReply()   
#     if 'buttons' in params:
#         hi_msg.buttons = [from_dict_button_to_msg_button(button) for button in params['buttons']] 
    
#     return hi_msg

# Converts the above dictionary to a ros message
# hi_msg = params_to_human_interface_reply_mock_msg(json.dumps(query_params))
mock_human_interface_pub = rospy.Publisher('/human_interface_confirmation', HumanInterfaceConfirmation, queue_size=1)

if __name__ == '__main__':
    rospy.init_node('human_server_msg_publisher', anonymous=True)
    rate = rospy.Rate(10)

    rospy.loginfo('Publishing HumanInterfaceRequest...')
    int_msg = HumanInterfaceConfirmation()
    int_msg.succeed = True
    # while not rospy.is_shutdown():
    mock_human_interface_pub.publish(int_msg)
    # rate.sleep()