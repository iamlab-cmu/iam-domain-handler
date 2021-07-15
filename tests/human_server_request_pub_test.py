import rospy
from iam_domain_handler.human_interface_msg_process import params_to_human_interface_request_msg
from domain_handler_msgs.msg import HumanInterfaceRequest

mock_human_interface_pub = rospy.Publisher(
            'mock_human_interface_publisher', HumanInterfaceRequest, queue_size=1000)
query_params = {
        'buttons' : [
            {
                'name' : 'grasp_button',
                'text' : 'Execute Grasp Skill',
            },
            {
                'name' : 'move_ee_to_pose_button',
                'text' : 'Execute Move EE to Pose Skill',
            },
            {
                'name' : 'stop_button',
                'text' : 'Click this to terminate behavior tree',
            },
        ]
    }

# Converts the above dictionary to a ros message
hi_msg = params_to_human_interface_request_msg(query_params)


if __name__ == '__main__':
    rospy.init_node('human_server_msg_publisher', anonymous=True)
    rate = rospy.Rate(100)

    rospy.loginfo('Publishing HumanInterfaceRequest...')
    while not rospy.is_shutdown():
        mock_human_interface_pub.publish(hi_msg)
        rate.sleep()