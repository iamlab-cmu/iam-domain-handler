import rospy 
from domain_handler_msgs.msg import HumanInterfaceRequest
from iam_domain_handler.human_interface_msg_process import human_interface_reply_handler

def sub_cb(data, handler):
    for k, v in handler(data).items():
        print(k,v)

mock_human_interface_sub = rospy.Subscriber('mock_human_interface_publisher', HumanInterfaceRequest, sub_cb, human_interface_reply_handler)

if __name__ == '__main__':
    rospy.init_node('human_interface_subscriber', anonymous=True)
    rospy.loginfo('Running State Server...')
    rospy.spin()