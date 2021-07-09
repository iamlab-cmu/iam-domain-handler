import rospy
from franka_interface_msgs.msg import RobotState


if __name__ == '__main__':
    rospy.init_node('mock_robot_publisher', anonymous=True)
    pub = rospy.Publisher('/robot_state_publisher_node_1/robot_state', RobotState, queue_size=10)
    rate = rospy.Rate(100)

    rospy.loginfo('Publishing...')
    while not rospy.is_shutdown():
        data = RobotState()
        pub.publish(data)
        rate.sleep()
