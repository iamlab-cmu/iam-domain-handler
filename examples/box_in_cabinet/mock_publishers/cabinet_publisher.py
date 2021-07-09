import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose


if __name__ == '__main__':
    rospy.init_node('mock_cabinet_publisher', anonymous=True)
    pub_open = rospy.Publisher('/cabinet_open', Bool, queue_size=10)
    pub_handle = rospy.Publisher('/cabinet_handle_pose', Pose, queue_size=10)
    rate = rospy.Rate(100)

    rospy.loginfo('Publishing...')
    while not rospy.is_shutdown():
        pub_open.publish(False)
        pub_handle.publish(Pose())
        rate.sleep()
