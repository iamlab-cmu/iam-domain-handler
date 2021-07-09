import rospy
from geometry_msgs.msg import Pose


if __name__ == '__main__':
    rospy.init_node('mock_box_pose_publisher', anonymous=True)
    pub = rospy.Publisher('/box_pose', Pose, queue_size=10)
    rate = rospy.Rate(100)

    rospy.loginfo('Publishing...')
    while not rospy.is_shutdown():
        data = Pose()
        pub.publish(data)
        rate.sleep()
