import numpy as np
import quaternion as qt
from franka_interface_msgs.msg import RobotState
from geometry_msgs.msg import Pose

from iam_domain_handler.state_server import StateServer


def robot_state_handler(data):
    np_ee_pose = np.array(data.O_T_EE).reshape(4, 4).T
    ee_pos = np_ee_pose[:3, 3]
    ee_quat = qt.as_float_array(qt.from_rotation_matrix(np_ee_pose[:3, :3]))
    return {
        'frame:franka:ee:pose/position': ee_pos,
        'frame:franka:ee:pose/quaternion': ee_quat,
        'frame:franka:joints/torque': data.tau_J,
        'frame:franka:joints/position': data.q,
        'frame:franka:joints/position_desired': data.q_d,
        'frame:franka:joints/velocity': data.dq,
        'frame:franka:gripper/width': [data.gripper_width],
        'frame:franka:gripper/is_grasped': [data.gripper_is_grasped],
    }

def aruco_handler(data):
    ee_pos = np.array([data.position.x, data.position.y, data.position.z])
    ee_quat = np.array([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
    return {
        'frame:block:ee:pose/position': ee_pos,
        'frame:block:ee:pose/quaternion': ee_quat
    }


if __name__ == '__main__':[ 0.41569083, -0.25251649,  0.24636923]
    sub_handlers = [
        ('/robot_state_publisher_node_1/robot_state', RobotState, robot_state_handler),
        ('/aruco_simple/pose', Pose, aruco_handler)
    ]
    state_server = StateServer(sub_handlers)
    