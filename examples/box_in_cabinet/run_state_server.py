import numpy as np
import quaternion as qt
from franka_interface_msgs.msg import RobotState
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose

from iam_domain_handler.state_server import StateServer


def box_pose_handler(data):
    return {
        'frame:box:pose/position': [data.position.x, data.position.y, data.position.z],
        'frame:box:pose/quaternion': [data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z]
    }


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


def cabinet_handle_pose_handler(data):
    return {
        'frame:cabinet:handle:pose/position': [data.position.x, data.position.y, data.position.z],
        'frame:cabinet:handle:pose/quaternion': [data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z]
    }


def cabinet_open_handler(data):
    return {
        'frame:cabinet:open': [data.position.x]
    }


if __name__ == '__main__':
    sub_handlers = [
        ('/robot_state_publisher_node_1/robot_state', RobotState, robot_state_handler),
        ('/box_pose', Pose, box_pose_handler),
        ('/cabinet_open', Bool, cabinet_open_handler),
        ('/cabinet_handle_pose', Pose, cabinet_handle_pose_handler),
    ]
    state_server = StateServer(sub_handlers)
    