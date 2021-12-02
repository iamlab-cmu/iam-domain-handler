from iam_skills import *

from iam_domain_handler.robot_server import RobotServer
from iam_domain_handler.utils import EE_RigidTransform_from_state
import pickle
import numpy as np

class StreamJointTrajSkill(BaseStreamTrajSkill):

    def make_policy(self, state, param):
        # stay in place for 2 seconds
        param_dict = json.loads(param)
        dt = param_dict['dt']
        traj = param_dict['traj']
        return StreamTrajPolicy(traj, dt, CmdType.JOINT)

if __name__ == '__main__':
    skills_dict = {
        'open_gripper': OpenGripperSkill(),
        'close_gripper': CloseGripperSkill(),
        'zero_force': ZeroForceSkill(),
        'record_trajectory': RecordSkill(),
        'stream_joint_traj': StreamJointTrajSkill(),
        'one_step_joint': GoToJointsSkill(),
        'one_step_pose': GoToPoseSkill(),
        'one_step_joint_dmp': JointDmpSkill(),
        'one_step_quat_pose_dmp': QuatPoseDmpSkill()
    }
    server = RobotServer(skills_dict)
    