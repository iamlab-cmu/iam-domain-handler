from iam_skills import *

from iam_domain_handler.robot_server import RobotServer
from iam_domain_handler.utils import EE_RigidTransform_from_state
import pickle
import numpy as np

class StayInPlaceEETrajSkill(BaseStreamTrajSkill):

    def make_policy(self, state, param):
        # stay in place for 2 seconds
        dt = 0.02
        traj = [EE_RigidTransform_from_state(state)] * 100
        return StreamTrajPolicy(traj, dt, CmdType.EE)

if __name__ == '__main__':
    skills_dict = {
        'stay_in_place': StayInPlaceEETrajSkill(),
        'open_gripper': OpenGripperSkill(),
        'close_gripper': CloseGripperSkill(),
        'zero_force': ZeroForceSkill(),
        'record_trajectory': RecordSkill()
    }
    server = RobotServer(skills_dict)
    