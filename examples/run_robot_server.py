from iam_skills import CmdType, StreamTrajPolicy, BaseStreamTrajSkill, OpenGripperSkill, CloseGripperSkill

from iam_domain_handler.robot_server import RobotServer
# from iam_domain_handler.utils import EE_RigidTransform_from_state
import pickle
import numpy as np

class StayInPlaceEETrajSkill(BaseStreamTrajSkill):

    def make_policy(self, state, param):
        # stay in place for 2 seconds
        dt = 0.02
        # traj = [EE_RigidTransform_from_state(state)] * 100
        # return StreamTrajPolicy(traj, dt, CmdType.EE)
        traj = list(pickle.load(open('./examples/franka_traj.pkl', 'rb')))
        traj = [list(item) for item in traj]
        return StreamTrajPolicy(traj, dt, CmdType.JOINT)


if __name__ == '__main__':
    skills_dict = {
        'stay_in_place': StayInPlaceEETrajSkill(),
        'open_gripper': OpenGripperSkill(),
        'close_gripper': CloseGripperSkill()
    }
    server = RobotServer(skills_dict)
    