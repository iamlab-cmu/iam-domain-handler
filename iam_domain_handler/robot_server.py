import rospy
from frankapy import FrankaArm

from domain_handler_msgs.srv import RunSkill

from .state_client import StateClient
from .skill_registry_client import SkillRegistryClient


class RobotServer:

    def __init__(self, skills_dict):
        self._skills_dict = skills_dict

        # self._fa = FrankaArm()
        self._state_client = StateClient()
        self._skill_registry_client = SkillRegistryClient()

        self._run_skill_srv = rospy.Service('run_skill', RunSkill, self._run_skill_srv_handler)

    def _run_skill_srv_handler(self, req):
        skill_info = self._skill_registry_client.get_skill_info(req.skill_id)
        self._skill_registry_client.set_skill_status(req.skill_id, 'running')

        skill = self._skills_dict[skill_info.skill_name]
        init_state = self._state_client.get_state()
        policy = skill.make_policy(init_state, skill_info.skill_param)

        t_step = 0
        while True:
            # TODO: run policy
            state = self._state_client.get_state()
            t_step += 1
            if skill.termination_condition_satisfied(state, skill_info.skill_param, policy, t_step) > 0.5:
                break

        end_state = self._state_client.get_state()

        if skill.skill_execution_successful(init_state, end_state, skill_info.skill_param, policy, t_step) > 0.5:
            skill_status = 'success'
        else:
            skill_status = 'failure'
        self._skill_status_client.set_skill_status(req.skill_id, skill_status)
