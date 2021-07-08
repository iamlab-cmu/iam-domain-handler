from shortuuid import uuid
import rospy

from domain_handler_msgs.srv import RegisterSkill, SetSkillStatus, GetSkillInfo, DoesSkillExist

from .state_client import StateClient


class SkillRegistryServer:

    def __init__(self, skills_dict):
        self._skills_dict = skills_dict

        self._state_client = StateClient()

        self._does_skill_exist_srv = rospy.Service('does_skill_exist', DoesSkillExist, self._does_skill_exist_srv_handler)
        self._register_skill_srv = rospy.Service('register_skill', RegisterSkill, self._register_skill_srv_handler)
        self._set_skill_status_srv = rospy.Service('set_skill_status', SetSkillStatus, self._set_skill_status_srv_handler)
        self._get_skill_status_srv = rospy.Service('get_skill_info', GetSkillInfo, self._get_skill_status_srv_handler)

        self._skill_registry = {}

    def _does_skill_exist_srv_handler(self, req):
        return req.skill_id in self._skill_registry

    def _register_skill_srv_handler(self, req):
        skill_id = f'{req.skill_name}_{uuid()}'

        self._skill_registry[skill_id] = {
            'skill_name': req.skill_name,
            'skill_param': req.skill_param,
            'skill_status': 'registered'
        }

        return skill_id

    def _set_skill_status_srv_handler(self, req):
        self._skill_registry[req.skill_id]['skill_status'] = req.skill_status

    def _get_skill_info_srv_handler(self, req):
        return \
            self._skill_registry[req.skill_id]['skill_name'], \
            self._skill_registry[req.skill_id]['skill_param'], \
            self._skill_registry[req.skill_id]['skill_status']
