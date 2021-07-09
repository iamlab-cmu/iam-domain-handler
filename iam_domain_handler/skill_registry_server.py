import rospy
from shortuuid import uuid

from domain_handler_msgs.srv import RegisterSkill, SetSkillStatus, GetSkillInfo, DoesSkillExist

from .state_client import StateClient


class SkillRegistryServer:

    def __init__(self):
        rospy.init_node('skill_registry_server', anonymous=True)

        self._skill_registry = {}

        self._state_client = StateClient()

        self._does_skill_exist_srv = rospy.Service('does_skill_exist', DoesSkillExist, self._does_skill_exist_srv_handler)
        self._register_skill_srv = rospy.Service('register_skill', RegisterSkill, self._register_skill_srv_handler)
        self._set_skill_status_srv = rospy.Service('set_skill_status', SetSkillStatus, self._set_skill_status_srv_handler)
        self._get_skill_status_srv = rospy.Service('get_skill_info', GetSkillInfo, self._get_skill_info_srv_handler)

        rospy.loginfo('Running Skill Registry Server...')
        rospy.spin()

    def _does_skill_exist_srv_handler(self, req):
        skill_exists = req.skill_id in self._skill_registry
        rospy.loginfo(f'Got req: Does Skill Exist? skill_id={req.skill_id} | ret: {skill_exists}')
        return skill_exists

    def _register_skill_srv_handler(self, req):
        skill_id = f'{req.skill_name}_{uuid()}'
        rospy.loginfo(f'Got req: Registering skill {req.skill_name} with param {req.skill_param} | skill_id={skill_id}')

        self._skill_registry[skill_id] = {
            'skill_name': req.skill_name,
            'skill_param': req.skill_param,
            'skill_status': 'registered'
        }

        return skill_id

    def _set_skill_status_srv_handler(self, req):
        prev_skill_status = self._skill_registry[req.skill_id]['skill_status']
        skill_name = self._skill_registry[req.skill_id]['skill_name']
        skill_param = self._skill_registry[req.skill_id]['skill_param']
        rospy.loginfo(f'Got req: Setting skill w/ id {req.skill_id}, name {skill_name}, param {skill_param} from {prev_skill_status} to {req.skill_status}')

        self._skill_registry[req.skill_id]['skill_status'] = req.skill_status

    def _get_skill_info_srv_handler(self, req):
        skill_name = self._skill_registry[req.skill_id]['skill_name']
        skill_param = self._skill_registry[req.skill_id]['skill_param']
        skill_status = self._skill_registry[req.skill_id]['skill_status']
        rospy.loginfo(f'Got req: Returning skill info w/ id {req.skill_id}, name {skill_name}, param {skill_param}, status {skill_status}')
        
        return req.skill_id, skill_name, skill_param, skill_status
