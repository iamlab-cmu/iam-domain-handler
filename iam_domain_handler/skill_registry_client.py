import rospy

from domain_handler_msgs.srv import RegisterSkill, SetSkillStatus, GetSkillStatus, DoesSkillExist


class SkillRegistryClient:

    def __init__(self):
        self._does_skill_exist_srv_name = 'does_skill_exist'
        self._register_skill_srv_name = 'register_skill'
        self._set_skill_status_srv_name = 'set_skill_status'
        self._get_skill_info_srv_name = 'get_skill_status'
        
        self._does_skill_exist_srv_proxy = rospy.ServiceProxy(self._does_skill_exist_srv_name, DoesSkillExist)
        self._register_skill_srv_proxy = rospy.ServiceProxy(self._register_skill_srv_name, RegisterSkill)
        self._set_skill_status_srv_proxy = rospy.ServiceProxy(self._set_skill_status_srv_name, SetSkillStatus)
        self._get_skill_info_srv_proxy = rospy.ServiceProxy(self._get_skill_info_srv_name, GetSkillStatus)

    def does_skill_exist(self, skill_id):
        rospy.wait_for_service(self._does_skill_exist_srv_proxy)
        return self._does_skill_exist_srv_proxy(skill_id).skill_exists

    def register_skill(self, skill_name, skill_param):
        rospy.wait_for_service(self._register_skill_srv_name)
        return self._register_skill_srv_proxy(skill_name, skill_param).skill_id

    def set_skill_status(self, skill_id, skill_status):
        rospy.wait_for_service(self._set_skill_status_srv_name)
        self._set_skill_status_srv_proxy(skill_id, skill_status)

    def get_skill_info(self, skill_id):
        rospy.wait_for_service(self._get_skill_info_srv_name)
        return self._get_skill_info_srv_proxy(skill_id)
