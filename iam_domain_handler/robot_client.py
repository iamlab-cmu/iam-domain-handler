from threading import Thread
import rospy

from domain_handler_msgs.srv import RunSkill

from .skill_registry_client import SkillRegistryClient


class RobotClient:

    def __init__(self):
        self._run_skill_srv_name = 'run_skill'
        
        self._skill_registry_client = SkillRegistryClient()

        self._run_skill_srv_proxy = rospy.ServiceProxy(self._run_skill_srv_name, RunSkill)

        self._runnning_skill_id = None
        self._req_run_skill_th = None

    def run_skill(self, skill_name, skill_param):

        assert self._req_run_skill_th is None, 'A skill is currently running!'

        skill_id = self._skill_registry_client.register_skill(skill_name, skill_param)

        th = Thread(target=req_run_skill)
        th.start()

        return skill_id

    def get_skill_status(self, skill_id):
        rospy.wait_for_service(self._get_skill_status_srv_name)
        return self._get_skill_status_srv_proxy(skill_id).skill_status
