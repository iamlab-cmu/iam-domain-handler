from threading import Thread
import rospy

from domain_handler_msgs.srv import RunSkill, GetSkillTraj
from web_interface_msgs.msg import Confirmation
from .action_registry_client import ActionRegistryClient

class RobotClient:

    def __init__(self):
        rospy.init_node('robot_client')
        self._run_skill_srv_name = 'run_skill'
        self._get_skill_traj_srv_name = 'get_skill_traj'
        
        self._action_registry_client = ActionRegistryClient()

        self._run_skill_srv_proxy = rospy.ServiceProxy(self._run_skill_srv_name, RunSkill)
        self._get_skill_traj_srv_proxy = rospy.ServiceProxy(self._get_skill_traj_srv_name, GetSkillTraj)

        self._state_trajectory_done_reset_pub = rospy.Publisher('/state_trajectory_done_reset', Confirmation, queue_size=1000)

        self._runnning_skill_id = None

    def get_skill_traj(self, skill_name, skill_param):
        reset_msg = Confirmation()
        reset_msg.succeed = False
        self._state_trajectory_done_reset_pub.publish(reset_msg)
        srv_response = self._get_skill_traj_srv_proxy(skill_name, skill_param)
        return srv_response.skill_status
    
    def run_skill(self, skill_name, skill_param):
        if self._runnning_skill_id is not None:
            assert self.get_skill_status(self._runnning_skill_id) in ('success', 'failure'), 'A skill is currently running!'

        self._runnning_skill_id = self._action_registry_client.register_action(skill_name, skill_param)

        # Make this request async so we don't wait until skill finishes
        def req_run_skill():
            self._run_skill_srv_proxy(self._runnning_skill_id)
        th = Thread(target=req_run_skill)
        th.start()

        return self._runnning_skill_id

    def get_skill_status(self, skill_id):
        return self._action_registry_client.get_action_status(skill_id)
