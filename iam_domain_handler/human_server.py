import rospy

from domain_handler_msgs.srv import RunSkill
from domain_handler_msgs.msg import HumanInterfaceRequest

from .state_client import StateClient
from .skill_registry_client import SkillRegistryClient
from .human_interface_msg_process import params_to_human_interface_request_msg


def query_subdict_to_expected_state_changes(k, d):
    expected_state_changes = []
    if 'value' in d:
        assert 'name' in d
        name = d['name']
        expected_state_changes.append(f'{k}:{name}')
    return expected_state_changes


def query_params_to_expected_state_changes(params):
    expected_state_changes = []
    for k, v in params.items():
        if type(v) is not dict or type(v) is not list:
            continue

        if type(v) is list:
            for vi in v:
                vi_changes = query_subdict_to_expected_state_changes(vi)
                expected_state_changes += vi_changes
        else:
            expected_state_changes += query_subdict_to_expected_state_changes(
                v)


class HumanServer:

    def __init__(self):
        self._state_client = StateClient()
        self._query_registry_client = SkillRegistryClient()

        self._human_interface_pub = rospy.Publisher(
            'mock_human_interface_publisher', HumanInterfaceRequest, queue_size=1000)
        self._run_skill_srv = rospy.Service(
            'run_query', RunSkill, self._run_query_srv_handler)

        rospy.loginfo('Running Human Server...')
        rospy.spin()

    def _run_query_srv_handler(self, req):
        query_info = self._query_registry_client.get_skill_info(req.skill_id)
        self._query_registry_client.set_skill_status(req.skill_id, 'running')
        hi_msg = params_to_human_interface_request_msg(query_info.skill_param)

        expected_state_changes = query_params_to_expected_state_changes(
            query_info.skill_param)
        # !!!! Comparing end_state with init_state to make sure changes have happened
        init_state = self._state_client.get_state()
        self._human_interface_pub.publish(hi_msg)
        end_state = self._state_client.get_state()

        skill_status = 'success'
        for k in expected_state_changes:
            # Assume that everytime the human interface publishes, triggers change in a different state
            if k not in end_state:
                skill_status = 'failure'

        self._query_registry_client.set_skill_status(
            req.skill_id, skill_status)
        return skill_status
