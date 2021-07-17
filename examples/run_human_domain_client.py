import rospy
import json
from time import sleep

from iam_domain_handler.domain_client import DomainClient



if __name__ == '__main__':
    rospy.init_node('run_domain_client')
    domain = DomainClient()

    state = domain.state
    print(state)

    query_params = {
        'buttons' : [
            {
                'name' : 'grasp_button',
                'text' : 'Execute Grasp Skill',
            },
            {
                'name' : 'move_ee_to_pose_button',
                'text' : 'Execute Move EE to Pose Skill',
            },
        ]
    }
    domain.run_query('Demo_dance', json.dumps(query_params))
