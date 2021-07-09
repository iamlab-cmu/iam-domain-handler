import rospy
from time import sleep

from iam_domain_handler.domain_client import DomainClient


def run_and_wait_for_skill(domain, skill_name):
    rospy.loginfo(f'Running {skill_name}')
    
    skill_id = domain.run_skill(skill_name)
    print(skill_id)
    
    while True:
        skill_status = domain.get_skill_status(skill_id)
        print(skill_status)

        if skill_status == 'success':
            break
        sleep(0.5)

    return skill_status

if __name__ == '__main__':
    rospy.init_node('run_domain_client', anonymous=True)
    domain = DomainClient()

    state = domain.state
    print(state)

    run_and_wait_for_skill(domain, 'close_gripper')
    run_and_wait_for_skill(domain, 'open_gripper')
    run_and_wait_for_skill(domain, 'stay_in_place')

    import IPython; IPython.embed(); exit()
