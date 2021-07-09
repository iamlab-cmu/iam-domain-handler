from time import sleep

from iam_domain_handler.domain_client import DomainClient


if __name__ == '__main__':
    domain_client = DomainClient()

    state = domain_client.state

    print(state)

    skill_id = domain_client.run_skill('close_gripper')
    print(skill_id)

    
    while True:
        skill_status = domain_client.get_skill_status(skill_id)
        print(skill_status)

        if skill_status == 'success':
            break
        sleep(0.5)

    import IPython; IPython.embed(); exit()
