import rospy
from time import sleep

from iam_domain_handler.domain_client import DomainClient



if __name__ == '__main__':
    rospy.init_node('run_domain_client', anonymous=True)
    domain = DomainClient()

    state = domain.state
    print(state)

    domain.run_query('Demo_dance','lalalalal')

    exit()
    ###import IPython; IPython.embed(); exit()
