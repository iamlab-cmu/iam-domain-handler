from iam_domain_handler.state_client import StateClient
from time import sleep



if __name__ == '__main__':
    state_client = StateClient()

    while True:
        state = state_client.get_state()
        print(state)
        sleep(1)
