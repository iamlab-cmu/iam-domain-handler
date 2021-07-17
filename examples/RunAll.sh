#!/bin/bash

trap 'kill %1; kill %2; kill %3; trap - EXIT; echo goodbye' EXIT

python examples/run_state_server_human_interface.py &
sleep 2
python examples/run_human_server.py &
sleep 2
python scripts/run_action_registry_server.py &
sleep 2
python examples/run_robot_server.py 