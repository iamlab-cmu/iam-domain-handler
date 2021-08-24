#!/bin/bash

trap 'kill %1; kill %2; kill %3; kill %4; kill %5; trap - EXIT; echo goodbye' EXIT

python examples/run_memory_server.py &
sleep 2
python examples/pen_in_jar/run_state_server.py &
sleep 2
python examples/run_human_memory_server.py &
sleep 2
python examples/run_human_server.py &
sleep 2
python scripts/run_action_registry_server.py &
sleep 2
python examples/run_robot_server.py 