#!/bin/bash

trap 'kill %1; kill %2; kill %3; trap - EXIT; echo goodbye' EXIT

# python examples/pen_in_jar/run_state_server.py &
# python examples/pen_in_jar/mock_publishers/run_pen_pose_publisher.py
python examples/run_state_server_human_interface.py &
sleep 2
python examples/run_human_server.py &
sleep 2
python scripts/run_action_registry_server.py &
sleep 2
# python examples/run_human_domain_client.py 