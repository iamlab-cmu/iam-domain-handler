#!/bin/bash

trap 'kill %1; kill %2; kill %3; trap - EXIT; echo goodbye' EXIT

python examples/pen_in_jar/run_state_server.py &
python examples/run_human_server.py &
python scripts/run_action_registry_server.py &
python examples/run_human_domain_client.py

