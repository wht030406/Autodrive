#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
export AUTODRIVE_PYTHON=/home/wht/carla-venv/bin/python
export SUMO_HOME=/home/wht/sumo-1.22.0
export OMNETPP_ROOT=/home/wht/v2x/omnetpp-6.1
export VEINS_ROOT=/home/wht/v2x/veins-5.3.1
export CPATH=/home/wht/sumo-1.22.0/src/foreign
export PATH="$SUMO_HOME/bin:/home/wht/carla-venv/bin:$PATH"
export PYTHONPATH="$ROOT:$SUMO_HOME/tools:/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$ROOT"
exec "$AUTODRIVE_PYTHON" -m vanet.v2i_glosa.run_experiment "$@"
