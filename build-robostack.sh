#!/usr/bin/env bash

set -euxo pipefail

# Robostack build wrapper
# Activates conda environment before building

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MINIFORGE_PATH="${HOME}/miniforge3"
CONDA_ENV="ros_env"

# Source conda
source "${MINIFORGE_PATH}/etc/profile.d/conda.sh"
conda activate "${CONDA_ENV}"

# Forward to the main build script
exec "${SCRIPT_DIR}/build.sh" "$@"
