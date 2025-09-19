#!/usr/bin/zsh

# config and propagate failure
set -euxo pipefail

# for colorful echos
RED="\e[1;31m"
GREEN="\e[1;32m"
BLUE="\e[1;34m"
NC="\e[0m"

echo -e "${BLUE}======= CONFIGURING DOCKER ENVIRONMENT =======${NC}"

# configuration vars
ROS2_WORKSPACE="/home/mrover/ros2_ws"
PYPROJECT="${ROS2_WORKSPACE}/src/mrover/pyproject.toml"
VENV_DIR="${ROS2_WORKSPACE}/src/mrover/venv"
PACKAGE_PATH="${ROS2_WORKSPACE}/src/mrover"
FRONTEND_PATH="${PACKAGE_PATH}/teleoperation/basestation_gui/frontend"
DJANGO_PATH="${PACKAGE_PATH}/teleoperation/basestation_gui/manage.py"

# run a step and display the exit status
run_step() {
    desc="$1"
    shift
    echo -e "${BLUE}${desc}...${NC}"
    if "$@"; then
        echo -e "${GREEN}✓ Success: ${desc}${NC}"
    else
        echo -e "${RED}✗ Failed: ${desc}${NC}"
        exit 1
    fi
}

# setup and source the python virtual environment
if [ -f "${PYPROJECT}" ]; then
    if [ ! -d "${VENV_DIR}" ]; then
        run_step "Configure venv" /usr/bin/python3 -m venv "$VENV_DIR"
        # shellcheck disable=SC1091
        source "$VENV_DIR/bin/activate"
        run_step "Upgrade pip" pip install -U pip
        run_step "Install deps" pip install "${PACKAGE_PATH}[dev]"
    else
        echo -e "${BLUE}Activating venv...${NC}"
        # shellcheck disable=SC1091
        source "$VENV_DIR/bin/activate"
    fi
else
    echo -e "${RED}✗ Failed: pyproject.toml not found${NC}"
    exit 1
fi

# build manif and manifpy on entry (depends on submodule being mounted)
run_step "Build manif" zsh -c "${PACKAGE_PATH}/scripts/build_manif.sh"
run_step "Build manifpy" zsh -c "${PACKAGE_PATH}/scripts/build_manifpy.sh"

# setup the bun deps
if [ -f "${FRONTEND_PATH}/package.json" ]; then
    run_step "Installing frontend dependencies with Bun" /home/mrover/.bun/bin/bun install --cwd "${FRONTEND_PATH}"
fi

# run the django migrations
if [ -f "${DJANGO_PATH}" ]; then
    run_step "Running Django makemigrations" python "${DJANGO_PATH}" makemigrations
    run_step "Running Django migrate" python "${DJANGO_PATH}" migrate
fi

echo -e "${GREEN}======= SUCCESS =======${NC}"

# pass through to provided shell command
exec "$@"
