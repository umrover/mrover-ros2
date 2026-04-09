# MRover ROS
readonly MROVER_ROS2_WS_PATH="$HOME/ros2_ws"

source /opt/ros/humble/setup.zsh

export ROS_DOMAIN_ID=5
export COLCON_TRACE=0

remove_ros2_ws_from_path(){
    export ${1}="$(echo ${${1}} | tr ':' '\n' | grep -v "ros2_ws" | paste -s -d ':')"
}

source_mrover_overlay(){
    source ~/ros2_ws/src/mrover/venv/bin/activate

    build_profiles=("RelWithDebInfo" "Release" "Debug")

    target_file=""

    for profile in "${build_profiles[@]}"; do
        file="${MROVER_ROS2_WS_PATH}/install/${profile}/setup.zsh"

        if [ -f "${file}" ]; then
            if [[ -z "${target_file}" ]]; then
                target_file="${file}"
            elif [[ "${file}" -nt "${target_file}" ]]; then
                target_file="${file}"
            fi
        fi
    done

    # clean up current ROS environment
    remove_ros2_ws_from_path LD_LIBRARY_PATH
    remove_ros2_ws_from_path AMENT_PREFIX_PATH
    remove_ros2_ws_from_path PYTHONPATH
    remove_ros2_ws_from_path COLCON_PREFIX_PATH
    remove_ros2_ws_from_path CMAKE_PREFIX_PATH

    if [ -f "${target_file}" ]; then
        source "${target_file}" >> /dev/null
    fi
}

# cuda
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

# bun
export BUN_INSTALL="$HOME/.bun"
export PATH="$BUN_INSTALL/bin:$PATH"
