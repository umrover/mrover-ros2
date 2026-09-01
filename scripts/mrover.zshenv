# MRover ROS
export MROVER_REPO="${MROVER_REPO:-$HOME/mrover-ros2}"
readonly MROVER_REPO

source /opt/ros/jazzy/setup.zsh

export ROS_DOMAIN_ID=5
export COLCON_TRACE=0

remove_mrover_from_path(){
    export ${1}="$(echo ${(P)1} | tr ':' '\n' | grep -vF "${MROVER_REPO}" | paste -s -d ':')"
}

source_mrover_overlay(){
    source "${MROVER_REPO}/venv/bin/activate"

    build_profiles=("RelWithDebInfo" "Release" "Debug")
    unset MROVER_BUILD_PROFILE

    target_file=""

    for profile in "${build_profiles[@]}"; do
        file="${MROVER_REPO}/install/${profile}/setup.zsh"

        if [ -f "${file}" ]; then
            if [[ -z "${target_file}" ]]; then
                export MROVER_BUILD_PROFILE="${profile}"
                target_file="${file}"
            elif [[ "${file}" -nt "${target_file}" ]]; then
                export MROVER_BUILD_PROFILE="${profile}"
                target_file="${file}"
            fi
        fi
    done

    # clean up current ROS environment
    remove_mrover_from_path LD_LIBRARY_PATH
    remove_mrover_from_path AMENT_PREFIX_PATH
    remove_mrover_from_path PYTHONPATH
    remove_mrover_from_path COLCON_PREFIX_PATH
    remove_mrover_from_path CMAKE_PREFIX_PATH

    if [ -f "${target_file}" ]; then
        source "${target_file}" >> /dev/null
    fi

    command -v register-python-argcomplete3 &>/dev/null && command -v colcon &>/dev/null && eval "$(register-python-argcomplete3 colcon)"
}

# cuda
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

# bun
export BUN_INSTALL="$HOME/.bun"
export PATH="$BUN_INSTALL/bin:$PATH"
[ -s "$BUN_INSTALL/_bun" ] && source "$BUN_INSTALL/_bun"

alias mrover="cd ${MROVER_REPO} && source_mrover_overlay"

build_mrover(){
    cd "${MROVER_REPO}" && ./build.sh "${1}" && mrover
}

alias clean_mrover="cd ${MROVER_REPO} && ./clean.sh && mrover"

if command -v register-python-argcomplete3 &>/dev/null; then
    eval "$(register-python-argcomplete3 ros2)"
    command -v colcon &>/dev/null && eval "$(register-python-argcomplete3 colcon)"
fi
