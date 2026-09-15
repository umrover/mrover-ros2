# MRover portable shell env
MROVER_REPO="${${(%):-%x}:A:h:h}"
MROVER_ROS2_WS_PATH="${MROVER_REPO:h:h}"

[[ -d "$HOME/.pixi/bin" ]] && export PATH="$HOME/.pixi/bin:$PATH"

remove_ros2_ws_install_from_path(){
  export ${1}="$(echo ${(P)1} | tr ':' '\n' | grep -v "^${MROVER_ROS2_WS_PATH}/install" | paste -s -d ':' -)"
}

activate_mrover() {
  if ! command -v pixi >/dev/null 2>&1; then
    echo "pixi not found on PATH" >&2
    return 1
  fi
  cd "$MROVER_REPO" || return 1
  # conda base environments also set CONDA_PREFIX
  if [[ "$PIXI_PROJECT_ROOT" != "$MROVER_REPO" ]]; then
    eval "$(pixi shell-hook --as-is)"
    export TERMINFO_DIRS="${CONDA_PREFIX}/share/terminfo:/usr/share/terminfo"
    # conda's ncurses terminfo does not include xterm-kitty; fall back to xterm-256color.
    [[ "$TERM" == "xterm-kitty" ]] && export TERM=xterm-256color
    # undo conda's compiler activation swapping $HOST
    autoload -Uz add-zsh-hook
    add-zsh-hook -d precmd _conda_clang_precmd 2>/dev/null
    add-zsh-hook -d preexec _conda_clang_preexec 2>/dev/null
    export HOST="${CONDA_BACKUP_HOST:-$HOST}"
  fi

  build_profiles=("RelWithDebInfo" "Release" "Debug")
  unset MROVER_BUILD_PROFILE

  target_file=""

  for profile in "${build_profiles[@]}"; do
    file="${MROVER_ROS2_WS_PATH}/install/${profile}/setup.zsh"

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
  remove_ros2_ws_install_from_path LD_LIBRARY_PATH
  remove_ros2_ws_install_from_path AMENT_PREFIX_PATH
  remove_ros2_ws_install_from_path PYTHONPATH
  remove_ros2_ws_install_from_path COLCON_PREFIX_PATH
  remove_ros2_ws_install_from_path CMAKE_PREFIX_PATH

  if [ -f "${target_file}" ]; then
    source "${target_file}" >> /dev/null
  fi

  # ROS's local_setup.zsh sets AMENT_SHELL=zsh without unsetting it
  unset AMENT_SHELL

  # Move DYLD_LIBRARY_PATH to fallback so it does not override system libs
  if [[ -n "$DYLD_LIBRARY_PATH" ]]; then
    export DYLD_FALLBACK_LIBRARY_PATH="${DYLD_LIBRARY_PATH}${DYLD_FALLBACK_LIBRARY_PATH:+:$DYLD_FALLBACK_LIBRARY_PATH}"
    unset DYLD_LIBRARY_PATH
  fi
}

source_mrover_overlay() { activate_mrover "$@"; }

# named function because it takes priority over autocd
mrover() { activate_mrover "$@"; }

build_mrover(){
  mrover && ./build.sh "${1}" && mrover
}

clean_mrover(){
  cd "${MROVER_REPO}" && ./clean.sh && mrover
}
