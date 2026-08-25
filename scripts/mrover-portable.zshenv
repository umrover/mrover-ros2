# MRover portable shell env
MROVER_REPO="${${(%):-%x}:A:h:h}"

[[ -d "$HOME/.pixi/bin" ]] && export PATH="$HOME/.pixi/bin:$PATH"

activate_mrover() {
  if ! command -v pixi >/dev/null 2>&1; then
    echo "pixi not found on PATH" >&2
    return 1
  fi
  cd "$MROVER_REPO" || return 1
  if [[ -z "$CONDA_PREFIX" ]]; then
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
  [[ -f install/setup.zsh ]] && source install/setup.zsh
  
  # ROS's local_setup.zsh sets AMENT_SHELL=zsh without unsetting it
  unset AMENT_SHELL

  # Move DYLD_LIBRARY_PATH to fallback so it does not override system libs
  if [[ -n "$DYLD_LIBRARY_PATH" ]]; then
    export DYLD_FALLBACK_LIBRARY_PATH="${DYLD_LIBRARY_PATH}${DYLD_FALLBACK_LIBRARY_PATH:+:$DYLD_FALLBACK_LIBRARY_PATH}"
    unset DYLD_LIBRARY_PATH
  fi
}

# named function because it takes priority over autocd
mrover() { activate_mrover "$@"; }
