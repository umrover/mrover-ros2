# MRover portable shell env
MROVER_REPO="${${(%):-%x}:A:h:h}"

activate_mrover() {
  if ! command -v pixi >/dev/null 2>&1; then
    echo "pixi not found on PATH" >&2
    return 1
  fi
  cd "$MROVER_REPO" || return 1
  if [[ -z "$CONDA_PREFIX" ]]; then
    eval "$(pixi shell-hook)"
    export TERMINFO_DIRS="${CONDA_PREFIX}/share/terminfo:/usr/share/terminfo"
    # conda's ncurses terminfo does not include xterm-kitty; fall back to xterm-256color.
    [[ "$TERM" == "xterm-kitty" ]] && export TERM=xterm-256color
    # conda's compiler activation swaps $HOST (used by zsh %m / tab title) to the toolchain triplet; undo it.
    autoload -Uz add-zsh-hook
    add-zsh-hook -d precmd _conda_clang_precmd 2>/dev/null
    add-zsh-hook -d preexec _conda_clang_preexec 2>/dev/null
    export HOST="${CONDA_BACKUP_HOST:-$HOST}"
  fi
  [[ -f install/setup.zsh ]] && source install/setup.zsh
}

alias mrover='activate_mrover'
