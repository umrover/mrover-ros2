# MRover portable (Tier 2) shell env: sourced from ~/.zshenv, defines activate_mrover.
MROVER_REPO="${${(%):-%x}:A:h:h}"

activate_mrover() {
  if ! command -v pixi >/dev/null 2>&1; then
    echo "pixi not found on PATH" >&2
    return 1
  fi
  cd "$MROVER_REPO" || return 1
  eval "$(pixi shell-hook)"
  [[ -f install/setup.zsh ]] && source install/setup.zsh
}

alias mrover='activate_mrover'
