function get_build_profile() {
  if [[ -v MROVER_BUILD_PROFILE ]] && [[ "$PWD" == "${MROVER_ROS2_WS_PATH}/src/mrover" ]]; then
    echo "%F{green}[${MROVER_BUILD_PROFILE}]%f "
  fi 
}

PROMPT="%(?:%{$fg_bold[green]%}➜ :%{$fg_bold[red]%}➜ ) %{$fg[yellow]%}%m %{$fg[gray]%}at %{$fg[yellow]%}%d%{$reset_color%}"
PROMPT+=' $(git_prompt_info)'
PROMPT+='$(get_build_profile)'

ZSH_THEME_GIT_PROMPT_PREFIX="%{$fg_bold[gray]%}on %{$fg[blue]%}git:(%{$fg[blue]%}"
ZSH_THEME_GIT_PROMPT_SUFFIX="%{$reset_color%} "
ZSH_THEME_GIT_PROMPT_DIRTY="%{$fg[blue]%}) %{$fg[yellow]%}*"
ZSH_THEME_GIT_PROMPT_CLEAN="%{$fg[blue]%})"
