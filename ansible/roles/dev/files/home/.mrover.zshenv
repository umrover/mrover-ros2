[ -f /opt/ros/jazzy/setup.zsh ] && source /opt/ros/jazzy/setup.zsh

export ROS_DOMAIN_ID=5
export COLCON_TRACE=0
export BUN_INSTALL="$HOME/.bun"
export PATH="$BUN_INSTALL/bin:$PATH"

[ -d /usr/local/cuda/bin ]   && export PATH=/usr/local/cuda/bin:$PATH
[ -d /usr/local/cuda/lib64 ] && export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

remove_ros2_ws_from_path() {
    export ${1}="$(echo ${(P)1} | tr ':' '\n' | grep -v "ros2_ws" | paste -s -d ':')"
}

source_mrover_overlay() {
    source ~/ros2_ws/src/mrover/venv/bin/activate

    local target_file=""
    for profile in RelWithDebInfo Release Debug; do
        local file="$HOME/ros2_ws/install/${profile}/setup.zsh"
        if [ -f "$file" ]; then
            [[ -z "$target_file" ]] || [[ "$file" -nt "$target_file" ]] && target_file="$file"
        fi
    done

    remove_ros2_ws_from_path LD_LIBRARY_PATH
    remove_ros2_ws_from_path AMENT_PREFIX_PATH
    remove_ros2_ws_from_path PYTHONPATH
    remove_ros2_ws_from_path COLCON_PREFIX_PATH
    remove_ros2_ws_from_path CMAKE_PREFIX_PATH

    [ -f "$target_file" ] && source "$target_file" > /dev/null
}

[[ -o interactive ]] || return

export ZSH="$HOME/.oh-my-zsh"
ZSH_THEME="mrover"
plugins=(git virtualenvwrapper fzf zsh-autosuggestions zsh-syntax-highlighting)
source $ZSH/oh-my-zsh.sh

alias mrover="cd ~/ros2_ws/src/mrover && source_mrover_overlay"

[ -s "$HOME/.bun/_bun" ] && source "$HOME/.bun/_bun"
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
