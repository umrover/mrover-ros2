# this file is deployed if the user does not have an existing ~/.zshrc

export ZSH="$HOME/.oh-my-zsh"
ZSH_THEME="mrover"
plugins=(git fzf zsh-autosuggestions zsh-syntax-highlighting)
source $ZSH/oh-my-zsh.sh
