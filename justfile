set shell := ["zsh", "-ceuo", "pipefail"]

ext := `nvidia-smi &>/dev/null && echo "gpu.yml" || echo "yml"`
compose := 'docker compose --file docker/compose'

alias u := up
alias d := down
alias a := attach
alias l := log

# list available recipes
@default:
    just --list

# launch the docker compose configuration
up:
    xhost +SI:localuser:root
    xhost +local:docker
    {{compose}}.{{ext}} up --detach --force-recreate

# terminate the docker compose configuration (data lost)
down:
    {{compose}}.{{ext}} down

# start containers from a suspended state
start:
    {{compose}}.{{ext}} start

# suspend containers (maintains disk state)
stop:
    {{compose}}.{{ext}} stop

# attach to a running container (supports parallel instances)
attach:
    docker exec -it mrover-ros zsh

# follow logs on container
log:
    docker logs -f mrover-ros
