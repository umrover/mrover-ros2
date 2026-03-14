set shell := ["zsh", "-ceuo", "pipefail"]

default_preset := "Debug"

alias b := build
alias d := docs
alias f := flash
alias m := monitor
alias v := venv

# list available recipes
@default:
    just --list

# build a project
@build src preset=default_preset:
    ./scripts/build.sh --src {{src}} --preset {{preset}}

# write an executable to STLINK (reset and run application)
@flash src preset=default_preset:
    just build {{src}} {{preset}}
    ./scripts/build.sh --src {{src}} --preset {{preset}} --flash

# start a local mkdocs server
docs:
    #!/usr/bin/env zsh
    source ./tools/venv/bin/activate
    mkdocs serve

# update cmake tooling
cmake src *libs:
    #!/usr/bin/env zsh
    source tools/venv/bin/activate
    PY_LIB_ARGS=()
    for lib in {{libs}}; do
        PY_LIB_ARGS+=(--lib "$lib")
    done
    python ./tools/scripts/update_cmake_cfg.py --src {{src}} --root . --ctx ./lib/stm32g4 "${PY_LIB_ARGS[@]}"

# run serial monitor on stlinkv3
monitor baud="115200" log="INFO":
    #!/usr/bin/env zsh
    source tools/venv/bin/activate
    python ./tools/scripts/monitor.py --baud {{baud}} --log-level {{log}}

# install/update the venv
venv:
    #!/usr/bin/env zsh
    pushd tools
    cmake -S . -B build
    cmake --build build --target python_env_ready
    rm -rf build/
    popd
