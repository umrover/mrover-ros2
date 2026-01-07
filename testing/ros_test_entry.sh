#!/usr/bin/env zsh

set -euxo pipefail

TEMP_FILE=$(mktemp) || exit 1

ros2 launch mrover test_approach_tag.py temp_file:="$TEMP_FILE"

if [[ "$(<"$TEMP_FILE")" == "0" ]]; then
    rm -f $TEMP_FILE
    exit 0
else
    rm -f $TEMP_FILE
    exit 1
fi

