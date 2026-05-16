#!/usr/bin/env bash

FOLDER=~/Desktop/science_$(date +%Y-%m-%d_%H-%M-%S)

# Make the science
mkdir $FOLDER

scp ~/ros2_ws/src/mrover/data/raw-pano-images/$(ls -1 ~/ros2_ws/src/mrover/data/raw-pano-images | tail -n 1)/pano.png john@10.1.0.2:$FOLDER

find ~/Downloads/ \
  -type f \
  -regextype posix-extended \
  -regex '.*/[^/]+_[0-9]{2}-[0-9]{2}-[0-9]{2}\.png' \
  -mmin -45 \
  -exec cp {} $FOLDER \;

now_secs=$(date +%s)
today=$(date +%Y-%m-%d)

find ~/Downloads -type f -name '*.png' | while read -r f; do
    base=$(basename "$f")

    if [[ $base =~ _([0-9]{2})-([0-9]{2})-([0-9]{2})\.png$ ]]; then
        hh="${BASH_REMATCH[1]}"
        mm="${BASH_REMATCH[2]}"
        ss="${BASH_REMATCH[3]}"

        file_secs=$(date -d "$today $hh:$mm:$ss" +%s)

        diff=$(( now_secs - file_secs ))

        if (( diff >= 0 && diff <= 2700 )); then
            echo "Copying: $f"
            cp "$f" "$FOLDER/"
        fi
    fi
done
