#!/usr/bin/env bash

FOLDER=~/Desktop/science_$(date +%Y-%m-%d_%H-%M-%S)

# Make the science
# shellcheck disable=SC2086
mkdir $FOLDER

# shellcheck disable=SC2086
scp -r jetson:/home/mrover/ros2_ws/src/mrover/data/raw-pano-images $FOLDER

now_secs=$(date +%s)
today=$(date +%Y-%m-%d)

# copy all of the downloaded graphs
find ~/Downloads -type f \( -name "*.png" -o -name "*.txt" \) | while read -r f; do
    base=$(basename "$f")

    if [[ $base =~ _([0-9]{2})-([0-9]{2})-([0-9]{2})\.(png|txt)$ ]]; then
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

# copy all of the screenshots
find ~/Pictures/Screenshots -type f -name "*.png" | while read -r f; do
    base=$(basename "$f")

    if [[ $base =~ ([0-9]{4})-([0-9]{2})-([0-9]{2})\ ([0-9]{2})-([0-9]{2})-([0-9]{2})\.png$ ]]; then
        yr="${BASH_REMATCH[1]}"
        mn="${BASH_REMATCH[2]}"
        dy="${BASH_REMATCH[3]}"

        hh="${BASH_REMATCH[4]}"
        mm="${BASH_REMATCH[5]}"
        ss="${BASH_REMATCH[6]}"

        file_secs=$(date -d "$yr-$mn-$dy $hh:$mm:$ss" +%s)

        diff=$(( now_secs - file_secs ))

        if (( diff >= 0 && diff <= 2700 )); then
            echo "Copying: $f"
            cp "$f" "$FOLDER/"
        fi
    fi
done
