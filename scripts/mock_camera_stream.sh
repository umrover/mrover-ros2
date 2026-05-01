#!/usr/bin/env bash
PORT=${1:-8082}
gst-launch-1.0 -q videotestsrc pattern=smpte ! video/x-raw,width=640,height=480,framerate=15/1 ! x265enc tune=zerolatency ! rtph265pay ! udpsink host=127.0.0.1 port="${PORT}"
