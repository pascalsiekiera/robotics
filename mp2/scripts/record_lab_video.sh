#!/usr/bin/env bash
# Task 2a: record the RViz window during every run of waypoint_nav.py (raw video, no labels).
# Waits until waypoint_nav.py is started, records the RViz window until the program ends, then
# waits for the next run. Stop with Ctrl+C. The start time of every recording is stored next to
# the video (<name>_t0.txt, wall clock = ROS time on the real robot), so the waypoint log
# (t_start/t_end) can be used for labels afterwards.
# Requires: ffmpeg, xwininfo, RViz running (lab.launch.py).
# Usage: bash mp2/scripts/record_lab_video.sh [output dir, default ~/mp2_videos]
# Notes (WSL2/WSLg): recording the whole screen gives black frames, single X windows work
# (-window_id); RViz needs software rendering (lab.launch.py) to show the map.
OUT_DIR="${1:-$HOME/mp2_videos}"
mkdir -p "$OUT_DIR"
trap 'kill -INT "$FF_PID" 2> /dev/null; wait "$FF_PID" 2> /dev/null; exit 0' INT TERM

while true; do
  echo "waiting for waypoint_nav.py ..."
  until PID=$(pgrep -f "waypoint_nav.py .*--tag" | head -1) && [ -n "$PID" ]; do sleep 0.5; done
  TAG=$(tr '\0' ' ' < /proc/"$PID"/cmdline | grep -oP -- '--tag \K\S+')
  WIN=$(xwininfo -root -tree | grep -E '"[^"]*RViz"' | head -1 | awk '{print $1}')
  if [ -z "$WIN" ]; then
    echo "RViz window not found, not recording $TAG"
  else
    NAME="task2a_${TAG}_$(date +%H%M%S)"
    date +%s.%N > "$OUT_DIR/${NAME}_t0.txt"
    setsid ffmpeg -nostdin -loglevel error -y -f x11grab -framerate 15 -window_id "$WIN" -i :0 \
      -c:v libx264 -preset veryfast -crf 20 "$OUT_DIR/${NAME}_raw.mkv" < /dev/null &
    FF_PID=$!
    echo "recording $NAME (waypoint_nav.py pid $PID)"
  fi
  while kill -0 "$PID" 2> /dev/null; do sleep 0.5; done
  if [ -n "$WIN" ]; then
    sleep 3
    kill -INT "$FF_PID"
    wait "$FF_PID"
    echo "saved $OUT_DIR/${NAME}_raw.mkv"
  fi
done
