#!/usr/bin/env bash
# Record the RViz window while drive_compare.py drives all pairs, then crop the map view,
# speed it up 2x and add a legend and a label for every section of the drive
# ("drive to the start of P2 (not measured)" / "measured run P2").
# Requires: ffmpeg, simulation running with RViz:
#     ros2 launch mp2/launch/sim.launch.py rviz:=true [controller:=teb]
# Usage: bash mp2/scripts/record_video.sh <output name> "<title>" [drive_compare.py arguments]
#     bash mp2/scripts/record_video.sh task1a_dijkstra_dwb "Task 1a - Dijkstra + DWB" --tag video_dwb
# Environment: CROP=w:h:x:y crops the 3D view out of the RViz window (default: RViz layout
# of mp2.rviz in a 1610x893 window).
# Notes (WSL2/WSLg): recording the whole screen gives black frames, single X windows work
# (-window_id). The Gazebo GUI did not show the robot moving in the recording, so only RViz.
set -e
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
MP2="$(cd "$(dirname "$0")/.." && pwd)"
NAME="${1:?output name}"
TITLE="${2:?title}"
shift 2
CROP="${CROP:-896:806:364:59}"
OUT_DIR="$MP2/videos"
RAW="$OUT_DIR/${NAME}_raw.mkv"
LOG="$OUT_DIR/${NAME}_drive.log"
FILTER="$OUT_DIR/${NAME}_filter.txt"
FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf
mkdir -p "$OUT_DIR"

WIN=$(xwininfo -root -tree | grep -E '"[^"]*RViz"' | head -1 | awk '{print $1}')
[ -n "$WIN" ] || { echo "RViz window not found (start sim.launch.py with rviz:=true)"; exit 1; }

# own session, so that only this ffmpeg is stopped below
T0=$(date +%s.%N)
setsid ffmpeg -nostdin -loglevel error -y -f x11grab -framerate 15 -window_id "$WIN" -i :0 \
  -c:v libx264 -preset veryfast -crf 20 "$RAW" < /dev/null &
FF_PID=$!
sleep 3
python3 "$MP2/scripts/drive_compare.py" "$@" 2>&1 | tee "$LOG" || true
sleep 4
kill -INT "$FF_PID"
wait "$FF_PID" || true

# section labels from the log of drive_compare.py: ROS log lines carry the wall time,
# "[INFO] [<time>] [basic_navigator]: P2: driving to start ..., attempt 1"
python3 - "$LOG" "$T0" "$TITLE" "$FONT" "$CROP" > "$FILTER" <<'EOF'
import re, sys
log, t0, title, font, crop = sys.argv[1], float(sys.argv[2]), sys.argv[3], sys.argv[4], sys.argv[5]
events = []  # (video time at 2x speed, label)
for line in open(log):
    m = re.search(r'\[(\d+\.\d+)\] \[[^\]]+\]: (P\d+): (driving to start|measured run).*?(?:attempt (\d+))?$', line.strip())
    if m:
        t = (float(m.group(1)) - t0) / 2
        pair, kind, attempt = m.group(2), m.group(3), m.group(4)
        if kind == 'measured run':
            events.append((t, f'measured run {pair}', 'yellow'))
        else:
            extra = f', attempt {attempt} (AMCL check failed)' if attempt and attempt != '1' else ''
            events.append((t, f'drive to the start of {pair} (not measured){extra}', 'white'))
    elif re.search(r'\]: saved ', line):
        events.append(((float(re.search(r'\[(\d+\.\d+)\]', line).group(1)) - t0) / 2, None, None))

def esc(s):  # drawtext text inside single quotes: escape ' : , \
    return s.replace('\\', '\\\\').replace("'", "\\'").replace(':', '\\:').replace(',', '\\,')

f = [f'crop={crop}', 'setpts=0.5*PTS',
     'drawbox=x=0:y=0:w=iw:h=84:color=black@0.6:t=fill',
     f"drawtext=fontfile={font}:text='{esc(title)}':x=12:y=8:fontsize=20:fontcolor=white",
     f"drawtext=fontfile={font}:text='{esc('red: planned path   green: estimated path (AMCL)   orange: true path (Gazebo)   (2x speed)')}'"
     ':x=12:y=34:fontsize=15:fontcolor=white',
     f"drawtext=fontfile={font}:text='{esc('blue: local plan (controller)')}':x=w-tw-12:y=60:fontsize=15:fontcolor=white"]
for (t, label, color), (t_next, _, _) in zip(events, events[1:] + [(1e9, None, None)]):
    if label:
        f.append(f"drawtext=fontfile={font}:text='{esc(label)}':x=12:y=58:fontsize=18:"
                 f"fontcolor={color}:enable='between(t,{t:.2f},{t_next:.2f})'")
print(',\n'.join(f))
EOF

ffmpeg -nostdin -loglevel error -y -i "$RAW" -filter_script:v "$FILTER" \
  -r 15 -c:v libx264 -crf 24 -preset medium -pix_fmt yuv420p -an "$OUT_DIR/$NAME.mp4"
rm -f "$RAW" "$FILTER" "$LOG"
echo "saved $OUT_DIR/$NAME.mp4"
