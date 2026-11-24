#!/usr/bin/env bash
# Task 2a: RViz video of a lab run, made afterwards from its rosbag (the live recording of the
# lab runs was not possible). Starts its own ROS graph on a separate domain (map_server with the
# lab map, robot_state_publisher for the robot model, replay_helpers.py, RViz with mp2.rviz),
# plays the bag with --clock from shortly before the first leg of the waypoint log to shortly
# after the last one, records the RViz window with ffmpeg, then crops the 3D view, speeds it up
# 2x and adds title, legend and a label per leg ("leg to waypoint k (result, duration)").
# Output: mp2/videos/task2a_<name>_rviz.mp4
# Usage: bash mp2/scripts/record_lab_replay.sh <bag dir> <waypoint log .json> <name> "<title>"
#     bash mp2/scripts/record_lab_replay.sh ~/mp2_bags/dijkstra_dwb_test \
#         mp2/data/task2a/waypoints_dijkstra_dwb_test_20261124_121107.json dijkstra_dwb_run1 \
#         "Task 2a - lab run 1: Dijkstra + DWB"
# Environment:
#     REPLAY_DOMAIN_ID (default 77): ROS domain of the replay; it runs with ROS_LOCALHOST_ONLY=1
#         and must differ from the domain of the robot (11), otherwise the bag would reach the
#         robot / Nav2.
#     VIEW="x y scale" (default "2.6 2.4 118"): center [m] and zoom [px/m] of the top-down view.
#     CROP=w:h:x:y: crop of the 3D view out of the RViz window (default: geometry of the RViz
#         render window from xwininfo).
#     LEAD / TAIL (default 3 / 4 s): bag time played before the first leg / after the last one.
#     MAP (default mp1/maps/map_2a_merged.yaml).
# Notes (WSL2/WSLg): RViz needs software rendering (LIBGL_ALWAYS_SOFTWARE=1), with the GPU the map
# stays empty (GLSL error). Only the single RViz window can be recorded (-window_id), the whole
# screen gives black frames. The bag's /tf_static is published once at its start, so it is
# republished by replay_helpers.py (bag played with --start-offset, without /tf_static);
# /tf of the bag is played on /bag_tf, replay_helpers.py forwards it without map -> odom and
# computes map -> odom from /amcl_pose (the recorder lost /tf messages in the lab).
# robot_state_publisher only provides /robot_description (its TF output is remapped away, the bag
# has all transforms incl. wheels).
set -e
ulimit -c 0   # rviz2 (Humble) segfaults when it is stopped
BAG="$(realpath "${1:?bag directory}")"
WP_LOG="$(realpath "${2:?waypoint log (.json)}")"
NAME="${3:?output name}"
TITLE="${4:?title}"

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${REPLAY_DOMAIN_ID:-77}" ROS_LOCALHOST_ONLY=1 DISPLAY="${DISPLAY:-:0}"
if [ "$ROS_DOMAIN_ID" = 11 ] || [ "$ROS_LOCALHOST_ONLY" != 1 ]; then
  echo "refusing to replay on the robot's domain"; exit 1
fi
MP2="$(cd "$(dirname "$0")/.." && pwd)"
MAP="${MAP:-$MP2/../mp1/maps/map_2a_merged.yaml}"
VIEW="${VIEW:-2.6 2.4 118}"
CROP="${CROP:-}"
LEAD="${LEAD:-3}"
TAIL="${TAIL:-4}"
FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf
OUT="$MP2/videos/task2a_${NAME}_rviz.mp4"
WORK="$(mktemp -d /tmp/mp2_replay_XXXXXX)"
CFG="$WORK/mp2_replay_${NAME}.rviz"   # unique window title "mp2_replay_<name>.rviz - RViz"

# every process in its own session; only these process groups are stopped at the end
PGIDS=()
start() {  # start <log name> <command ...>
  local log="$WORK/$1.log"; shift
  setsid nice -n 10 "$@" > "$log" 2>&1 < /dev/null &
  PGIDS+=($!)
  disown   # no job messages (rviz2 segfaults when it is stopped)
}
stop_group() {
  kill -INT -- "-$1" 2> /dev/null || return 0
  for _ in $(seq 20); do kill -0 -- "-$1" 2> /dev/null || return 0; sleep 0.25; done
  kill -KILL -- "-$1" 2> /dev/null || true
}
cleanup() {
  for ((i = ${#PGIDS[@]} - 1; i >= 0; i--)); do stop_group "${PGIDS[i]}"; done
  [ -n "$KEEP_WORK" ] || rm -rf "$WORK"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# times: first t_start / last t_end of the log, start of the bag -> --start-offset
read -r T_FIRST T_LAST BAG_START < <(python3 - "$WP_LOG" "$BAG" <<'EOF'
import json, sys, rosbag2_py
log = json.load(open(sys.argv[1]))
info = rosbag2_py.Info().read_metadata(sys.argv[2], 'sqlite3')
a = log['attempts']
print(min(x['t_start'] for x in a), max(x['t_end'] for x in a),
      info.starting_time.nanoseconds * 1e-9)
EOF
)
OFFSET=$(python3 -c "print(max(0.0, $T_FIRST - $LEAD - $BAG_START))")
T_STOP=$(python3 -c "print($T_LAST + $TAIL)")
echo "bag $BAG: start offset ${OFFSET} s, playing until bag time $T_STOP"

# RViz config: mp2.rviz with the robot description transient local (published once by
# robot_state_publisher), view centered on the lab, TF axes off, laser scan in one color, global
# plan without pose arrows, local plan thicker
python3 - "$MP2/config/mp2.rviz" "$CFG" $VIEW <<'EOF'
import sys, yaml
src, dst, x, y, scale = sys.argv[1], sys.argv[2], *map(float, sys.argv[3:6])
cfg = yaml.safe_load(open(src))
for d in cfg['Visualization Manager']['Displays']:
    if d['Class'] == 'rviz_default_plugins/RobotModel':
        d['Description Topic']['Durability Policy'] = 'Transient Local'
    elif d['Class'] == 'rviz_default_plugins/TF':
        d['Enabled'] = False
    elif d['Class'] == 'rviz_default_plugins/LaserScan':
        d['Color Transformer'] = 'FlatColor'
        d['Color'] = '255; 255; 0'
        d['Size (Pixels)'] = 4
    elif d.get('Name') == 'MP2 driven path (ground truth)':
        d['Enabled'] = False
    for sub in d.get('Displays', []):
        if sub['Class'] == 'rviz_default_plugins/Path' and sub['Topic']['Value'] == '/plan':
            sub['Pose Style'] = 'None'   # the arrows make the red line look pink
        elif sub['Class'] == 'rviz_default_plugins/Path' and sub['Topic']['Value'] == '/local_plan':
            sub.update({'Line Style': 'Billboards', 'Line Width': 0.03})
view = cfg['Visualization Manager']['Views']['Current']
view.update({'X': x, 'Y': y, 'Scale': scale, 'Angle': 0.0})
cfg['Window Geometry'].update({'X': 0, 'Y': 0, 'Width': 1610, 'Height': 893})
yaml.safe_dump(cfg, open(dst, 'w'), sort_keys=False)
EOF

# robot_state_publisher parameters: Waffle Pi URDF (xacro, empty namespace)
python3 - "$WORK/rsp.yaml" <<'EOF'
import subprocess, sys, yaml
urdf = subprocess.run(['xacro', '/opt/ros/humble/share/turtlebot3_description/urdf/'
                       'turtlebot3_waffle_pi.urdf', 'namespace:='],
                      check=True, capture_output=True, text=True).stdout
yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {
    'robot_description': urdf, 'use_sim_time': True}}}, open(sys.argv[1], 'w'))
EOF

start map_server ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:="$MAP" -p use_sim_time:=true
start rsp ros2 run robot_state_publisher robot_state_publisher --ros-args \
  --params-file "$WORK/rsp.yaml" -r /tf:=/mp2_replay/rsp_tf -r /tf_static:=/mp2_replay/rsp_tf_static
start helpers python3 "$MP2/scripts/replay_helpers.py" --bag "$BAG" \
  --waypoints "$MP2/config/waypoints_lsdc3.yaml" --clock-file "$WORK/clock.txt"
for step in configure activate; do
  for _ in $(seq 30); do
    ros2 lifecycle set --no-daemon --spin-time 2 /map_server "$step" 2> /dev/null | grep -q successful && break
    sleep 1
  done
done
ros2 lifecycle get --no-daemon --spin-time 2 /map_server | grep -q active || { echo "map_server not active"; exit 1; }

start rviz env LIBGL_ALWAYS_SOFTWARE=1 rviz2 -d "$CFG" --ros-args -p use_sim_time:=true
WIN=""
for _ in $(seq 60); do
  # title "<config>.rviz - RViz", with "*" after .rviz once RViz marks the config as modified
  WIN=$(xwininfo -root -tree | grep -F "mp2_replay_${NAME}.rviz" | grep -F -- "- RViz" | head -1 | awk '{print $1}')
  [ -n "$WIN" ] && break
  sleep 1
done
[ -n "$WIN" ] || { echo "RViz window not found"; cat "$WORK/rviz.log"; exit 1; }
sleep 8   # RViz loads the plugins, map and robot model
# crop: the 3D view is the largest native child window of RViz (absolute position - window's)
if [ -z "$CROP" ]; then
  CROP=$(python3 - "$(xwininfo -id "$WIN")" "$(xwininfo -id "$WIN" -tree)" <<'EOF'
import re, sys
info, tree = sys.argv[1], sys.argv[2]
ax = int(re.search(r'Absolute upper-left X: +(-?\d+)', info).group(1))
ay = int(re.search(r'Absolute upper-left Y: +(-?\d+)', info).group(1))
# child lines: 0x600011 "rviz2": ("rviz2" "rviz2")  879x789+373+68  +379+95
children = [re.search(r'(\d+)x(\d+)[+-]-?\d+[+-]-?\d+ +([+-]-?\d+)([+-]-?\d+)$', l.strip())
            for l in tree.splitlines()[1:] if l.strip().startswith('0x')]
w, h, x, y = max((m for m in children if m), key=lambda m: int(m[1]) * int(m[2])).groups()
print(f'{int(w) // 2 * 2}:{int(h) // 2 * 2}:{int(x) - ax}:{int(y) - ay}')
EOF
)
fi
echo "RViz window $WIN, crop $CROP"

start bag ros2 bag play "$BAG" --clock 50 --start-offset "$OFFSET" --disable-keyboard-controls \
  --topics /amcl_pose /plan /local_plan /odom /scan /tf /cmd_vel --remap /tf:=/bag_tf
BAG_PGID=${PGIDS[-1]}
# record as soon as the bag clock runs (replay_helpers.py writes clock.txt)
for _ in $(seq 100); do [ -f "$WORK/clock.txt" ] && break; sleep 0.1; done
T0=$(date +%s.%N)
start ffmpeg ffmpeg -nostdin -loglevel error -y -f x11grab -draw_mouse 0 -framerate 15 -window_id "$WIN" -i :0 \
  -c:v libx264 -preset veryfast -crf 20 "$WORK/raw.mkv"
FF_PGID=${PGIDS[-1]}
# wait until the end of the last leg (+ TAIL) is played (bag time from replay_helpers.py)
while kill -0 -- "-$BAG_PGID" 2> /dev/null; do
  if [ -f "$WORK/clock.txt" ] && python3 -c "import sys; sys.exit(float(open('$WORK/clock.txt').read().split()[1]) < $T_STOP)"; then
    break
  fi
  sleep 0.5
done
stop_group "$BAG_PGID"   # the last state stays visible (no /clock -> RViz time stops)
sleep 3
stop_group "$FF_PGID"
CLOCK_OFFSET=$(cut -d' ' -f1 "$WORK/clock.txt")

# labels per leg from the waypoint log: bag time + (wall - bag offset) - recording start, 2x speed
python3 - "$WP_LOG" "$CLOCK_OFFSET" "$T0" "$TITLE" "$FONT" "$CROP" > "$WORK/filter.txt" <<'EOF'
import json, sys
log, offset, t0, title, font, crop = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), *sys.argv[4:7]
vt = lambda t_bag: (t_bag + offset - t0) / 2   # video time (2x speed)
attempts = sorted(json.load(open(log))['attempts'], key=lambda a: a['t_start'])

def esc(s):  # drawtext text inside single quotes: escape ' : , \ %
    return (s.replace('\\', '\\\\').replace("'", "\\'").replace(':', '\\:').replace(',', '\\,')
            .replace('%', '\\%'))

def text(s, x, y, size, color='white', enable=None):
    out = f"drawtext=fontfile={font}:text='{esc(s)}':x={x}:y={y}:fontsize={size}:fontcolor={color}"
    return out + (f":enable='between(t,{enable[0]:.2f},{enable[1]:.2f})'" if enable else '')

f = [f'crop={crop}', 'setpts=0.5*PTS',
     'drawbox=x=0:y=0:w=iw:h=84:color=black@0.6:t=fill',
     text(title, 12, 8, 20),
     text('red: global plan   green: estimated path (AMCL)   blue: local plan   (2x speed)', 12, 34, 15),
     text('yellow dots: laser scan   orange: waypoints', 'w-tw-12', 60, 15)]
for i, a in enumerate(attempts):
    t_next = attempts[i + 1]['t_start'] if i + 1 < len(attempts) else a['t_end'] + 1.0
    extra = f", attempt {a['attempt']}" if a.get('attempt', 1) != 1 else ''
    rnd = f"round {a['round']} ({a['planner']}): " if a.get('planner') else ''
    label = f"{rnd}leg to waypoint {a['waypoint']} ({a['result']}, {a['duration_s']:.1f} s{extra})"
    color = 'yellow' if a['result'] == 'succeeded' else 'orange'
    f.append(text(label, 12, 58, 18, color, (vt(a['t_start']), vt(t_next))))
n_ok = sum(a['result'] == 'succeeded' for a in attempts)
total = attempts[-1]['t_end'] - attempts[0]['t_start']
f.append(text(f'round finished: {n_ok}/{len(attempts)} legs succeeded, {total:.1f} s', 12, 58, 18,
              'white', (vt(attempts[-1]['t_end'] + 1.0), 1e6)))
print(',\n'.join(f))
EOF

mkdir -p "$MP2/videos"
nice -n 10 ffmpeg -nostdin -loglevel error -y -i "$WORK/raw.mkv" -filter_script:v "$WORK/filter.txt" \
  -r 15 -c:v libx264 -crf 24 -preset medium -pix_fmt yuv420p -an "$OUT"
echo "saved $OUT"
