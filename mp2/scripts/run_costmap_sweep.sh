#!/usr/bin/env bash
# Task 1a, part 1: vary one costmap parameter at a time (baseline: inflation_radius 0.5,
# cost_scaling_factor 5.0) and compare the planned paths. Runs without Gazebo.
# Usage: bash mp2/scripts/run_costmap_sweep.sh [reps]
set -e
source /opt/ros/jazzy/setup.bash
MP2="$(cd "$(dirname "$0")/.." && pwd)"
REPS="${1:-10}"
LOG_DIR="$MP2/data/task1a/logs"
mkdir -p "$LOG_DIR"

# tag  launch arguments
CONFIGS=(
  "infl0.4 inflation_radius:=0.4"
  "infl0.3 inflation_radius:=0.3"
  "infl0.2 inflation_radius:=0.2"
  "csf2 cost_scaling_factor:=2.0"
  "csf10 cost_scaling_factor:=10.0"
)

for config in "${CONFIGS[@]}"; do
  read -r tag launch_args <<< "$config"
  echo "=== $tag ($launch_args)"
  # own session/process group: background jobs of a script ignore SIGINT, and stopping the
  # whole group also stops all nodes (incl. static_transform_publisher)
  setsid ros2 launch "$MP2/launch/planning_only.launch.py" $launch_args > "$LOG_DIR/$tag.log" 2>&1 &
  pgid=$!
  for _ in $(seq 30); do
    grep -q "Managed nodes are active" "$LOG_DIR/$tag.log" && break
    sleep 1
  done
  python3 "$MP2/scripts/plan_compare.py" --reps "$REPS" --tag "$tag" 2>&1 | grep -E "WARN|ERROR|saved" || true
  kill -INT -- "-$pgid"
  for _ in $(seq 30); do
    pgrep -g "$pgid" > /dev/null || break
    sleep 1
  done
  kill -KILL -- "-$pgid" 2> /dev/null || true
done
