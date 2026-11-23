#!/usr/bin/env bash
# Task 1b: RRT baseline (plain RRT) and one variant per parameter, each changed alone from the
# baseline (step 0.2, goal bias 0, cost threshold 253, no smoothing, seed 1). Runs without Gazebo.
# With the fixed seed, every configuration starts the same random sequence, so the runs are
# reproducible (same planner start + same request order = same paths).
# Usage: bash mp2/scripts/run_rrt_experiments.sh [reps]
set -e
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
MP2="$(cd "$(dirname "$0")/.." && pwd)"
REPS="${1:-50}"
OUT="$MP2/data/task1b"
LOG_DIR="$OUT/logs"
mkdir -p "$LOG_DIR"

# tag  launch arguments
CONFIGS=(
  "rrt_base"
  "rrt_step0.5 rrt_step:=0.5"
  "rrt_bias0.1 rrt_goal_bias:=0.1"
  "rrt_thr128 rrt_cost_threshold:=128"
  "rrt_smooth rrt_smoothing:=true"
)

for config in "${CONFIGS[@]}"; do
  read -r tag launch_args <<< "$config"
  echo "=== $tag (${launch_args:-baseline})"
  # own session/process group: background jobs of a script ignore SIGINT, and stopping the
  # whole group also stops all nodes (incl. static_transform_publisher)
  setsid ros2 launch "$MP2/launch/planning_only.launch.py" $launch_args > "$LOG_DIR/$tag.log" 2>&1 &
  pgid=$!
  for _ in $(seq 30); do
    grep -q "Managed nodes are active" "$LOG_DIR/$tag.log" && break
    sleep 1
  done
  python3 "$MP2/scripts/plan_compare.py" --planners RRT --reps "$REPS" --tag "$tag" --out "$OUT" \
    2>&1 | grep -E "ERROR|saved" || true
  kill -INT -- "-$pgid"
  for _ in $(seq 30); do
    pgrep -g "$pgid" > /dev/null || break
    sleep 1
  done
  kill -KILL -- "-$pgid" 2> /dev/null || true
done
