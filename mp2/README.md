# MP2 – Path Planning, Path Following, Q-Learning

Mini-Project 2 of Introduction to Robotics (IST 2026/27), Group 6.
Status: Task 1a done (part 1: planner comparison, part 2: driving with Dijkstra + DWB).

Working notes with all decisions, parameters and results: [notes/mp2_notes.tex](notes/mp2_notes.tex)
(compile in Overleaf).

## Requirements

- ROS 2 Jazzy, Gazebo Harmonic, Nav2 (`ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`)
- TurtleBot3 simulation: `sudo apt install ros-jazzy-turtlebot3-simulations`
- `export TURTLEBOT3_MODEL=waffle_pi` (in `~/.bashrc`)
- Python: numpy, scipy, matplotlib, pillow, pyyaml

All commands below are run from the repo root (`robotics/`).

## Structure

```
mp2/
├── config/
│   ├── nav2_params.yaml      Nav2 parameters (copy of turtlebot3_navigation2 waffle_pi.yaml, changes marked "MP2")
│   └── test_pairs.yaml       start/goal pairs P1-P3 in turtlebot3_world
├── launch/
│   ├── sim.launch.py             Gazebo + Nav2 (+ ground truth), headless by default
│   └── planning_only.launch.py   map_server + planner_server only (no Gazebo), for the planner comparison
├── scripts/
│   ├── metrics.py            path length, minimum clearance, deviation (shared by all tasks)
│   ├── plan_compare.py       part 1: requests paths for all pairs/planners, saves paths + metrics
│   ├── plot_plans.py         part 1: summary table + figure of the planned paths
│   ├── run_costmap_sweep.sh  part 1: varies inflation_radius / cost_scaling_factor, one at a time
│   ├── drive_compare.py      part 2: drives all pairs, records planned/estimated/true path
│   └── plot_drives.py        part 2: summary table + figure of the driven paths
├── data/task1a/              results (plans_<tag>.json / .csv), launch logs of the sweep
├── figures/                  generated figures
└── notes/mp2_notes.tex       working notes for the report
```

## Changes in `config/nav2_params.yaml` (vs. `waffle_pi.yaml`)

| Change | Why |
|---|---|
| `amcl/set_initial_pose: true` at (-2.0, -0.5, 0) | Gazebo spawn pose in `turtlebot3_world`, no clicking in RViz |
| behavior tree `navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml` | plan once (default replans every second); exactly one planned path per run, same conditions for Dijkstra, A* and later RRT |
| second planner `AStar` (NavFn, `use_astar: true`) | compare with `GridBased` = NavFn with `use_astar: false` = **Dijkstra** |
| goal checker `yaw_goal_tolerance: 6.29` (was 0.25) | heading at the goal is not evaluated; with 0.25 the robot rotates at the goal, the controller gets an empty (pruned) plan, the plan-once tree does not replan and the goal fails |

`GridBased` keeps its name because the behavior tree uses it as the default planner id.
Figures and tables show it as "Dijkstra".

## Task 1a

### Simulation (Gazebo + Nav2 + RViz)

```bash
# everything in one terminal, with our parameters (headless: without gui/rviz)
ros2 launch mp2/launch/sim.launch.py gui:=true rviz:=true
```

`sim.launch.py` starts the same world, robot and Nav2 bringup as the tutorial, with two changes to
the robot model (generated at launch time, the installed files are not modified):
- **camera removed**: rendering it in software and bridging the raw images (≈27 MB/s) overloaded the
  CPU under WSL2 (Gazebo at 0.3× real time, laser scans only every few seconds, Nav2 did not start);
  Nav2 does not use the camera,
- **PosePublisher plugin**: ground truth on `/ground_truth/pose` (`geometry_msgs/PoseStamped`, 30 Hz,
  Gazebo world frame = map frame).

The tutorial way (two terminals, with camera) still works:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True \
    params_file:=$PWD/mp2/config/nav2_params.yaml
```

Then set a goal with "Nav2 Goal" in RViz, or from the command line:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.5}, orientation: {w: 1.0}}}}"
```

With the default `waffle_pi.yaml` (no `params_file`), AMCL waits for an initial pose; until then the
costmap logs `Timed out waiting for transform from base_link to map` (expected). Set it with
"2D Pose Estimate" or:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}"
```

The map is the one shipped with ROBOTIS: `/opt/ros/jazzy/share/turtlebot3_navigation2/map/map.yaml`.

### Part 1: planner comparison (no Gazebo)

Runs only `map_server` + `planner_server` with the same planner/costmap parameters, but
- global costmap = static layer + inflation layer only (no live laser scan, identical costmap for every request),
- `use_sim_time: false` (planning time in wall-clock time, not in steps of the Gazebo clock).

Do not run it at the same time as the simulation (same node names).

```bash
# terminal 1
ros2 launch mp2/launch/planning_only.launch.py
# optional costmap overrides: inflation_radius:=0.3 cost_scaling_factor:=10.0 robot_radius:=0.21

# terminal 2: 20 requests per pair and planner -> mp2/data/task1a/plans_baseline.{json,csv}
python3 mp2/scripts/plan_compare.py --planners GridBased AStar --reps 20 --tag baseline

# table + figure -> mp2/figures/plans_baseline.png
python3 mp2/scripts/plot_plans.py mp2/data/task1a/plans_baseline.json
```

Costmap parameter sweep (starts and stops the planner by itself, ~4 min):

```bash
bash mp2/scripts/run_costmap_sweep.sh 10
python3 mp2/scripts/plot_plans.py mp2/data/task1a/plans_{baseline,infl0.4,infl0.3,infl0.2}.json \
    --planners GridBased --name sweep_inflation_dijkstra
python3 mp2/scripts/plot_plans.py mp2/data/task1a/plans_{csf2,baseline,csf10}.json \
    --planners GridBased --name sweep_csf_dijkstra
```

### Metrics (`scripts/metrics.py`)

| Metric | Definition | Evaluates |
|---|---|---|
| path length | sum of segment lengths of the planned path [m] | planner |
| planning time | time reported by `planner_server` (wall clock in the planning-only setup) [ms], reported as median | planner |
| min. clearance | smallest distance of the path to an occupied cell of the static map, from the robot centre [m] | planner |
| deviation (estimated) | distance of each estimated pose (tf map → base_footprint, 10 Hz) to the nearest point of the planned path, mean and max [m] | controller (part 2) |
| deviation (true) | same with the Gazebo ground truth | controller + localization (part 2) |
| localization error | estimated vs. true pose (for interpreting the two deviations) [m] | AMCL (part 2) |

### Part 2: driving

Dijkstra + DWB only (A* gives almost the same paths; the controller comparison is not required).
The robot drives to the start of each pair (not evaluated), then the measured run. The planned path is
the one published on `/plan` during the run (plan-once tree: the path the controller follows).

```bash
# terminal 1
ros2 launch mp2/launch/sim.launch.py
# terminal 2: ~3 min -> mp2/data/task1a/drives_dijkstra_dwb.json
python3 mp2/scripts/drive_compare.py --tag dijkstra_dwb
python3 mp2/scripts/plot_drives.py mp2/data/task1a/drives_dijkstra_dwb.json
# repetitions: --tag dijkstra_dwb_r2, dijkstra_dwb_r3; all in one figure:
python3 mp2/scripts/plot_drives.py mp2/data/task1a/drives_dijkstra_dwb{,_r2,_r3}.json --name drives_dijkstra_dwb_all
```

Start and end of a run differ from the pair coordinates by up to `xy_goal_tolerance` (0.25 m), so the
planned lengths differ from part 1.

### Results and figures

Tables and observations: [notes/mp2_notes.tex](notes/mp2_notes.tex), section "Results".

| Figure | Content |
|---|---|
| `test_pairs_proposal.png` | start/goal pairs P1–P3 |
| `plans_baseline.png` | part 1: Dijkstra vs. A*, default costmap |
| `inflation_cost.png` | inflation cost over distance for the tested parameters |
| `sweep_inflation_dijkstra.png`, `sweep_csf_dijkstra.png` | part 1: inflation radius / cost scaling factor |
| `drives_dijkstra_dwb.png`, `drives_dijkstra_dwb_all.png` | part 2: planned, estimated and true path |

## Pitfalls

- **Leftover processes break the next run.** `gz sim` sometimes survives Ctrl+C and keeps publishing
  `/clock`, `/scan`, `/tf`; a leftover `static_transform_publisher` (map → base_link from
  `planning_only.launch.py`) splits the TF tree of the simulation (`base_link` gets two parents,
  "Timed out waiting for transform from base_link to odom"). Before starting, check:
  `pgrep -fa "gz sim|ros2 launch|static_transform|component_container"` (should be empty).
- Messages that queue up while a node does not spin are delivered later. `drive_compare.py` therefore
  only keeps messages with a time stamp after the start of the run.

- `BasicNavigator.waitUntilNav2Active()` (nav2_simple_commander) publishes an initial pose at (0, 0) if it
  does not receive an `amcl_pose` and thereby resets AMCL. Our scripts only wait for the node they need.
- In a bash script, background jobs ignore SIGINT, so `ros2 launch ... &` cannot be stopped with
  `kill -INT`; `run_costmap_sweep.sh` starts it with `setsid`.
- `robot_radius` in `waffle_pi.yaml` is 0.15 m, the Waffle Pi footprint (281 × 306 mm) has a circumscribed
  radius of about 0.21 m.
