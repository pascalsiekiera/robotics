# MP2 – Path Planning, Path Following, Q-Learning

Mini-Project 2 of Introduction to Robotics (IST 2026/27), Group 6.
Status: Task 1a done under **ROS 2 Humble** (part 1: planner comparison, part 2: driving with
Dijkstra + DWB and Dijkstra + TEB, videos). All results are from Humble (the lab robot runs Humble;
TEB and the RRT package of Task 1b need Humble).
An earlier version under Jazzy is archived and not used: `jazzy/`, `data/task1a_jazzy/`, `figures/jazzy/`.

Working notes with all decisions, parameters and results: [notes/mp2_notes.tex](notes/mp2_notes.tex)
(compile in Overleaf). All design decisions per task, as the basis for the report:
[notes/report-notes.md](notes/report-notes.md) (LaTeX: `notes/report_notes.tex`).

## Requirements

- Ubuntu 22.04 + ROS 2 Humble, Gazebo Classic 11, Nav2, TurtleBot3 simulation
- source packages in `~/ros2_ws/src`: `teb_local_planner` (branch `humble-devel`),
  `costmap_converter` (branch `humble`), and a link to `mp2/rrt_planner` (our RRT, Task 1b)
- Python: numpy, scipy, matplotlib, pillow, pyyaml; ffmpeg + xwininfo for the videos

Setup on Windows/WSL2 (a second distro next to the Jazzy one):

```bash
# Windows PowerShell
wsl --install -d Ubuntu-22.04
# inside Ubuntu-22.04
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone -b humble-devel https://github.com/rst-tu-dortmund/teb_local_planner.git
git clone -b humble https://github.com/rst-tu-dortmund/costmap_converter.git
ln -s <path>/mp2/rrt_planner rrt_planner   # our RRT planner (Task 1b), lives in this repo
git -C teb_local_planner apply <path>/mp2/setup/teb_plan_transform.patch   # see Pitfalls
sudo bash <path>/mp2/setup/install_humble.sh   # ROS, Nav2, TB3, Gazebo, rosdep, ~/.bashrc
cd ~/ros2_ws && MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 1
```

`MAKEFLAGS=-j2`: compiling `teb_local_planner` needs about 1.5 GB per compiler process; with one
process per core the compiler is killed for lack of memory under WSL2 (`Killed signal terminated
program cc1plus`).

`install_humble.sh` also mounts this repo from the Jazzy distro at `/mnt/robotics` (one copy of the
repo; at every start via `/etc/wsl.conf`, by hand: `sudo mount-robotics`). The path in the script
is for our machine (`\\wsl.localhost\Ubuntu\home\maya\...`); with the repo cloned inside 22.04
this part is not needed.
`mp2/rrt_planner` is our RRT planner (based on https://github.com/irob-labs-ist/rrt_planner, see its README).
`mp2/COLCON_IGNORE` keeps a colcon build in the Jazzy workspace (which builds `mp1` from this repo)
out of `mp2`: the planner uses the Humble Nav2 interface. The link in the Humble workspace points to
the package directly and is not affected.
Do not run the Jazzy and the Humble simulation at the same time: both distros share the network,
so the ROS nodes would see each other (or use different `ROS_DOMAIN_ID`s).

All commands below are run from the repo root (`robotics/`).

## Structure

```
mp2/
├── config/
│   ├── nav2_params.yaml      Nav2 parameters (Humble structure, TB3 waffle_pi values, changes marked "MP2")
│   ├── teb_controller.yaml   TEB controller, replaces DWB with sim.launch.py controller:=teb
│   ├── mp2.rviz              RViz config (copy of tb3_navigation2.rviz + driven paths)
│   └── test_pairs.yaml       start/goal pairs P1-P3 in turtlebot3_world
├── launch/
│   ├── sim.launch.py             Gazebo Classic + Nav2 (+ ground truth), headless by default, controller:=dwb|teb
│   └── planning_only.launch.py   map_server + planner_server only (no Gazebo), for the planner comparison
├── scripts/
│   ├── metrics.py            path length, minimum clearance, deviation (shared by all tasks)
│   ├── plan_compare.py       part 1: requests paths for all pairs/planners, saves paths + metrics
│   ├── plot_plans.py         part 1: summary table + figure of the planned paths
│   ├── run_costmap_sweep.sh  part 1: varies inflation_radius / cost_scaling_factor, one at a time
│   ├── compare_setups.py     part 1: control run, planning-only setup vs. full simulation
│   ├── record_video.sh       records the RViz window during drive_compare.py (ffmpeg)
│   ├── drive_compare.py      part 2: drives all pairs, records planned/estimated/true path
│   ├── plot_drives.py        part 2: summary table + figure of the driven paths
│   ├── run_rrt_experiments.sh  1b: RRT baseline + one variant per parameter (no Gazebo)
│   └── plot_rrt.py           1b: summary table + figures of the RRT plans
├── rrt_planner/              Task 1b: RRT global planner plugin, linked into ~/ros2_ws/src
├── setup/install_humble.sh   one-time setup of the Humble environment
├── jazzy/                    Jazzy versions of nav2_params.yaml, sim.launch.py, mp2.rviz (archived)
├── data/task1a/              results (plans_<tag>.json / .csv), launch logs of the sweep
├── data/task1b/              RRT results (plans_rrt_<tag>.json / .csv), launch logs
├── data/task1a_jazzy/        the same under Jazzy (archived)
├── figures/                  generated figures
├── videos/                   RViz recordings (task1a_*.mp4; videos/jazzy/: archived Jazzy videos)
└── notes/
    ├── mp2_notes.tex         working notes for the report (facts, parameters, results)
    ├── report-notes.md       all design decisions per task, basis for the report
    └── report_notes.tex      the same as LaTeX (generated from report-notes.md)
```

## Changes in `config/nav2_params.yaml` (vs. `waffle_pi.yaml`)

The Humble file is built from the structure of `nav2_bringup/params/nav2_params.yaml` (Humble) with
the values of the Jazzy `turtlebot3_navigation2/param/waffle_pi.yaml` (same values as our Jazzy runs).
The Humble file shipped by ROBOTIS (`param/humble/waffle_pi.yaml`) still uses Foxy names
(`recoveries_server`) and other values (robot_radius 0.22, max_vel_x 0.22).

| Change | Why |
|---|---|
| `amcl/set_initial_pose: true` at (-2.0, -0.5, 0) | Gazebo spawn pose in `turtlebot3_world`, no clicking in RViz |
| behavior tree `navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml` | plan once (default replans every second); exactly one planned path per run, same conditions for Dijkstra, A* and later RRT |
| second planner `AStar` (NavFn, `use_astar: true`) | compare with `GridBased` = NavFn with `use_astar: false` = **Dijkstra** |
| `use_sim_time: True` in every node section | Humble's bringup only rewrites keys that are present in the file (Jazzy passes `use_sim_time` to every node); without the key the planner and controller run on wall-clock time, the plan gets a wall-clock stamp, the controller fails with "Transform data too old when converting from map to odom" and the robot does not move. `planning_only.launch.py` sets it back to false |
| goal checker `yaw_goal_tolerance: 6.29` (was 0.25) | heading at the goal is not evaluated; with 0.25 the robot rotates at the goal, the controller gets an empty (pruned) plan, the plan-once tree does not replan and the goal fails |

`GridBased` keeps its name because the behavior tree uses it as the default planner id.
Figures and tables show it as "Dijkstra".

## Task 1a

### Simulation (Gazebo + Nav2 + RViz)

```bash
# everything in one terminal, with our parameters (headless: without gui/rviz)
ros2 launch mp2/launch/sim.launch.py gui:=true rviz:=true
# with TEB instead of DWB
ros2 launch mp2/launch/sim.launch.py rviz:=true controller:=teb
```

`sim.launch.py` starts the same world, robot and Nav2 bringup as the tutorial, with two changes to
the robot model (generated at launch time, the installed files are not modified):
- **camera removed**: rendering it in software and bridging the raw images (≈27 MB/s) overloaded the
  CPU under WSL2 (Gazebo at 0.3× real time, laser scans only every few seconds, Nav2 did not start);
  Nav2 does not use the camera,
- **gazebo_ros_p3d plugin**: ground truth on `/ground_truth/odom` (`nav_msgs/Odometry`, 30 Hz,
  Gazebo world frame = map frame). (Jazzy: PosePublisher plugin, `/ground_truth/pose`.)

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

The map is the one shipped with ROBOTIS: `/opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml`.

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

**Control run** (is the planning-only setup equivalent?): the same requests in the full simulation,
robot localized at the spawn pose, then compare the paths:

```bash
ros2 launch mp2/launch/sim.launch.py                                      # terminal 1
python3 mp2/scripts/plan_compare.py --reps 10 --tag fullsim               # terminal 2
python3 mp2/scripts/compare_setups.py
python3 mp2/scripts/plot_plans.py mp2/data/task1a/plans_{baseline,fullsim}.json --name control_run_fullsim
```

### Metrics (`scripts/metrics.py`)

| Metric | Definition | Evaluates |
|---|---|---|
| path length | sum of segment lengths of the planned path [m] | planner |
| success rate, iterations, tree nodes | RRT only (`/rrt/stats`); failed plans run up to `max_num_nodes` iterations | planner (1b) |
| planning time | time reported by `planner_server` (wall clock in the planning-only setup) [ms], reported as median | planner |
| min. clearance | smallest distance of the path to an occupied cell of the static map, from the robot centre, checked along the segments [m] | planner |
| deviation (estimated) | distance of each estimated pose (tf map → base_footprint, 10 Hz) to the nearest point of the planned path that was active at that time (after a replan: the new plan), mean and max [m] | controller (part 2) |
| deviation (true) | same with the Gazebo ground truth | controller + localization (part 2) |
| localization error | estimated vs. true pose (for interpreting the two deviations) [m] | AMCL (part 2) |

### Part 2: driving

Dijkstra (A* gives almost the same paths) with two controllers: DWB and TEB.
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

**Localization check at the start:** after the drive to the start, `drive_compare.py` compares the AMCL
pose with the ground truth. If they differ by more than 0.10 m (`--max-start-error`) or the start was
not reached, the attempt is rejected (stored under `rejected` in the JSON), AMCL is set to the
ground-truth pose and the robot drives to the start again (at most 3 attempts, `--attempts`). The
measured run itself is never corrected. Reason: on the way to the start of P2 the robot can get stuck
at a pillar; during the recoveries AMCL lost ≈1 m and the measured run started at the wrong place
(seen under Jazzy, video `jazzy/task1a_localization_failure_p2.mp4`; under Humble once in the video run,
AMCL 4.2 m off, video `task1a_dijkstra_dwb.mp4`).
The number of rejected attempts is a result of its own (how often localization fails on the way).

Same with TEB (terminal 1: `ros2 launch mp2/launch/sim.launch.py controller:=teb`):
```bash
python3 mp2/scripts/drive_compare.py --tag dijkstra_teb      # and _r2, _r3
python3 mp2/scripts/plot_drives.py mp2/data/task1a/drives_dijkstra_teb{,_r2,_r3}.json --name drives_dijkstra_teb_all
```

During each measured run, `drive_compare.py` publishes the driven paths for RViz (`config/mp2.rviz`):
`/mp2/driven_estimated` (green, tf map → base_footprint) and `/mp2/driven_true` (orange, ground truth).
The planned path is the red `/plan` display ("Global Planner" → "Path").

### Video / screenshots

Videos in `mp2/videos/` (RViz map view, 2x speed, legend). Each video contains six drives: for every
pair the drive to the start (not measured) and the measured run; a label at the top names the section.
RViz shows the robot at the AMCL pose, not at the true pose.
- `task1a_dijkstra_dwb.mp4` (Humble): P1, P2, P3 with Dijkstra + DWB. The first drive to the start of P2
  was rejected by the localization check (AMCL 4.2 m off, particles spread over the map); AMCL is reset
  and the robot drives to the start again.
- `task1a_dijkstra_teb.mp4` (Humble): P1, P2, P3 with Dijkstra + TEB. On P3 the blue local plan of TEB
  repeatedly jumps to a loop between the pillars (alternative route around a pillar); in this run the
  robot stays on the global path (in repetition 3 of the measurements it took the detour).
- `jazzy/task1a_localization_failure_p2.mp4` (Jazzy, before the localization check existed): the robot gets
  stuck at a pillar on the way to the start of P2, AMCL loses ≈1 m during the recoveries and the measured
  run P2 starts at the wrong place

Screenshot: `figures/rviz_p3_dijkstra_dwb.png` (end of P3, from the DWB video).

Record a new video (needs `ffmpeg`; under WSL2 only single windows can be recorded, the whole screen is black):

```bash
ros2 launch mp2/launch/sim.launch.py rviz:=true                                   # terminal 1
bash mp2/scripts/record_video.sh task1a_dijkstra_dwb "Task 1a - Dijkstra + DWB (ROS 2 Humble)" --tag video_dwb   # terminal 2, ~3 min
# TEB: terminal 1 with controller:=teb, then
bash mp2/scripts/record_video.sh task1a_dijkstra_teb "Task 1a - Dijkstra + TEB (ROS 2 Humble)" --tag video_teb
```

The section labels come from the log of `drive_compare.py` (wall-clock time stamps of "driving to
start" / "measured run"). RViz is started with software rendering (`LIBGL_ALWAYS_SOFTWARE=1` in
`sim.launch.py`): with the GPU, the 3D view was empty in the recording under WSLg.

`config/mp2.rviz` is set up for readable videos: costmap displays off, robot model on, top-down view
(x right, y up) centred on the world, planned/driven paths as thick lines. The Gazebo window was not
recorded: in the recording it did not show the robot moving.

### Results and figures

Tables and observations: [notes/mp2_notes.tex](notes/mp2_notes.tex), section "Results".

| Figure | Content |
|---|---|
| `test_pairs_proposal.png` | start/goal pairs P1–P3 |
| `plans_baseline.png` | part 1: Dijkstra vs. A*, default costmap |
| `inflation_cost.png` | inflation cost over distance for the tested parameters |
| `sweep_inflation_dijkstra.png`, `sweep_csf_dijkstra.png` | part 1: inflation radius / cost scaling factor |
| `drives_dijkstra_dwb.png`, `drives_dijkstra_dwb_all.png` | part 2, DWB: planned, estimated and true path |
| `drives_dijkstra_teb.png`, `drives_dijkstra_teb_all.png` | part 2, TEB: the same (repetition 3 / P3: detour between the pillars) |
| `control_run_fullsim.png` | part 1 control run: planning-only vs. full simulation |
| `rviz_p3_dijkstra_dwb.png` | RViz screenshot at the end of P3 (planned, estimated, true path) |
| `rrt_summary.png` | 1b: success rate, length, clearance, iterations per RRT configuration and pair, Dijkstra as reference |
| `rrt_paths_<config>.png` | 1b: all 50 RRT paths per pair of one configuration, Dijkstra for reference |
| `drives_rrt_dwb_all.png`, `drives_rrt_smooth_dwb_all.png`, `drives_rrt_thr128_dwb_all.png` | 1b driving: planned (first and later plans), estimated and true path |
| `rviz_p3_dijkstra_teb_local_plan.png` | TEB on P3: the local plan (blue) loops between the pillars (alternative route) |

## Task 1b

RRT as Nav2 global planner plugin: `rrt_planner/` (algorithm, parameters and topics in its README).
It is loaded as planner `RRT` next to `GridBased` (Dijkstra) and `AStar`.

### Planner comparison (no Gazebo)

Same setup and pairs as Task 1a part 1. Baseline = plain RRT (step 0.2 m, goal bias 0, cost
threshold 253, no smoothing, seed 1); every variant changes one parameter:

| Tag | Change |
|---|---|
| `rrt_base` | - |
| `rrt_step0.5` | `rrt.step` 0.5 m |
| `rrt_bias0.1` | `rrt.goal_bias` 0.1 |
| `rrt_thr128` | `rrt.cost_threshold` 128 (robot centre > 0.29 m from obstacles instead of > 0.15 m) |
| `rrt_smooth` | `rrt.smoothing` true |

```bash
bash mp2/scripts/run_rrt_experiments.sh 50          # 5 configurations x 3 pairs x 50 plans, ~1 min
python3 mp2/scripts/plot_rrt.py mp2/data/task1b/plans_rrt_*.json   # table + figures
```

A single configuration by hand:
```bash
ros2 launch mp2/launch/planning_only.launch.py rrt_goal_bias:=0.1            # terminal 1
python3 mp2/scripts/plan_compare.py --planners RRT --reps 50 --tag rrt_bias0.1 --out mp2/data/task1b
```

### Driving (Gazebo)

RRT + DWB: plain, smoothed and with cost threshold 128, 3 repetitions each, compared with
Dijkstra + DWB (Task 1a part 2). `planner:=rrt` uses a copy of the plan-once behavior tree with
`planner_id="RRT"`:

```bash
ros2 launch mp2/launch/sim.launch.py planner:=rrt                         # terminal 1
python3 mp2/scripts/drive_compare.py --planner RRT --tag rrt_dwb --out mp2/data/task1b   # and _r2, _r3
# smoothed: terminal 1 with planner:=rrt rrt_smoothing:=true, tags rrt_smooth_dwb{,_r2,_r3}
# threshold: terminal 1 with planner:=rrt rrt_cost_threshold:=128, tags rrt_thr128_dwb{,_r2,_r3}
python3 mp2/scripts/plot_drives.py mp2/data/task1b/drives_rrt_dwb{,_r2,_r3}.json --name drives_rrt_dwb_all
```

**Replanning:** RRT paths (cost threshold 253) pass the pillars at the minimum distance. During a
run Nav2 then often plans again (path invalid in the live costmap, controller stuck, or RRT found no
path), and RRT returns a different random route. The deviation is therefore computed against the
plan that was active at each moment (all plans of a run are stored under `plans`), and the number of
plans per run is reported. With `rrt_cost_threshold:=128` no measured run needed a second plan.

Additional metrics for RRT (from `/rrt/stats`): success rate, iterations and tree nodes per plan.
With the fixed seed the runs are reproducible: the same configuration started fresh with the same
request order gives the same paths. Dijkstra reference: `data/task1a/plans_baseline.json`.

## Pitfalls

- **Leftover processes break the next run.** `gzserver` (Jazzy: `gz sim`) sometimes survives Ctrl+C and keeps publishing
  `/clock`, `/scan`, `/tf`; a leftover `static_transform_publisher` (map → base_link from
  `planning_only.launch.py`) splits the TF tree of the simulation (`base_link` gets two parents,
  "Timed out waiting for transform from base_link to odom"). Before starting, check:
  `pgrep -fa "gzserver|ros2 launch|static_transform|component_container"` (should be empty).
- **Humble, first Gazebo start:** in a fresh environment the first `gzserver` start took longer than
  the 30 s that `spawn_entity.py` waits by default, the robot was never spawned and Nav2 waited for
  `odom` forever. `sim.launch.py` waits 120 s (after the first start, Gazebo is up in 3-4 s).
- **TEB fails after 10 s of driving** ("Could not transform the global plan to the frame of the
  controller", "Controller patience exceeded"): the ROS 2 port of TEB (humble-devel) looks up
  map → odom at the time stamp of the plan instead of the latest one (ROS 1 TEB). With the plan-once
  behavior tree the plan is never renewed and falls out of the 10 s tf buffer. Fixed by the one-line
  patch `setup/teb_plan_transform.patch` (needs a rebuild of `teb_local_planner`). Without it, short
  runs (P1) work and every run longer than 10 s fails.
- **Humble, 10 Hz sim clock:** Gazebo Classic publishes `/clock` at 10 Hz, so time stamps of
  different nodes can differ by one step (0.1 s); `drive_compare.py` accepts plans up to 0.5 s older
  than the start of the run. Planning times measured in sim time are 0 (below one clock step), which
  is one more reason why part 1 runs without Gazebo.
- **Humble, sim time:** see `use_sim_time` in the parameter table above. Symptom: the robot does not
  move, "Data time" in the error is the wall clock (~1.8e9 s), "Transform time" the sim time.
- **TEB drives slower than DWB** (≈0.20 vs. ≈0.27 m/s) although both have `max_vel_x: 0.3`:
  `penalty_epsilon: 0.1` (TEB example configuration) makes TEB's velocity penalty start at
  0.3 − 0.1 = 0.2 m/s. Keep in mind when comparing times.
- **RViz recording empty under WSLg (Ubuntu 22.04):** with GPU rendering, the 3D view was captured as
  plain background. `sim.launch.py` starts RViz with `LIBGL_ALWAYS_SOFTWARE=1`.
- Messages that queue up while a node does not spin are delivered later. `drive_compare.py` therefore
  only keeps messages with a time stamp after the start of the run.

- `BasicNavigator.waitUntilNav2Active()` (nav2_simple_commander) publishes an initial pose at (0, 0) if it
  does not receive an `amcl_pose` and thereby resets AMCL. Our scripts only wait for the node they need.
- In a bash script, background jobs ignore SIGINT, so `ros2 launch ... &` cannot be stopped with
  `kill -INT`; `run_costmap_sweep.sh` starts it with `setsid`.
- `robot_radius` in `waffle_pi.yaml` is 0.15 m, the Waffle Pi footprint (281 × 306 mm) has a circumscribed
  radius of about 0.21 m.
