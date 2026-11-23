# MP2 – Decisions (basis for the report)

Group 6 (Maya Aul, Pascal Siekiera). Collected 2026-11-23. Every design decision of Task 1 so far,
sorted by task: what was decided, the alternatives, why, and what it means for the results.
Details, numbers and figures: `notes/mp2_notes.tex`. The same content as LaTeX: `notes/report_notes.tex`.

Marked **(technical)**: implementation choices made while building, not discussed as a design
question; they still belong in the report where they affect the results.

---

## 0. General

### 0.1 ROS 2 Humble only (2026-11-23)
- **Decision:** all results of Task 1 come from ROS 2 Humble (Ubuntu 22.04, Gazebo Classic 11, Nav2 1.1.20).
- **Alternatives:** keep Jazzy (first version of Task 1a); compare Jazzy and Humble.
- **Why:** TEB (controller comparison, requested in the progress meeting 2026-11-11) only exists as a
  ROS 2 port up to Humble; DWB and TEB must run in the same environment; the lab robot runs Humble.
- **Consequence:** Jazzy files archived (`jazzy/`, `data/task1a_jazzy/`, `figures/jazzy/`,
  `videos/jazzy/`), not compared and not reported.

### 0.2 Simulation: ROBOTIS TurtleBot3 simulation, Waffle Pi, `turtlebot3_world`
- **Alternatives:** `nav2_bringup/tb3_simulation_launch.py` (other world, model *waffle*, MPPI); TurtleBot 4.
- **Why:** follows the assignment tutorial; Waffle Pi = robot planned for Task 2; DWB is the default controller.
- **Open:** TurtleBot3 vs. TurtleBot4 for Task 2 not confirmed yet.

### 0.3 Few, interpretable metrics
- **Decision:** path length, planning time, minimum clearance (planner); deviation planned vs. driven
  path (controller). Task 1b adds success rate and iterations/tree nodes (needed for a random planner).
- **Not used:** smoothness, execution time as main metric, AMCL error as metric (topic of MP1;
  reported only to interpret the deviation).
- **Why:** quality over quantity; every number must be explainable.

### 0.4 Plan once instead of replanning every second
- **Decision:** behavior tree `navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml`
  for all quantitative runs (1a and 1b).
- **Alternative:** default tree (replans at 1 Hz).
- **Why:** exactly one planned path per run, so length, clearance and deviation are well defined;
  with RRT a 1 Hz replan would give a new random path every second.
- **Consequence:** replanning still happens if the path becomes invalid or the controller fails
  (relevant for RRT, see 2.10). Weakness found in 1a: with `yaw_goal_tolerance` active the robot
  could rotate at the goal without a path (see 1.7).

### 0.5 Start/goal pairs P1–P3 (`config/test_pairs.yaml`)
- P1 short, straight, between two pillar rows (1.1 m); P2 along the left wall (3.2 m);
  P3 bottom left → top right, straight line through three pillars (4.5 m).
- **Why:** three typical situations (open lane, wall + pillars, obstacles in the direct line);
  the same pairs for 1a and 1b.

### 0.6 Use of AI (DUIA)
- Code of the RRT functions (Task 1b) was written by Claude (AI) at our request and read and
  checked by us; scripts, launch files and analysis also with AI assistance. Must be declared in the DUIA.

---

## 1. Task 1a – Global planning and path following in simulation

### 1.1 Part 1 (planner comparison) without Gazebo
- **Decision:** planning-only setup (`launch/planning_only.launch.py`): only `map_server`,
  `planner_server` and a static transform; global costmap = static + inflation layer; wall-clock time.
- **Alternative:** request the paths in the full simulation.
- **Why:** in the full simulation the global costmap contains the live laser scan placed with the
  AMCL pose → the same request gave different paths or none; planning time with the sim clock is not
  usable (10 Hz clock under Gazebo Classic → always 0 ms).
- **Control run:** the same requests in the full simulation give the same paths (within 2 cm),
  except A*/P3 (other way around the pillar field, almost equally expensive). Answers the question
  from the progress meeting "how was this done without Gazebo?".

### 1.2 Global planners: NavFn Dijkstra vs. NavFn A*
- 20 requests per pair and planner (deterministic, so repetitions only for the planning time).
- **Open for the report:** why A* is not faster than Dijkstra here.

### 1.3 Costmap parameters: one at a time
- **Decision:** vary `inflation_radius` (0.5 / 0.4 / 0.3 / 0.2 m) and `cost_scaling_factor`
  (2 / 5 / 10) one at a time from the baseline, Dijkstra only, 10 requests each.
- **Alternative:** full factorial design.
- **Why:** each effect can be explained on its own (short vs. safe).

### 1.4 Part 2 (driving): Dijkstra with DWB and TEB
- **Decision:** drive Dijkstra only (A* gives almost the same paths) with two controllers, DWB and
  TEB, 3 repetitions each. (2026-11-10 first reduced to Dijkstra + DWB; TEB added after the
  progress meeting 2026-11-11.)
- **Ground truth:** `gazebo_ros_p3d` plugin added to the robot model (`/ground_truth/odom`, 30 Hz).
- **Deviation:** of the estimated path (controller error) and of the true path (controller +
  localization error); localization error reported to interpret the difference.

### 1.5 Localization check at the start – option B (2026-11-23)
- **Decision:** after the drive to the start, compare AMCL with the ground truth; error > 0.10 m or
  start not reached → reject the attempt, reset AMCL to the ground truth, drive to the start again
  (max. 3 attempts). The measured run itself is never corrected.
- **Alternative (option A):** reset AMCL to the ground truth before every measured run.
- **Why:** under Jazzy the robot got stuck at a pillar on the way to P2, AMCL ended up ≈1 m off and
  the measured run started at the wrong place (video `videos/jazzy/task1a_localization_failure_p2.mp4`).
  Option B keeps the measured run a normal run and makes such failures visible.
- **Consequence:** number of rejected attempts is a result of its own (seen again in the Humble DWB
  video run: AMCL 4.2 m off, caught).

### 1.6 TEB: default homotopy class planning kept – option A (2026-11-23)
- **Decision:** `enable_homotopy_class_planning: true` (default).
- **Alternative (B):** switch it off → TEB only follows one route; smoother comparison, but not the
  usual TEB.
- **Why:** the detour in TEB repetition 3 / P3 (robot turns into the pillar gap, oscillates ≈30 s,
  returns) is a real property: TEB may leave the global path, DWB cannot. Visible in the TEB video
  (local plan loops between the pillars).

### 1.7 Further settings in Part 2
- `yaw_goal_tolerance` disabled (6.29 rad): with plan-once, rotating at the goal left DWB with an
  empty plan and the goal was aborted.
- `xy_goal_tolerance` 0.25 m kept → start/end differ from the pair coordinates by up to 0.25 m.
- Camera removed from the robot model (CPU load under WSL2).
- **(technical)** TEB bug fixed (`setup/teb_plan_transform.patch`): the ROS 2 port looked up
  map → odom at the time stamp of the plan; with plan-once every run longer than 10 s failed.
- **(technical)** `use_sim_time` in every section of `nav2_params.yaml` (Humble bringup only
  rewrites existing keys).

### 1.8 TEB speed (open, kept for now)
- TEB drives ≈0.20 m/s, DWB ≈0.27 m/s, both with `max_vel_x` 0.3: `penalty_epsilon` 0.1 (TEB example
  configuration) makes TEB's velocity penalty start at 0.2 m/s.
- **Status:** kept as is ("1a is fine for now"); must be stated in the report. Alternative: tune TEB
  to the same effective speed and rerun TEB (≈10 min). Question for the TA in the presentation.

### 1.9 Videos (2026-11-23)
- **Decision:** re-record under Humble: one video DWB, one TEB (P1–P3, RViz, 2x speed), with a label
  for every section ("drive to the start … (not measured)" / "measured run …") and a legend
  (planned, estimated, true path, local plan).
- **Why:** the old videos did not show which drives were measured (6 drives per video).
- Old Jazzy videos kept in `videos/jazzy/`; the localization-failure video can be shown as the
  reason for the localization check.

### 1.10 Clearance metric along the segments (technical, 2026-11-23)
- Clearance was evaluated only at the path poses; now also on points of segments longer than a cell
  diagonal. Checked: all 1a values unchanged (NavFn has one pose per cell); needed for RRT.

---

## 2. Task 1b – RRT global planner plugin

### 2.1 Who writes the code
- **Decision:** first "together step by step", then (2026-11-23) Claude writes all five functions
  and the extensions, we read and check them, declared in the DUIA.

### 2.2 Where the code lives
- **Decision:** copy of the skeleton in `mp2/rrt_planner` (in our repo), linked into `~/ros2_ws/src`.
- **Alternatives:** fork of the skeleton repo; leave it in the cloned skeleton (not versioned).
- **Why:** versioned with everything else, Pascal gets it with the repo.
- **(technical)** `mp2/COLCON_IGNORE`: the Jazzy workspace builds `mp1` from the same repo and
  would otherwise try to build the Humble plugin.
- READMEs describe our planner, without the skeleton/student instructions.

### 2.3 Algorithm scope: RRT + goal bias + shortcut smoothing
- **Decision:** the required RRT plus two switchable extensions: goal bias (fraction of samples =
  goal) and shortcut smoothing of the finished path. Both off in the baseline.
- **Alternatives:** plain RRT only; additionally RRT*.
- **Why:** the effect of each extension can be measured against the required RRT.

### 2.4 Collision check: cost threshold as a parameter
- **Decision:** a cell is free if its cost < `rrt.cost_threshold` (baseline 253 = inscribed: the
  robot circle does not touch an obstacle); unknown cells (255) and points outside the costmap are
  never free; only the robot centre is checked (robot radius is in the inflation layer).
- **Alternatives:** fixed "< 253"; only cost 0 (impossible: P1 lies ≈0.4 m from the pillars).
- **Consequence:** threshold 128 → robot centre > 0.29 m from obstacles.
- For Task 2: `robot_radius` in the costmap must fit the real robot (≈0.21 m instead of 0.15 m).

### 2.5 Sampling range: whole costmap
- **Decision:** uniform in the whole costmap (origin to origin + size, 19.2 × 19.2 m).
- **Alternatives:** bounding box of the free space (recommended at the time); keep the skeleton
  (±map size around (0, 0)).
- **Consequence:** the free hexagon is only 5.4 % of the map → ≈90 % of the iterations add no node,
  14–22 % of the plans fail after 5000 iterations. A result to discuss in the report.

### 2.6 Path end: goal appended if free
- **Decision:** if the segment from the last node to the goal is free, the goal is appended.
- **Alternative:** skeleton (path ends up to 0.2 m before the goal).
- **Why:** path ends exactly at the goal like Dijkstra → lengths comparable.

### 2.7 Random numbers: seed parameter
- **Decision:** one generator per planner, seeded once with `rrt.seed` (experiments: seed 1);
  0 = random.
- **Alternative:** skeleton (new random seed for every number, not reproducible).
- **Consequence:** reproducible: a second run of the whole experiment gave identical paths for
  all 750 plans.

### 2.8 Experiment design (planning, no Gazebo)
- **Baseline** = plain RRT as in the skeleton: step 0.2 m, goal bias 0, threshold 253, no smoothing.
- **Variants**, one parameter at a time, at most 2 values per parameter: step 0.5 m; goal bias 0.1;
  threshold 128; smoothing on.
- **50 plans** per pair and configuration (750 in total).
- **Metrics:** length, planning time, min. clearance + success rate + iterations/tree nodes.
- **Reference:** Dijkstra from Task 1a.

### 2.9 Driving (Gazebo): RRT + DWB
- **Decision:** RRT + DWB (no TEB), 3 repetitions per variant: baseline and smoothed (2026-11-23),
  plus threshold 128 after the first run showed many replans ("only threshold 128", not combined
  with smoothing, so only one parameter changes).
- **Comparison:** Dijkstra + DWB from Task 1a.
- **(technical)** `sim.launch.py planner:=rrt` uses a copy of the plan-once tree with
  `planner_id="RRT"`.

### 2.10 Evaluation with replanning: against the active plan
- **Decision:** deviation of each sample against the plan that was active at that time; number of
  plans per run reported as a metric.
- **Alternatives:** only runs without replanning (none left for P3); against the first plan
  (measures route changes, up to 3 m).
- **Why:** with RRT a replan returns a different random route; only the active plan measures path
  following.
- **Consequence:** with one plan the result is the same as before (1a values unchanged).

### 2.11 Costmap lock
- **Decision:** the costmap is locked during planning and smoothing (`getMutex`).
- **Why:** in the full simulation the costmap is updated continuously; no relevant drawback
  (planning takes milliseconds).

### 2.12 Technical choices in the plugin
- Sampling range and costmap origin/resolution read before every plan (the costmap gets its final
  size only when the map arrives).
- Plugin publishes on `rrt/plan` instead of `plan` (the planner server already publishes `/plan`;
  every path arrived twice) and `rrt/stats` (success, iterations, nodes, poses) for the metrics.
- Nearest node: linear search; extension: one step of `step` towards the sample (or the sample
  itself if closer).
- Smoothing: greedy shortcutting, then the straight segments are resampled at the costmap
  resolution (5 cm); without it DWB had no pose inside its 3 × 3 m local costmap on long segments.

---

## 3. Task 2 – Real robot (not started)

Decisions still to take:
- TurtleBot3 Waffle Pi or TurtleBot4 (proposal: Waffle Pi, map and AMCL parameters from MP1).
- `robot_radius` ≈0.21 m (or footprint polygon) for the real robot.
- 2a: waypoints (4, as in Figure 1 of the assignment), planner (Dijkstra and/or RRT), controller.
- 2b: grid size (4×4 or 5×5), states/actions/rewards, marker for waypoint 3, number of episodes,
  learning rate / discount / exploration.

---

## 4. Open points to remember for the report

- Why A* is not faster than Dijkstra in NavFn (1a).
- DWB cuts the corner at the start of P3 (20–28 cm): check the critic weights (1a).
- TEB speed / `penalty_epsilon` (1.8): state in the report or rerun.
- Replanning cause in 1b: most likely the path became invalid in the live costmap (not logged by
  Nav2) – state as the most likely explanation, with the evidence (none with threshold 128).
- Video for 1b (RRT + DWB, possibly with the RRT tree in RViz); presentation not updated for 1b.
- DUIA: list where AI was used (code of 1b, scripts, analysis, text).
