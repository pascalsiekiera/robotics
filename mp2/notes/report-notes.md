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
- **Task 2:** TurtleBot3 Waffle Pi confirmed (3.1).

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
- For Task 2: `robot_radius` kept at 0.15 m although the real robot reaches ≈0.21 m (3.9).

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

## 3. Task 2a – Waypoint navigation with the real robot

Decisions taken on 2026-11-24.

### 3.1 Robot and computer
- **Decision:** TurtleBot3 Waffle Pi (as in MP1; Ubuntu 22.04 + Humble on the robot). Nav2 and our
  program run on our own laptop (WSL2, Ubuntu 22.04 + Humble) on the `deec-robots` Wi-Fi with
  `ROS_DOMAIN_ID` = robot id (changed on 2026-11-24 from "lab PC"); the lab PC (Humble installed)
  stays the fallback, our code gets there with git.
- **Alternatives:** lab PC as the main computer (first proposal: no network risk).
- **Why:** our environment with all scripts; no copying to the lab PC.
- **Consequence (technical):** WSL2 in the default NAT mode does not let the ROS 2 (DDS) discovery
  reach the robot → WSL "mirrored networking" (`.wslconfig`) + Hyper-V firewall rule; to be tested
  before the lab.

### 3.2 Map: map_2a_merged, unchanged
- **Decision:** `mp1/maps/map_2a_merged` used as it is for AMCL, Nav2 and the Gazebo test world.
- **Alternatives:** `map_2a_new` (final MP1 map); a cleaned copy (unknown cells inside the room set
  to free, wall gaps closed). The dataset map (`turtlebot_datasets/data/map.yaml`) is the 3.8 × 4 m
  motion-capture arena, not the LSDC3 lab.
- **Why:** fewer unknown cells inside the room (6.2 % instead of 15.4 % for `map_2a_new`); a map used
  as mapped needs no editing step to justify.
- **Consequence:** unknown speckles (upper part of the room, around the table) and small wall gaps
  (1–2 cells) remain; handled where they matter (3.3; walls thickened by one cell only in the Gazebo
  world).

### 3.3 RRT: unknown cells count as free (`rrt.allow_unknown`)
- **Decision:** new parameter `rrt.allow_unknown`, default false (behaviour of Task 1b), true for 2a.
- **Alternatives:** unknown stays blocked.
- **Why:** same meaning as NavFn's `allow_unknown`; otherwise every segment through a speckle is
  rejected and RRT avoids or fails in these areas.
- **Consequence:** Task 1b results unchanged (default false).

### 3.4 Test before the lab: Gazebo world from the map
- **Decision:** a Gazebo Classic world generated from `map_2a_merged` (walls from the occupied
  cells); program, waypoints and both variants are tested there first.
- **Alternatives:** only `turtlebot3_world` with placeholder waypoints; no test before the lab.
- **Why:** lab time is short.

### 3.5 Waypoints
- **Decision:** (2.8, 3.5), (5.1, 0.9), (5.2, 3.0), (0.5, 1.4) m in the `map` frame of
  `map_2a_merged`, placed as in Figure 1 of the assignment (1 above the right end of the table,
  2 in the niche bottom right, 3 top right at the wall, 4 left below the table); ≥0.55 m from the
  walls, not in unknown cells. Stored in a YAML file; in the lab check that nothing stands there.
- **Orientation:** yaw = direction to the next waypoint (no turning on the spot at the goal, next leg
  starts forward).
- **Note:** leg 3→4 can pass the table above or below – a likely difference between Dijkstra and RRT.

### 3.6 Program: own NavigateToPose action client (rclpy)
- **Decision:** small Python program with an rclpy `ActionClient` for `NavigateToPose`: send one
  waypoint, wait for the result, log result and time, next waypoint.
- **Alternatives:** `nav2_simple_commander` (`BasicNavigator.goToPose`, also NavigateToPose inside);
  NavigateThroughPoses / waypoint follower (not what the assignment asks for).
- **Why:** shows the action mechanics named in the assignment; short and easy to explain.
- **Failure handling:** an aborted waypoint is tried once more, then skipped (logged).

### 3.7 Start and rounds
- **Decision:** robot placed by eye roughly at waypoint 4, initial pose set in RViz ("2D Pose
  Estimate", laser points on the map walls); one round = 1→2→3→4, ends at the start, next round
  without moving the robot.
- **Alternatives:** fixed start mark on the floor; start anywhere (first leg not comparable).
- **Why:** exact placement is not needed: AMCL only needs an initial guess (≈±30 cm, ±20°), and from
  round 2 on Nav2 itself has brought the robot to waypoint 4.

### 3.8 Variants and repetitions
- **Decision:** Dijkstra + DWB and RRT + DWB, RRT as the baseline (like the skeleton: no goal bias,
  no smoothing); 2 rounds per variant.
- **Alternatives:** RRT with goal bias 0.1 + smoothing (recommended: fewer failed plans and replans);
  only smoothing; additionally TEB; 3 or 5 rounds.
- **Why:** the plain RRT as implemented in 1b on the real robot; 2 rounds save lab time.
- **Consequence:** zig-zag plans and replans are expected (1b: 7 of 9 runs replanned) – part of the
  result; with 2 rounds the spread can hardly be estimated.

### 3.9 Robot radius unchanged (0.15 m)
- **Decision:** `robot_radius` 0.15 m as in Task 1.
- **Alternatives:** 0.22 m (half diagonal of the Waffle Pi ≈0.21 m + margin); footprint polygon.
- **Why:** comparable with Task 1.
- **Risk:** the real corners reach ≈0.21 m from the centre; the inflation (0.5 m) keeps the plan away
  from obstacles, but DWB may pass close at corners (cf. the P3 corner in 1a). Mention in the report.

### 3.10 Metrics and recording
- **Decision:** the metrics of Task 1 per leg: path length (plan and driven), planning time,
  deviation of the estimated path from the active plan, minimum clearance to the map. Estimated
  path = AMCL pose (no ground truth in the lab).
- **Recording:** rosbag (`/amcl_pose`, `/plan`, `/odom`, `/scan`, `/tf`, `/tf_static`, `/local_plan`,
  `/cmd_vel`); all plots and metrics are computed from the bags afterwards.
- **Video (required by the assignment):** RViz video made afterwards from the rosbag (replay +
  screen recording as in 1a); no phone video (in MP1 nobody was interested in it).
- **Consequence:** deviation and clearance include the localisation error of AMCL – state in the
  report.

### 3.11 First lab test (2026-11-24) and goal tolerance 0.10 m
- **Test:** one round Dijkstra + DWB, all 4 waypoints reached (24, 21, 13, 39 s), no recoveries;
  laser points on the map walls (median 5 cm) → localisation good.
- **Observation 1:** leg 3→4 planned around the top of the table (8.1 m instead of ≈5 m below it).
  Likely causes: NavFn with `allow_unknown` gives unknown cells almost obstacle cost (253 instead of
  50 for free cells), the passage below passes unknown speckles of the map; plus an obstacle not
  in the map (chair/person) at the lower right end of the table seen by the laser.
- **Observation 2:** the robot stopped 25 cm before waypoint 4 = `xy_goal_tolerance` 0.25 m.
- **Decision:** `xy_goal_tolerance` 0.10 m for the lab (`lab.launch.py`, launch arg); Task 1 keeps
  0.25 m. Alternatives: keep 0.25 (comparable with Task 1); 0.15 (proposed).
- **Note:** with `yaw_goal_tolerance` 6.29 (Task 1) the yaw sent with the waypoints (3.5) is not
  enforced: the robot arrives with any heading.
- **Open:** keep the long Dijkstra route as a result (proposed) or `track_unknown_space: false` in
  the global costmap.

### 3.12 Wi-Fi delays and transform tolerance 1.0 s (technical)
- **Problem:** two lab runs (12:46, 14:40) failed with "Transform data too old when converting from
  odom to map" and "Controller patience exceeded"; Nav2 then ran its recovery behaviors (spin, back
  up), which looked like random driving. In both runs `/odom` arrived at 5–9 Hz instead of 20 Hz and
  the delay of `/scan`/`/odom` varied up to ≈1 s; AMCL dropped scans, so map → odom was 0.5–0.85 s
  old while DWB allowed 0.5 s. The successful runs had the same setup with 20 Hz odometry.
- **Decision:** all transform tolerances 1.0 s in the lab (`lab.launch.py`: AMCL, bt_navigator,
  DWB, both costmaps, behavior server); Task 1 unchanged.
- **Also:** the clock offset robot – laptop changed during the day (−0.07 s, −0.4 s, −0.05 s).
- **Consequence:** the controller may use a pose up to 1 s old (≤ 0.26 m at full speed); to
  mention in the report as a limitation of running Nav2 off-board over Wi-Fi.
- **Program (technical):** `waypoint_nav.py --planners dijkstra rrt` drives one round per planner
  without restarting Nav2 (behavior tree file in the NavigateToPose goal), so both rounds share the
  same AMCL initialisation; the AMCL start pose is set in RViz (an automatic start pose at waypoint 4
  gave a 1 m / 130° wrong pose because the robot stood elsewhere).

### 3.13 Lab run 3: Dijkstra and RRT in one run (2026-11-24, 15:42)
- **Setup:** `waypoint_nav.py --planners dijkstra rrt`: round 1 Dijkstra, round 2 RRT (baseline,
  `allow_unknown` true), same Nav2/AMCL session, xy tolerance 0.10 m, transform tolerance 1.0 s.
  Bag `dijkstra_rrt_3`, log `waypoints_dijkstra_rrt_20261124_154211.json` (split per round for the
  figures: `..._round1_dijkstra.json`, `..._round2_rrt.json`).
- **Result:** all 8 legs succeeded, final distance to the waypoint ≤ 0.11 m.
  Dijkstra: 23.3 / 23.8 / 13.9 / 33.0 s (93.9 s), first plans 17.4 m in total.
  RRT: 33.5 / 27.2 / 24.0 / 68.9 s (153.6 s), first plans 24.7 m in total; leg 3→4 with 18
  replans and 4 recovery behaviors, the robot got caught on table legs (observed in the lab).
- **Clearance of the plans to the map:** Dijkstra ≥ 0.43 m on every leg; RRT 0.14–0.22 m (first
  plans), down to 0.10 m in the replans of leg 3→4.
- **Explanation:** the RRT baseline accepts every cell with cost < 253 (robot circle of
  `robot_radius` 0.15 m does not touch an obstacle) and does not prefer clearance, NavFn minimises
  the inflation cost and stays in the middle of free space. The real Waffle Pi reaches ≈0.21 m from
  its centre (3.9) → contact with table legs. In 1b the threshold 128 gave ≈0.29 m clearance (2.4).
- **(technical) Recording:** the bag lost many /tf messages (map → odom gaps up to 17 s while
  /amcl_pose was complete); `plot_lab_run.py` and the RViz replay compute map → odom from
  /amcl_pose and /odom instead.
- **Figures/videos:** `figures/task2a_run3_round{1_dijkstra,2_rrt}.png`,
  `videos/task2a_run3_round{1_dijkstra,2_rrt}{,_rviz}.mp4`.

---

## 4. Task 2b – Q-learning (postponed)

Decisions still to take: where the agent learns (Python grid world from the map vs. real robot),
grid size (4×4 or 5×5) and states (cell + heading), meaning of the actions, rewards, role of the
marker at waypoint 3 (unclear in the assignment: the goal is waypoint 4 → ask the professor),
localisation on the robot (AMCL vs. odometry), number of episodes, learning rate / discount /
exploration.

---

## 5. Open points to remember for the report

- Why A* is not faster than Dijkstra in NavFn (1a).
- DWB cuts the corner at the start of P3 (20–28 cm): check the critic weights (1a).
- TEB speed / `penalty_epsilon` (1.8): state in the report or rerun.
- Replanning cause in 1b: most likely the path became invalid in the live costmap (not logged by
  Nav2) – state as the most likely explanation, with the evidence (none with threshold 128).
- Video for 1b (RRT + DWB, possibly with the RRT tree in RViz); presentation not updated for 1b.
- 2a: WSL mirrored networking must be tested before the lab (3.1).
- 2a: robot radius 0.15 m vs. real ≈0.21 m (3.9); waypoints to check in the lab (3.5).
- DUIA: list where AI was used (code of 1b, scripts, analysis, text).
