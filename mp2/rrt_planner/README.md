# rrt_planner — RRT global planner for Nav2 (ROS 2 Humble)

RRT (Rapidly-exploring Random Tree) as a global planner plugin for Nav2 (`nav2_core::GlobalPlanner`),
MP2 Task 1b, group 6. Based on https://github.com/irob-labs-ist/rrt_planner (commit 3fa2102,
BSD-3-Clause, see `LICENSE`). Linked into `~/ros2_ws/src`, see `mp2/README.md`.

## Algorithm

1. The tree starts at the start pose.
2. Every iteration: draw a sample (uniform in the whole costmap; with probability `rrt.goal_bias`
   the goal itself), find the nearest tree node (Euclidean distance), and go one step of `rrt.step`
   from it towards the sample (the sample itself if it is closer).
3. If the straight segment from the nearest node to the new point is free, the point becomes a node.
4. After `rrt.min_num_nodes` iterations: once a new node is within `goal_tolerance` of the goal, the
   goal is appended (if the last segment is free) and the path is read back from the goal to the root.
5. Optional (`rrt.smoothing`): shortcut smoothing — from the current pose jump to the farthest later
   pose that can be reached on a free straight segment, repeat from there; the straight segments are
   then resampled at the costmap resolution (a controller only sees the part of the path inside its
   local costmap).

No path after `rrt.max_num_nodes` iterations = failure.

**Collision check** (`CollisionDetector`): a point is free if its cell in the global costmap has a
cost below `rrt.cost_threshold`; points outside the costmap and unknown cells (255) are never free.
Only the robot centre is checked, the robot radius is contained in the inflation layer. A segment is
checked at the costmap resolution (`obstacleBetween`). The costmap is locked while planning.

## Parameters

Under the planner name in the `planner_server` parameters (here `RRT.`, see `mp2/config/nav2_params.yaml`):

| Parameter | Default | Description |
|---|---|---|
| `goal_tolerance` | 0.20 m | a node this close to the goal ends the search |
| `rrt.step` | 0.20 m | extension step |
| `rrt.min_num_nodes` | 200 | iterations before the goal is checked |
| `rrt.max_num_nodes` | 5000 | iterations before giving up |
| `rrt.goal_bias` | 0.0 | fraction of samples that are the goal (0 = plain RRT) |
| `rrt.cost_threshold` | 253 | cell free if cost < this (253 = inscribed: the robot circle does not touch an obstacle) |
| `rrt.smoothing` | false | shortcut smoothing of the finished path |
| `rrt.seed` | 0 | random seed; fixed seed = the same sequence of requests gives the same paths, 0 = random |

## Topics

| Topic | Type | Description |
|---|---|---|
| `/plan` | `nav_msgs/msg/Path` | planned path (published by the planner server) |
| `rrt/plan` | `nav_msgs/msg/Path` | the same path, published by the plugin |
| `rrt/stats` | `std_msgs/msg/Float64MultiArray` | after every plan: [success (0/1), iterations, tree nodes, path poses] |

## Build and use

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select rrt_planner
```

In `mp2/config/nav2_params.yaml` the plugin is loaded as planner `RRT` next to `GridBased` (Dijkstra)
and `AStar`; a request selects it with `planner_id: RRT` (e.g. `mp2/scripts/plan_compare.py --planners RRT`).

## Changes compared to the original package

Besides the five planner functions and the parameters/extensions above:

- random numbers: one engine per generator, seeded once (originally a new engine from
  `std::random_device` for every number)
- sampling range read from the costmap before every plan (originally `[-width, width] x [-height, height]`
  around (0, 0), computed when the plugin was configured, possibly before the map arrived)
- `CollisionDetector` reads resolution and origin from the costmap when needed (same reason)
- the plugin publishes on `rrt/plan` instead of `plan` (the planner server already publishes every
  plan on `plan`, so each path arrived twice)
- `rrt/stats` added; `launch/rrt_turtlebot.launch.py` and `config/rrt_planner*.yaml` are unchanged
  from the original package and not used by the mp2 scripts
