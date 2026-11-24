#include "rrt_planner/rrt_planner_ros.h"
#include "pluginlib/class_list_macros.hpp"

// Register this planner as a nav2_core::GlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlannerROS, nav2_core::GlobalPlanner)

namespace rrt_planner {

void RRTPlannerROS::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  auto node = node_.lock();

  // Declare parameters (name-prefixed so multiple planner instances don't clash)
  node->declare_parameter(name_ + ".goal_tolerance", 0.20);
  node->declare_parameter(name_ + ".rrt.step", 0.20);
  node->declare_parameter(name_ + ".rrt.min_num_nodes", 200);
  node->declare_parameter(name_ + ".rrt.max_num_nodes", 5000);
  // MP2 additions (defaults = plain RRT as in the skeleton)
  node->declare_parameter(name_ + ".rrt.goal_bias", 0.0);
  node->declare_parameter(name_ + ".rrt.cost_threshold", 253);  // 253 = INSCRIBED_INFLATED_OBSTACLE
  node->declare_parameter(name_ + ".rrt.smoothing", false);
  node->declare_parameter(name_ + ".rrt.allow_unknown", false);
  node->declare_parameter(name_ + ".rrt.seed", 0);

  params_.goal_tolerance = node->get_parameter(name_ + ".goal_tolerance").as_double();
  params_.step           = node->get_parameter(name_ + ".rrt.step").as_double();
  params_.min_num_nodes  = node->get_parameter(name_ + ".rrt.min_num_nodes").as_int();
  params_.max_num_nodes  = node->get_parameter(name_ + ".rrt.max_num_nodes").as_int();
  params_.goal_bias      = node->get_parameter(name_ + ".rrt.goal_bias").as_double();
  params_.cost_threshold = static_cast<unsigned char>(
    node->get_parameter(name_ + ".rrt.cost_threshold").as_int());
  params_.smoothing      = node->get_parameter(name_ + ".rrt.smoothing").as_bool();
  params_.allow_unknown  = node->get_parameter(name_ + ".rrt.allow_unknown").as_bool();
  params_.seed           = static_cast<unsigned int>(node->get_parameter(name_ + ".rrt.seed").as_int());

  // Publisher for the planned path (visualised in RViz2 / Foxglove).
  // MP2: "rrt/plan" instead of "plan": the planner server itself publishes every plan on "plan",
  // with the plugin also publishing there, every path arrived twice.
  plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("rrt/plan", 1);
  stats_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("rrt/stats", 10);

  planner_ = std::make_shared<RRTPlanner>(costmap_ros.get(), params_);
  initialized_ = true;

  RCLCPP_INFO(node->get_logger(),
    "[RRTPlannerROS] Configured with goal_tolerance=%.2f, step=%.2f, "
    "min_nodes=%d, max_nodes=%d, goal_bias=%.2f, cost_threshold=%d, smoothing=%d, "
    "allow_unknown=%d, seed=%u",
    params_.goal_tolerance, params_.step,
    params_.min_num_nodes, params_.max_num_nodes, params_.goal_bias,
    static_cast<int>(params_.cost_threshold), static_cast<int>(params_.smoothing),
    static_cast<int>(params_.allow_unknown), params_.seed);
}

void RRTPlannerROS::cleanup()
{
  plan_pub_.reset();
  stats_pub_.reset();
  planner_.reset();
  initialized_ = false;
}

void RRTPlannerROS::activate()
{
  plan_pub_->on_activate();
  stats_pub_->on_activate();
}

void RRTPlannerROS::deactivate()
{
  plan_pub_->on_deactivate();
  stats_pub_->on_deactivate();
}

nav_msgs::msg::Path RRTPlannerROS::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;

  auto node = node_.lock();

  if (!initialized_) {
    RCLCPP_ERROR(node->get_logger(),
      "[RRTPlannerROS] Planner not initialized — call configure() first.");
    return path;
  }

  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(node->get_logger(),
      "[RRTPlannerROS] Start pose is in frame '%s', expected '%s'.",
      start.header.frame_id.c_str(), global_frame_.c_str());
    return path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_ERROR(node->get_logger(),
      "[RRTPlannerROS] Goal pose is in frame '%s', expected '%s'.",
      goal.header.frame_id.c_str(), global_frame_.c_str());
    return path;
  }

  double world_start[2]{start.pose.position.x, start.pose.position.y};
  double world_goal[2]{goal.pose.position.x,  goal.pose.position.y};

  unsigned int map_x, map_y;
  if (!costmap_->worldToMap(world_start[0], world_start[1], map_x, map_y)) {
    RCLCPP_WARN(node->get_logger(),
      "[RRTPlannerROS] Start position (%.2f, %.2f) is off the costmap.",
      world_start[0], world_start[1]);
    return path;
  }
  if (!costmap_->worldToMap(world_goal[0], world_goal[1], map_x, map_y)) {
    RCLCPP_WARN(node->get_logger(),
      "[RRTPlannerROS] Goal position (%.2f, %.2f) is off the costmap.",
      world_goal[0], world_goal[1]);
    return path;
  }

  RCLCPP_INFO(node->get_logger(),
    "[RRTPlannerROS] Planning from (%.2f, %.2f) to (%.2f, %.2f)",
    world_start[0], world_start[1], world_goal[0], world_goal[1]);

  planner_->setStart(world_start);
  planner_->setGoal(world_goal);

  // MP2: lock the costmap while planning and smoothing, so that the costmap thread does not
  // change it in between (in the full simulation the obstacle layers are updated continuously)
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  if (!planner_->planPath()) {
    RCLCPP_WARN(node->get_logger(), "[RRTPlannerROS] Failed to find a path.");
    publishStats(false, 0);
    return path;
  }

  // Build the path message by walking back from the last node to the root
  rrt_tree_ = planner_->getTree();
  path.header.stamp = node->now();

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = path.header;
  pose_stamped.pose.orientation = goal.pose.orientation;

  int current_id = static_cast<int>(rrt_tree_.size()) - 1;
  int prev_id;
  tf2::Quaternion quat_tf;
  double dy, dx, yaw;

  while (current_id != 0) {
    pose_stamped.pose.position.x = rrt_tree_[current_id].pos[0];
    pose_stamped.pose.position.y = rrt_tree_[current_id].pos[1];
    pose_stamped.pose.position.z = 0.0;
    path.poses.push_back(pose_stamped);

    prev_id    = current_id;
    current_id = rrt_tree_[current_id].parent_id;

    // Compute heading from current node toward its parent
    dy  = rrt_tree_[prev_id].pos[1] - rrt_tree_[current_id].pos[1];
    dx  = rrt_tree_[prev_id].pos[0] - rrt_tree_[current_id].pos[0];
    yaw = std::atan2(dy, dx);
    quat_tf.setRPY(0.0, 0.0, yaw);
    pose_stamped.pose.orientation = tf2::toMsg(quat_tf);
  }

  // Add start pose
  pose_stamped.pose.position.x = rrt_tree_[0].pos[0];
  pose_stamped.pose.position.y = rrt_tree_[0].pos[1];
  pose_stamped.pose.position.z = 0.0;
  pose_stamped.pose.orientation = start.pose.orientation;
  path.poses.push_back(pose_stamped);

  // Reverse so the path runs start → goal
  std::reverse(path.poses.begin(), path.poses.end());

  if (params_.smoothing) {
    smoothPath(path);
  }
  lock.unlock();

  publishStats(true, path.poses.size());

  plan_pub_->publish(path);
  return path;
}

void RRTPlannerROS::smoothPath(nav_msgs::msg::Path & path)
{
  // MP2: greedy shortcutting. From the current pose, jump to the farthest later pose that can be
  // reached on a straight free segment; repeat from there. Start and goal pose are kept.
  if (path.poses.size() <= 2) {
    return;
  }
  const auto & poses = path.poses;
  std::vector<geometry_msgs::msg::PoseStamped> smooth{poses.front()};
  size_t i = 0;
  while (i < poses.size() - 1) {
    double a[2]{poses[i].pose.position.x, poses[i].pose.position.y};
    size_t j = poses.size() - 1;
    while (j > i + 1) {
      double b[2]{poses[j].pose.position.x, poses[j].pose.position.y};
      if (planner_->segmentFree(a, b)) {
        break;
      }
      j--;
    }
    smooth.push_back(poses[j]);  // j = i + 1 is always reachable (edge of the tree)
    i = j;
  }

  // Resample the straight segments at the costmap resolution (as NavFn paths): the controller
  // only uses the part of the path inside its local costmap (3 x 3 m); a segment of a few metres
  // with no pose in between would leave it without a path. Heading of the inserted and inner
  // poses = direction of the segment; the first and last pose keep the start and goal orientation.
  double resolution = costmap_->getResolution();
  std::vector<geometry_msgs::msg::PoseStamped> dense{smooth.front()};
  tf2::Quaternion quat_tf;
  for (size_t k = 1; k < smooth.size(); k++) {
    const auto & a = smooth[k - 1].pose.position;
    const auto & b = smooth[k].pose.position;
    double dx = b.x - a.x, dy = b.y - a.y;
    quat_tf.setRPY(0.0, 0.0, std::atan2(dy, dx));
    int n = std::max(1, static_cast<int>(std::ceil(std::hypot(dx, dy) / resolution)));
    for (int m = 1; m <= n; m++) {
      geometry_msgs::msg::PoseStamped pose = smooth[k];
      pose.pose.position.x = a.x + dx * m / n;
      pose.pose.position.y = a.y + dy * m / n;
      if (!(k + 1 == smooth.size() && m == n)) {
        pose.pose.orientation = tf2::toMsg(quat_tf);
      }
      dense.push_back(pose);
    }
  }
  path.poses = dense;
}

void RRTPlannerROS::publishStats(bool success, size_t path_poses)
{
  std_msgs::msg::Float64MultiArray stats;
  stats.data = {success ? 1.0 : 0.0, static_cast<double>(planner_->getNumIterations()),
    static_cast<double>(planner_->getTree().size()), static_cast<double>(path_poses)};
  stats_pub_->publish(stats);
}

}  // namespace rrt_planner
