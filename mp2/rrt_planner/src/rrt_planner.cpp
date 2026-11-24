#include "rrt_planner/rrt_planner.h"

#include <limits>

namespace rrt_planner {

RRTPlanner::RRTPlanner(nav2_costmap_2d::Costmap2DROS * costmap,
        const rrt_params & params) :
        params_(params), collision_dect_(costmap, params.cost_threshold, params.allow_unknown) {

    costmap_ = costmap->getCostmap();

    // MP2: seeded once; with a fixed seed, the same sequence of plan requests after a start of
    // the planner gives the same paths (0 = random seed for each generator)
    unsigned int s = params_.seed;
    random_double_x.setSeed(s);
    random_double_y.setSeed(s != 0 ? s + 1 : 0);
    random_unit_.setSeed(s != 0 ? s + 2 : 0);
    random_unit_.setRange(0.0, 1.0);

    nodes_.reserve(params_.max_num_nodes);
}

void RRTPlanner::updateSamplingRange() {

    // MP2: the skeleton sampled in [-width, width] x [-height, height] around (0, 0), computed in
    // the constructor. Our map covers [-10, 9.2]^2 m (origin (-10, -10), 19.2 m), so samples are
    // drawn from the whole costmap instead, read before every plan (the global costmap gets its
    // final size and origin only when the map arrives).
    map_width_  = costmap_->getSizeInMetersX();
    map_height_ = costmap_->getSizeInMetersY();
    random_double_x.setRange(costmap_->getOriginX(), costmap_->getOriginX() + map_width_);
    random_double_y.setRange(costmap_->getOriginY(), costmap_->getOriginY() + map_height_);
}

bool RRTPlanner::planPath() {

    // Clear everything before planning
    nodes_.clear();
    num_iterations_ = 0;
    updateSamplingRange();

    // Start node (root of the tree)
    createNewNode(start_, -1);

    double * p_rand, * p_new;
    Node nearest_node;

    for (unsigned int k = 1; k <= static_cast<unsigned int>(params_.max_num_nodes); k++) {

        num_iterations_ = k;
        p_rand      = sampleRandomPoint();
        nearest_node = nodes_[getNearestNodeId(p_rand)];
        p_new       = extendTree(nearest_node.pos, p_rand);

        if (!collision_dect_.obstacleBetween(nearest_node.pos, p_new)) {
            createNewNode(p_new, nearest_node.node_id);
        } else {
            continue;
        }

        if (k > static_cast<unsigned int>(params_.min_num_nodes)) {
            if (computeDistance(p_new, goal_) <= params_.goal_tolerance) {
                // MP2: end exactly at the goal if the last segment is free (the skeleton ended at
                // the last node, up to goal_tolerance before the goal). Not if the node already
                // is the goal (possible with goal bias), that would give a zero-length segment.
                if (computeDistance(p_new, goal_) > 1e-6 &&
                        !collision_dect_.obstacleBetween(p_new, goal_)) {
                    createNewNode(goal_, static_cast<int>(nodes_.size()) - 1);
                }
                return true;
            }
        }
    }

    return false;
}

int RRTPlanner::getNearestNodeId(const double * point) {

    // linear search over all nodes; node_id = index in nodes_ (see createNewNode)
    int nearest_id = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (const Node & node : nodes_) {
        double dist = computeDistance(node.pos, point);
        if (dist < min_dist) {
            min_dist   = dist;
            nearest_id = node.node_id;
        }
    }
    return nearest_id;
}

void RRTPlanner::createNewNode(const double * pos, int parent_node_id) {

    // the id is the index in nodes_, so that nodes_[id] and the parent ids can be used directly
    Node new_node(pos, static_cast<int>(nodes_.size()), parent_node_id);

    nodes_.emplace_back(new_node);
}

double * RRTPlanner::sampleRandomPoint() {

    // goal bias (MP2): with probability goal_bias the sample is the goal itself. With goal_bias 0,
    // random_unit_ is not used, so the sequence of samples is that of plain RRT.
    if (params_.goal_bias > 0.0 && random_unit_.generate() < params_.goal_bias) {
        rand_point_[0] = goal_[0];
        rand_point_[1] = goal_[1];
    } else {
        // uniform in the whole costmap (range set in updateSamplingRange)
        rand_point_[0] = random_double_x.generate();
        rand_point_[1] = random_double_y.generate();
    }

    return rand_point_;
}

double * RRTPlanner::extendTree(const double * point_nearest, const double * point_rand) {

    // one step of length params_.step from the nearest node towards the sample;
    // if the sample is closer than one step, the sample itself
    double dx   = point_rand[0] - point_nearest[0];
    double dy   = point_rand[1] - point_nearest[1];
    double dist = std::hypot(dx, dy);

    if (dist <= params_.step) {
        candidate_point_[0] = point_rand[0];
        candidate_point_[1] = point_rand[1];
    } else {
        candidate_point_[0] = point_nearest[0] + params_.step * dx / dist;
        candidate_point_[1] = point_nearest[1] + params_.step * dy / dist;
    }

    return candidate_point_;
}

const std::vector<Node> & RRTPlanner::getTree() {
    return nodes_;
}

void RRTPlanner::setStart(double * start) {
    start_[0] = start[0];
    start_[1] = start[1];
}

void RRTPlanner::setGoal(double * goal) {
    goal_[0] = goal[0];
    goal_[1] = goal[1];
}

}  // namespace rrt_planner
