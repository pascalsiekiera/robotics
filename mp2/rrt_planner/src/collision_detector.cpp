#include "rrt_planner/collision_detector.h"

namespace rrt_planner {

CollisionDetector::CollisionDetector(nav2_costmap_2d::Costmap2DROS * costmap,
        unsigned char cost_threshold, bool allow_unknown) :
        cost_threshold_(cost_threshold), allow_unknown_(allow_unknown) {

    costmap_ = costmap->getCostmap();
}

bool CollisionDetector::inFreeSpace(const double * world_pos) {

    // world coordinates (x, y) [m] -> cell (mx, my); false if the point is outside the costmap
    unsigned int mx, my;
    if (!costmap_->worldToMap(world_pos[0], world_pos[1], mx, my)) {
        return false;
    }

    unsigned char cost = costmap_->getCost(mx, my);

    // unknown cells: free only with allow_unknown_ (as NavFn's allow_unknown), independent of
    // cost_threshold_ (255 would never be below it)
    if (cost == nav2_costmap_2d::NO_INFORMATION) {
        return allow_unknown_;
    }

    // 253 (INSCRIBED_INFLATED_OBSTACLE) and above: the robot circle touches an obstacle;
    // a lower threshold keeps an additional distance to the obstacles (inflation layer)
    return cost < cost_threshold_;
}

bool CollisionDetector::obstacleBetween(const double * point_a, const double * point_b) {

    double dist = computeDistance(point_a, point_b);
    double resolution = costmap_->getResolution();

    if (dist < resolution) {
        return (!inFreeSpace(point_b)) ? true : false;
    } else {
        int num_steps = static_cast<int>(std::floor(dist / resolution));

        double point_i[2];
        for (int n = 1; n <= num_steps; n++) {
            point_i[0] = point_a[0] + n * (point_b[0] - point_a[0]) / num_steps;
            point_i[1] = point_a[1] + n * (point_b[1] - point_a[1]) / num_steps;

            if (!inFreeSpace(point_i)) return true;
        }
        return false;
    }
}

}  // namespace rrt_planner
