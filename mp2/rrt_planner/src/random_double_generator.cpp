#include <rrt_planner/random_double_generator.h>

namespace rrt_planner {

    void RandomDoubleGenerator::setRange(const double& min, const double& max) {
        min_value_ = min;
        max_value_ = max;
    }

    void RandomDoubleGenerator::setSeed(unsigned int seed) {
        gen_.seed(seed != 0 ? seed : std::random_device{}());
    }

    double RandomDoubleGenerator::generate() {
        // Note: uniform_real_distribution does [start, stop), but we want to do [start, stop].
        // Pass the next largest value instead.
        return std::uniform_real_distribution<> {min_value_, std::nextafter(max_value_, DBL_MAX)}(gen_);
    }
}
