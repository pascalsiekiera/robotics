#ifndef _RTT_PLANNER_RANDOM_DOUBLE_GENERATOR_H
#define _RTT_PLANNER_RANDOM_DOUBLE_GENERATOR_H

#include <random>
#include <cfloat>

namespace rrt_planner {

    class RandomDoubleGenerator {

        public:
            RandomDoubleGenerator() = default;

            void setRange(const double& min, const double& max);

            /**
             * @brief   Seed the generator (MP2). 0 = seed from std::random_device.
             *          With a fixed seed, the same sequence of plan requests gives the same paths.
             */
            void setSeed(unsigned int seed);

            double generate();

        private:
            // MP2: one engine per generator, seeded once. The skeleton created a new engine from
            // std::random_device for every number (not reproducible, and slow).
            std::mt19937 gen_{std::random_device{}()};
            double min_value_{-1.0};
            double max_value_{1.0};
    };
};

#endif // _RTT_PLANNER_RANDOM_DOUBLE_GENERATOR_H
