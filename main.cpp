#include <iostream>
#include <vector>
#include <random>
#include "robot.h"

int main(int argc, char *argv[])
{
    constexpr int num_robots = 10;
    constexpr int num_iterations = 1000000;
    constexpr float dt = 0.01f;

    std::vector<Robot> robots;
    robots.reserve(num_robots);
    for (int i = 0; i < num_robots; ++i)
    {
        robots.emplace_back(1.0f);
    }

    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

    double sum = 0.0;
    for (int iter = 0; iter < num_iterations; ++iter)
    {
        for (int i = 0; i < num_robots; ++i)
        {
            float vx = dist(rng);
            float vy = dist(rng);
            float omega = dist(rng);
            robots[i].drive(vx, vy, omega);
            robots[i].update(dt);
            // Accumulate some values to prevent optimization
            sum += robots[i].getTopLeftModule().getSteerOutput();
            sum += robots[i].getTopRightModule().getSteerOutput();
            sum += robots[i].getBottomLeftModule().getSteerOutput();
            sum += robots[i].getBottomRightModule().getSteerOutput();
        }
    }

    std::cout << "Profiling sum: " << sum << std::endl;
    return 0;
}