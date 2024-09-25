#include "time_tests.hpp"

#include <iostream>
#include <cstdlib>

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <number of trials>\n";
        return 1;
    }

    int trials = std::atoi(argv[1]);

    const double
        arena_w = 7.0,
        arena_h = 5.0,
        resolution = 0.01,
        robot_rad = 0.7;

    const int
        w = std::ceil(arena_w / resolution),
        h = std::ceil(arena_h / resolution),
        rad = std::ceil(robot_rad / resolution);

    multi_trial(trials, w, h, 50, rad, 0.2f, "lts_data.csv");

    return 0;
}
