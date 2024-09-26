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
        resolution = 0.02,
        robot_rad = 0.7;

    const int
        w = std::ceil(arena_w / resolution),
        h = std::ceil(arena_h / resolution),
        rad = std::ceil(robot_rad / resolution);

    ALGORITHM tests = LAZY_THETA_STAR | THETA_STAR;

    std::string add_info = "conno-zepherus";

    multi_trial(trials, w, h, 50, rad, 0.2f, "set_locations.csv", tests, add_info, 10, 10, w - 10, h - 10, 25);

    return 0;
}
