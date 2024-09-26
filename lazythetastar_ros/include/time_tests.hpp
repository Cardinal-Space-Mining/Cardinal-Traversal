#ifndef TIME_TESTS_H
#define TIME_TESTS_H

#include <chrono>
#include <numeric>

#include <fstream>
#include <iostream>

#include "grid_utils.hpp"
#include "node_grid.hpp"
#include "image_proc.hpp"

#include "mapping.hpp"

enum ALGORITHM : int
{
    LAZY_THETA_STAR = 0b001,
    THETA_STAR = 0b010
};

inline ALGORITHM operator|(ALGORITHM a, ALGORITHM b)
{

    return static_cast<ALGORITHM>(static_cast<std::underlying_type_t<ALGORITHM>>(a) | static_cast<std::underlying_type_t<ALGORITHM>>(b));
}

using thetastarGrid = NavMap<int64_t, float>;

double path_length(lazythetastar::NodeGrid::Path &path)
{
    double ret = 0.0;
    for (size_t i = 1; i < path.size(); i++)
    {
        auto p1 = path[i - 1];
        auto p2 = path[i];
        double dx = static_cast<double>(p1.x) - static_cast<double>(p2.x);
        double dy = static_cast<double>(p1.y) - static_cast<double>(p2.y);
        ret += std::sqrt(dx * dx + dy * dy);
    }
    return ret;
}

double path_length(thetastarGrid::Path &path)
{
    double ret = 0.0;
    for (size_t i = 1; i < path.size(); i++)
    {
        auto p1 = path[i - 1];
        auto p2 = path[i];
        double dx = static_cast<double>(p1.x()) - static_cast<double>(p2.x());
        double dy = static_cast<double>(p1.y()) - static_cast<double>(p2.y());
        ret += std::sqrt(dx * dx + dy * dy);
    }
    return ret;
}

void time_one_path(const int w, const int h,
                   const int sx, const int sy,
                   const int gx, const int gy,
                   const int num_obstacles, const int obstacle_radius,
                   const float obstacle_falloff,
                   std::ofstream &output_file,
                   ALGORITHM algos,
                   bool show_im, std::string additional_info)
{
    auto t1 = std::chrono::high_resolution_clock::now();

    // Example occupancy grid (grayscale image)
    std::vector<uint8_t> weights;
    weights.resize(w * h, 0);

    for (int i = 0; i < num_obstacles; i++)
        weights[rand() % (w * h)] = rand() % 127 + 127;

    auto t2 = std::chrono::high_resolution_clock::now();

    lazythetastar::expand(weights.data(), w, h, obstacle_radius, obstacle_falloff);

    auto t3 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> diff1 = t2 - t1; // Map generation time
    std::chrono::duration<double> diff2 = t3 - t2; // Expansion time
    double time1 = diff1.count() * 1E6;
    double time2 = diff2.count() * 1E6;

    if (algos & LAZY_THETA_STAR)
    {
        lazythetastar::NodeGrid grid;

        auto t4 = std::chrono::high_resolution_clock::now();

        auto path = grid.get_path(weights.data(), w, h, 0.f, 0.f, 1.f, sx, sy, gx, gy, 1);

        auto t5 = std::chrono::high_resolution_clock::now();

        double length = path_length(path);
        size_t joints = path.size();

#ifdef DEBUG_TOOLS
        if (show_im)
        {
            lazythetastar::show_path(weights.data(), {w, h}, path);
        }
#endif

        std::chrono::duration<double> diff3 = t5 - t4; // Path time
        double time3 = diff3.count() * 1E6;

        if (additional_info.size() > 0)
            output_file << additional_info << ",";

        output_file << "LAZY_THETA_STAR" << ","
                    << w << "," << h << ","
                    << sx << "," << sy << ","
                    << gx << "," << gy << ","
                    << num_obstacles << "," << obstacle_radius << "," << obstacle_falloff
                    << "," << time1 << "," << time2 << "," << time3 << ","
                    << length << "," << joints << std::endl;
    }
    if (algos & THETA_STAR)
    {

        thetastarGrid grid;

        auto t6 = std::chrono::high_resolution_clock::now();

        auto path = grid.navigate<uint8_t, false>(weights.data(), w, h, 0.f, 0.f, 1.f,
                                                  static_cast<float>(sx), static_cast<float>(sy), static_cast<float>(gx), static_cast<float>(gy), 1);

        auto t7 = std::chrono::high_resolution_clock::now();

        double length = path_length(path);
        size_t joints = path.size();

        std::chrono::duration<double> diff4 = t7 - t6; // Path time
        double time4 = diff4.count() * 1E6;

        if (additional_info.size() > 0)
            output_file << additional_info << ",";

        output_file << "THETA_STAR" << ","
                    << w << "," << h << ","
                    << sx << "," << sy << ","
                    << gx << "," << gy << ","
                    << num_obstacles << "," << obstacle_radius << "," << obstacle_falloff
                    << "," << time1 << "," << time2 << "," << time4 << ","
                    << length << "," << joints << std::endl;
    }
}

void multi_trial(const int trial_num,
                 const int width, const int height,
                 const int max_obstacles, const int obstacle_radius, float obstacle_falloff,
                 std::string filepath,
                 ALGORITHM algorithm,
                 std::string additional_info = "",
                 const int start_x = -1, const int start_y = -1,
                 const int goal_x = -1, const int goal_y = -1,
                 const int number_of_obstacles = -1)
{
    std::ofstream file{filepath, std::ios::app};
    if (!file.is_open())
    {
        printf("Error opening file: %s\n", filepath.c_str());
        return;
    }

    srand(time(NULL));

    printf("Beginning %ld trials\n", trial_num);

    for (long i = 0; i < trial_num; i++)
    {
        int sx = (start_x >= 0) ? start_x : rand() % width;
        int sy = (start_y >= 0) ? start_y : rand() % height;
        int gx = (goal_x >= 0) ? goal_x : rand() % width;
        int gy = (goal_y >= 0) ? goal_y : rand() % height;
        int n_obs = (number_of_obstacles >= 0) ? number_of_obstacles : rand() % max_obstacles;
        time_one_path(width, height, sx, sy, gx, gy, n_obs, obstacle_radius, obstacle_falloff, file, algorithm, trial_num == 1, additional_info);

        printf("\rTrial %ld complete", i + 1);
        std::cout << std::flush;
    }

    printf("\n");
}

#endif // TIME_TESTS_H