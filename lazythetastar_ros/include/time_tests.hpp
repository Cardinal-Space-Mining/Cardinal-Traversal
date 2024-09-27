#ifndef TIME_TESTS_H
#define TIME_TESTS_H

#include <chrono>
#include <numeric>

#include <fstream>

#include "grid_utils.hpp"
#include "node_grid.hpp"
#include "image_proc.hpp"

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

void time_one_path(const int w, const int h,
                   const int sx, const int sy,
                   const int gx, const int gy,
                   const int num_obstacles, const int obstacle_radius,
                   const float obstacle_falloff,
                   std::ofstream &output_file, std::string extra_data,
                   bool show_im)
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

    lazythetastar::NodeGrid grid;

    auto t4 = std::chrono::high_resolution_clock::now();

    auto path = grid.get_path(weights.data(), w, h, 0.f, 0.f, 1.f, sx, sy, gx, gy, 1);

    auto t5 = std::chrono::high_resolution_clock::now();

    double length = path_length(path);
    size_t joints = path.size();

    if (show_im)
    {
        lazythetastar::show_path(weights.data(), {w, h}, path);
    }
    std::chrono::duration<double> diff1 = t2 - t1; // Map generation time
    std::chrono::duration<double> diff2 = t3 - t2; // Expansion time
    std::chrono::duration<double> diff3 = t5 - t4; // Path time
    double time1 = diff1.count() * 1E6;
    double time2 = diff2.count() * 1E6;
    double time3 = diff3.count() * 1E6;

    if (extra_data.size() > 0)
        output_file << extra_data << ",";

    output_file << w << "," << h << "," << sx << "," << sy << "," << gx << "," << gy << "," << num_obstacles << "," << obstacle_radius << "," << obstacle_falloff << "," << time1 << "," << time2 << "," << time3 << "," << length << "," << joints << std::endl;
}

void multi_trial(const int trial_num,
                 const int width, const int height,
                 const int max_obstacles, const int obstacle_radius, float obstacle_falloff,
                 std::string filepath, std::string extra_data)
{
    std::ofstream file{filepath, std::ios::app};
    if (!file.is_open())
    {
        printf("Error opening file: %s\n", filepath.c_str());
        return;
    }

    srand(time(NULL));

    printf("LAZY THETA*\n");

    for (long i = 0; i < trial_num; i++)
    {
        // int sx = rand() % width;
        // int sy = rand() % height;
        // int gx = rand() % width;
        // int gy = rand() % height;
        // int n_obs = rand() % max_obstacles;

        int sx = 10;
        int sy = 10;
        int gx = width - 10;
        int gy = height - 10;
        int n_obs = 10;
        time_one_path(width, height, sx, sy, gx, gy, n_obs, obstacle_radius, obstacle_falloff, file, extra_data, trial_num == 1);

        printf("\rTrial %ld complete", i + 1);
    }

    printf("\n");
}

void show_spread(float gradient)
{
    const int w = 200, h = 200;

    std::vector<uint8_t> weights;
    weights.resize(w * h, 0);

    weights[(100) + (100) * w] = 255;

    lazythetastar::expand(weights.data(), w, h, 75, gradient);

    lazythetastar::NodeGrid::Path path;
    path.reserve(2);
    path.push_back(lazythetastar::NodeGrid::Vec2m(0, 0));
    path.push_back(lazythetastar::NodeGrid::Vec2m(0, 0));

    lazythetastar::show_path(weights.data(), {w, h}, path);
}

#endif // TIME_TESTS_H