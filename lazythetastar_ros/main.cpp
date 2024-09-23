#include "grid_utils.hpp"
#include "node_grid.hpp"

#include <opencv2/opencv.hpp>
#include <chrono>

double time_path(const int w, const int h)
{
    // Example occupancy grid (grayscale image)
    cv::Mat occupancyGrid = cv::Mat::zeros(h, w, CV_8UC1);

    // Mark some points as obstacles

    srand(time(NULL));

    for (int i = 0; i < 50; i++)
        occupancyGrid.at<uint8_t>(rand() % h, rand() % w) = rand() % 127 + 127;

    // Parameters for dilation and blur
    int radius = std::max(25, std::min(w, h)) / 25; // Radius for spreading weights

    // Expand the weights in the grid with a falloff
    lazythetastar::GridUtils::expand(occupancyGrid, occupancyGrid, radius, 1.0f);

    lazythetastar::NodeGrid grid;

    auto start = std::chrono::high_resolution_clock::now();

    auto path = grid.get_path(occupancyGrid.data,
                              w, h, 0.f, 0.f, 1.f,
                              10, 10,
                              w - 10, h - 10,
                              1);

    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> diff = end - start;
    return diff.count();
}

int main()
{
    const int
        width = 750,
        height = 500;
    const size_t trials = 10;

    double total_time, min_time, max_time;
    for (size_t i = 0; i < trials; i++)
    {
        double t = time_path(width, height);
        total_time += t;
        min_time = std::min(min_time, t);
        max_time = std::max(max_time, t);
    }

    std::cout << "Grid size: (" << width << ", " << height << ")," << trials << " trials" << std::endl;
    std::cout << "Average time: " << (total_time / trials) << "s" << std::endl;
    std::cout << "Minimum time: " << min_time << "s" << std::endl;
    std::cout << "Maximum time: " << max_time << "s" << std::endl;

    return 0;
}
