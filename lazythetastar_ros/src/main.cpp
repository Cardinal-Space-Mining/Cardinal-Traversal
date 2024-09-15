#include <opencv2/opencv.hpp>
#include <grid_utils.hpp>

int main()
{
    const int
        width = 750,
        height = 500;

    // Example occupancy grid (grayscale image)
    cv::Mat occupancyGrid = cv::Mat::zeros(height, width, CV_8UC1);

    // Mark some points as obstacles
    occupancyGrid.at<uint8_t>(height / 2, width / 2) = 255;

    // Parameters for dilation and blur
    int radius = 50; // Radius for spreading weights

    // Expand the weights in the grid with a falloff
    lazythetastar::GridUtils::expand(occupancyGrid, occupancyGrid, radius, 0.9f);

    cv::rectangle(occupancyGrid, cv::Rect(width / 2 - radius, height / 2 - radius, 2 * radius + 1, 2 * radius + 1), cv::Scalar(100));

    // Show the result
    cv::imshow("Expanded Occupancy Grid", occupancyGrid);
    cv::waitKey(0);

    return 0;
}
