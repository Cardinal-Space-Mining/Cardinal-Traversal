#ifndef IMAGE_PROC_H
#define IMAGE_PROC_H

#include <opencv2/imgproc.hpp>

namespace lazythetastar
{

    void expand(cv::Mat &src, cv::Mat &dst, int radius, float falloffSmoothness)
    {
        int diameter = 2 * radius + 1;
        int blurSize = radius - static_cast<int>((1.0 - falloffSmoothness) * radius);
        int dilateSize = diameter - blurSize;
        // Step 1: Dilate the occupancy grid
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilateSize, dilateSize));
        cv::Mat dilatedGrid;
        cv::dilate(src, dilatedGrid, kernel);

        if (blurSize > 0)
        {
            cv::Mat blurredGrid;
            cv::GaussianBlur(dilatedGrid, blurredGrid, cv::Size(blurSize | 1, blurSize | 1), 0.0, 0.0); // blur size must be odd

            double temp, maxValBefore, maxValAfter;
            cv::minMaxLoc(dilatedGrid, &temp, &maxValBefore);
            cv::minMaxLoc(blurredGrid, &temp, &maxValAfter);

            if (maxValAfter > 0)
            {
                blurredGrid.convertTo(blurredGrid, -1, maxValBefore / maxValAfter);
            }

            dst = blurredGrid;
        }
        else
        {
            // Don't blur if there is no falloff
            dst = dilatedGrid;
        }
    }

    void expand(uint8_t *data, int width, int height, int radius, float falloffSmoothness)
    {
        cv::Mat mat(height, width, CV_8UC1, data);

        expand(mat, mat, radius, falloffSmoothness);

        std::memcpy(data, mat.data, mat.total() * mat.elemSize());
    }
}

#endif // IMAGE_PROC_H