#ifndef GRID_UTILS_H
#define GRID_UTILS_H

#include <Eigen/Core>
#include <opencv2/imgproc.hpp>

namespace lazythetastar
{
    /** Generic grid helpers */
    namespace GridUtils
    {
        /** Align a point to a box grid of the given resolution and offset origin. Result may be negative if lower than current offset. */
        template <typename IntT = int, typename FloatT = float>
        inline static Eigen::Vector2<IntT> gridAlign(FloatT x, FloatT y, const Eigen::Vector2<FloatT> &off, FloatT res)
        {
            return Eigen::Vector2<IntT>{
                static_cast<IntT>(std::floor((x - off.x()) / res)), // always floor since grid cells are indexed by their "bottom left" corner's raw position
                static_cast<IntT>(std::floor((y - off.y()) / res))};
        }
        template <typename IntT = int, typename FloatT = float>
        inline static Eigen::Vector2<IntT> gridAlign(const Eigen::Vector4<FloatT> &pt, const Eigen::Vector2<FloatT> &off, FloatT res)
        {
            return gridAlign<IntT, FloatT>(pt.x(), pt.y(), off, res);
        }

        /** Get a raw buffer idx from a 2d index and buffer size (templated on major-order) */
        template <typename IntT = int>
        inline static int64_t gridIdx(const IntT x, const IntT y, const Eigen::Vector2<IntT> &size)
        {
            return static_cast<int64_t>(y) * size.x() + x; // y-major = "contiguous blocks along [parallel to] x-axis" --> idx = (y * xmax) + x
        }

        template <typename IntT = int>
        inline static int64_t gridIdx(const Eigen::Vector2<IntT> &loc, const Eigen::Vector2<IntT> &size)
        {
            return gridIdx<IntT>(loc.x(), loc.y(), size);
        }

        /** Get the 2d location corresponding to a raw buffer idx for the provded grid size (templated on major-order) */
        template <typename IntT = int>
        inline static Eigen::Vector2<IntT> gridLoc(const size_t idx, const Eigen::Vector2<IntT> &size)
        {
            return Eigen::Vector2<IntT>{// y-major = "contiguous blocks along [parallel to] x-axis" --> x = idx % xmax, y = idx / xmax
                                        static_cast<IntT>(idx % size.x()),
                                        static_cast<IntT>(idx / size.x())};
        }

        /** Check if v is GEQ than min and LESS than max */
        template <typename IntT = int>
        inline static bool inRange(const Eigen::Vector2<IntT> &v, const Eigen::Vector2<IntT> &min, const Eigen::Vector2<IntT> &max)
        {
            return (
                min.x() <= v.x() && v.x() < max.x() &&
                min.y() <= v.y() && v.y() < max.y());
        }

        void expand(cv::Mat &src, cv::Mat &dst, int radius, float gradientFalloff)
        {
            int diameter = 2 * radius + 1;
            int blurSize = radius - static_cast<int>(gradientFalloff * radius);
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
    };

}

#endif // GRID_UTILS_H