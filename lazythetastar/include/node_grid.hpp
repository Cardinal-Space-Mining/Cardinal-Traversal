#ifndef NODE_GRID_H
#define NODE_GRID_H

#include "grid_utils.hpp"

#include <vector>
#include <cstdint>
#include <limits>
#include <algorithm>

#ifdef DEBUG_TOOLS
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#endif

namespace lazythetastar
{
    class NodeGrid
    {
    public:
        using mapsize_t = int64_t;
        using Vec2m = GridUtils::Vector2m;
        using Vec2f = GridUtils::Vector2f;
        using Path = std::vector<Vec2m>;

    public:
        struct Node
        {
            mapsize_t
                self_idx,
                parent_idx;
            float
                g,
                h;
        };
        struct NodeCmp
        {
            bool operator()(const Node &a, const Node &b) { return a.g + a.h > b.g + b.h; }
            bool operator()(const Node *a, const Node *b) { return a->g + a->h > b->g + b->h; }
        };
        static constexpr float SQRT_2 = 1.41421f;

        inline static const Vec2m NBR_MOVES[] = {
            Vec2m{1, 0}, Vec2m{-1, 0}, Vec2m{0, 1}, Vec2m{0, -1},
            Vec2m{1, 1}, Vec2m{1, -1}, Vec2m{-1, 1}, Vec2m{-1, -1}};
        inline static const float NBR_MULTS[]{
            1.f, 1.f, 1.f, 1.f, SQRT_2, SQRT_2, SQRT_2, SQRT_2};

        static constexpr mapsize_t INVALID_IDX = std::numeric_limits<mapsize_t>::max();

    public:
        NodeGrid() = default;
        ~NodeGrid() = default;

        NodeGrid &resize(size_t len)
        {
            size_t i = this->size();
            this->grid.resize(len);

            while (i < len)
            {
                this->grid[i++].self_idx = static_cast<mapsize_t>(i);
                i++;
            }
            return *this;
        }

        inline NodeGrid &clear()
        {
            this->grid.clear();
            return *this;
        }

        inline size_t size() const
        {
            return this->grid.size();
        }

    public:
        NodeGrid::Path get_path(
            const uint8_t *weights,
            const Vec2m &wsize,
            const Vec2m &start,
            const Vec2m &goal,
            const uint8_t line_of_sight_threshold);

        NodeGrid::Path get_path(
            const uint8_t *weights,
            const size_t weight_cell_w, const size_t weight_cell_h,
            const float weight_origin_x, const float weight_origin_y,
            const float weight_resolution,
            const float start_x, const float start_y,
            const float goal_x, const float goal_y,
            const uint8_t line_of_sight_threshold);

    private:
        NodeGrid::Path backtrace_path(
            Node *_start_node,
            Node *_end_node,
            const uint8_t *weights,
            const Vec2m &wsize,
            const uint8_t threshold);

        bool line_of_sight(
            const uint8_t *weights,
            const Vec2m &wsize,
            const mapsize_t a_idx,
            const mapsize_t b_idx,
            const uint8_t threshold);

    private:
        std::vector<Node> grid;
    };

#ifdef DEBUG_TOOLS
    void show_path(uint8_t *weights, const NodeGrid::Vec2m &wsize, NodeGrid::Path &path)
    {
        int w = wsize.x,
            h = wsize.y;
        cv::Mat image_r{h, w, CV_8U, weights};
        cv::Mat image_g = cv::Mat::zeros(h, w, CV_8U);
        cv::Mat image_b = cv::Mat::zeros(h, w, CV_8U);

        for (size_t i = 0; i < path.size() - 1; i++)
        {
            auto n1 = path[i];
            auto n2 = path[i + 1];
            cv::Point p1(n1.x, n1.y);
            cv::Point p2(n2.x, n2.y);
            cv::circle(image_b, p1, 3, {255});
            cv::line(image_g, p1, p2, {255});
        }

        cv::Mat image;
        std::vector<cv::Mat> channels = {image_b, image_g, image_r};
        cv::merge(channels, image);

        cv::imshow("LZ TH*", image);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }
#endif
}

#endif // NODE_GRID_H