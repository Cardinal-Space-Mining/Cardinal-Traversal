#ifndef NODE_GRID_H
#define NODE_GRID_H

#include <vector>
#include <cstdint>
#include <limits>
#include <algorithm>

#include <Eigen/Core>

namespace lazythetastar
{
    class NodeGrid
    {
    public:
        using mapsize_t = uint64_t;
        using fweight_t = float;

        using Vec2m = Eigen::Vector2<mapsize_t>; // represents a location on the map
        using NodeMove = std::tuple<Vec2m, fweight_t>;
        using Path = std::vector<Vec2m>;

    public:
        struct Node
        {
            mapsize_t
                self_idx,
                parent_idx;
            fweight_t
                g,
                h;
        };

        struct NodeCmp
        {
            bool operator()(const Node &a, const Node &b) { return a.g + a.h > b.g + b.h; }
            bool operator()(const Node *a, const Node *b) { return a->g + a->h > b->g + b->h; }
        };

        static constexpr float SQRT_2 = 1.415f;

        inline static const NodeMove MOVES[] = {
            {Vec2m{1, 0}, 1.f},
            {Vec2m{-1, 0}, 1.f},
            {Vec2m{0, 1}, 1.f},
            {Vec2m{0, -1}, 1.f},
            {Vec2m{1, 1}, SQRT_2},
            {Vec2m{1, -1}, SQRT_2},
            {Vec2m{-1, 1}, SQRT_2},
            {Vec2m{-1, -1}, SQRT_2}};
        static constexpr size_t NUM_MOVES = sizeof(MOVES) / sizeof(MOVES[0]);

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

}

#endif // NODE_GRID_H