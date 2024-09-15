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
        template <typename WeightT = uint8_t, typename IntT = mapsize_t>
        NodeGrid::Path get_path(
            const WeightT *weights,
            const Eigen::Vector2<IntT> &wsize,
            const Eigen::Vector2<IntT> &start,
            const Eigen::Vector2<IntT> &goal,
            const WeightT line_of_sight_threshold);

    private:
        template <typename WeightT = uint8_t, typename IntT = mapsize_t>
        NodeGrid::Path backtrace_path(
            Node *_start_node,
            Node *_end_node,
            const WeightT *weights,
            const Eigen::Vector2<IntT> &wsize,
            const WeightT threshold);

        template <typename WeightT = uint8_t, typename IntT = mapsize_t>
        bool line_of_sight(
            const WeightT *weights,
            const Eigen::Vector2<IntT> &wsize,
            const mapsize_t a_idx,
            const mapsize_t b_idx,
            const WeightT threshold);

    private:
        std::vector<Node> grid;
    };

}

#endif // NODE_GRID_H