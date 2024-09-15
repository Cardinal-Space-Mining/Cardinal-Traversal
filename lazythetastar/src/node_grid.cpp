#include "node_grid.hpp"
#include "grid_utils.hpp"

#include <queue>

namespace lazythetastar
{
    template <typename WeightT, typename IntT>
    inline NodeGrid::Path NodeGrid::get_path(
        const WeightT *weights,
        const Eigen::Vector2<IntT> &wsize,
        const Eigen::Vector2<IntT> &start,
        const Eigen::Vector2<IntT> &goal,
        const WeightT line_of_sight_threshold)
    {
        const int64_t
            _area = static_cast<int64_t>(wsize.x()) * wsize.y(),
            _start_idx = GridUtils::gridIdx<IntT>(start, wsize),
            _goal_idx = GridUtils::gridIdx<IntT>(goal, wsize);

        static const Eigen::Vector2<IntT>
            _zero = Eigen::Vector2<IntT>::Zero();

        if (
            !GridUtils::inRange<IntT>(start, _zero, wsize) ||
            !GridUtils::inRange<IntT>(goal, _zero, wsize) ||
            (_start_idx == _goal_idx))
        {
            return NodeGrid::Path{}; // Return empty path
        }

        this->resize(_area);
        for (size_t i = 0; i < this->grid.size(); i++)
        {
            this->grid[i].h = static_cast<fweight_t>(
                (goal - GridUtils::gridLoc<IntT>(i, wsize)).norm());
            this->grid[i].g = std::numeric_limits<fweight_t>::infinity();
            this->grid[i].parent_idx = NodeGrid::INVALID_IDX;
        }

        Node &_start_node = this->grid[_start_idx];
        _start_node.g = static_cast<fweight_t>(0);

        std::vector<Node *> queue_backer;
        queue_backer.reserve(static_cast<size_t>(_area));

        std::priority_queue<Node *,
                            std::vector<Node *>,
                            NodeCmp>
            queue{NodeCmp{}, std::move(queue_backer)};
        queue.push(&_start_node);

        while (!queue.empty())
        {
            Node &_node = *queue.top();
            queue.pop();

            if (_node.self_idx == _goal_idx)
            {
                return backtrace_path(&_start_node, &_node,
                                      weights, wsize,
                                      line_of_sight_threshold);
            }

            // else
            const Eigen::Vector2<IntT>
                _loc = GridUtils::gridLoc<IntT>(_node.self_idx, wsize);

            for (size_t i = 0; i < NUM_MOVES; i++)
            {
                const Eigen::Vector2<IntT> _neighbor_loc = _loc + std::get<0>(MOVES[i]).template cast<IntT>();
                if (!GridUtils::inRange<IntT>(_neighbor_loc, _zero, wsize))
                    continue;

                const int64_t _idx = GridUtils::gridIdx<IntT>(_neighbor_loc, wsize);
                Node &_neighbor = this->grid[_idx];

                const fweight_t tentative_g = _node.g + (weights[_node.self_idx] + weights[_neighbor.self_idx]) * 0.5 * std::get<1>(MOVES[i]);

                if (tentative_g < _neighbor.g)
                {
                    _neighbor.parent_idx = _node.self_idx;
                    _neighbor.g = tentative_g;
                    queue.push(&_neighbor);
                }
            }
        }
        // Should never reach here, but handled gracefully
        return Path{};
    }

    template <typename WeightT, typename IntT>
    inline NodeGrid::Path NodeGrid::backtrace_path(
        Node *_start_node, Node *_end_node,
        const WeightT *weights, const Eigen::Vector2<IntT> &wsize,
        const WeightT threshold)
    {
        Path result;

        result.push_back(GridUtils::gridLoc<IntT>(_start_node->self_idx, wsize).template cast<mapsize_t>());

        Node *_current_node = _start_node;
        while (_current_node->self_idx != _end_node->self_idx)
        {
            Node *_itr_node = _end_node;
            while (!(
                line_of_sight(weights, wsize, _current_node->self_idx, _itr_node->self_idx, threshold) ||
                // If A* path goes through weight above threshold, all LOS checks fail, so just go to the neighbor node
                _itr_node->parent_idx == _current_node->self_idx))
            {
                if (_itr_node->parent_idx == INVALID_IDX)
                {
                    // something has gone terribly wrong
                    // return empty path and cry
                    return Path{};
                }
                _itr_node = &this->grid[_itr_node->parent_idx];
            }
            result.push_back(GridUtils::gridLoc<IntT>(_itr_node->self_idx, wsize).template cast<mapsize_t>());
            _current_node = _itr_node;
        }

        return result;
    }

    template <typename WeightT, typename IntT>
    inline bool NodeGrid::line_of_sight(
        const WeightT *weights, const Eigen::Vector2<IntT> &wsize,
        const mapsize_t a_idx, const mapsize_t b_idx,
        const WeightT threshold)
    {
        Eigen::Vector2<IntT>
            a_loc = GridUtils::gridLoc<IntT>(a_idx, wsize),
            b_loc = GridUtils::gridLoc<IntT>(b_idx, wsize);

        IntT
            x1 = a_loc.x(),
            y1 = a_loc.y(),
            x2 = b_loc.x(),
            y2 = b_loc.y(),

            dx = std::abs(x2 - x1),
            dy = std::abs(y2 - y1),
            sx = (x1 < x2) ? 1 : -1,
            sy = (y1 < y2) ? 1 : -1,
            err = dx - dy;

        while (true)
        {
            mapsize_t idx = GridUtils::gridIdx<IntT>(Eigen::Vector2<IntT>(x1, y1), wsize);
            if (weights[idx] > threshold)
                return false;

            if (x1 == x2 && y1 == y2)
                return true;

            IntT e2 = err * 2;
            if (e2 > -dy)
            {
                err -= dy;
                x1 += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                y1 += sy;
            }
        }
    }
}