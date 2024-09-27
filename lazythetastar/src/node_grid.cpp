#include "node_grid.hpp"
#include "grid_utils.hpp"

#include <queue>
#include <unordered_set>
#include <cmath>

namespace lazythetastar
{
    NodeGrid::Path NodeGrid::get_path(
        const uint8_t *weights,
        const size_t weight_cell_w, const size_t weight_cell_h,
        const float weight_origin_x, const float weight_origin_y,
        const float weight_resolution,
        const float start_x, const float start_y,
        const float goal_x, const float goal_y,
        const uint8_t line_of_sight_threshold)
    {
        Vec2m _wsize(
            static_cast<int64_t>(weight_cell_w),
            static_cast<int64_t>(weight_cell_h));
        Vec2f _worigin(weight_origin_x, weight_origin_y);

        Vec2m _start = GridUtils::gridAlign(start_x, start_y, _worigin, weight_resolution);
        Vec2m _goal = GridUtils::gridAlign(goal_x, goal_y, _worigin, weight_resolution);

        return get_path(weights, _wsize, _start, _goal, line_of_sight_threshold);
    }

    NodeGrid::Path
    NodeGrid::get_path(
        const uint8_t *weights,
        const Vec2m &wsize,
        const Vec2m &start,
        const Vec2m &goal,
        const uint8_t line_of_sight_threshold)
    {
        const int64_t
            _area = wsize.x * wsize.y,
            _start_idx = GridUtils::gridIdx(start, wsize),
            _goal_idx = GridUtils::gridIdx(goal, wsize);

        static const Vec2m
            _zero = Vec2m{0, 0};

        if (
            !GridUtils::inRange(start, _zero, wsize) ||
            !GridUtils::inRange(goal, _zero, wsize) ||
            (_start_idx == _goal_idx))
        {
            return NodeGrid::Path{}; // Return empty path
        }

        this->resize(_area);
        for (size_t i = 0; i < _area; i++)
        {
            float dx = static_cast<float>(goal.x) - (static_cast<float>(i % wsize.x));
            float dy = static_cast<float>(goal.y) - (static_cast<float>(i / wsize.x));
            this->grid[i].h = std::sqrt(dx * dx + dy * dy);
            this->grid[i].g = std::numeric_limits<float>::infinity();
            this->grid[i].parent_idx = NodeGrid::INVALID_IDX;
            this->grid[i].self_idx = i;
        }

        Node &_start_node = this->grid[_start_idx];
        _start_node.g = 0.f;

        std::deque<Node *> queue_backer;

        std::priority_queue<Node *, std::deque<Node *>, NodeCmp> queue;
        queue.push(&_start_node);

        std::unordered_set<mapsize_t> visited_node_idxs;
        visited_node_idxs.reserve(static_cast<size_t>(_area));

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

            const Vec2m _loc = GridUtils::gridLoc(_node.self_idx, wsize);

            // Add a constant 1.f so no path from a node to another is free, it is at least 1
            // (this also makes the heuritic easily admissible+monotonic w/ euclidean distance)
            const float _node_weightf = static_cast<float>(weights[_node.self_idx]) + 1.f;

            for (size_t i = 0; i < 8; i++)
            {
                const Vec2m _offset = NBR_MOVES[i];
                const Vec2m _neighbor_loc{_loc.x + _offset.x, _loc.y + _offset.y};
                if (!GridUtils::inRange(_neighbor_loc, _zero, wsize))
                    continue;

                const int64_t _idx = GridUtils::gridIdx(_neighbor_loc, wsize);
                Node &_neighbor = this->grid[_idx];

                const float tentative_g =
                    _node.g + (_node_weightf + static_cast<float>(weights[_neighbor.self_idx])) * NBR_MULTS[i];

                if (tentative_g < _neighbor.g)
                {
                    _neighbor.parent_idx = _node.self_idx;
                    _neighbor.g = tentative_g;
                    queue.push(&_neighbor);
                }
            }

            // Check if neighbor has already been visited, if not, add to queue and closed set
            if (visited_node_idxs.find(_node.self_idx) == visited_node_idxs.end())
            {
                visited_node_idxs.insert(_node.self_idx);
            }
        }
        // Should never reach here, but handled gracefully
        return Path{};
    }

    NodeGrid::Path NodeGrid::backtrace_path(
        Node *_start_node, Node *_end_node,
        const uint8_t *weights, const Vec2m &wsize,
        const uint8_t threshold)
    {
        Path result;

        result.push_back(GridUtils::gridLoc(_start_node->self_idx, wsize));

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
            result.push_back(GridUtils::gridLoc(_itr_node->self_idx, wsize));
            _current_node = _itr_node;
        }

        return result;
    }

    bool NodeGrid::line_of_sight(
        const uint8_t *weights, const Vec2m &wsize,
        const mapsize_t a_idx, const mapsize_t b_idx,
        const uint8_t threshold)
    {
        Vec2m
            a_loc = GridUtils::gridLoc(a_idx, wsize),
            b_loc = GridUtils::gridLoc(b_idx, wsize);

        int64_t
            x1 = a_loc.x,
            y1 = a_loc.y,
            x2 = b_loc.x,
            y2 = b_loc.y,

            dx = std::abs(static_cast<int64_t>(x2) - static_cast<int64_t>(x1)),
            dy = std::abs(static_cast<int64_t>(y2) - static_cast<int64_t>(y1)),
            sx = (x1 < x2) ? 1 : -1,
            sy = (y1 < y2) ? 1 : -1,
            err = dx - dy;

        while (true)
        {
            mapsize_t idx = x1 + y1 * wsize.x;
            if (weights[idx] > threshold)
                return false;

            if (x1 == x2 && y1 == y2)
                return true;

            int64_t e2 = err * 2;
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