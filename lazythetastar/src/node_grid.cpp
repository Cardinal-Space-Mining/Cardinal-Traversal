#include "node_grid.hpp"
#include "grid_utils.hpp"

#include <queue>

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
        Vec2m _wsize{weight_cell_w, weight_cell_h};
        Eigen::Vector2<float> _worigin{weight_origin_x, weight_origin_y};

        Vec2m _start = GridUtils::gridAlign<mapsize_t, float>(start_x, start_y, _worigin, weight_resolution);
        Vec2m _goal = GridUtils::gridAlign<mapsize_t, float>(goal_x, goal_y, _worigin, weight_resolution);

        return get_path(weights, _wsize, _start, _goal, line_of_sight_threshold);
    }

    NodeGrid::Path NodeGrid::get_path(
        const uint8_t *weights,
        const Vec2m &wsize,
        const Vec2m &start,
        const Vec2m &goal,
        const uint8_t line_of_sight_threshold)
    {
        const int64_t
            _area = static_cast<int64_t>(wsize.x()) * wsize.y(),
            _start_idx = GridUtils::gridIdx<mapsize_t>(start, wsize),
            _goal_idx = GridUtils::gridIdx<mapsize_t>(goal, wsize);

        static const Vec2m
            _zero = Vec2m::Zero();

        if (
            !GridUtils::inRange<mapsize_t>(start, _zero, wsize) ||
            !GridUtils::inRange<mapsize_t>(goal, _zero, wsize) ||
            (_start_idx == _goal_idx))
        {
            return NodeGrid::Path{}; // Return empty path
        }

        this->resize(_area);
        for (size_t i = 0; i < this->grid.size(); i++)
        {
            this->grid[i].h = static_cast<fweight_t>(
                (goal - GridUtils::gridLoc<mapsize_t>(i, wsize)).norm());
            this->grid[i].g = std::numeric_limits<fweight_t>::infinity();
            this->grid[i].parent_idx = NodeGrid::INVALID_IDX;
            this->grid[i].self_idx = i;
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
            const Vec2m
                _loc = GridUtils::gridLoc<mapsize_t>(_node.self_idx, wsize);

            for (size_t i = 0; i < NUM_MOVES; i++)
            {
                const Vec2m _neighbor_loc = _loc + std::get<0>(MOVES[i]).template cast<mapsize_t>();
                if (!GridUtils::inRange<mapsize_t>(_neighbor_loc, _zero, wsize))
                    continue;

                const int64_t _idx = GridUtils::gridIdx<mapsize_t>(_neighbor_loc, wsize);
                Node &_neighbor = this->grid[_idx];

                const fweight_t tentative_g = _node.g + (static_cast<uint64_t>(weights[_node.self_idx]) + weights[_neighbor.self_idx] + 1) * 0.5 * std::get<1>(MOVES[i]);

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

    NodeGrid::Path NodeGrid::backtrace_path(
        Node *_start_node, Node *_end_node,
        const uint8_t *weights, const Vec2m &wsize,
        const uint8_t threshold)
    {
        Path result;

        result.push_back(GridUtils::gridLoc<mapsize_t>(_start_node->self_idx, wsize).template cast<mapsize_t>());

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
            result.push_back(GridUtils::gridLoc<mapsize_t>(_itr_node->self_idx, wsize).template cast<mapsize_t>());
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
            a_loc = GridUtils::gridLoc<mapsize_t>(a_idx, wsize),
            b_loc = GridUtils::gridLoc<mapsize_t>(b_idx, wsize);

        int64_t
            x1 = a_loc.x(),
            y1 = a_loc.y(),
            x2 = b_loc.x(),
            y2 = b_loc.y(),

            dx = std::abs(static_cast<int64_t>(x2) - static_cast<int64_t>(x1)),
            dy = std::abs(static_cast<int64_t>(y2) - static_cast<int64_t>(y1)),
            sx = (x1 < x2) ? 1 : -1,
            sy = (y1 < y2) ? 1 : -1,
            err = dx - dy;

        while (true)
        {
            mapsize_t idx = GridUtils::gridIdx<mapsize_t>(Vec2m(x1, y1), wsize);
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