#ifndef GRID_UTILS_H
#define GRID_UTILS_H

#include <stdint.h>
#include <cmath>

namespace lazythetastar
{
    /** Generic grid helpers */
    namespace GridUtils
    {
        class Vector2m
        {
        public:
            int64_t x, y;
            Vector2m(int64_t _x, int64_t _y) : x(_x), y(_y) {}
        };

        class Vector2f
        {
        public:
            float x, y;
            Vector2f(float _x, float _y) : x(_x), y(_y) {}
        };

        /** Align a point to a box grid of the given resolution and offset origin. Result may be negative if lower than current offset. */
        inline static Vector2m
        worldToGrid(Vector2f &pos, const Vector2f &off, const float res)
        {
            return Vector2m{
                static_cast<int32_t>((pos.x - off.x) / res), // always floor since grid cells are indexed by their "bottom left" corner's raw position
                static_cast<int32_t>((pos.y - off.y) / res)};
        }

        inline static Vector2m
        worldToGrid(float x, float y, const Vector2f &off, const float res)
        {
            return worldToGrid(Vector2f{x, y}, off, res);
        }

        inline static Vector2f
        gridToWorld(const Vector2m &pos, const Vector2f &off, const float res)
        {
            return Vector2f{
                (static_cast<float>(pos.x) * res) + off.x,
                (static_cast<float>(pos.y) * res) + off.y};
        }

        /** Get a raw buffer idx from a 2d index and buffer size (templated on major-order) */
        inline static int64_t gridIdx(const int64_t x, const int64_t y, const Vector2m &size)
        {
            return static_cast<int64_t>(y) * size.x + x; // y-major = "contiguous blocks along [parallel to] x-axis" --> idx = (y * xmax) + x
        }

        inline static int64_t gridIdx(const Vector2m &loc, const Vector2m &size)
        {
            return gridIdx(loc.x, loc.y, size);
        }

        /** Get the 2d location corresponding to a raw buffer idx for the provded grid size (templated on major-order) */
        inline static Vector2m gridLoc(const int64_t idx, const Vector2m &size)
        {
            return Vector2m{// y-major = "contiguous blocks along [parallel to] x-axis" --> x = idx % xmax, y = idx / xmax
                            static_cast<int32_t>(idx % size.x),
                            static_cast<int32_t>(idx / size.x)};
        }

        /** Check if v is GEQ than min and LESS than max */
        inline static bool inRange(const Vector2m &v, const Vector2m &min, const Vector2m &max)
        {
            return (
                min.x <= v.x && v.x < max.x &&
                min.y <= v.y && v.y < max.y);
        }
    };
}

#endif // GRID_UTILS_H