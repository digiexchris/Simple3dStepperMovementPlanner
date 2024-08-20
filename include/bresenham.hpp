#pragma once

#include <cstdint>
#include <vector>

using Vector3i = std::vector<int32_t>;
using Vector3iList = std::vector<Vector3i>;

Vector3iList Bresenham3D(int32_t x1, int32_t y1, int32_t z1, int32_t x2,
                         int32_t y2, int32_t z2);

// turn points returned from the Bresenham function into how far each point
// changes from the previous point
Vector3iList PointsToDeltas(Vector3iList aListOfPoints);