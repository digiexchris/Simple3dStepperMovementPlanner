#pragma once

#include "Types.hpp"
#include <cstdint>

struct BresenhamPoints {
  Vector3Int32List points;
  uint8_t drivingAxis;
};

BresenhamPoints Bresenham3D(Vector3Int32 start, Vector3Int32 end);

// turn points returned from the Bresenham function into how far each point
// changes from the previous point
Vector3Int32List PointsToDeltas(Vector3Int32List &aListOfPoints);
