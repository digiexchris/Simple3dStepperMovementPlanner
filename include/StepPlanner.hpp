#pragma once

#include "Types.hpp"
#include <cstdint>
namespace StepTimingPlanner {

struct BresenhamPoints {
  Vec3Int32List points;
  uint8_t drivingAxis;
};

BresenhamPoints Bresenham3D(Vec3Int32 start, Vec3Int32 end);

// turn points returned from the Bresenham function into how far each point
// changes from the previous point
Vec3Int8List PointsToDeltas(Vec3Int32List &aListOfPoints);

} // namespace StepTimingPlanner