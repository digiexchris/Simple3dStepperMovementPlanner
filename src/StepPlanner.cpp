#include <cmath>
#include <iostream>
#include <vector>

#include "StepPlanner.hpp"
#include "Types.hpp"

BresenhamPoints Bresenham3D(Vector3Int32 start, Vector3Int32 end) {

  uint8_t drivingAxis = 16;
  if (start.size() != 3 || end.size() != 3) {
    throw std::invalid_argument("Both start and end must be 3D points");
  }
  int32_t x1 = start[0];
  int32_t y1 = start[1];
  int32_t z1 = start[2];
  int32_t x2 = end[0];
  int32_t y2 = end[1];
  int32_t z2 = end[2];

  Vector3Int32List ListOfPoints;
  ListOfPoints.push_back({x1, y1, z1});
  int32_t dx = abs(x2 - x1);
  int32_t dy = abs(y2 - y1);
  int32_t dz = abs(z2 - z1);
  int32_t xs;
  int32_t ys;
  int32_t zs;
  if (x2 > x1)
    xs = 1;
  else
    xs = -1;
  if (y2 > y1)
    ys = 1;
  else
    ys = -1;
  if (z2 > z1)
    zs = 1;
  else
    zs = -1;

  // Driving axis is X-axis"
  if (dx >= dy && dx >= dz) {
    drivingAxis = 0;
    int32_t p1 = 2 * dy - dx;
    int32_t p2 = 2 * dz - dx;
    while (x1 != x2) {
      x1 += xs;
      if (p1 >= 0) {
        y1 += ys;
        p1 -= 2 * dx;
      }
      if (p2 >= 0) {
        z1 += zs;
        p2 -= 2 * dx;
      }
      p1 += 2 * dy;
      p2 += 2 * dz;
      ListOfPoints.push_back({x1, y1, z1});
    }

    // Driving axis is Y-axis"
  } else if (dy >= dx && dy >= dz) {
    drivingAxis = 1;
    int32_t p1 = 2 * dx - dy;
    int32_t p2 = 2 * dz - dy;
    while (y1 != y2) {
      y1 += ys;
      if (p1 >= 0) {
        x1 += xs;
        p1 -= 2 * dy;
      }
      if (p2 >= 0) {
        z1 += zs;
        p2 -= 2 * dy;
      }
      p1 += 2 * dx;
      p2 += 2 * dz;
      ListOfPoints.push_back({x1, y1, z1});
    }

    // Driving axis is Z-axis"
  } else {
    drivingAxis = 2;
    int32_t p1 = 2 * dy - dz;
    int32_t p2 = 2 * dx - dz;
    while (z1 != z2) {
      z1 += zs;
      if (p1 >= 0) {
        y1 += ys;
        p1 -= 2 * dz;
      }
      if (p2 >= 0) {
        x1 += xs;
        p2 -= 2 * dz;
      }
      p1 += 2 * dy;
      p2 += 2 * dx;
      ListOfPoints.push_back({x1, y1, z1});
    }
  }
  return BresenhamPoints{ListOfPoints, drivingAxis};
}

Vector3Int8List PointsToDeltas(Vector3Int32List &aListOfPoints) {
  Vector3Int8List listOfDeltas;
  for (size_t i = 0; i < aListOfPoints.size() - 1; i++) {
    Vector3Int8 delta = {
        static_cast<int8_t>(aListOfPoints[i + 1][0] - aListOfPoints[i][0]),
        static_cast<int8_t>(aListOfPoints[i + 1][1] - aListOfPoints[i][1]),
        static_cast<int8_t>(aListOfPoints[i + 1][2] - aListOfPoints[i][2])};
    listOfDeltas.push_back(delta);
  }
  return listOfDeltas;
}
