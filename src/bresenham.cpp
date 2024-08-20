#include <cmath>
#include <iostream>
#include <vector>

#include "bresenham.hpp"

// todo change the params to vector3i32
Vector3iList Bresenham3D(int32_t x1, int32_t y1, int32_t z1, int32_t x2,
                         int32_t y2, int32_t z2) {
  std::vector<std::vector<int>> ListOfPoints;
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
  return ListOfPoints;
}

Vector3iList PointsToDeltas(Vector3iList aListOfPoints) {
  Vector3iList ListOfDeltas;
  for (size_t i = 0; i < aListOfPoints.size() - 1; i++) {
    std::vector<int32_t> delta = {aListOfPoints[i + 1][0] - aListOfPoints[i][0],
                                  aListOfPoints[i + 1][1] - aListOfPoints[i][1],
                                  aListOfPoints[i + 1][2] -
                                      aListOfPoints[i][2]};
    ListOfDeltas.push_back(delta);
  }
  return ListOfDeltas;
}
