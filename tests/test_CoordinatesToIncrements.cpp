#include "StepPlanner.hpp"
#include <gtest/gtest.h>
#include <vector>

TEST(CoordinatesToIncrementsTest, XAxisIncrementing) {
  int x1 = 0;
  int y1 = 0;
  int z1 = 0;
  int x2 = 5;
  int y2 = 2;
  int z2 = 1;
  Vector3Int32List ListOfPoints =
      Bresenham3D(Vector3Int32({x1, y1, z1}), Vector3Int32({x2, y2, z2})).points;
  Vector3Int32List listOfDeltas = PointsToDeltas(ListOfPoints);

  Vector3Int32List expectedPoints = {{0, 0, 0}, {1, 0, 0}, {2, 1, 0},
                                   {3, 1, 1}, {4, 2, 1}, {5, 2, 1}};
  Vector3Int32List expectedDeltas = {
      {1, 0, 0}, {1, 1, 0}, {1, 0, 1}, {1, 1, 0}, {1, 0, 0}};
  ASSERT_EQ(ListOfPoints, expectedPoints);
  ASSERT_EQ(listOfDeltas, expectedDeltas);
}

TEST(CoordinatesToIncrementsTest, XAxisDecrementing) {
  int x1 = 5;
  int y1 = 2;
  int z1 = 1;
  int x2 = 0;
  int y2 = 0;
  int z2 = 0;
  Vector3Int32List ListOfPoints =
      Bresenham3D(Vector3Int32({x1, y1, z1}), Vector3Int32({x2, y2, z2})).points;
  Vector3Int32List listOfDeltas = PointsToDeltas(ListOfPoints);

  Vector3Int32List expectedPoints = {{5, 2, 1}, {4, 2, 1}, {3, 1, 1},
                                   {2, 1, 0}, {1, 0, 0}, {0, 0, 0}};
  Vector3Int32List expectedDeltas = {
      {-1, 0, 0}, {-1, -1, 0}, {-1, 0, -1}, {-1, -1, 0}, {-1, 0, 0}};
  ASSERT_EQ(ListOfPoints, expectedPoints);
  ASSERT_EQ(listOfDeltas, expectedDeltas);
}

TEST(CoordinatesToIncrementsTest, YAxisIncrementing) {
  int x1 = 0;
  int y1 = 0;
  int z1 = 0;
  int x2 = 2;
  int y2 = 5;
  int z2 = 1;
  Vector3Int32List ListOfPoints =
      Bresenham3D(Vector3Int32({x1, y1, z1}), Vector3Int32({x2, y2, z2})).points;
  Vector3Int32List listOfDeltas = PointsToDeltas(ListOfPoints);

  Vector3Int32List expectedPoints = {{0, 0, 0}, {0, 1, 0}, {1, 2, 0},
                                   {1, 3, 1}, {2, 4, 1}, {2, 5, 1}};
  Vector3Int32List expectedDeltas = {
      {0, 1, 0}, {1, 1, 0}, {0, 1, 1}, {1, 1, 0}, {0, 1, 0}};
  ASSERT_EQ(ListOfPoints, expectedPoints);
  ASSERT_EQ(listOfDeltas, expectedDeltas);
}

TEST(CoordinatesToIncrementsTest, YAxisDecrementing) {
  int x1 = 2;
  int y1 = 5;
  int z1 = 1;
  int x2 = 0;
  int y2 = 0;
  int z2 = 0;
  Vector3Int32List ListOfPoints =
      Bresenham3D(Vector3Int32({x1, y1, z1}), Vector3Int32({x2, y2, z2})).points;
  Vector3Int32List listOfDeltas = PointsToDeltas(ListOfPoints);

  Vector3Int32List expectedPoints = {{2, 5, 1}, {2, 4, 1}, {1, 3, 1},
                                   {1, 2, 0}, {0, 1, 0}, {0, 0, 0}};
  Vector3Int32List expectedDeltas = {
      {0, -1, 0}, {-1, -1, 0}, {0, -1, -1}, {-1, -1, 0}, {0, -1, 0}};

  ASSERT_EQ(ListOfPoints, expectedPoints);
  ASSERT_EQ(listOfDeltas, expectedDeltas);
}

TEST(CoordinatesToIncrementsTest, ZAxisIncrementing) {
  int x1 = 0;
  int y1 = 0;
  int z1 = 0;
  int x2 = 1;
  int y2 = 2;
  int z2 = 5;
  Vector3Int32List ListOfPoints =
      Bresenham3D(Vector3Int32({x1, y1, z1}), Vector3Int32({x2, y2, z2})).points;
  Vector3Int32List listOfDeltas = PointsToDeltas(ListOfPoints);

  Vector3Int32List expectedPoints = {{0, 0, 0}, {0, 0, 1}, {0, 1, 2},
                                   {1, 1, 3}, {1, 2, 4}, {1, 2, 5}};
  Vector3Int32List expectedDeltas = {
      {0, 0, 1}, {0, 1, 1}, {1, 0, 1}, {0, 1, 1}, {0, 0, 1}};

  ASSERT_EQ(ListOfPoints, expectedPoints);
  ASSERT_EQ(listOfDeltas, expectedDeltas);
}

TEST(CoordinatesToIncrementsTest, ZAxisDecrementing) {
  int x1 = 1;
  int y1 = 2;
  int z1 = 5;
  int x2 = 0;
  int y2 = 0;
  int z2 = 0;
  Vector3Int32List ListOfPoints =
      Bresenham3D(Vector3Int32({x1, y1, z1}), Vector3Int32({x2, y2, z2})).points;
  Vector3Int32List listOfDeltas = PointsToDeltas(ListOfPoints);

  Vector3Int32List expectedPoints = {{1, 2, 5}, {1, 2, 4}, {1, 1, 3},
                                   {0, 1, 2}, {0, 0, 1}, {0, 0, 0}};
  Vector3Int32List expectedDeltas = {
      {0, 0, -1}, {0, -1, -1}, {-1, 0, -1}, {0, -1, -1}, {0, 0, -1}};

  ASSERT_EQ(ListOfPoints, expectedPoints);
  ASSERT_EQ(listOfDeltas, expectedDeltas);
}
