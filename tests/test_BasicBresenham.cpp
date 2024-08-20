#include "bresenham.hpp"
#include <gtest/gtest.h>
#include <vector>

TEST(Bresenham3DTest, BasicTest) {
  int x1 = -1;
  int y1 = 1;
  int z1 = 1;
  int x2 = 5;
  int y2 = 3;
  int z2 = -1;
  std::vector<std::vector<int>> ListOfPoints =
      Bresenham3D(x1, y1, z1, x2, y2, z2);
  std::vector<std::vector<int>> expectedPoints = {
      {-1, 1, 1}, {0, 1, 1},  {1, 2, 0}, {2, 2, 0},
      {3, 2, 0},  {4, 3, -1}, {5, 3, -1}};
  ASSERT_EQ(ListOfPoints, expectedPoints);
}

TEST(Bresenham3DTest, AnotherTest) {

  int x1 = -7;
  int y1 = 0;
  int z1 = -3;
  int x2 = 2;
  int y2 = -5;
  int z2 = -1;
  std::vector<std::vector<int>> ListOfPoints =
      Bresenham3D(x1, y1, z1, x2, y2, z2);
  std::vector<std::vector<int>> expectedPoints = {
      {-7, 0, -3},  {-6, -1, -3}, {-5, -1, -3}, {-4, -2, -2}, {-3, -2, -2},
      {-2, -3, -2}, {-1, -3, -2}, {0, -4, -1},  {1, -4, -1},  {2, -5, -1}};
  ASSERT_EQ(ListOfPoints, expectedPoints);
}
TEST(Bresenham3DTest, AdditionalTest) {
  int x1 = 5;
  int y1 = 3;
  int z1 = 11;
  int x2 = -3;
  int y2 = -3;
  int z2 = -3;
  std::vector<std::vector<int>> ListOfPoints =
      Bresenham3D(x1, y1, z1, x2, y2, z2);
  std::vector<std::vector<int>> expectedPoints = {
      {5, 3, 11},  {4, 3, 10},  {4, 2, 9},    {3, 2, 8},    {3, 1, 7},
      {2, 1, 6},   {2, 0, 5},   {1, 0, 4},    {0, 0, 3},    {0, -1, 2},
      {-1, -1, 1}, {-1, -2, 0}, {-2, -2, -1}, {-2, -3, -2}, {-3, -3, -3}};

  ASSERT_EQ(ListOfPoints, expectedPoints);
}
