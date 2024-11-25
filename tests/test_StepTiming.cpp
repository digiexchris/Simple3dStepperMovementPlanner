#include "StepTiming.hpp"

#include "gtest/gtest.h"

TEST(StepTimingTest, BasicTest) {
  AxisLimits x = {1, 2};
  AxisLimits y = {3, 4};
  AxisLimits z = {5, 6};
  StepTimingGenerator timing(x, y, z);
}