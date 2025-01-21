#include "StepTimingPlanner.hpp"

#include "gtest/gtest.h"

using namespace StepTimingPlanner;

TEST(StepTimingTest, BasicTest)
{
	AxisLimits x = {1, 2};
	AxisLimits y = {3, 4};
	AxisLimits z = {5, 6};
	Generator3d timing(x, y, z);
}