#include "Delay.hpp"
#include "gtest/gtest.h"

using namespace StepTimingPlanner;

TEST(Delay, CalculateIntervalBasic) {
  int stepsPerRotation = 200;
  double acceleration = 10.0; // in steps/s^2

  // Hard-coded first interval (t1)
  double previousInterval = 0.079;

  // Calculate the next interval (t2) using the result from t1
  double nextInterval =
      calculateNextInterval(stepsPerRotation, acceleration, previousInterval);

  EXPECT_NEAR(nextInterval, 0.044, 0.001);

  previousInterval = nextInterval;
  nextInterval =
      calculateNextInterval(stepsPerRotation, acceleration, previousInterval);

  EXPECT_NEAR(nextInterval, 0.031, 0.001);
}
