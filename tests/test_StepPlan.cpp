#include "StepPlanner.hpp"
#include "StepTiming.hpp"
#include "gtest/gtest.h"
#include <algorithm>
#include <bits/stdint-uintn.h>
#include <chrono>
#include <cmath>

// Sample AxisLimits for testing
AxisLimits xLimits = {1000, 1000, 500}; // maxAcceleration, maxSpeed
AxisLimits yLimits = {1000, 1000, 400}; // Different maxSpeed for Y
AxisLimits zLimits = {1000, 1000, 300}; // Different maxSpeed for Z

class TestGenerator : public StepTimingGenerator {
public:
  TestGenerator(AxisLimits anXLimit, AxisLimits aYLimit, AxisLimits aZLimit)
      : StepTimingGenerator(anXLimit, aYLimit, aZLimit) {}

  // redeclearing as public for testing
  Vector<uint16_t> TestGetComponentSpeeds(const uint16_t straightLineSpeed,
                                          const Vector3Int32 &start,
                                          const Vector3Int32 &end) const {
    return GetComponentSpeeds(straightLineSpeed, start, end);
  }

  Vector3Int32 TestGetComponentDistances(const Vector3Int32 &start,
                                         const Vector3Int32 &end) const {
    return GetComponentDistances(start, end);
  };

  double TestGetStraightLineDistance(const Vector3Int32 &first,
                                     const Vector3Int32 &second) const {
    return GetStraightLineDistance(first, second);
  }
};

TestGenerator generator(xLimits, yLimits, zLimits);

TEST(StepTimingGenerator, GetComponentDistances) {
  Vector3Int32 start({0, 0, 0});
  Vector3Int32 end({10, 11, 12});

  Vector3Int32 distances = generator.TestGetComponentDistances(start, end);

  ASSERT_EQ(distances, Vector3Int32({10, 11, 12}));

  start = {10, 11, 12};
  end = {0, 0, 0};
  distances = generator.TestGetComponentDistances(start, end);

  ASSERT_EQ(distances, Vector3Int32({10, 11, 12}));
}

TEST(StepTimingGenerator, GetStraightLineDistance) {
  Vector3Int32 start({0, 0, 0});
  Vector3Int32 end({10, 11, 12});

  double distance = generator.TestGetStraightLineDistance(start, end);

  ASSERT_NEAR(distance, 19.105, 0.001);

  start = {10, 11, 12};
  end = {0, 0, 0};
  distance = generator.TestGetStraightLineDistance(start, end);

  ASSERT_NEAR(distance, 19.105, 0.001);

  // one last round number easy to do in head math with
  start = {10, 10, 10};
  end = {0, 0, 0};
  distance = generator.TestGetStraightLineDistance(start, end);

  ASSERT_NEAR(distance, 17.320, 0.001);
}

TEST(StepTimingGenerator, GetComponentSpeedsSingleAxis) {
  Vector3Int32 start({0, 0, 0});
  Vector3Int32 end({10, 0, 0});
  uint16_t requestedSpeed = 200;

  Vector<uint16_t> actualSpeed =
      generator.TestGetComponentSpeeds(requestedSpeed, start, end);

  ASSERT_EQ(actualSpeed, Vector<uint16_t>({200, 0, 0}));

  end = Vector3Int32({0, 10, 0});

  actualSpeed = generator.TestGetComponentSpeeds(requestedSpeed, start, end);

  ASSERT_EQ(actualSpeed, Vector<uint16_t>({0, 200, 0}));

  end = Vector3Int32({0, 0, 10});

  actualSpeed = generator.TestGetComponentSpeeds(requestedSpeed, start, end);

  ASSERT_EQ(actualSpeed, Vector<uint16_t>({0, 0, 200}));

  end = Vector3Int32({-10, 0, 0});

  actualSpeed = generator.TestGetComponentSpeeds(requestedSpeed, start, end);

  ASSERT_EQ(actualSpeed, Vector<uint16_t>({200, 0, 0}));

  end = Vector3Int32({0, -10, 0});

  actualSpeed = generator.TestGetComponentSpeeds(requestedSpeed, start, end);

  ASSERT_EQ(actualSpeed, Vector<uint16_t>({0, 200, 0}));

  end = Vector3Int32({0, 0, -10});

  actualSpeed = generator.TestGetComponentSpeeds(requestedSpeed, start, end);

  ASSERT_EQ(actualSpeed, Vector<uint16_t>({0, 0, 200}));
}

TEST(StepTimingGenerator, GetComponentSpeedsMultiAxis) {
  Vector3Int32 start({0, 0, 0});
  Vector3Int32 end({10, 10, 10});
  uint16_t requestedSpeed = 200;

  Vector<uint16_t> actualSpeed =
      generator.TestGetComponentSpeeds(requestedSpeed, start, end);

  ASSERT_EQ(actualSpeed, Vector<uint16_t>({115, 115, 115}));

  end = Vector3Int32({0, 10, 50});

  actualSpeed = generator.TestGetComponentSpeeds(requestedSpeed, start, end);

  ASSERT_EQ(actualSpeed, Vector<uint16_t>({0, 39, 196}));

  end = Vector3Int32({0, -10, 50});

  actualSpeed = generator.TestGetComponentSpeeds(requestedSpeed, start, end);

  ASSERT_EQ(actualSpeed, Vector<uint16_t>({0, 39, 196}));

  end = Vector3Int32({0, -10, -50});

  actualSpeed = generator.TestGetComponentSpeeds(requestedSpeed, start, end);

  ASSERT_EQ(actualSpeed, Vector<uint16_t>({0, 39, 196}));
}

// Testing when the speed is capped by the X axis limit
TEST(StepTimingGeneratorDifferentLimitsTest, SpeedCappingXAxis) {
  StepTimingGenerator generator(xLimits, yLimits, zLimits);
  Vector3Int32 start = {0, 0, 0};
  Vector3Int32 end = {50, 0, 0}; // Move primarily on the X axis

  // Expected speed capping
  uint16_t cappedSpeed = xLimits.maxSpeed;

  Vector<uint16_t> startingSpeedDirection({0, 0, 0});
  uint16_t requestedSpeed = 1000;
  uint16_t endingSpeed = 0;

  auto steps = generator.GenerateSteps(start, end, startingSpeedDirection,
                                       requestedSpeed, endingSpeed);

  ASSERT_EQ(steps.size(), 50);

  uint32_t expectedDelay = 2000;

  for (size_t i = 0; i < steps.size(); ++i) {
    EXPECT_EQ(steps[i].delta, Vector3Int32({1, 0, 0}));
    EXPECT_EQ(steps[i].delay, expectedDelay);
  }

  // EXPECT_EQ(steps.back().delay, 0);
}

// Testing when the speed is capped by the Y axis limit
TEST(StepTimingGeneratorDifferentLimitsTest, SpeedCappingYAxis) {
  StepTimingGenerator generator(xLimits, yLimits, zLimits);
  Vector3Int32 start = {0, 0, 0};
  Vector3Int32 end = {0, 50, 0}; // Move primarily on the Y axis

  // Expected speed capping
  uint16_t cappedSpeed = yLimits.maxSpeed;

  Vector<uint16_t> startingSpeedDirection({0, 0, 0});
  uint16_t requestedSpeed = 1000;
  uint16_t endingSpeed = 0;

  auto steps = generator.GenerateSteps(start, end, startingSpeedDirection,
                                       requestedSpeed, endingSpeed);

  ASSERT_EQ(steps.size(), 50);

  uint32_t expectedDelay = 1.0 / 400.0 * 1000000;

  for (size_t i = 0; i < steps.size(); ++i) {
    EXPECT_EQ(steps[i].delta, Vector3Int32({0, 1, 0}));
    EXPECT_EQ(steps[i].delay, expectedDelay);
  }

  // EXPECT_EQ(steps.back().delay, 0);
}

// Testing when the speed is capped by the Z axis limit
TEST(StepTimingGeneratorDifferentLimitsTest, SpeedCappingZAxis) {
  StepTimingGenerator generator(xLimits, yLimits, zLimits);
  Vector3Int32 start = {0, 0, 0};
  Vector3Int32 end = {0, 0, 50}; // Move primarily on the Z axis

  // Expected speed capping
  uint16_t cappedSpeed = zLimits.maxSpeed;

  Vector<uint16_t> startingSpeedDirection({0, 0, 0});
  uint16_t requestedSpeed = 1000;
  uint16_t endingSpeed = 0;

  auto steps = generator.GenerateSteps(start, end, startingSpeedDirection,
                                       requestedSpeed, endingSpeed);

  ASSERT_EQ(steps.size(), 50);

  uint32_t expectedDelay = 1.0 / 300.0 * 1000000;

  for (size_t i = 0; i < steps.size(); ++i) {
    EXPECT_EQ(steps[i].delta, Vector3Int32({0, 0, 1}));
    EXPECT_EQ(steps[i].delay, expectedDelay);
  }

  // EXPECT_EQ(steps.back().delay, 0);
}

// Testing when the requested speed is within the limits of all axes
TEST(StepTimingGeneratorDifferentLimitsTest, NoSpeedCappingXAxis) {
  StepTimingGenerator generator(xLimits, yLimits, zLimits);
  Vector3Int32 start = {0, 0, 0};
  Vector3Int32 end = {10, 0, 0}; // Move primarily on the X axis
  Vector<uint16_t> startingSpeedDirection({0, 0, 0});
  uint16_t requestedSpeed = 300;
  uint16_t endingSpeed = 0;

  auto steps = generator.GenerateSteps(start, end, startingSpeedDirection,
                                       requestedSpeed, endingSpeed);

  ASSERT_EQ(steps.size(),
            10); // The number of steps should match the X-axis movement

  uint32_t expectedDelay = 1.0 / 300.0 * 1000000;

  for (size_t i = 0; i < steps.size(); ++i) {
    EXPECT_EQ(steps[i].delta, Vector3Int32({1, 0, 0}));
    EXPECT_EQ(steps[i].delay, expectedDelay);
  }

  // EXPECT_EQ(steps.back().delay, 0); // The last step should have zero delay
}

TEST(StepTimingGeneratorDifferentLimitsTest, NoSpeedCappingYAxis) {
  StepTimingGenerator generator(xLimits, yLimits, zLimits);
  Vector3Int32 start = {0, 0, 0};
  Vector3Int32 end = {0, 10, 0}; // Move primarily on the Y axis
  Vector<uint16_t> startingSpeedDirection({0, 0, 0});
  uint16_t requestedSpeed = 300;
  uint16_t endingSpeed = 0;

  auto steps = generator.GenerateSteps(start, end, startingSpeedDirection,
                                       requestedSpeed, endingSpeed);

  ASSERT_EQ(steps.size(),
            10); // The number of steps should match the Y-axis movement

  uint32_t expectedDelay = 1.0 / 300.0 * 1000000;

  for (size_t i = 0; i < steps.size(); ++i) {
    EXPECT_EQ(steps[i].delta, Vector3Int32({0, 1, 0}));
    EXPECT_EQ(steps[i].delay, expectedDelay);
  }

  // EXPECT_EQ(steps.back().delay, 0); // The last step should have zero delay
}

TEST(StepTimingGeneratorDifferentLimitsTest, NoSpeedCappingZAxis) {
  StepTimingGenerator generator(xLimits, yLimits, zLimits);
  Vector3Int32 start = {0, 0, 0};
  Vector3Int32 end = {0, 0, 10}; // Move primarily on the Z axis
  Vector<uint16_t> startingSpeedDirection({0, 0, 0});
  uint16_t requestedSpeed = 200;
  uint16_t endingSpeed = 0;

  auto steps = generator.GenerateSteps(start, end, startingSpeedDirection,
                                       requestedSpeed, endingSpeed);

  ASSERT_EQ(steps.size(),
            10); // The number of steps should match the Z-axis movement

  uint32_t expectedDelay = 1.0 / 200.0 * 1000000;

  for (size_t i = 0; i < steps.size(); ++i) {
    EXPECT_EQ(steps[i].delta, Vector3Int32({0, 0, 1}));
    EXPECT_EQ(steps[i].delay, expectedDelay);
  }

  // EXPECT_EQ(steps.back().delay, 0); // The last step should have zero delay
}

// A simple linear move on the X-axis with different AxisLimits
TEST(StepTimingGeneratorDifferentLimitsTest, SimpleLinearMove) {
  StepTimingGenerator generator(xLimits, yLimits, zLimits);
  Vector3Int32 start = {0, 0, 0};
  Vector3Int32 end = {10, 0, 0};
  Vector<uint16_t> startingSpeedDirection({0, 0, 0});
  uint16_t requestedSpeed = 100;
  uint16_t endingSpeed = 0;

  auto steps = generator.GenerateSteps(start, end, startingSpeedDirection,
                                       requestedSpeed, endingSpeed);

  ASSERT_EQ(steps.size(), 10);

  uint32_t expectedDelay = 1.0 / 100 * 1000000;

  for (size_t i = 0; i < steps.size(); ++i) {
    EXPECT_EQ(steps[i].delta, Vector3Int32({1, 0, 0}));
    EXPECT_EQ(steps[i].delay, expectedDelay);
  }

  // EXPECT_EQ(steps.back().delay, 0); // The last step should have zero delay
}

// Testing when start and end are the same with different AxisLimits
TEST(StepTimingGeneratorDifferentLimitsTest, NoMovement) {
  StepTimingGenerator generator(xLimits, yLimits, zLimits);
  Vector3Int32 start = {5, 5, 5};
  Vector3Int32 end = {5, 5, 5};
  Vector<uint16_t> startingSpeedDirection({0, 0, 0});
  uint16_t requestedSpeed = 100;
  uint16_t endingSpeed = 0;

  auto steps = generator.GenerateSteps(start, end, startingSpeedDirection,
                                       requestedSpeed, endingSpeed);

  EXPECT_EQ(steps.size(), 0); // No movement should produce no steps
}

// Testing a diagonal move with all axes with different AxisLimits
TEST(StepTimingGeneratorDifferentLimitsTest, DiagonalMove) {
  StepTimingGenerator generator(xLimits, yLimits, zLimits);
  Vector3Int32 start = {0, 0, 0};
  Vector3Int32 end = {25, 25, 25};
  Vector<uint16_t> startingSpeedDirection({0, 0, 0});
  uint16_t requestedSpeed = 1000;
  uint16_t endingSpeed = 0;

  auto steps = generator.GenerateSteps(start, end, startingSpeedDirection,
                                       requestedSpeed, endingSpeed);

  ASSERT_EQ(steps.size(), 25);

  for (size_t i = 0; i < steps.size(); ++i) {
    EXPECT_EQ(steps[i].delta, Vector3Int32({1, 1, 1}));
    // EXPECT_EQ(steps[i].delay, expectedDelay);
  }
  EXPECT_EQ(steps[0].delay, 999999);
  EXPECT_EQ(steps[1].delay, 999999);
  EXPECT_EQ(steps[2].delay, 999999);
  EXPECT_EQ(steps[3].delay, 999999);
  EXPECT_EQ(steps[4].delay, 999999);
  EXPECT_EQ(steps[5].delay, 999999);
  EXPECT_EQ(steps[6].delay, 999999);
  EXPECT_EQ(steps[7].delay, 999999);
  EXPECT_EQ(steps[8].delay, 999999);
  EXPECT_EQ(steps[9].delay, 999999);
  EXPECT_EQ(steps[10].delay, 999999);
  EXPECT_EQ(steps[11].delay, 999999);
  EXPECT_EQ(steps[12].delay, 999999);
  EXPECT_EQ(steps[13].delay, 999999);
  EXPECT_EQ(steps[14].delay, 999999);
  EXPECT_EQ(steps[15].delay, 999999);
  EXPECT_EQ(steps[16].delay, 999999);
  EXPECT_EQ(steps[17].delay, 999999);
  EXPECT_EQ(steps[18].delay, 999999);
  EXPECT_EQ(steps[19].delay, 999999);
  EXPECT_EQ(steps[20].delay, 999999);
  EXPECT_EQ(steps[21].delay, 999999);
  EXPECT_EQ(steps[22].delay, 999999);
  EXPECT_EQ(steps[23].delay, 999999);
  EXPECT_EQ(steps[24].delay, 999999);
  EXPECT_EQ(steps[25].delay, 999999);

  // EXPECT_EQ(steps.back().delay, 0);
}

// Testing a non-uniform move with different distances on each axis with
// different AxisLimits
TEST(StepTimingGeneratorDifferentLimitsTest, NonUniformMove) {
  StepTimingGenerator generator(xLimits, yLimits, zLimits);
  Vector3Int32 start = {0, 0, 0};
  Vector3Int32 end = {10, 5, 2};
  Vector<uint16_t> startingSpeedDirection({0, 0, 0});
  uint16_t requestedSpeed = 1000;
  uint16_t endingSpeed = 0;

  auto steps = generator.GenerateSteps(start, end, startingSpeedDirection,
                                       requestedSpeed, endingSpeed);

  ASSERT_EQ(steps.size(), 10);

  // Calculate the movement along each axis
  Vector3Int32 movement = {10, 5, 2}; // Corresponds to end - start

  // Identify the driving axis (the axis with the largest movement)
  int drivingAxis = 0;

  uint16_t expectedDelayX = 1.0 / 500 * 1000000;

  for (size_t i = 0; i < steps.size(); ++i) {
    EXPECT_EQ(steps[i].delay, expectedDelayX);
  }

  // EXPECT_EQ(steps.back().delay, 0);
}

TEST(StepTimingGeneratorDifferentLimitsTest,
     NonUniformMoveCappedByNonDrivingAxis) {
  StepTimingGenerator generator(xLimits, yLimits, zLimits);
  Vector3Int32 start = {0, 0, 0};
  Vector3Int32 end = {10, 5, 9};
  Vector<uint16_t> startingSpeedDirection({0, 0, 0});
  uint16_t requestedSpeed = 1000;
  uint16_t endingSpeed = 0;

  auto steps = generator.GenerateSteps(start, end, startingSpeedDirection,
                                       requestedSpeed, endingSpeed);

  ASSERT_EQ(steps.size(), 10);

  // Calculate the movement along each axis
  Vector3Int32 movement = {10, 5, 9}; // Corresponds to end - start

  // Identify the driving axis (the axis with the largest movement)
  int drivingAxis = 0;
  int cappedAxis = 2;
  float effectiveZSpeed = 9.0 / 10 * xLimits.maxSpeed;
  float effectiveYSpeed = 5.0 / 10 * xLimits.maxSpeed;
  float cappedSpeed = zLimits.maxSpeed;
  float scaleRatio = cappedSpeed / effectiveZSpeed;
  uint16_t scaledSpeed = 500 * scaleRatio;

  uint16_t expectedDelayX = 1.0 / scaledSpeed * 1000000;

  for (size_t i = 0; i < steps.size(); ++i) {
    EXPECT_EQ(steps[i].delay, expectedDelayX);
  }

  // EXPECT_EQ(steps.back().delay, 0);
}
