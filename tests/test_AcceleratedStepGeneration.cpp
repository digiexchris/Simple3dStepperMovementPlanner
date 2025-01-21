#include "StepTimingPlanner.hpp"
#include "Types.hpp"
#include "gtest/gtest.h"
#include <algorithm>
#include <bits/stdint-uintn.h>
#include <cmath>

using namespace StepTimingPlanner;

class TestGenerator : public Generator3d
{
public:
	TestGenerator(AxisLimits anXLimit, AxisLimits aYLimit, AxisLimits aZLimit)
		: Generator3d(anXLimit, aYLimit, aZLimit) {}

	// redeclearing as public for testing
	Vector<uint16_t> TestGetComponentSpeeds(const uint16_t straightLineSpeed,
											const Vec3Int32 &start,
											const Vec3Int32 &end) const
	{
		return GetComponentSpeeds(straightLineSpeed, start, end);
	}

	Vec3Int32 TestGetComponentDistances(const Vec3Int32 &start,
										const Vec3Int32 &end) const
	{
		return GetComponentDistances(start, end);
	};

	double TestGetStraightLineDistance(const Vec3Int32 &first,
									   const Vec3Int32 &second) const
	{
		return GetStraightLineDistance(first, second);
	}
};

TEST(StepTimingGeneratorAcceleration, NonUniformMoveCappedByNonDrivingAxis)
{
	AxisLimits xLimits = {50, 50, 500};	  // maxAcceleration, maxSpeed
	AxisLimits yLimits = {80, 80, 400};	  // Different maxSpeed for Y
	AxisLimits zLimits = {100, 100, 300}; // Different maxSpeed for Z
	Generator3d generator(xLimits, yLimits, zLimits);
	Vec3Int32 start = {0, 0, 0};
	Vec3Int32 end = {10, 5, 9};
	Vector<uint16_t> startingSpeedDirection({0, 0, 0});
	uint16_t requestedSpeed = 1000;
	uint16_t endingSpeed = 0;

	auto steps = generator.GenerateSteps(start, end, startingSpeedDirection,
										 requestedSpeed, endingSpeed);

	ASSERT_EQ(steps.size(), 10);

	// Calculate the movement along each axis
	Vec3Int32 movement = {10, 5, 9}; // Corresponds to end - start

	// Identify the driving axis (the axis with the largest movement)
	int drivingAxis = 0;
	int cappedAxis = 2;
	float effectiveZSpeed = 9.0 / 10 * xLimits.maxSpeed;
	float effectiveYSpeed = 5.0 / 10 * xLimits.maxSpeed;
	float cappedSpeed = zLimits.maxSpeed;
	float scaleRatio = cappedSpeed / effectiveZSpeed;
	uint16_t scaledSpeed = 500 * scaleRatio;

	uint16_t expectedDelayX = 1.0 / scaledSpeed * 1000000;

	std::array<uint32_t, 10> expected({750751, 501502, 252253, 3003, 3003, 3003,
									   252252, 501501, 750751, 750750});

	Vec3Int32 position({0, 0, 0});
	for (int i = 0; i < steps.size(); ++i)
	{
		char buffer[50];
		std::snprintf(buffer, sizeof(buffer), "index: %u", i);
		// GTEST_MESSAGE_(buffer, testing::TestPartResult::Type::kFatalFailure);
		EXPECT_NEAR(steps[i].delay, expected[i], 1);
		std::transform(position.begin(), position.end(), steps[i].delta.begin(),
					   position.begin(), std::plus<>());
	}

	EXPECT_EQ(position, Vec3Int32({10, 5, 9}));

	// EXPECT_EQ(steps.back().delay, 0);
}
