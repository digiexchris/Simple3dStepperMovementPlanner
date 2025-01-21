#pragma once

#include "Types.hpp"
#include <cmath>
#include <stdint.h>

namespace StepTimingPlanner
{

	inline uint32_t SecondsToMicroseconds(float seconds)
	{
		return std::round(seconds * 1000000.0);
	}

	enum class Axis : uint8_t
	{
		X = 0,
		Y = 1,
		Z = 2
	};

	struct AxisLimits
	{
		uint16_t maxAcceleration;
		uint16_t maxDeceleration;
		uint16_t maxSpeed;
	};

	struct StepTiming
	{
		StepTiming(Vec3Int8 aDelta, uint32_t aDelay) : delta(aDelta), delay(aDelay) {}
		// which axis to increment
		Vec3Int8 delta;
		// delay in microseconds to wait after this step is executed (optional if last
		// step)
		uint32_t delay;
	};

	enum class StepTimingPlanType
	{
		ACCELERATE,
		COAST,
		DECELERATE,
		ACCELERATE_DECELERATE,
		DECELERATE_ACCELERATE,

	};

	struct StepTimingPlan
	{
		std::vector<StepTiming> steps;
		uint32_t endingSpeed;
	};

	struct StepTimingPlan1d
	{
		uint32_t steps;
		uint32_t endingSpeed;
		StepTimingPlanType type;
	};

	class Generator1d
	{
	public:
		Generator1d(AxisLimits limits);
		StepTimingPlan GenerateSteps(const int32_t distance,
									 const uint16_t startingSpeed, const uint16_t requestedSpeed,
									 const uint16_t requestedEndingSpeed);

	protected:
		std::vector<StepTimingPlan1d> SelectPlan(uint16_t startingSpeed, uint16_t requestedSpeed, uint16_t requestedEndingSpeed, uint32_t distance) const;

	private:
		AxisLimits limit;
	};

	class Generator3d
	{
	public:
		Generator3d(AxisLimits x, AxisLimits y, AxisLimits z);
		std::vector<StepTiming>
		GenerateSteps(const Vec3Int32 &start, const Vec3Int32 &end,
					  const Vector<uint16_t> startingSpeedDirection,
					  const uint16_t requestedStraightLineSpeed,
					  const uint16_t requestedEndingStraightLineSpeed);

	protected:
		uint16_t
		GetAcclerationStepCount(const Vector<uint16_t> startingSpeeds,
								const Vector<uint16_t> requestedComponentSpeeds,
								const uint8_t drivingAxis) const;

		Vector<uint16_t> GetComponentSpeeds(const uint16_t straightLineSpeed,
											const Vec3Int32 &start,
											const Vec3Int32 &end) const;

		Vec3Int32 GetComponentDistances(const Vec3Int32 &start,
										const Vec3Int32 &end) const;

		double GetStraightLineDistance(const Vec3Int32 &first,
									   const Vec3Int32 &second) const;

		AxisLimits axisLimits[3];
	};

} // namespace StepTimingPlanner