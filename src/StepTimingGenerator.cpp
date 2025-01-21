#include "MovementProfile.hpp"
#include "StepPlanner.hpp"
#include "StepTimingPlanner.hpp"
#include "Types.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <numeric>
#include <stdint.h>
#include <vector>

namespace StepTimingPlanner
{

	Generator1d::Generator1d(AxisLimits limits) : limit(limits) {}

	std::vector<StepTimingPlan1d> Generator1d::SelectPlan(uint16_t startingSpeed, uint16_t requestedSpeed, uint16_t requestedEndingSpeed, uint32_t distance) const
	{

		std::vector<StepTimingPlan1d> plan;

		if (startingSpeed == requestedSpeed && requestedSpeed == requestedEndingSpeed)
		{
			if (requestedSpeed > limit.maxSpeed)
			{
				requestedSpeed = limit.maxSpeed;
			}

			plan.insert(plan.begin(), StepTimingPlan1d{distance, requestedSpeed, StepTimingPlanType::COAST});

			return plan;
		}

		if (startingSpeed == requestedSpeed && requestedSpeed < requestedEndingSpeed)
		{
			auto stepCount = GetAccelerationDistance(startingSpeed, requestedEndingSpeed);
			auto endingSpeed = requestedEndingSpeed;
			if (stepCount > distance)
			{
				// determine the speed reached once it reaches distance
				auto endSpeed = std::sqrt(2 * axisLimits.maxAcceleration * distance + startingSpeed * startingSpeed);
			}
			return StepTimingPlanType::ACCELERATE;
		}

		if (startingSpeed < requestedSpeed && requestedSpeed == requestedEndingSpeed)
		{
			return StepTimingPlanType::ACCELERATE;
		}

		if (startingSpeed == requestedSpeed && requestedSpeed > requestedEndingSpeed)
		{
			return StepTimingPlanType::DECELERATE;
		}

		if (startingSpeed > requestedSpeed && requestedSpeed == requestedEndingSpeed)
		{
			return StepTimingPlanType::DECELERATE;
		}

		if (startingSpeed > requestedSpeed && requestedSpeed < requestedEndingSpeed)
		{
			return StepTimingPlanType::DECELERATE_ACCELERATE;
		}

		if (startingSpeed < requestedSpeed && requestedSpeed > requestedEndingSpeed)
		{
			return StepTimingPlanType::ACCELERATE_DECELERATE;
		}
	}

	Generator3d::Generator3d(AxisLimits x, AxisLimits y, AxisLimits z)
	{
		axisLimits[0] = x;
		axisLimits[1] = y;
		axisLimits[2] = z;
	}

	StepTimingPlan Generator1d::GenerateSteps(const int32_t distance,
											  const uint16_t startingSpeed,
											  const uint16_t requestedSpeed,
											  const uint16_t requestedEndingSpeed)
	{
		StepTimingPlan plan;
		// if there is no movement, return an empty plan
		if (distance == 0)
		{
			return plan;
		}

		StepTimingPlanType planType = SelectPlan(startingSpeed, requestedSpeed, requestedEndingSpeed);

		// Determine if any of the axis are going to limit the speed, and if so, scale
		// the speed by the ratio of the desired speed to the maximum speed
		float speedScale = 1.0;
		if (requestedSpeed > axisLimits.maxSpeed)
		{
			speedScale = static_cast<float>(axisLimits.maxSpeed) /
						 static_cast<float>(requestedSpeed);
		}

		if (speedScale < 1.0)
		{
			requestedSpeed = std::round(requestedSpeed * speedScale);
		}

		uint16_t accelerationSteps = 0;

		if (startingSpeed < requestedSpeed)
		{
			// accelerate up to speed
			accelerationSteps = GetAcclerationStepCount({startingSpeed}, {requestedSpeed}, 0);
		}

		uint16_t decelerationSteps = 0;

		if (requestedEndingSpeed < requestedSpeed)
		{
			// decelerate from coast to end
			decelerationSteps += GetAcclerationStepCount({requestedSpeed}, {endingSpeed}, 0);
		}

		// Calculate the number of steps required to decelerate to the requested ending
		// speed
		uint16_t decelerationSteps = GetAcclerationStepCount(
			{requestedSpeed}, {requestedEndingSpeed}, 0);

		// Calculate the number of steps required to coast at the requested speed
		uint16_t coastingSteps = distance - accelerationSteps - decelerationSteps;

		// Calculate the total number of steps
		uint16_t totalSteps = accelerationSteps + coastingSteps + decelerationSteps;

		// Calculate the delay between steps
		uint32_t startingSpeedDelayMicroseconds =

			std::vector<StepTiming>
			Generator3d::GenerateSteps(const Vec3Int32 &start, const Vec3Int32 &end,
									   const Vector<uint16_t> startingSpeedDirection,
									   const uint16_t requestedStraightLineSpeed,
									   const uint16_t requestedEndingStraightLineSpeed)
		{

			// uint16_t speed = requestedStraightLineSpeed;

			if (start == end)
			{
				return {};
			}
			std::vector<StepTiming> steps;

			// Determine if any of the axis are going to limit the speed, and if so, scale
			// the speed by the ratio of the desired speed to the maximum speed

			float speedScale = 1.0;

			Vector<uint16_t> requestedComponentSpeed =
				GetComponentSpeeds(requestedStraightLineSpeed, start, end);
			Vector<uint16_t> requestedEndingComponentSpeed =
				GetComponentSpeeds(requestedEndingStraightLineSpeed, start, end);

			for (int i = 0; i < 3; i++)
			{

				if (requestedComponentSpeed[i] > axisLimits[i].maxSpeed)
				{
					float scale = static_cast<float>(axisLimits[i].maxSpeed) /
								  static_cast<float>(requestedComponentSpeed[i]);
					if (scale < speedScale)
					{
						speedScale = scale;
					}
				}
			}

			// if an axis restricts maximum speed, scale the speed for all axes and get
			// their component speeds again
			if (speedScale < 1.0)
			{
				auto startingSpeed = std::round(requestedStraightLineSpeed * speedScale);
				requestedComponentSpeed = GetComponentSpeeds(startingSpeed, start, end);

				auto endingSpeed = std::round(requestedEndingStraightLineSpeed * speedScale);
				requestedEndingComponentSpeed = GetComponentSpeeds(endingSpeed, start, end);
			}

			// get a vector full of deltas between one position and the next position
			// (the steps to execute in order, basically)
			BresenhamPoints points = Bresenham3D(start, end);

			uint32_t accelerationSteps = 0;
			uint32_t decelerationSteps = 0;
			uint32_t coastingSteps = 0;
			uint32_t totalSteps = points.points.size();
			uint32_t plannedSteps = totalSteps;

			// Determine limiting acceleration

			uint16_t acceleration = axisLimits[points.drivingAxis].maxAcceleration;
			uint16_t deceleration = axisLimits[points.drivingAxis].maxDeceleration;

			// calculate acceleration for every axis
			for (int i = 0; i != 3; i++)
			{
				if (requestedComponentSpeed[i] == 0)
				{
					continue;
				}

				float movementRatio =
					static_cast<float>(requestedComponentSpeed[i]) /
					static_cast<float>(requestedComponentSpeed[points.drivingAxis]);
				uint16_t accel = acceleration * movementRatio;
				uint16_t decel = deceleration * movementRatio;

				if (accel > axisLimits[i].maxAcceleration)
				{
					acceleration = axisLimits[i].maxAcceleration;
				}

				if (decel > axisLimits[i].maxDeceleration)
				{
					deceleration = axisLimits[i].maxDeceleration;
				}
			}

			Vec3Int8List deltas = PointsToDeltas(points.points);

			MovementProfile movementProfile = CalculateMovementProfile(
				deltas.size(), startingSpeedDirection[points.drivingAxis],
				requestedComponentSpeed[points.drivingAxis],
				requestedEndingComponentSpeed[points.drivingAxis], acceleration,
				deceleration);

			uint32_t startingSpeedDelayMicroseconds =
				startingSpeedDirection[points.drivingAxis]
					? SecondsToMicroseconds(startingSpeedDirection[points.drivingAxis])
					: SecondsToMicroseconds(1);

			uint32_t endingSpeedDelayMicroseconds =
				requestedEndingComponentSpeed[points.drivingAxis]
					? SecondsToMicroseconds(
						  requestedEndingComponentSpeed[points.drivingAxis])
					: SecondsToMicroseconds(1);

			uint32_t constantSpeedDelayMicroseconds =
				SecondsToMicroseconds(1.0 / requestedComponentSpeed[points.drivingAxis]);

			uint32_t currentDelay = startingSpeedDelayMicroseconds;

			uint32_t accelerationDelayDiffMicroseconds =
				(startingSpeedDelayMicroseconds - constantSpeedDelayMicroseconds);

			uint32_t decelerationDelayDiffMicroseconds =
				(endingSpeedDelayMicroseconds - constantSpeedDelayMicroseconds);

			uint32_t deltaIndex = 0;

			for (Profile section : movementProfile)
			{
				ProfileType type = section.type;

				for (int i = 0; i != section.steps; i++)
				{
					switch (type)
					{

					// calculate delay
					case ProfileType::ACCELERATION:
					{
						// √(2*i/a) - √(2*(i-1)/a)
						uint32_t increment = std::round(
							(1.0 * accelerationDelayDiffMicroseconds) / section.steps);
						currentDelay -= increment;
						if (currentDelay < constantSpeedDelayMicroseconds)
						{
							currentDelay = constantSpeedDelayMicroseconds;
						}
					}
					break;
					case ProfileType::COASTING:
						currentDelay = constantSpeedDelayMicroseconds;
						break;
					case ProfileType::DECELLERATION:
					{
						uint32_t increment = std::round(
							(1.0 * decelerationDelayDiffMicroseconds) / section.steps);
						currentDelay += increment;

						if (currentDelay > endingSpeedDelayMicroseconds)
						{
							currentDelay = endingSpeedDelayMicroseconds;
						}
					}
					break;
					}
					// create StepTiming step
					StepTiming step(deltas[deltaIndex], currentDelay);

					steps.push_back(step);
					deltaIndex++;
				}
			}

			// make the very last one have no delay. as I don't know how  to handle this
			// if we chained more steps on, it would start with a step, not a delay, so
			// this one should probably end in a delayanyway.

			// modified to be the same delay as the previous step because a zero delay
			// violates the maximum speed rules in other areas
			steps.back().delay = steps.at(steps.size() - 2).delay;
			return steps;
		}

		uint16_t Generator3d::GetAcclerationStepCount(
			const Vector<uint16_t> startingSpeeds,
			const Vector<uint16_t> requestedComponentSpeeds,
			const uint8_t drivingAxis) const
		{
			uint16_t requestedAcceleration = axisLimits[drivingAxis].maxAcceleration;
			Vector<uint16_t> speedDelta;
			std::transform(requestedComponentSpeeds.begin(),
						   requestedComponentSpeeds.end(), startingSpeeds.begin(),
						   speedDelta.begin(), std::minus<uint16_t>());
			uint32_t drivingAxisAccelerationSteps = 0;

			for (int i = 0; i < speedDelta.size(); i++)
			{
				uint32_t componentRequiredAccelerationSteps =
					static_cast<uint16_t>(std::ceil(static_cast<double>(speedDelta[i]) /
													axisLimits[i].maxAcceleration));
				if (componentRequiredAccelerationSteps > drivingAxisAccelerationSteps)
				{
					drivingAxisAccelerationSteps = componentRequiredAccelerationSteps;
				}
			}

			return drivingAxisAccelerationSteps;
		}

		Vector<uint16_t> Generator3d::GetComponentSpeeds(const uint16_t straightLineSpeed,
														 const Vec3Int32 &start,
														 const Vec3Int32 &end) const
		{

			// Calculate the movement along each axis
			Vec3Int32 componentDistance = GetComponentDistances(start, end);
			// Calculate the total distance
			double straightLineDistance = GetStraightLineDistance(start, end);

			// Calculate speed along each axis
			std::array<uint16_t, 3> speeds;
			std::transform(
				componentDistance.begin(), componentDistance.end(), speeds.begin(),
				[straightLineDistance, straightLineSpeed](int32_t move) -> uint16_t
				{
					if (straightLineDistance == 0)
					{
						return 0;
					}
					double speed = (static_cast<double>(move) / straightLineDistance) *
								   straightLineSpeed;
					return static_cast<uint16_t>(std::round(speed));
				});

			return speeds;
		}

		Vec3Int32 Generator3d::GetComponentDistances(const Vec3Int32 &start,
													 const Vec3Int32 &end) const
		{
			Vec3Int32 movement;
			std::transform(end.begin(), end.end(), start.begin(), movement.begin(),
						   [](int a, int b)
						   { return std::abs(a - b); });
			return movement;
		}

		double Generator3d::GetStraightLineDistance(const Vec3Int32 &first,
													const Vec3Int32 &second) const
		{
			double sumOfSquares = std::inner_product(
				first.begin(), first.end(), second.begin(), 0.0, std::plus<double>(),
				[](int a, int b)
				{ return std::pow(b - a, 2); });

			return std::sqrt(sumOfSquares);
		}

	} // namespace StepTimingPlanner