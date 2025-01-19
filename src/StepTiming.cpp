#include "StepTiming.hpp"
#include "MovementProfile.hpp"
#include "StepPlanner.hpp"
#include "Types.hpp"
#include <algorithm>
#include <bits/stdint-uintn.h>
#include <cmath>
#include <functional>
#include <numeric>
#include <sys/types.h>

StepTimingGenerator::StepTimingGenerator(AxisLimits x, AxisLimits y,
                                         AxisLimits z) {
  axisLimits[0] = x;
  axisLimits[1] = y;
  axisLimits[2] = z;
}

std::vector<StepTiming> StepTimingGenerator::GenerateSteps(
    const Vector3Int32 &start, const Vector3Int32 &end,
    const Vector<uint16_t> startingSpeedDirection,
    const uint16_t requestedStraightLineSpeed,
    const uint16_t requestedEndingStraightLineSpeed) {

  uint16_t speed = requestedStraightLineSpeed;

  if (start == end) {
    return {};
  }
  std::vector<StepTiming> steps;

  // Determine if any of the axis are going to limit the speed, and if so, scale
  // the speed by the ratio of the desired speed to the maximum speed

  float speedScale = 1.0;

  Vector<uint16_t> requestedComponentSpeed =
      GetComponentSpeeds(speed, start, end);

  for (int i = 0; i < 3; i++) {

    if (requestedComponentSpeed[i] > axisLimits[i].maxSpeed) {
      float scale = static_cast<float>(axisLimits[i].maxSpeed) /
                    static_cast<float>(requestedComponentSpeed[i]);
      if (scale < speedScale) {
        speedScale = scale;
      }
    }
  }

  // if an axis restricts maximum speed, scale the speed for all axes and get
  // their component speeds again
  if (speedScale < 1.0) {
    speed = std::round(speed * speedScale);
    requestedComponentSpeed = GetComponentSpeeds(speed, start, end);
  }

  Vector<uint16_t> requestedEndingComponentSpeeds =
      GetComponentSpeeds(requestedEndingStraightLineSpeed, start, end);

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
  for (int i = 0; i != 3; i++) {
    if (requestedComponentSpeed[i] == 0) {
      continue;
    }

    float movementRatio =
        static_cast<float>(requestedComponentSpeed[i]) /
        static_cast<float>(requestedComponentSpeed[points.drivingAxis]);
    uint16_t accel = acceleration * movementRatio;
    uint16_t decel = deceleration * movementRatio;

    if (accel > axisLimits[i].maxAcceleration) {
      acceleration = axisLimits[i].maxAcceleration;
    }

    if (decel > axisLimits[i].maxDeceleration) {
      deceleration = axisLimits[i].maxDeceleration;
    }
  }

  Vector3Int8List deltas = PointsToDeltas(points.points);

  MovementProfile movementProfile = CalculateMovementProfile(
      deltas.size(), startingSpeedDirection[points.drivingAxis],
      requestedComponentSpeed[points.drivingAxis],
      requestedEndingComponentSpeeds[points.drivingAxis], acceleration,
      deceleration);

  uint32_t startingSpeedDelayMicroseconds =
      startingSpeedDirection[points.drivingAxis]
          ? SecondsToMicroseconds(startingSpeedDirection[points.drivingAxis])
          : SecondsToMicroseconds(1);

  uint32_t endingSpeedDelayMicroseconds =
      requestedEndingComponentSpeeds[points.drivingAxis]
          ? SecondsToMicroseconds(
                requestedEndingComponentSpeeds[points.drivingAxis])
          : SecondsToMicroseconds(1);

  uint32_t constantSpeedDelayMicroseconds =
      SecondsToMicroseconds(1.0 / requestedComponentSpeed[points.drivingAxis]);

  uint32_t currentDelay = startingSpeedDelayMicroseconds;

  uint32_t accelerationDelayDiffMicroseconds =
      (startingSpeedDelayMicroseconds - constantSpeedDelayMicroseconds);

  uint32_t decelerationDelayDiffMicroseconds =
      (endingSpeedDelayMicroseconds - constantSpeedDelayMicroseconds);

  uint32_t deltaIndex = 0;

  for (Profile section : movementProfile) {
    ProfileType type = section.type;

    for (int i = 0; i != section.steps; i++) {
      switch (type) {

      // calculate delay
      case ProfileType::ACCELERATION: {
        // √(2*i/a) - √(2*(i-1)/a)
        uint32_t increment = std::round(
            (1.0 * accelerationDelayDiffMicroseconds) / section.steps);
        currentDelay -= increment;
        if (currentDelay < constantSpeedDelayMicroseconds) {
          currentDelay = constantSpeedDelayMicroseconds;
        }
      } break;
      case ProfileType::COASTING:
        currentDelay = constantSpeedDelayMicroseconds;
        break;
      case ProfileType::DECELLERATION: {
        uint32_t increment = std::round(
            (1.0 * decelerationDelayDiffMicroseconds) / section.steps);
        currentDelay += increment;

        if (currentDelay > endingSpeedDelayMicroseconds) {
          currentDelay = endingSpeedDelayMicroseconds;
        }
      } break;
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

uint16_t StepTimingGenerator::GetAcclerationStepCount(
    const Vector<uint16_t> startingSpeeds,
    const Vector<uint16_t> requestedComponentSpeeds,
    const uint8_t drivingAxis) const {
  uint16_t requestedAcceleration = axisLimits[drivingAxis].maxAcceleration;
  Vector<uint16_t> speedDelta;
  std::transform(requestedComponentSpeeds.begin(),
                 requestedComponentSpeeds.end(), startingSpeeds.begin(),
                 speedDelta.begin(), std::minus<uint16_t>());
  uint32_t drivingAxisAccelerationSteps = 0;

  for (int i = 0; i < speedDelta.size(); i++) {
    uint32_t componentRequiredAccelerationSteps =
        static_cast<uint16_t>(std::ceil(static_cast<double>(speedDelta[i]) /
                                        axisLimits[i].maxAcceleration));
    if (componentRequiredAccelerationSteps > drivingAxisAccelerationSteps) {
      drivingAxisAccelerationSteps = componentRequiredAccelerationSteps;
    }
  }

  return drivingAxisAccelerationSteps;
}

Vector<uint16_t>
StepTimingGenerator::GetComponentSpeeds(const uint16_t straightLineSpeed,
                                        const Vector3Int32 &start,
                                        const Vector3Int32 &end) const {

  // Calculate the movement along each axis
  Vector3Int32 componentDistance = GetComponentDistances(start, end);
  // Calculate the total distance
  double straightLineDistance = GetStraightLineDistance(start, end);

  // Calculate speed along each axis
  std::array<uint16_t, 3> speeds;
  std::transform(
      componentDistance.begin(), componentDistance.end(), speeds.begin(),
      [straightLineDistance, straightLineSpeed](int32_t move) -> uint16_t {
        if (straightLineDistance == 0) {
          return 0;
        }
        double speed = (static_cast<double>(move) / straightLineDistance) *
                       straightLineSpeed;
        return static_cast<uint16_t>(std::round(speed));
      });

  return speeds;
}

Vector3Int32
StepTimingGenerator::GetComponentDistances(const Vector3Int32 &start,
                                           const Vector3Int32 &end) const {
  Vector3Int32 movement;
  std::transform(end.begin(), end.end(), start.begin(), movement.begin(),
                 [](int a, int b) { return std::abs(a - b); });
  return movement;
}

double
StepTimingGenerator::GetStraightLineDistance(const Vector3Int32 &first,
                                             const Vector3Int32 &second) const {
  double sumOfSquares = std::inner_product(
      first.begin(), first.end(), second.begin(), 0.0, std::plus<double>(),
      [](int a, int b) { return std::pow(b - a, 2); });

  return std::sqrt(sumOfSquares);
}
