#pragma once

#include "StepPlanner.hpp"
#include <bits/stdint-uintn.h>
#include <cmath>

inline uint32_t SecondsToMicroseconds(float seconds) {
  return std::round(seconds * 1000000.0);
}

enum class Axis : uint8_t { X = 0, Y = 1, Z = 2 };

struct AxisLimits {
  uint16_t maxAcceleration;
  uint16_t maxDeceleration;
  uint16_t maxSpeed;
};

struct StepTiming {
  StepTiming(Vector3Int32 aDelta, uint32_t aDelay)
      : delta(aDelta), delay(aDelay) {}
  // which axis to increment
  Vector3Int32 delta;
  // delay in microseconds to wait after this step is executed (optional if last
  // step)
  uint32_t delay;
};

class StepTimingGenerator {
public:
  StepTimingGenerator(AxisLimits x, AxisLimits y, AxisLimits z);
  std::vector<StepTiming>
  GenerateSteps(const Vector3Int32 &start, const Vector3Int32 &end,
                const Vector<uint16_t> startingSpeedDirection,
                const uint16_t requestedStraightLineSpeed,
                const uint16_t requestedEndingStraightLineSpeed);

protected:
  uint16_t
  GetAcclerationStepCount(const Vector<uint16_t> startingSpeeds,
                          const Vector<uint16_t> requestedComponentSpeeds,
                          const uint8_t drivingAxis) const;

  Vector<uint16_t> GetComponentSpeeds(const uint16_t straightLineSpeed,
                                      const Vector3Int32 &start,
                                      const Vector3Int32 &end) const;

  Vector3Int32 GetComponentDistances(const Vector3Int32 &start,
                                     const Vector3Int32 &end) const;

  double GetStraightLineDistance(const Vector3Int32 &first,
                                 const Vector3Int32 &second) const;

  AxisLimits axisLimits[3];
};
