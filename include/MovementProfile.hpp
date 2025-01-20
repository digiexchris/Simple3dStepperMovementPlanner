#pragma once

#include <cmath>
#include <stdint.h>
#include <vector>

namespace StepTimingPlanner {

enum class ProfileType { ACCELERATION, DECELLERATION, COASTING };

struct Profile {
  int32_t steps;
  ProfileType type;
  uint16_t fromSpeed;
  uint16_t toSpeed;
};

using MovementProfile = std::vector<Profile>;

MovementProfile CalculateMovementProfile(uint32_t totalSteps,
                                         uint16_t startSpeed,
                                         uint16_t coastSpeed, uint16_t endSpeed,
                                         uint16_t maxAcceleration,
                                         uint16_t maxDeceleration);

} // namespace StepTimingPlanner