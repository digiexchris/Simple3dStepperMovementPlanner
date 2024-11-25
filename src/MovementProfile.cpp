#include "MovementProfile.hpp"
#include <algorithm>
#include <bits/stdint-intn.h>
#include <bits/stdint-uintn.h>
#include <cmath>
#include <exception>
#include <stdexcept>

uint32_t GetAccelerationDistance(uint16_t startSpeed, uint16_t endSpeed,
                                 uint16_t maxAcceleration) {
  double distance =
      static_cast<double>(endSpeed * endSpeed - startSpeed * startSpeed) /
      (2.0 * maxAcceleration);

  uint32_t accelerationSteps = static_cast<uint32_t>(std::max(0.0, distance));

  return accelerationSteps;
}

MovementProfile CalculateMovementProfile(uint32_t totalSteps,
                                         uint16_t startSpeed,
                                         uint16_t coastSpeed, uint16_t endSpeed,
                                         uint16_t maxAcceleration,
                                         uint16_t maxDeceleration) {
  if (startSpeed > coastSpeed) {
    throw std::runtime_error(
        "start speed must be equal or less than coast speed");
  }

  if (coastSpeed < endSpeed) {
    throw std::runtime_error(
        "end speed must be equal or less than coast speed");
  }

  std::vector<Profile> profile;
  uint32_t startSteps = 0;
  uint32_t endSteps = 0;
  uint32_t coastingSteps = 0;
  uint32_t requiredAccelDecelSteps = 0;

  // if (startSpeed < coastSpeed) {
  //   startSteps =
  //       GetAccelerationDistance(startSpeed, coastSpeed, maxAcceleration);
  // }

  // if (coastSpeed > endSpeed) {
  //   endSteps = GetAccelerationDistance(endSpeed, coastSpeed,
  //   maxDeceleration);
  // }

  // requiredAccelDecelSteps = startSteps + endSteps;

  // Calculate the distance required to accelerate to the coast speed
  uint32_t accelDistance =
      std::floor((coastSpeed * coastSpeed - startSpeed * startSpeed) /
                 (2.0 * maxAcceleration));

  // Calculate the distance required to decelerate from the coast speed to the
  // end speed
  uint32_t decelDistance =
      std::floor((coastSpeed * coastSpeed - endSpeed * endSpeed) /
                 (2.0 * maxDeceleration));

  uint32_t accelPlusDecel = accelDistance + decelDistance;

  if (accelPlusDecel <= totalSteps) {
    // There is enough distance to reach the coast speed and decelerate
    startSteps = accelDistance;
    endSteps = decelDistance;
    coastingSteps = totalSteps - accelPlusDecel;
    // profile.finalSpeedAfterAccel = coastSpeed;
  } else {
    // Not enough distance for full accel and decel, calculate the maximum
    // possible speed within the distance
    uint16_t adjustedSpeed = std::floor(
        std::sqrt((2.0 * maxAcceleration * maxDeceleration * totalSteps +
                   maxDeceleration * startSpeed * startSpeed +
                   maxAcceleration * endSpeed * endSpeed) /
                  (maxAcceleration + maxDeceleration)));
    adjustedSpeed = std::floor(
        std::min(adjustedSpeed, coastSpeed)); // Ensure the speed doesn't exceed
                                              // the requested coast speed
    coastSpeed = adjustedSpeed;
    // Recalculate distances based on the adjusted speed
    startSteps = (adjustedSpeed * adjustedSpeed - startSpeed * startSpeed) /
                 (2.0 * maxAcceleration);
    endSteps = (adjustedSpeed * adjustedSpeed - endSpeed * endSpeed) /
               (2.0 * maxDeceleration);
    coastingSteps = totalSteps - startSteps -
                    endSteps; // No coasting when the profile is truncated but
                              // adjust for rounding error
  }

  // if (requiredAccelDecelSteps > totalSteps) {
  //   if (startSteps > endSteps) {

  //     if (startSteps > totalSteps) {
  //       startSteps = totalSteps;
  //       endSteps = 0;
  //       coastingSteps = 0;
  //     } else {
  //       coastingSteps = totalSteps - startSteps;
  //     }
  //   }

  //   if (endSteps > startSteps) {
  //     // not lots of acceleration, but lots of deceleration. like, if we're
  //     // starting fast.
  //     endSteps = endSteps - startSteps;
  //     if (endSteps > totalSteps) {
  //       startSteps = 0;
  //       endSteps = totalSteps;
  //       coastingSteps = 0;
  //     } else {
  //       coastingSteps = totalSteps - endSteps;
  //     }
  //   }

  //   if (endSteps == startSteps) {
  //     if (endSteps + startSteps > totalSteps) {
  //       endSteps = std::floor(totalSteps / 2.0);
  //       startSteps = std::floor(totalSteps / 2.0);
  //       coastingSteps =
  //           totalSteps - endSteps - startSteps; // deal with rounding error
  //     }
  //   }
  // }

  if (startSteps > 0) {
    Profile startProfile;
    startProfile.steps = startSteps;
    startProfile.fromSpeed = startSpeed;
    startProfile.toSpeed = coastSpeed;
    startProfile.type = ProfileType::ACCELERATION;
    profile.emplace_back(startProfile);
  }

  if (coastingSteps > 0) {
    Profile coastProfile;
    coastProfile.steps = coastingSteps;
    coastProfile.fromSpeed = coastSpeed;
    coastProfile.toSpeed = coastSpeed;
    coastProfile.type = ProfileType::COASTING;
    profile.emplace_back(coastProfile);
  }

  if (endSteps > 0) {
    Profile endProfile;
    endProfile.steps = endSteps;
    endProfile.fromSpeed = coastSpeed;
    endProfile.toSpeed = endSpeed;
    endProfile.type = ProfileType::DECELLERATION;
    profile.emplace_back(endProfile);
  }

  return profile;
}
