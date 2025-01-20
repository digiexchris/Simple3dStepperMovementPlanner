#include "MovementProfile.hpp" // Replace with the actual header file name
#include "gtest/gtest.h"
#include <stdexcept>

using namespace StepTimingPlanner;

TEST(StepProfileTest, SufficientSteps) {
  MovementProfile profile =
      CalculateMovementProfile(1000, 10.0, 50.0, 10.0, 5.0, 5.0);
  EXPECT_EQ(profile[0].steps, 240); // Expected from calculation
  EXPECT_EQ(profile[0].type, ProfileType::ACCELERATION);
  EXPECT_EQ(profile[1].steps, 520); // Expected from calculation
  EXPECT_EQ(profile[1].type, ProfileType::COASTING);
  EXPECT_EQ(profile[2].steps, 240); // Expected from calculation
  EXPECT_EQ(profile[2].type, ProfileType::DECELLERATION);
  EXPECT_EQ(profile[0].steps + profile[1].steps + profile[2].steps, 1000);
}

TEST(StepProfileTest, InsufficientSteps) {
  MovementProfile profile =
      CalculateMovementProfile(100, 10.0, 100.0, 10.0, 5.0, 5.0);
  EXPECT_EQ(profile[0].steps,
            47); // Proportionally adjusted from expected 400
  EXPECT_EQ(profile[0].type, ProfileType::ACCELERATION);
  EXPECT_EQ(profile[1].steps, 6); // Proportionally adjusted from expected 400
  EXPECT_EQ(profile[1].type, ProfileType::COASTING);
  EXPECT_EQ(profile[2].steps, 47); // Proportionally adjusted from expected 400
  EXPECT_EQ(profile[2].type, ProfileType::DECELLERATION);
  EXPECT_EQ(profile[0].steps + profile[1].steps + profile[2].steps, 100);
}

TEST(StepProfileTest, EqualStartAndCoastSpeed) {
  MovementProfile profile =
      CalculateMovementProfile(500, 50.0, 50.0, 10.0, 5.0, 5.0);
  EXPECT_EQ(profile[0].steps, 260); // Total - Deceleration steps
  EXPECT_EQ(profile[0].type, ProfileType::COASTING);
  EXPECT_EQ(profile[1].steps, 240); // Expected from calculation
  EXPECT_EQ(profile[1].type, ProfileType::DECELLERATION);
  EXPECT_EQ(profile[0].steps + profile[1].steps, 500);
}

TEST(StepProfileTest, EqualEndAndCoastSpeed) {
  MovementProfile profile =
      CalculateMovementProfile(500, 10.0, 50.0, 50.0, 5.0, 5.0);
  EXPECT_EQ(profile[0].steps, 240); // Expected from calculation
  EXPECT_EQ(profile[0].type, ProfileType::ACCELERATION);
  EXPECT_EQ(profile[1].steps, 260); // Total - Acceleration steps
  EXPECT_EQ(profile[1].type, ProfileType::COASTING);
  EXPECT_EQ(profile.size(), 2); // No deceleration needed
  EXPECT_EQ(profile[0].steps + profile[1].steps, 500);
}

TEST(StepProfileTest, StartSpeedGreaterThanCoastSpeed) {
  EXPECT_THROW(CalculateMovementProfile(500, 60.0, 50.0, 10.0, 5.0, 5.0),
               std::runtime_error);
}

TEST(StepProfileTest, EndSpeedGreaterThanCoastSpeed) {
  EXPECT_THROW(CalculateMovementProfile(500, 10.0, 50.0, 60.0, 5.0, 5.0),
               std::runtime_error);
}

TEST(StepProfileTest, StartSpeedLessThanCoastSpeed) {
  MovementProfile profile =
      CalculateMovementProfile(500, 10.0, 50.0, 50.0, 5.0, 5.0);
  EXPECT_EQ(profile[0].steps, 240); // Expected from calculation
  EXPECT_EQ(profile[0].type, ProfileType::ACCELERATION);
  EXPECT_EQ(profile[1].steps, 260); // Remainder after acceleration
  EXPECT_EQ(profile[1].type, ProfileType::COASTING);
  EXPECT_EQ(profile.size(), 2); // No deceleration needed
  EXPECT_EQ(profile[0].steps + profile[1].steps, 500);
}

TEST(StepProfileTest, EndSpeedLessThanCoastSpeed) {
  MovementProfile profile =
      CalculateMovementProfile(500, 50.0, 50.0, 10.0, 5.0, 5.0);
  EXPECT_EQ(profile[0].steps, 260); // Total - Deceleration steps
  EXPECT_EQ(profile[0].type, ProfileType::COASTING);
  EXPECT_EQ(profile[1].steps, 240); // Expected from calculation
  EXPECT_EQ(profile[1].type, ProfileType::DECELLERATION);
  EXPECT_EQ(profile.size(), 2); // No acceleration needed
  EXPECT_EQ(profile[0].steps + profile[1].steps, 500);
}

TEST(StepProfileTest, FewerendStepsThanstartSteps) {
  MovementProfile profile =
      CalculateMovementProfile(150, 10.0, 50.0, 30.0, 5.0, 5.0);
  EXPECT_EQ(profile[0].steps, 112); // Proportional calculation
  EXPECT_EQ(profile[0].type, ProfileType::ACCELERATION);
  EXPECT_EQ(profile[1].steps, 6); // Proportional calculation
  EXPECT_EQ(profile[1].type, ProfileType::COASTING);
  EXPECT_EQ(profile[2].steps, 32); // Proportional calculation
  EXPECT_EQ(profile[2].type, ProfileType::DECELLERATION);
  EXPECT_EQ(profile.size(), 3); // No coasting needed
  EXPECT_EQ(profile[0].steps + profile[1].steps + profile[2].steps, 150);
}

TEST(StepProfileTest, SameEndStepsThanstartSteps_less_than_total) {
  MovementProfile profile =
      CalculateMovementProfile(150, 10.0, 50.0, 10.0, 5.0, 5.0);
  EXPECT_EQ(profile[0].steps, 74); // Proportional calculation
  EXPECT_EQ(profile[0].type, ProfileType::ACCELERATION);
  EXPECT_EQ(profile[1].steps, 2); // Proportional calculation
  EXPECT_EQ(profile[1].type, ProfileType::COASTING);
  EXPECT_EQ(profile[2].steps, 74); // Proportional calculation
  EXPECT_EQ(profile[2].type, ProfileType::DECELLERATION);
  EXPECT_EQ(profile.size(), 3); // No coasting needed
  EXPECT_EQ(profile[0].steps + profile[1].steps + profile[2].steps, 150);
}

TEST(StepProfileTest, Normal_Zero_to_Zero_even_accel) {
  MovementProfile profile =
      CalculateMovementProfile(1000, 0.0, 50.0, 0.0, 5.0, 5.0);
  EXPECT_EQ(profile[0].steps, 250); // Proportional calculation
  EXPECT_EQ(profile[0].type, ProfileType::ACCELERATION);
  EXPECT_EQ(profile[1].steps, 500); // Proportional calculation
  EXPECT_EQ(profile[1].type, ProfileType::COASTING);
  EXPECT_EQ(profile[2].steps, 250); // Proportional calculation
  EXPECT_EQ(profile[2].type, ProfileType::DECELLERATION);
  EXPECT_EQ(profile.size(), 3); // No coasting needed
  EXPECT_EQ(profile[0].steps + profile[1].steps + profile[2].steps, 1000);
}

TEST(StepProfileTest, Normal_Zero_to_Zero) {
  MovementProfile profile =
      CalculateMovementProfile(1000, 0.0, 50.0, 0.0, 5.0, 15.0);
  EXPECT_EQ(profile[0].steps, 250); // Proportional calculation
  EXPECT_EQ(profile[0].type, ProfileType::ACCELERATION);
  EXPECT_EQ(profile[1].steps, 667); // Proportional calculation
  EXPECT_EQ(profile[1].type, ProfileType::COASTING);
  EXPECT_EQ(profile[2].steps, 83); // Proportional calculation
  EXPECT_EQ(profile[2].type, ProfileType::DECELLERATION);
  EXPECT_EQ(profile.size(), 3); // No coasting needed
  EXPECT_EQ(profile[0].steps + profile[1].steps + profile[2].steps, 1000);
}
