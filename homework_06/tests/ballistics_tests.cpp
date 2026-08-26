#include "ballistics.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

// Fixed test values mirror the original homework examples.
// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)
TEST(Ballistics, ComputesKnownVog17DropPoint)
{
  const BallisticsInput kInput{
    .drone_x_ = 100.0,
    .drone_y_ = 100.0,
    .drone_z_ = 100.0,
    .target_x_ = 200.0,
    .target_y_ = 200.0,
    .attack_speed_ = 10.0,
    .acceleration_path_ = 10.0,
    .ammo_name_ = "VOG-17",
  };

  const DropSolution kSolution = compute_drop_solution(kInput);

  EXPECT_FALSE(kSolution.maneuver_required_);
  EXPECT_NEAR(kSolution.time_s_, 5.74976, 0.0001);
  EXPECT_NEAR(kSolution.horizontal_fall_m_, 37.1102, 0.0001);
  EXPECT_NEAR(kSolution.fire_x_, 173.759, 0.001);
  EXPECT_NEAR(kSolution.fire_y_, 173.759, 0.001);
}

TEST(Ballistics, RejectsUnknownAmmo)
{
  const BallisticsInput kInput{
    .drone_x_ = 100.0,
    .drone_y_ = 100.0,
    .drone_z_ = 100.0,
    .target_x_ = 200.0,
    .target_y_ = 200.0,
    .attack_speed_ = 10.0,
    .acceleration_path_ = 10.0,
    .ammo_name_ = "UNKNOWN",
  };

  EXPECT_THROW(static_cast<void>(compute_drop_solution(kInput)), std::runtime_error);
}

TEST(Ballistics, ComputesManeuverPoint)
{
  const BallisticsInput kInput{
    .drone_x_ = 100.0,
    .drone_y_ = 100.0,
    .drone_z_ = 100.0,
    .target_x_ = 120.0,
    .target_y_ = 120.0,
    .attack_speed_ = 10.0,
    .acceleration_path_ = 10.0,
    .ammo_name_ = "VOG-17",
  };

  const DropSolution kSolution = compute_drop_solution(kInput);

  EXPECT_TRUE(kSolution.maneuver_required_);
  EXPECT_NEAR(kSolution.intermediate_x_, 86.688, 0.001);
  EXPECT_NEAR(kSolution.intermediate_y_, 86.688, 0.001);
  EXPECT_NEAR(kSolution.fire_x_, 93.7591, 0.001);
  EXPECT_NEAR(kSolution.fire_y_, 93.7591, 0.001);
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)
