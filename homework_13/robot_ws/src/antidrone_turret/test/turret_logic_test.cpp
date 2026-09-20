#include <gtest/gtest.h>

#include "antidrone_turret/turret_logic.hpp"

namespace {

TEST(TurretLogicTest, LowConfidenceTargetIsIdleAndSkipsTrigger)
{
  const auto status = antidrone_turret::TurretLogic::make_status(
    antidrone_turret::TargetInput{
      true,
      320.0F,
      240.0F,
      20.0F,
      0.79F,
    },
    0.80F,
    30.0F,
    true);

  EXPECT_EQ(
    status.target_state,
    antidrone_turret::TargetState::kLowConfidence);
  EXPECT_EQ(
    status.action,
    antidrone_turret::TurretAction::kIdle);
  EXPECT_EQ(
    status.trigger_state,
    antidrone_turret::TriggerState::kSkip);
}

TEST(TurretLogicTest, ServoTurnsRightForTargetRightOfCenter)
{
  const auto command =
    antidrone_turret::TurretLogic::make_servo_command(420.0F);

  EXPECT_EQ(
    command.direction,
    antidrone_turret::ServoDirection::kRight);
  EXPECT_FLOAT_EQ(command.target_x, 420.0F);
  EXPECT_FLOAT_EQ(command.error_x, 100.0F);
}

TEST(TurretLogicTest, GimbalMovesUpForTargetAboveCenter)
{
  const auto command =
    antidrone_turret::TurretLogic::make_gimbal_command(180.0F);

  EXPECT_EQ(
    command.direction,
    antidrone_turret::GimbalDirection::kUp);
  EXPECT_FLOAT_EQ(command.target_y, 180.0F);
  EXPECT_FLOAT_EQ(command.error_y, 60.0F);
}

TEST(TurretLogicTest, CloseTargetRequestsTriggerWhenActuatorReady)
{
  const auto state =
    antidrone_turret::TurretLogic::evaluate_trigger(
      25.0F,
      30.0F,
      true);

  EXPECT_EQ(
    state,
    antidrone_turret::TriggerState::kRequested);
}

TEST(TurretLogicTest, CloseTargetReportsReloadingWhenActuatorNotReady)
{
  const auto state =
    antidrone_turret::TurretLogic::evaluate_trigger(
      25.0F,
      30.0F,
      false);

  EXPECT_EQ(
    state,
    antidrone_turret::TriggerState::kReloading);
}

TEST(TurretLogicTest, FarValidTargetTracksAndSkipsTrigger)
{
  const auto status = antidrone_turret::TurretLogic::make_status(
    antidrone_turret::TargetInput{
      true,
      420.0F,
      180.0F,
      50.0F,
      0.90F,
    },
    0.80F,
    30.0F,
    true);

  EXPECT_EQ(
    status.target_state,
    antidrone_turret::TargetState::kLocked);
  EXPECT_EQ(
    status.action,
    antidrone_turret::TurretAction::kTrack);
  EXPECT_EQ(
    status.trigger_state,
    antidrone_turret::TriggerState::kSkip);
}

}  // namespace
