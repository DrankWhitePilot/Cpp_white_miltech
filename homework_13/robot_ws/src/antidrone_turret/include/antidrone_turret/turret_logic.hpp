#pragma once

#include <cstdint>

namespace antidrone_turret {

enum class TargetState : std::uint8_t {
  kNone = 0,
  kLowConfidence = 1,
  kLocked = 2,
};

enum class TurretAction : std::uint8_t {
  kIdle = 0,
  kTrack = 1,
};

enum class TriggerState : std::uint8_t {
  kSkip = 0,
  kRequested = 1,
  kReloading = 2,
};

enum class ServoDirection : std::int8_t {
  kLeft = -1,
  kCenter = 0,
  kRight = 1,
};

enum class GimbalDirection : std::int8_t {
  kDown = -1,
  kCenter = 0,
  kUp = 1,
};

struct TargetInput {
  bool visible{false};
  float x{0.0F};
  float y{0.0F};
  float distance_m{0.0F};
  float confidence{0.0F};
};

struct ServoCommandData {
  ServoDirection direction{ServoDirection::kCenter};
  float target_x{0.0F};
  float error_x{0.0F};
};

struct GimbalCommandData {
  GimbalDirection direction{GimbalDirection::kCenter};
  float target_y{0.0F};
  float error_y{0.0F};
};

struct TurretStatusData {
  TargetState target_state{TargetState::kNone};
  TurretAction action{TurretAction::kIdle};
  TriggerState trigger_state{TriggerState::kSkip};
  float confidence{0.0F};
  float distance_m{0.0F};
};

class TurretLogic {
public:
  [[nodiscard]] static TargetState evaluate_target(
    bool visible,
    float confidence,
    float confidence_threshold)
  {
    if (!visible) {
      return TargetState::kNone;
    }

    if (confidence < confidence_threshold) {
      return TargetState::kLowConfidence;
    }

    return TargetState::kLocked;
  }

  [[nodiscard]] static ServoCommandData make_servo_command(float target_x)
  {
    const float error_x = target_x - 320.0F;

    auto direction = ServoDirection::kCenter;
    if (error_x > 0.0F) {
      direction = ServoDirection::kRight;
    } else if (error_x < 0.0F) {
      direction = ServoDirection::kLeft;
    }

    return ServoCommandData{direction, target_x, error_x};
  }

  [[nodiscard]] static GimbalCommandData make_gimbal_command(float target_y)
  {
    const float error_y = 240.0F - target_y;

    auto direction = GimbalDirection::kCenter;
    if (error_y > 0.0F) {
      direction = GimbalDirection::kUp;
    } else if (error_y < 0.0F) {
      direction = GimbalDirection::kDown;
    }

    return GimbalCommandData{direction, target_y, error_y};
  }

  [[nodiscard]] static TriggerState evaluate_trigger(
    float distance_m,
    float max_distance_m,
    bool actuator_ready)
  {
    if (distance_m > max_distance_m) {
      return TriggerState::kSkip;
    }

    if (!actuator_ready) {
      return TriggerState::kReloading;
    }

    return TriggerState::kRequested;
  }

  [[nodiscard]] static TurretStatusData make_status(
    const TargetInput& target,
    float confidence_threshold,
    float max_distance_m,
    bool actuator_ready)
  {
    const auto target_state = evaluate_target(
      target.visible,
      target.confidence,
      confidence_threshold);

    if (target_state != TargetState::kLocked) {
      return TurretStatusData{
        target_state,
        TurretAction::kIdle,
        TriggerState::kSkip,
        target.confidence,
        target.distance_m,
      };
    }

    return TurretStatusData{
      TargetState::kLocked,
      TurretAction::kTrack,
      evaluate_trigger(target.distance_m, max_distance_m, actuator_ready),
      target.confidence,
      target.distance_m,
    };
  }
};

}  // namespace antidrone_turret
