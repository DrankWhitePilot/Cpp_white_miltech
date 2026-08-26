#pragma once

#include <iosfwd>
#include <string>

struct BallisticsInput {
  double drone_x_{};
  double drone_y_{};
  double drone_z_{};
  double target_x_{};
  double target_y_{};
  double attack_speed_{};
  double acceleration_path_{};
  std::string ammo_name_;
};

struct AmmoParameters {
  double mass_kg_{};
  double diameter_m_{};
  double lift_factor_{};
};

struct DropSolution {
  double time_s_{};
  double horizontal_fall_m_{};
  double fire_x_{};
  double fire_y_{};
  bool maneuver_required_{};
  double intermediate_x_{};
  double intermediate_y_{};
};

auto get_ammo_parameters(const std::string& ammo_name) -> AmmoParameters;
auto read_ballistics_input(const std::string& path) -> BallisticsInput;
auto compute_drop_solution(BallisticsInput input) -> DropSolution;
void write_drop_solution(std::ostream& output, const DropSolution& solution);
