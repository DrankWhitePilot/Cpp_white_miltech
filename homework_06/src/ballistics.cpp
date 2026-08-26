#include "ballistics.hpp"

#include <cmath>
#include <fstream>
#include <numbers>
#include <ostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace {
constexpr double kGravity = 9.81;
constexpr double kEpsilon = 1e-9;

auto calculate_distance(const BallisticsInput& input) -> double
{
  const double kDeltaX = input.target_x_ - input.drone_x_;
  const double kDeltaY = input.target_y_ - input.drone_y_;
  return std::sqrt(kDeltaX * kDeltaX + kDeltaY * kDeltaY);
}

// Course ballistic formula intentionally keeps numeric coefficients from the
// original homework statement.
// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)
auto calculate_time(const BallisticsInput& input, const AmmoParameters& ammo) -> double
{
  const double kMass = ammo.mass_kg_;
  const double kDiameter = ammo.diameter_m_;
  const double kLift = ammo.lift_factor_;
  const double kSpeed = input.attack_speed_;

  const double kCoefficientA = kDiameter * kGravity * kMass - 2.0 * kDiameter * kDiameter * kLift * kSpeed;
  const double kCoefficientB = -3.0 * kGravity * kMass * kMass + 3.0 * kDiameter * kLift * kMass * kSpeed;
  const double kCoefficientC = 6.0 * kMass * kMass * input.drone_z_;

  if (std::abs(kCoefficientA) < kEpsilon) {
    throw std::runtime_error("Invalid ballistic equation");
  }

  const double kDepressedCubicP = -(kCoefficientB * kCoefficientB) / (3.0 * kCoefficientA * kCoefficientA);
  if (kDepressedCubicP > -kEpsilon) {
    throw std::runtime_error("Invalid ballistic equation parameter p");
  }

  const double kDepressedCubicQ =
    (2.0 * kCoefficientB * kCoefficientB * kCoefficientB) / (27.0 * kCoefficientA * kCoefficientA * kCoefficientA) +
    kCoefficientC / kCoefficientA;

  const double kAcosArgument = (3.0 * kDepressedCubicQ / (2.0 * kDepressedCubicP)) * std::sqrt(-3.0 / kDepressedCubicP);

  if (kAcosArgument < -1.0 || kAcosArgument > 1.0) {
    throw std::runtime_error("Invalid ballistic equation acos argument");
  }

  const double kPhi = std::acos(kAcosArgument);
  const double kTime =
    2.0 * std::sqrt(-kDepressedCubicP / 3.0) * std::cos((kPhi + 4.0 * std::numbers::pi) / 3.0) - kCoefficientB / (3.0 * kCoefficientA);

  if (kTime <= 0.0) {
    throw std::runtime_error("Invalid drop time");
  }

  return kTime;
}

auto calculate_horizontal_fall(const BallisticsInput& input, const AmmoParameters& ammo, double drop_time) -> double
{
  const double kSpeed = input.attack_speed_;
  const double kDiameter = ammo.diameter_m_;
  const double kMass = ammo.mass_kg_;
  const double kLift = ammo.lift_factor_;

  return kSpeed * drop_time - (drop_time * drop_time * kDiameter * kSpeed) / (2.0 * kMass) +
         (std::pow(drop_time, 3.0) *
          (6.0 * kDiameter * kGravity * kLift * kMass - 6.0 * kDiameter * kDiameter * (kLift * kLift - 1.0) * kSpeed)) /
           (36.0 * kMass * kMass) +
         (std::pow(drop_time, 4.0) *
          (-6.0 * kDiameter * kDiameter * kGravity * kLift * (1.0 + kLift * kLift + kLift * kLift * kLift * kLift) * kMass +
           3.0 * kDiameter * kDiameter * kDiameter * kLift * kLift * (1.0 + kLift * kLift) * kSpeed +
           6.0 * kDiameter * kDiameter * kDiameter * kLift * kLift * kLift * kLift * (1.0 + kLift * kLift) * kSpeed)) /
           (36.0 * std::pow(1.0 + kLift * kLift, 2.0) * std::pow(kMass, 3.0)) +
         (std::pow(drop_time, 5.0) * (3.0 * kDiameter * kDiameter * kDiameter * kGravity * kLift * kLift * kLift * kMass -
                                      3.0 * std::pow(kDiameter, 4.0) * kLift * kLift * (1.0 + kLift * kLift) * kSpeed)) /
           (36.0 * (1.0 + kLift * kLift) * std::pow(kMass, 4.0));
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)

auto calculate_fire_point(const BallisticsInput& input, double distance, double horizontal_fall) -> std::pair<double, double>
{
  const double kRatio = (distance - horizontal_fall) / distance;
  const double kFireX = input.drone_x_ + (input.target_x_ - input.drone_x_) * kRatio;
  const double kFireY = input.drone_y_ + (input.target_y_ - input.drone_y_) * kRatio;
  return {kFireX, kFireY};
}
}  // namespace

// Ammo table values are domain constants from the original homework.
// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)
auto get_ammo_parameters(const std::string& ammo_name) -> AmmoParameters
{
  if (ammo_name == "VOG-17") {
    return {.mass_kg_ = 0.35, .diameter_m_ = 0.07, .lift_factor_ = 0.0};
  }
  if (ammo_name == "M67") {
    return {.mass_kg_ = 0.6, .diameter_m_ = 0.10, .lift_factor_ = 0.0};
  }
  if (ammo_name == "RKG-3") {
    return {.mass_kg_ = 1.2, .diameter_m_ = 0.10, .lift_factor_ = 0.0};
  }
  if (ammo_name == "GLIDING-VOG") {
    return {.mass_kg_ = 0.45, .diameter_m_ = 0.10, .lift_factor_ = 1.0};
  }
  if (ammo_name == "GLIDING-RKG") {
    return {.mass_kg_ = 1.4, .diameter_m_ = 0.10, .lift_factor_ = 1.0};
  }

  throw std::runtime_error("Unknown ammo type: " + ammo_name);
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)

auto read_ballistics_input(const std::string& path) -> BallisticsInput
{
  std::ifstream input_file(path);
  if (!input_file.is_open()) {
    throw std::runtime_error("Cannot open input file: " + path);
  }

  BallisticsInput input;
  input_file >> input.drone_x_ >> input.drone_y_ >> input.drone_z_ >> input.target_x_ >> input.target_y_ >> input.attack_speed_ >>
    input.acceleration_path_ >> input.ammo_name_;

  if (!input_file) {
    throw std::runtime_error("Invalid input file format: " + path);
  }

  return input;
}

auto compute_drop_solution(BallisticsInput input) -> DropSolution
{
  if (input.drone_z_ <= 0.0) {
    throw std::runtime_error("Drone altitude must be positive");
  }
  if (input.attack_speed_ <= 0.0) {
    throw std::runtime_error("Attack speed must be positive");
  }
  if (input.acceleration_path_ < 0.0) {
    throw std::runtime_error("Acceleration path cannot be negative");
  }

  const AmmoParameters kAmmo = get_ammo_parameters(input.ammo_name_);
  double distance = calculate_distance(input);
  if (distance <= 0.0) {
    throw std::runtime_error("Distance to target must be positive");
  }

  const double kDropTime = calculate_time(input, kAmmo);
  const double kHorizontalFall = calculate_horizontal_fall(input, kAmmo, kDropTime);
  if (kHorizontalFall <= 0.0) {
    throw std::runtime_error("Horizontal fall distance must be positive");
  }

  DropSolution solution;
  solution.time_s_ = kDropTime;
  solution.horizontal_fall_m_ = kHorizontalFall;
  solution.maneuver_required_ = kHorizontalFall + input.acceleration_path_ > distance;

  if (solution.maneuver_required_) {
    solution.intermediate_x_ =
      input.target_x_ - (input.target_x_ - input.drone_x_) * (kHorizontalFall + input.acceleration_path_) / distance;
    solution.intermediate_y_ =
      input.target_y_ - (input.target_y_ - input.drone_y_) * (kHorizontalFall + input.acceleration_path_) / distance;

    input.drone_x_ = solution.intermediate_x_;
    input.drone_y_ = solution.intermediate_y_;
    distance = calculate_distance(input);
  }

  const auto [kFireX, kFireY] = calculate_fire_point(input, distance, kHorizontalFall);
  solution.fire_x_ = kFireX;
  solution.fire_y_ = kFireY;

  return solution;
}

void write_drop_solution(std::ostream& output, const DropSolution& solution)
{
  if (solution.maneuver_required_) {
    output << solution.intermediate_x_ << ' ' << solution.intermediate_y_ << '\n';
  }

  output << solution.fire_x_ << ' ' << solution.fire_y_ << '\n';
}
