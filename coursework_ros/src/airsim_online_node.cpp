#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "api/RpcLibAdaptorsBase.hpp"
#include "rpc/client.h"

#include "drone_controller.hpp"
#include "json.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

struct ScenarioConfig
{
  Coord start_position{};
  double altitude{};
  double initial_direction{};
  double attack_speed{};
  double acceleration_path{};
  double angular_speed{};
  double turn_threshold{};
  double simulation_step{};
  double time_scale{};
  double target_array_step{};
  double hit_radius{};
  std::string ammo_name;
};

struct ScenarioData
{
  ScenarioConfig config;
  dlink::AmmoCfg ammo{};
  std::vector<std::vector<Coord>> target_paths;
};

std::string joinPath(const std::string & directory, const std::string & file)
{
  if (directory.empty() || directory.back() == '/') {
    return directory + file;
  }
  return directory + "/" + file;
}

nlohmann::json loadJson(const std::string & path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open " + path);
  }
  nlohmann::json value;
  input >> value;
  return value;
}

ScenarioData loadScenario(const std::string & directory, const std::string & ammo_override)
{
  const auto config_json = loadJson(joinPath(directory, "config.json"));
  const auto ammo_json = loadJson(joinPath(directory, "ammo.json"));
  const auto targets_json = loadJson(joinPath(directory, "targets.json"));

  ScenarioData result;
  const auto & drone = config_json.at("drone");
  const auto & simulation = config_json.at("simulation");
  result.config.start_position = {
    drone.at("position").at("x").get<double>(),
    drone.at("position").at("y").get<double>()};
  result.config.altitude = drone.at("altitude").get<double>();
  result.config.initial_direction = drone.at("initialDirection").get<double>();
  result.config.attack_speed = drone.at("attackSpeed").get<double>();
  result.config.acceleration_path = drone.at("accelerationPath").get<double>();
  result.config.angular_speed = drone.at("angularSpeed").get<double>();
  result.config.turn_threshold = drone.at("turnThreshold").get<double>();
  result.config.simulation_step = simulation.at("timeStep").get<double>();
  result.config.time_scale = simulation.value("timeScale", 1.0);
  result.config.hit_radius = simulation.at("hitRadius").get<double>();
  result.config.target_array_step = config_json.at("targetArrayTimeStep").get<double>();
  result.config.ammo_name = ammo_override.empty() ?
    config_json.at("ammo").get<std::string>() : ammo_override;

  bool ammo_found = false;
  for (const auto & item : ammo_json) {
    if (item.at("name").get<std::string>() != result.config.ammo_name) {
      continue;
    }
    std::memset(&result.ammo, 0, sizeof(result.ammo));
    std::strncpy(
      result.ammo.name, result.config.ammo_name.c_str(), sizeof(result.ammo.name) - 1);
    result.ammo.mass = item.at("mass").get<float>();
    result.ammo.drag = item.at("drag").get<float>();
    result.ammo.lift = item.at("lift").get<float>();
    result.ammo.hitRadius = static_cast<float>(result.config.hit_radius);
    ammo_found = true;
    break;
  }
  if (!ammo_found) {
    throw std::runtime_error("selected ammo is absent from ammo.json");
  }

  const int declared_count = targets_json.at("targetCount").get<int>();
  const int declared_steps = targets_json.at("timeSteps").get<int>();
  const auto & targets = targets_json.at("targets");
  if (!targets.is_array() || static_cast<int>(targets.size()) != declared_count) {
    throw std::runtime_error("targetCount does not match targets array");
  }
  result.ammo.nTargets = static_cast<uint8_t>(declared_count);
  result.target_paths.reserve(targets.size());
  for (const auto & target : targets) {
    const auto & positions = target.at("positions");
    if (!positions.is_array() || static_cast<int>(positions.size()) != declared_steps) {
      throw std::runtime_error("timeSteps does not match target positions");
    }
    std::vector<Coord> path;
    path.reserve(positions.size());
    for (const auto & position : positions) {
      path.push_back({
          position.at("x").get<double>(),
          position.at("y").get<double>()});
    }
    result.target_paths.push_back(std::move(path));
  }
  return result;
}

double radiansToDegrees(double radians)
{
  return radians * 180.0 / kPi;
}
}  // namespace

class AirSimOnlineNode final : public rclcpp::Node
{
public:
  AirSimOnlineNode()
  : Node("airsim_online_node")
  {
    const std::string scenario_directory = declare_parameter<std::string>("scenario_directory");
    const std::string ammo = declare_parameter("ammo", "");
    const std::string host = declare_parameter("host", "127.0.0.1");
    const int port = declare_parameter("port", 41451);
    const bool start_at_altitude = declare_parameter("start_at_altitude", true);
    const bool auto_takeoff = declare_parameter("auto_takeoff", false);
    const bool reset_on_start = declare_parameter("reset_on_start", false);
    const std::string dashboard_host = declare_parameter("dashboard_host", host);
    const int dashboard_port = declare_parameter("dashboard_port", 8091);
    world_scale_ = declare_parameter("world_scale", 0.25);
    target_texture_directory_ = declare_parameter(
      "target_texture_directory", "");
    if (world_scale_ <= 0.0) {
      throw std::runtime_error("world_scale must be positive");
    }

    scenario_ = loadScenario(scenario_directory, ammo);
    scenario_name_ = std::filesystem::path(scenario_directory).filename().string();
    run_id_ = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    configureSceneTransform();
    configureController();
    configureDashboardSender(dashboard_host, dashboard_port);

    telemetry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("telemetry", 10);
    target_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>("selected_target", 10);

    client_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>(
      host, static_cast<uint16_t>(port), 10.0F);
    client_->confirmConnection();
    world_rpc_client_ = std::make_unique<rpc::client>(host, static_cast<uint16_t>(port));
    world_rpc_client_->set_timeout(10000);
    if (reset_on_start) {
      client_->reset();
    }
    client_->enableApiControl(true);
    client_->simFlushPersistentMarkers();
    cleanupPreviousRunObjects();
    spawnTargetVehicles();
    client_->simPrintLogMessage("[RESULT]", " RUNNING", 0);
    if (start_at_altitude) {
      msr::airlib::Pose start_pose;
      start_pose.position = toAirSim(
        scenario_.config.start_position, static_cast<float>(-scenario_.config.altitude));
      start_pose.orientation = msr::airlib::Quaternionr::Identity();
      client_->simSetVehiclePose(start_pose, true);
    }
    if (!client_->armDisarm(true)) {
      throw std::runtime_error("AirSim refused arm command");
    }
    if (!start_at_altitude && auto_takeoff) {
      client_->takeoffAsync(15.0F)->waitOnLastTask();
    }

    if (start_at_altitude) {
      client_->moveByVelocityAsync(0.0F, 0.0F, 0.0F, 1.0F);
      mission_started_ = true;
      started_at_ = std::chrono::steady_clock::now();
    }

    plotTargetPaths();
    const std::chrono::duration<double> period(scenario_.config.simulation_step);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {tick();});

    RCLCPP_INFO(
      get_logger(), "ONLINE START: %zu targets, ammo=%s, altitude=%.1f, direct_start=%s",
      scenario_.target_paths.size(), scenario_.config.ammo_name.c_str(),
      scenario_.config.altitude, start_at_altitude ? "yes" : "no");
  }

  ~AirSimOnlineNode() override
  {
    if (dashboard_socket_ >= 0) {
      close(dashboard_socket_);
    }
  }

private:
  enum class MissionPhase {Guidance, Projectile, Impact, Complete};

  void configureDashboardSender(const std::string & host, int port)
  {
    if (port <= 0 || port > 65535) {
      RCLCPP_WARN(get_logger(), "Dashboard telemetry disabled: invalid UDP port %d", port);
      return;
    }
    dashboard_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (dashboard_socket_ < 0) {
      RCLCPP_WARN(get_logger(), "Dashboard telemetry disabled: cannot create UDP socket");
      return;
    }
    std::memset(&dashboard_address_, 0, sizeof(dashboard_address_));
    dashboard_address_.sin_family = AF_INET;
    dashboard_address_.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, host.c_str(), &dashboard_address_.sin_addr) != 1) {
      RCLCPP_WARN(
        get_logger(), "Dashboard telemetry disabled: invalid host '%s'", host.c_str());
      close(dashboard_socket_);
      dashboard_socket_ = -1;
      return;
    }
    RCLCPP_INFO(get_logger(), "Dashboard telemetry: udp://%s:%d", host.c_str(), port);
  }

  void cleanupPreviousRunObjects()
  {
    client_->simDestroyObject(projectile_object_name_);
    const std::vector<std::string> old_effects =
      client_->simListSceneObjects("coursework_explosion_.*");
    for (const std::string & object_name : old_effects) {
      client_->simDestroyObject(object_name);
    }
  }

  void configureSceneTransform()
  {
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    for (const auto & path : scenario_.target_paths) {
      for (const Coord & point : path) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
      }
    }
    world_center_ = {(min_x + max_x) * 0.5, (min_y + max_y) * 0.5};
    world_min_ = {min_x, min_y};
    world_max_ = {max_x, max_y};
  }

  void configureController()
  {
    dlink::DroneCfg config{};
    config.attackSpeed = static_cast<float>(scenario_.config.attack_speed);
    config.accelerationPath = static_cast<float>(scenario_.config.acceleration_path);
    config.angularSpeed = static_cast<float>(scenario_.config.angular_speed);
    config.turnThreshold = static_cast<float>(scenario_.config.turn_threshold);
    config.timeStep = static_cast<float>(scenario_.config.simulation_step);
    config.timeScale = static_cast<float>(scenario_.config.time_scale);
    controller_.updateConfig(config);
    controller_.updateAmmo(scenario_.ammo);
  }

  void spawnTargetVehicles()
  {
    const std::vector<std::string> texture_files{
      "target_1.png", "target_2.png", "target_3.png", "target_4.png", "target_5.png"};
    target_object_names_.clear();
    target_object_names_.reserve(scenario_.target_paths.size());
    const std::vector<std::string> existing_targets =
      client_->simListSceneObjects("coursework_target_car_.*");
    for (std::size_t i = 0; i < scenario_.target_paths.size(); ++i) {
      const std::string requested_name = "coursework_target_car_" + std::to_string(i + 1);
      const auto & path = scenario_.target_paths.at(i);
      const Coord & position = path.front();
      const Coord & next = path.at(1 % path.size());
      const float yaw = static_cast<float>(std::atan2(next.y - position.y, next.x - position.x));
      msr::airlib::Pose pose;
      pose.position = toAirSim(position, -0.45F);
      pose.orientation = msr::airlib::VectorMath::toQuaternion(0.0F, 0.0F, yaw);
      std::string object_name = requested_name;
      if (std::find(existing_targets.begin(), existing_targets.end(), requested_name) ==
        existing_targets.end())
      {
        const msr::airlib::Vector3r scale(2.20F, 1.00F, 0.45F);
        object_name = world_rpc_client_->call(
          "simSpawnObject", requested_name, "Cube",
          msr::airlib_rpclib::RpcLibAdaptorsBase::Pose(pose),
          msr::airlib_rpclib::RpcLibAdaptorsBase::Vector3r(scale), false, false).as<std::string>();
        if (object_name.empty()) {
          throw std::runtime_error("AirSim could not spawn target car " + requested_name);
        }
      } else {
        client_->simSetObjectPose(object_name, pose, true);
      }
      client_->simSetObjectScale(object_name, msr::airlib::Vector3r(2.20F, 1.00F, 0.45F));
      target_object_names_.push_back(object_name);
      if (!target_texture_directory_.empty() && i < texture_files.size()) {
        const std::string texture_path =
          target_texture_directory_ + "/" + texture_files.at(i);
        if (!client_->simSetObjectMaterialFromTexture(object_name, texture_path, 0)) {
          RCLCPP_WARN(
            get_logger(), "Could not apply color texture to %s", object_name.c_str());
        }
      }
    }
  }

  msr::airlib::Vector3r toAirSim(const Coord & point, float z = -0.5F) const
  {
    return msr::airlib::Vector3r(
      static_cast<float>((point.x - world_center_.x) * world_scale_),
      static_cast<float>((point.y - world_center_.y) * world_scale_), z);
  }

  void plotTargetPaths()
  {
    const std::vector<std::vector<float>> colors{
      {0.86F, 0.14F, 0.14F, 1.0F},
      {0.12F, 0.73F, 0.27F, 1.0F},
      {0.16F, 0.53F, 1.00F, 1.0F},
      {0.86F, 0.22F, 0.75F, 1.0F},
      {1.00F, 0.61F, 0.08F, 1.0F}};
    for (std::size_t i = 0; i < scenario_.target_paths.size(); ++i) {
      const auto & path = scenario_.target_paths.at(i);
      std::vector<msr::airlib::Vector3r> points;
      points.reserve(path.size() + 1);
      for (const Coord & point : path) {
        points.push_back(toAirSim(point));
      }
      if (!points.empty()) {
        points.push_back(points.front());
      }
      client_->simPlotLineStrip(points, colors.at(i % colors.size()), 6.0F, -1.0F, true);
    }
  }

  double elapsedSimulationSeconds() const
  {
    const std::chrono::duration<double> wall_elapsed =
      std::chrono::steady_clock::now() - started_at_;
    return wall_elapsed.count() * scenario_.config.time_scale;
  }

  Coord targetPositionAt(std::size_t target_index, double simulation_time) const
  {
    const auto & path = scenario_.target_paths.at(target_index);
    const double sample_position = simulation_time / scenario_.config.target_array_step;
    const auto sample = static_cast<std::size_t>(std::floor(sample_position));
    const double interpolation = sample_position - std::floor(sample_position);
    const Coord & from = path.at(sample % path.size());
    const Coord & to = path.at((sample + 1) % path.size());
    return {
      from.x + (to.x - from.x) * interpolation,
      from.y + (to.y - from.y) * interpolation};
  }

  const char * phaseName() const
  {
    switch (mission_phase_) {
      case MissionPhase::Guidance:
        return mission_started_ ? "guidance" : "preparing";
      case MissionPhase::Projectile:
        return "projectile";
      case MissionPhase::Impact:
        return "impact";
      case MissionPhase::Complete:
        return "complete";
    }
    return "unknown";
  }

  void sendDashboardState()
  {
    if (dashboard_socket_ < 0) {
      return;
    }

    nlohmann::json targets = nlohmann::json::array();
    for (std::size_t i = 0; i < current_targets_.size(); ++i) {
      const Coord & target = current_targets_.at(i);
      targets.push_back({
        {"id", static_cast<int>(i + 1)},
        {"x", target.x},
        {"y", target.y},
        {"selected", static_cast<int>(i) == selected_target_index_},
        {"destroyed", result_hit_ && mission_phase_ == MissionPhase::Complete &&
          static_cast<int>(i) == projectile_target_index_}});
    }

    double impact_progress = 0.0;
    if (mission_phase_ == MissionPhase::Impact) {
      const std::chrono::duration<double> elapsed =
        std::chrono::steady_clock::now() - impact_started_at_;
      impact_progress = std::clamp(elapsed.count() / impact_visual_duration_, 0.0, 1.0);
    } else if (mission_phase_ == MissionPhase::Complete && drop_reported_) {
      impact_progress = 1.0;
    }

    nlohmann::json state{
      {"schema", 1},
      {"run_id", run_id_},
      {"scenario", scenario_name_},
      {"phase", phaseName()},
      {"sim_time", latest_simulation_time_},
      {"ammo", scenario_.config.ammo_name},
      {"drone", {
          {"x", latest_telemetry_.x}, {"y", latest_telemetry_.y},
          {"z", latest_telemetry_.z}, {"speed", latest_telemetry_.speed},
          {"heading", latest_telemetry_.dir}}},
      {"bounds", {
          {"min_x", world_min_.x}, {"max_x", world_max_.x},
          {"min_y", world_min_.y}, {"max_y", world_max_.y}}},
      {"targets", std::move(targets)},
      {"selected_target", selected_target_index_ >= 0 ? selected_target_index_ + 1 : 0},
      {"projectile", {
          {"released", drop_reported_},
          {"active", mission_phase_ == MissionPhase::Projectile},
          {"release_x", projectile_release_.x}, {"release_y", projectile_release_.y},
          {"x", projectile_current_.x}, {"y", projectile_current_.y},
          {"z", projectile_current_altitude_},
          {"impact_x", projectile_impact_.x}, {"impact_y", projectile_impact_.y}}},
      {"impact", {{"active", mission_phase_ == MissionPhase::Impact},
          {"progress", impact_progress}}},
      {"result", {
          {"ready", mission_phase_ == MissionPhase::Complete},
          {"hit", result_hit_}, {"error", result_miss_},
          {"limit", scenario_.config.hit_radius},
          {"target", projectile_target_index_ >= 0 ? projectile_target_index_ + 1 : 0}}}
    };

    const std::string payload = state.dump();
    (void)sendto(
      dashboard_socket_, payload.data(), payload.size(), MSG_DONTWAIT,
      reinterpret_cast<const sockaddr *>(&dashboard_address_), sizeof(dashboard_address_));
  }

  void updateTargets(double simulation_time)
  {
    const double sample_position = simulation_time / scenario_.config.target_array_step;
    const auto sample = static_cast<std::size_t>(std::floor(sample_position));
    const double interpolation = sample_position - std::floor(sample_position);
    const std::vector<std::vector<float>> colors{
      {0.86F, 0.14F, 0.14F, 1.0F},
      {0.12F, 0.73F, 0.27F, 1.0F},
      {0.16F, 0.53F, 1.00F, 1.0F},
      {0.86F, 0.22F, 0.75F, 1.0F},
      {1.00F, 0.61F, 0.08F, 1.0F}};
    current_targets_.clear();
    current_targets_.reserve(scenario_.target_paths.size());
    const bool update_visuals = target_visual_tick_ % 3 == 0;
    ++target_visual_tick_;

    for (std::size_t i = 0; i < scenario_.target_paths.size(); ++i) {
      const auto & path = scenario_.target_paths.at(i);
      const Coord & from = path.at(sample % path.size());
      const Coord & to = path.at((sample + 1) % path.size());
      const Coord position{
        from.x + (to.x - from.x) * interpolation,
        from.y + (to.y - from.y) * interpolation};
      current_targets_.push_back(position);
      dlink::TargetPos target{};
      target.id = static_cast<uint8_t>(i);
      target.x = static_cast<float>(position.x);
      target.y = static_cast<float>(position.y);
      controller_.updateTarget(target);
      if (update_visuals && i < target_object_names_.size()) {
        const float yaw = static_cast<float>(std::atan2(to.y - from.y, to.x - from.x));
        msr::airlib::Pose pose;
        pose.position = toAirSim(position, -0.45F);
        pose.orientation = msr::airlib::VectorMath::toQuaternion(0.0F, 0.0F, yaw);
        (void)world_rpc_client_->async_call(
          "simSetObjectPose", target_object_names_.at(i),
          msr::airlib_rpclib::RpcLibAdaptorsBase::Pose(pose), true);
      }
    }
  }

  dlink::Telemetry readTelemetry(double simulation_time)
  {
    const auto state = client_->getMultirotorState();
    const auto & position = state.kinematics_estimated.pose.position;
    const auto & velocity = state.kinematics_estimated.twist.linear;
    float pitch = 0.0F;
    float roll = 0.0F;
    float yaw = 0.0F;
    msr::airlib::VectorMath::toEulerianAngle(
      state.kinematics_estimated.pose.orientation, pitch, roll, yaw);

    dlink::Telemetry telemetry{};
    telemetry.t_ms = static_cast<uint32_t>(simulation_time * 1000.0);
    telemetry.x = static_cast<float>(world_center_.x + position.x() / world_scale_);
    telemetry.y = static_cast<float>(world_center_.y + position.y() / world_scale_);
    telemetry.z = -position.z();
    telemetry.vx = static_cast<float>(velocity.x() / world_scale_);
    telemetry.vy = static_cast<float>(velocity.y() / world_scale_);
    telemetry.speed = std::hypot(telemetry.vx, telemetry.vy);
    telemetry.dir = yaw;
    telemetry.state = state.landed_state == msr::airlib::LandedState::Landed ? 0 : 1;

    nav_msgs::msg::Odometry message;
    message.header.stamp = now();
    message.header.frame_id = "airsim_ned";
    message.pose.pose.position.x = telemetry.x;
    message.pose.pose.position.y = telemetry.y;
    message.pose.pose.position.z = telemetry.z;
    message.twist.twist.linear.x = telemetry.vx;
    message.twist.twist.linear.y = telemetry.vy;
    telemetry_publisher_->publish(message);
    latest_telemetry_ = telemetry;
    return telemetry;
  }

  void sendControl(const dlink::Telemetry & telemetry, const ControlDecision & decision)
  {
    const double dt = scenario_.config.simulation_step;
    if (!command_initialized_) {
      commanded_speed_ = telemetry.speed;
      command_initialized_ = true;
    }
    commanded_speed_ = std::clamp(
      commanded_speed_ +
      static_cast<double>(decision.accel) * scenario_.config.acceleration_path * dt,
      0.0, scenario_.config.attack_speed * 1.25);
    const double next_heading =
      static_cast<double>(telemetry.dir) +
      static_cast<double>(decision.turnRate) * scenario_.config.angular_speed * dt;
    const float vx = static_cast<float>(commanded_speed_ * std::cos(next_heading));
    const float vy = static_cast<float>(commanded_speed_ * std::sin(next_heading));
    const double desired_z = -scenario_.config.altitude;
    const double current_z = -static_cast<double>(telemetry.z);
    const float vz = static_cast<float>(std::clamp((desired_z - current_z) * 0.8, -3.0, 3.0));
    client_->moveByVelocityAsync(
      static_cast<float>(vx * world_scale_), static_cast<float>(vy * world_scale_), vz, 1.0F,
      msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
      msr::airlib::YawMode(false, static_cast<float>(radiansToDegrees(next_heading))));
  }

  void publishSelectedTarget(const ControlDecision & decision)
  {
    if (decision.targetIndex < 0 ||
      decision.targetIndex >= static_cast<int>(current_targets_.size()))
    {
      return;
    }
    const Coord & selected = current_targets_.at(
      static_cast<std::size_t>(decision.targetIndex));
    geometry_msgs::msg::PointStamped message;
    message.header.stamp = now();
    message.header.frame_id = "teacher_sim";
    message.point.x = selected.x;
    message.point.y = selected.y;
    target_publisher_->publish(message);
  }

  double estimateVisualFallTime(const dlink::Telemetry & telemetry) const
  {
    constexpr double gravity = 9.80665;
    const double mass = scenario_.ammo.mass > 0.001F ? scenario_.ammo.mass : 1.0;
    const double drag = scenario_.ammo.drag > 0.0F ? scenario_.ammo.drag : 0.0;
    const double lift = scenario_.ammo.lift > 0.0F ? scenario_.ammo.lift : 0.0;
    double altitude = telemetry.z;
    double vertical_speed = 0.0;
    constexpr double dt = 0.002;
    for (double time = 0.0; time < 30.0; time += dt) {
      const double vertical_drag = drag * vertical_speed * std::fabs(vertical_speed) / mass;
      const double acceleration = std::max(gravity * 0.25, gravity - vertical_drag - lift / mass);
      vertical_speed += acceleration * dt;
      altitude -= vertical_speed * dt;
      if (altitude <= 0.0) {
        return time + dt;
      }
    }
    return std::sqrt((2.0 * telemetry.z) / gravity);
  }

  void startProjectile(
    double simulation_time, const dlink::Telemetry & telemetry,
    const ControlDecision & decision)
  {
    drop_reported_ = true;
    mission_phase_ = MissionPhase::Projectile;
    drop_started_at_ = std::chrono::steady_clock::now();
    drop_simulation_time_ = simulation_time;
    projectile_fall_time_ = estimateVisualFallTime(telemetry);
    projectile_release_ = {telemetry.x, telemetry.y};
    projectile_release_altitude_ = telemetry.z;
    projectile_current_ = projectile_release_;
    projectile_current_altitude_ = projectile_release_altitude_;
    projectile_impact_ = decision.aimPoint;
    projectile_target_index_ = decision.targetIndex;
    projectile_trail_.clear();

    client_->hoverAsync();
    client_->simDestroyObject(projectile_object_name_);
    msr::airlib::Pose pose;
    pose.position = toAirSim(
      projectile_release_, static_cast<float>(-(projectile_release_altitude_ - 2.0)));
    pose.orientation = msr::airlib::Quaternionr::Identity();
    const msr::airlib::Vector3r scale(0.65F, 0.65F, 0.65F);
    projectile_object_name_ = world_rpc_client_->call(
      "simSpawnObject", projectile_object_name_, "Sphere",
      msr::airlib_rpclib::RpcLibAdaptorsBase::Pose(pose),
      msr::airlib_rpclib::RpcLibAdaptorsBase::Vector3r(scale), false, false).as<std::string>();
    if (!target_texture_directory_.empty()) {
      client_->simSetObjectMaterialFromTexture(
        projectile_object_name_, target_texture_directory_ + "/ammo.png", 0);
    }
    projectile_trail_.push_back(pose.position);
    RCLCPP_WARN(
      get_logger(), "DROP: target=%d aim=(%.2f, %.2f) fall_time=%.2f s",
      projectile_target_index_, projectile_impact_.x, projectile_impact_.y,
      projectile_fall_time_);
  }

  void finishProjectile()
  {
    client_->simDestroyObject(projectile_object_name_);
    const Coord target_at_impact = targetPositionAt(
      static_cast<std::size_t>(projectile_target_index_),
      drop_simulation_time_ + projectile_fall_time_);
    const double miss = std::hypot(
      projectile_impact_.x - target_at_impact.x,
      projectile_impact_.y - target_at_impact.y);
    result_miss_ = miss;
    result_hit_ = miss <= scenario_.config.hit_radius;
    impact_started_at_ = std::chrono::steady_clock::now();
    explosion_object_name_ = "coursework_explosion_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count());
    msr::airlib::Pose explosion_pose;
    explosion_pose.position = toAirSim(projectile_impact_, -1.5F);
    explosion_pose.orientation = msr::airlib::Quaternionr::Identity();
    const msr::airlib::Vector3r explosion_scale(0.25F, 0.25F, 0.25F);
    explosion_object_name_ = world_rpc_client_->call(
      "simSpawnObject", explosion_object_name_, "Sphere",
      msr::airlib_rpclib::RpcLibAdaptorsBase::Pose(explosion_pose),
      msr::airlib_rpclib::RpcLibAdaptorsBase::Vector3r(explosion_scale),
      false, false).as<std::string>();
    if (!target_texture_directory_.empty() && !explosion_object_name_.empty()) {
      client_->simSetObjectMaterialFromTexture(
        explosion_object_name_, target_texture_directory_ + "/target_5.png", 0);
    }

    impact_effect_object_names_.clear();
    const std::vector<Coord> fire_offsets{
      {-4.0, 0.0}, {4.0, 0.0}, {0.0, -4.0}, {0.0, 4.0}, {-2.8, -2.8}, {2.8, 2.8}};
    for (std::size_t i = 0; i < fire_offsets.size(); ++i) {
      const std::string requested_name = explosion_object_name_ + "_fire_" + std::to_string(i);
      const Coord fire_position{
        projectile_impact_.x + fire_offsets.at(i).x,
        projectile_impact_.y + fire_offsets.at(i).y};
      msr::airlib::Pose fire_pose;
      fire_pose.position = toAirSim(fire_position, static_cast<float>(-1.0 - 0.35 * i));
      fire_pose.orientation = msr::airlib::Quaternionr::Identity();
      const float initial_scale = 0.10F + static_cast<float>(i % 3) * 0.04F;
      const std::string object_name = world_rpc_client_->call(
        "simSpawnObject", requested_name, "Sphere",
        msr::airlib_rpclib::RpcLibAdaptorsBase::Pose(fire_pose),
        msr::airlib_rpclib::RpcLibAdaptorsBase::Vector3r(
          msr::airlib::Vector3r(initial_scale, initial_scale, initial_scale)),
        false, false).as<std::string>();
      if (!object_name.empty()) {
        impact_effect_object_names_.push_back(object_name);
        if (!target_texture_directory_.empty()) {
          const std::string texture = i < 4 ? "/target_5.png" : "/ammo.png";
          client_->simSetObjectMaterialFromTexture(
            object_name, target_texture_directory_ + texture, 0);
        }
      }
    }
    mission_phase_ = MissionPhase::Impact;
  }

  void showResult()
  {
    const std::string target = "T" + std::to_string(projectile_target_index_ + 1);
    std::ostringstream error;
    error << std::fixed << std::setprecision(2) << result_miss_ << " m";
    std::ostringstream limit;
    limit << std::fixed << std::setprecision(2) << scenario_.config.hit_radius << " m";
    const std::string status = result_hit_ ? "HIT" : "MISS";
    result_text_ = status + " | " + target + " | error=" + error.str() +
      " | limit=" + limit.str();

    const int severity = result_hit_ ? 0 : 2;
    const std::string result_banner = result_hit_ ?
      " HIT | TARGET " + target + " DESTROYED | ERROR " + error.str() +
      " | LIMIT " + limit.str() :
      " MISS | TARGET " + target + " SURVIVED | ERROR " + error.str() +
      " | LIMIT " + limit.str();
    client_->simPrintLogMessage("[RESULT]", result_banner, severity);

    if (result_hit_ && projectile_target_index_ >= 0 &&
      static_cast<std::size_t>(projectile_target_index_) < target_object_names_.size())
    {
      const std::string & target_object = target_object_names_.at(
        static_cast<std::size_t>(projectile_target_index_));
      if (!target_texture_directory_.empty()) {
        client_->simSetObjectMaterialFromTexture(
          target_object, target_texture_directory_ + "/ammo.png", 0);
      }
      client_->simSetObjectScale(target_object, msr::airlib::Vector3r(2.20F, 1.00F, 0.18F));
      client_->simDestroyObject(explosion_object_name_);
    } else if (!result_hit_ && !explosion_object_name_.empty()) {
      client_->simSetObjectScale(
        explosion_object_name_, msr::airlib::Vector3r(0.45F, 0.45F, 0.45F));
      if (!target_texture_directory_.empty()) {
        client_->simSetObjectMaterialFromTexture(
          explosion_object_name_, target_texture_directory_ + "/target_1.png", 0);
      }
    }
    for (const std::string & object_name : impact_effect_object_names_) {
      client_->simDestroyObject(object_name);
    }
    impact_effect_object_names_.clear();
    RCLCPP_WARN(get_logger(), "RESULT: %s", result_text_.c_str());
    mission_phase_ = MissionPhase::Complete;
  }

  void updateImpact()
  {
    const std::chrono::duration<double> elapsed =
      std::chrono::steady_clock::now() - impact_started_at_;
    const double progress = std::clamp(elapsed.count() / impact_visual_duration_, 0.0, 1.0);
    const double grow = std::clamp(progress / 0.65, 0.0, 1.0);
    const double fade = progress <= 0.65 ? 1.0 :
      std::clamp((1.0 - progress) / 0.35, 0.0, 1.0);
    const float scale = static_cast<float>(0.25 + 5.5 * grow * fade);
    if (!explosion_object_name_.empty()) {
      (void)world_rpc_client_->async_call(
        "simSetObjectScale", explosion_object_name_,
        msr::airlib_rpclib::RpcLibAdaptorsBase::Vector3r(
          msr::airlib::Vector3r(scale, scale, scale)));
    }
    for (std::size_t i = 0; i < impact_effect_object_names_.size(); ++i) {
      const double delay = 0.06 * static_cast<double>(i);
      const double local_progress = std::clamp((progress - delay) / (1.0 - delay), 0.0, 1.0);
      const double local_grow = std::clamp(local_progress / 0.55, 0.0, 1.0);
      const double local_fade = local_progress <= 0.55 ? 1.0 :
        std::clamp((1.0 - local_progress) / 0.45, 0.0, 1.0);
      const float fragment_scale = static_cast<float>(
        0.10 + (1.30 + 0.18 * static_cast<double>(i % 3)) * local_grow * local_fade);
      (void)world_rpc_client_->async_call(
        "simSetObjectScale", impact_effect_object_names_.at(i),
        msr::airlib_rpclib::RpcLibAdaptorsBase::Vector3r(
          msr::airlib::Vector3r(fragment_scale, fragment_scale, fragment_scale)));
    }
    if (progress >= 1.0) {
      showResult();
    }
  }

  void updateProjectile()
  {
    const std::chrono::duration<double> elapsed =
      std::chrono::steady_clock::now() - drop_started_at_;
    const double progress = std::clamp(elapsed.count() / projectile_visual_duration_, 0.0, 1.0);
    const Coord position{
      projectile_release_.x + (projectile_impact_.x - projectile_release_.x) * progress,
      projectile_release_.y + (projectile_impact_.y - projectile_release_.y) * progress};
    const double altitude =
      projectile_release_altitude_ * (1.0 - progress * progress) - 2.0 * (1.0 - progress);
    projectile_current_ = position;
    projectile_current_altitude_ = std::max(0.0, altitude);
    msr::airlib::Pose pose;
    pose.position = toAirSim(position, static_cast<float>(-std::max(0.5, altitude)));
    pose.orientation = msr::airlib::Quaternionr::Identity();
    (void)world_rpc_client_->async_call(
      "simSetObjectPose", projectile_object_name_,
      msr::airlib_rpclib::RpcLibAdaptorsBase::Pose(pose), true);
    if (!projectile_trail_.empty()) {
      client_->simPlotLineStrip(
        {projectile_trail_.back(), pose.position},
        {1.0F, 0.75F, 0.0F, 1.0F}, 4.0F, -1.0F, false);
    }
    projectile_trail_.push_back(pose.position);
    if (progress >= 1.0) {
      finishProjectile();
    }
  }

  void tick()
  {
    try {
      if (!mission_started_) {
        const dlink::Telemetry telemetry = readTelemetry(0.0);
        const double altitude_error = scenario_.config.altitude - telemetry.z;
        if (altitude_error > altitude_tolerance_) {
          const float vertical_speed = static_cast<float>(
            -std::clamp(altitude_error * 0.4, 1.0, 8.0));
          client_->moveByVelocityAsync(0.0F, 0.0F, vertical_speed, 1.0F);
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "ASCENDING altitude=%.1f/%.1f", telemetry.z, scenario_.config.altitude);
          sendDashboardState();
          return;
        }
        mission_started_ = true;
        started_at_ = std::chrono::steady_clock::now();
        command_initialized_ = false;
        RCLCPP_INFO(
          get_logger(), "ONLINE READY: altitude %.1f reached; guidance enabled",
          telemetry.z);
      }

      if (mission_phase_ == MissionPhase::Complete) {
        sendDashboardState();
        return;
      }

      if (mission_phase_ == MissionPhase::Impact) {
        updateImpact();
        sendDashboardState();
        return;
      }

      if (mission_phase_ == MissionPhase::Projectile) {
        const std::chrono::duration<double> elapsed =
          std::chrono::steady_clock::now() - drop_started_at_;
        const double progress = std::clamp(
          elapsed.count() / projectile_visual_duration_, 0.0, 1.0);
        updateTargets(drop_simulation_time_ + projectile_fall_time_ * progress);
        updateProjectile();
        latest_simulation_time_ = drop_simulation_time_ + projectile_fall_time_ * progress;
        sendDashboardState();
        return;
      }

      const double simulation_time = elapsedSimulationSeconds();
      latest_simulation_time_ = simulation_time;
      updateTargets(simulation_time);
      const dlink::Telemetry telemetry = readTelemetry(simulation_time);
      controller_.updateTelemetry(telemetry);
      const ControlDecision decision = controller_.decide();
      selected_target_index_ = decision.targetIndex;
      publishSelectedTarget(decision);
      if (decision.drop && !drop_reported_) {
        startProjectile(simulation_time, telemetry, decision);
      } else {
        sendControl(telemetry, decision);
      }

      const Coord drone_position{telemetry.x, telemetry.y};
      client_->simPlotPoints(
        {toAirSim(drone_position, -1.0F)},
        {0.0F, 0.20F, 1.00F, 1.0F}, 13.0F, 0.25F, false);
      if (decision.targetIndex >= 0 &&
        static_cast<std::size_t>(decision.targetIndex) < current_targets_.size())
      {
        client_->simPlotLineStrip(
          {toAirSim(drone_position, -0.8F),
            toAirSim(current_targets_.at(static_cast<std::size_t>(decision.targetIndex)), -0.8F)},
          {0.0F, 0.15F, 1.0F, 1.0F}, 5.0F, 0.25F, false);
      }

      ++path_tick_;
      if (path_tick_ % 2 == 0) {
        client_->simPlotPoints(
          {toAirSim(drone_position, -1.0F)},
          {0.0F, 0.20F, 1.00F, 1.0F}, 7.0F, -1.0F, false);
      }

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "ONLINE t=%.1f pos=(%.1f, %.1f, %.1f) speed=%.1f target=%d control=(%.2f, %.2f)",
        simulation_time, telemetry.x, telemetry.y, telemetry.z, telemetry.speed,
        decision.targetIndex, decision.accel, decision.turnRate);
      sendDashboardState();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "AirSim update failed: %s", error.what());
    }
  }

  ScenarioData scenario_;
  DroneController controller_;
  std::unique_ptr<msr::airlib::MultirotorRpcLibClient> client_;
  std::unique_ptr<rpc::client> world_rpc_client_;
  std::chrono::steady_clock::time_point started_at_;
  std::vector<Coord> current_targets_;
  std::vector<std::string> target_object_names_;
  std::string target_texture_directory_;
  std::string scenario_name_;
  Coord world_center_{};
  Coord world_min_{};
  Coord world_max_{};
  double world_scale_ = 0.25;
  MissionPhase mission_phase_ = MissionPhase::Guidance;
  std::chrono::steady_clock::time_point drop_started_at_;
  std::chrono::steady_clock::time_point impact_started_at_;
  Coord projectile_release_{};
  Coord projectile_impact_{};
  Coord projectile_current_{};
  double projectile_current_altitude_ = 0.0;
  double projectile_release_altitude_ = 0.0;
  double projectile_fall_time_ = 0.0;
  double drop_simulation_time_ = 0.0;
  int projectile_target_index_ = -1;
  const double projectile_visual_duration_ = 2.5;
  const double impact_visual_duration_ = 2.0;
  std::string projectile_object_name_ = "coursework_projectile";
  std::string explosion_object_name_;
  std::vector<std::string> impact_effect_object_names_;
  std::string result_text_;
  double result_miss_ = 0.0;
  bool result_hit_ = false;
  std::vector<msr::airlib::Vector3r> projectile_trail_;
  bool drop_reported_ = false;
  bool mission_started_ = false;
  bool command_initialized_ = false;
  double commanded_speed_ = 0.0;
  dlink::Telemetry latest_telemetry_{};
  double latest_simulation_time_ = 0.0;
  int selected_target_index_ = -1;
  int64_t run_id_ = 0;
  int dashboard_socket_ = -1;
  sockaddr_in dashboard_address_{};
  const double altitude_tolerance_ = 2.0;
  std::size_t path_tick_ = 0;
  std::size_t target_visual_tick_ = 0;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr telemetry_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<AirSimOnlineNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("airsim_online_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
