#include "mission_processor.hpp"
#include "interfaces.hpp"

#include <cstring>

MissionProcessor::MissionProcessor(
    ITargetProvider* targets,
    IBallisticSolver* solver,
    IConfigLoader* loader)
    : targets_(targets),
      solver_(solver),
      loader_(loader)
{
}

int MissionProcessor::init()
{
    if (targets_ == nullptr || solver_ == nullptr || loader_ == nullptr)
    {
        return 1;
    }

    if (targets_->load() != 0)
    {
        return 1;
    }

    if (loader_->load() != 0)
    {
        return 1;
    }

    prepareInputData();
    reset();
    initialized_ = true;

    return 0;
}

bool MissionProcessor::hasNext() const
{
    return initialized_ && targets_ != nullptr && currentIdx_ < targets_->getTargetCount();
}

DropPoint MissionProcessor::step()
{
    DropPoint result{};
    result.valid = false;
    result.targetIdx = currentIdx_;
    result.point = {0.0, 0.0};
    result.aimPoint = {0.0, 0.0};
    result.totalTime = 0.0;
    result.needManeuver = false;

    if (!hasNext() || solver_ == nullptr)
    {
        return result;
    }

    Coord* targetPath = targets_->getTarget(currentIdx_);
    Coord target = targetPath[0];
    const DroneConfig& config = loader_->getConfig();

    double aimX = 0.0;
    double aimY = 0.0;
    double fireX = 0.0;
    double fireY = 0.0;
    double totalTime = 0.0;
    bool needManeuver = false;

    bool ok = solver_->solve(
        data_,
        config.startPos.x,
        config.startPos.y,
        target.x,
        target.y,
        aimX,
        aimY,
        fireX,
        fireY,
        totalTime,
        needManeuver);

    result.valid = ok;
    result.point = {fireX, fireY};
    result.aimPoint = {aimX, aimY};
    result.totalTime = totalTime;
    result.needManeuver = needManeuver;

    currentIdx_++;
    return result;
}

void MissionProcessor::reset()
{
    currentIdx_ = 0;
}

void MissionProcessor::changeSolver(IBallisticSolver* solver)
{
    solver_ = solver;
}

void MissionProcessor::prepareInputData()
{
    const DroneConfig& config = loader_->getConfig();
    const AmmoParams& ammo = loader_->getAmmoParams();

    data_.xd = config.startPos.x;
    data_.yd = config.startPos.y;
    data_.zd = config.altitude;
    data_.initialDir = config.initialDir;
    data_.attackSpeed = config.attackSpeed;
    data_.accelerationPath = config.accelPath;
    std::strncpy(data_.ammo_name, config.ammoName, sizeof(data_.ammo_name) - 1);
    data_.ammo_name[sizeof(data_.ammo_name) - 1] = '\0';
    data_.arrayTimeStep = config.arrayTimeStep;
    data_.simTimeStep = config.simTimeStep;
    data_.hitRadius = config.hitRadius;
    data_.angularSpeed = config.angularSpeed;
    data_.turnThreshold = config.turnThreshold;
    data_.m = ammo.mass;
    data_.d = ammo.drag;
    data_.l = ammo.lift;
    data_.targetX = 0.0;
    data_.targetY = 0.0;
}
