#include <iostream>

#include "interfaces.hpp"
#include "factory.hpp"
#include "mission_processor.hpp"

int main()
{
    ITargetProvider* targets = createProvider(
        ProviderType::JSON,
        "homework_07/data/targets.json");

    IBallisticSolver* solver = createSolver(SolverType::ANALYTICAL);
    IConfigLoader* loader = createLoader(LoaderType::FILE, "homework_07/data/config.json", "homework_07/data/ammo.json");

    if (targets == nullptr || solver == nullptr || loader == nullptr)
    {
        std::cout << "Failed to create mission components\n";
        delete targets;
        delete solver;
        delete loader;
        return 1;
    }

    MissionProcessor mission(targets, solver, loader);

    if (mission.init() != 0)
    {
        std::cout << "Mission init failed\n";
        delete targets;
        delete solver;
        delete loader;
        return 1;
    }

    while (mission.hasNext())
    {
        DropPoint drop = mission.step();

        std::cout << "Target " << drop.targetIdx
                  << " valid=" << drop.valid
                  << " drop=(" << drop.point.x << ", " << drop.point.y << ")"
                  << " aim=(" << drop.aimPoint.x << ", " << drop.aimPoint.y << ")"
                  << " time=" << drop.totalTime
                  << " needManeuver=" << drop.needManeuver
                  << "\n";
    }

    delete targets;
    delete solver;
    delete loader;

    return 0;
}
