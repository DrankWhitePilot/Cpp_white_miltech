#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "drone_physics.hpp"
#include "factory.hpp"
#include "interfaces.hpp"
#include "json.hpp"
#include "mission_processor.hpp"
#include "model_math.hpp"

using json = nlohmann::json;

namespace
{
std::string joinPath(const std::string& directory,
                     const std::string& fileName)
{
    if (directory.empty() || directory == ".")
    {
        return fileName;
    }
    const char last = directory.back();
    if (last == '/' || last == '\\')
    {
        return directory + fileName;
    }
    return directory + "/" + fileName;
}

json coordToJson(Coord coord)
{
    return {{"x", coord.x}, {"y", coord.y}};
}

bool saveSimulation(const std::string& fileName,
                    const SimStep* steps,
                    int stepCount)
{
    if (steps == nullptr || stepCount <= 0)
    {
        return false;
    }

    json output;
    output["steps"] = json::array();

    for (int i = 0; i < stepCount; ++i)
    {
        json step;
        step["position"] = coordToJson(steps[i].position);
        step["direction"] = steps[i].direction;
        step["state"] = steps[i].state;
        step["targetIndex"] = steps[i].targetIndex;
        step["dropPoint"] = coordToJson(steps[i].dropPoint);
        step["aimPoint"] = coordToJson(steps[i].aimPoint);
        step["predictedTarget"] = coordToJson(steps[i].predictedTarget);
        step["timeSecSinceStart"] = steps[i].timeSecSinceStart;
        output["steps"].push_back(std::move(step));
    }

    std::ofstream out(fileName);
    if (!out.is_open())
    {
        return false;
    }
    out << std::fixed << std::setprecision(12)
        << output.dump(2) << '\n';
    return static_cast<bool>(out);
}
}

int main(int argc, char* argv[])
{
    const std::string inputDir =
        argc >= 2 ? argv[1] : "homework_10/data";
    const bool useTable =
        argc >= 3 && std::string(argv[2]) == "table";

    const std::string configPath = joinPath(inputDir, "config.json");
    const std::string ammoPath = joinPath(inputDir, "ammo.json");
    const std::string targetsPath = joinPath(inputDir, "targets.json");
    const std::string tablePath = joinPath(inputDir, "ballistic_table.txt");
    const std::string outputPath = joinPath(inputDir, "simulation.json");

    auto loader = createLoader(LoaderType::FILE, configPath, ammoPath);
    if (loader == nullptr || loader->load() != 0)
    {
        std::cerr << "Configuration load failed\n";
        return 1;
    }

    const DroneConfig config = loader->getConfig();
    const AmmoParams ammo = loader->getAmmoParams();

    auto providerUnique = createProvider(
        ProviderType::JSON,
        targetsPath,
        config.arrayTimeStep,
        config.targetTimeStep,
        config.timeScale);
    auto solver = createSolver(
        useTable ? SolverType::TABLE : SolverType::ANALYTICAL,
        tablePath);

    if (providerUnique == nullptr || solver == nullptr ||
        providerUnique->load() != 0)
    {
        std::cerr << "Component creation failed\n";
        return 1;
    }

    std::shared_ptr<ITargetProvider> provider(std::move(providerUnique));
    auto physics = std::make_shared<DronePhysics>(config);
    MissionProcessor mission(provider,
                             physics,
                             std::move(solver),
                             config,
                             ammo);

    std::thread providerThread([&provider] { provider->run(); });
    std::thread physicsThread([&physics] { physics->run(); });
    std::thread missionThread([&mission] { mission.run(); });

    while (!provider->isThreadReady() ||
           !physics->isThreadReady() ||
           !mission.isThreadReady())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    provider->start();
    physics->start();
    mission.start();

    missionThread.join();
    mission.stop();
    physics->stop();
    provider->stop();
    physicsThread.join();
    providerThread.join();

    const SimStep* steps = mission.getSteps();
    const int stepCount = mission.getStepCount();
    if (!saveSimulation(outputPath, steps, stepCount))
    {
        std::cerr << "Cannot save " << outputPath << '\n';
        return 1;
    }

    const SimStep& last = steps[stepCount - 1];
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "solver=" << (useTable ? "table" : "analytical") << '\n';
    std::cout << "output=" << outputPath << '\n';
    std::cout << "steps=" << stepCount << '\n';
    std::cout << "timeSecSinceStart=" << last.timeSecSinceStart << '\n';
    std::cout << "aimError="
              << model::length(last.aimPoint - last.predictedTarget)
              << '\n';
    return 0;
}
