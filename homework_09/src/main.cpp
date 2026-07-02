#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "factory.hpp"
#include "interfaces.hpp"
#include "json.hpp"
#include "mission_processor.hpp"
#include "model_math.hpp"

using json = nlohmann::json;

namespace
{
std::string joinPath(const std::string& directory, const std::string& fileName)
{
    if (directory.empty() || directory == ".")
    {
        return fileName;
    }
    char last = directory.back();
    if (last == '/' || last == '\\')
    {
        return directory + fileName;
    }
    return directory + "/" + fileName;
}

json coordToJson(Coord coord)
{
    json value;
    value["x"] = coord.x;
    value["y"] = coord.y;
    return value;
}

bool saveSimulation(
    const std::string& fileName,
    const SimStep* steps,
    int stepCount)
{
    if (steps == nullptr || stepCount <= 0)
    {
        return false;
    }

    json output;
    output["totalSteps"] = stepCount;
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
        output["steps"].push_back(step);
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
    std::string inputDir =
        argc >= 2 ? argv[1] : "homework_09/data";
    const bool useTable = !(argc >= 3 && std::string(argv[2]) == "analytical");

    std::string configPath = joinPath(inputDir, "config.json");
    std::string ammoPath = joinPath(inputDir, "ammo.json");
    std::string targetsPath = joinPath(inputDir, "targets.json");
    std::string tablePath = joinPath(inputDir, "ballistic_table.txt");
    std::string outputPath = joinPath(inputDir, "simulation.json");

    auto targets = createProvider(ProviderType::JSON, targetsPath);
    auto solver = createSolver(
        useTable ? SolverType::TABLE : SolverType::ANALYTICAL,
        tablePath);
    auto loader = createLoader(
        LoaderType::FILE,
        configPath,
        ammoPath);

    if (targets == nullptr || solver == nullptr || loader == nullptr)
    {
        std::cerr << "Failed to create mission components\n";
        return 1;
    }

    MissionProcessor mission(
        std::move(targets),
        std::move(solver),
        std::move(loader));
    if (mission.init() != 0)
    {
        std::cerr << "Mission init failed\n";
        return 1;
    }

    while (mission.hasNext())
    {
        mission.step();
    }

    const SimStep* steps = mission.getSteps();
    int stepCount = mission.getStepCount();
    if (stepCount <= 0 || !saveSimulation(outputPath, steps, stepCount))
    {
        std::cerr << "Cannot save " << outputPath << '\n';
        return 1;
    }

    std::cout << std::fixed << std::setprecision(12);
    std::cout << "solver=" << (useTable ? "table" : "analytical") << '\n';
    std::cout << "inputDir=" << inputDir << '\n';
    std::cout << "output=" << outputPath << '\n';
    std::cout << "totalSteps=" << stepCount << '\n';

    const SimStep& last = steps[stepCount - 1];
    std::cout << "lastTarget=" << last.targetIndex << '\n';
    std::cout << "lastState=" << last.state << '\n';
    std::cout << "lastPosition=(" << last.position.x
              << ", " << last.position.y << ")\n";
    std::cout << "lastDropPoint=(" << last.dropPoint.x
              << ", " << last.dropPoint.y << ")\n";
    std::cout << "lastAimPoint=(" << last.aimPoint.x
              << ", " << last.aimPoint.y << ")\n";
    std::cout << "lastPredictedTarget=("
              << last.predictedTarget.x << ", "
              << last.predictedTarget.y << ")\n";
    std::cout << "aimToPredicted="
              << model::length(last.aimPoint - last.predictedTarget)
              << '\n';

    return 0;
}
