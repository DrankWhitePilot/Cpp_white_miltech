#include "factory.hpp"

#include <memory>

#include "analytical_solver.hpp"
#include "file_config_loader.hpp"
#include "json_target_provider.hpp"
#include "table_solver.hpp"

std::unique_ptr<IBallisticSolver> createSolver(
    SolverType type,
    const std::string& param)
{
    switch (type)
    {
    case SolverType::ANALYTICAL:
        return std::make_unique<AnalyticalSolver>();
    case SolverType::TABLE:
        return std::make_unique<TableSolver>(param);
    }
    return nullptr;
}

std::unique_ptr<ITargetProvider> createProvider(
    ProviderType type,
    const std::string& param)
{
    switch (type)
    {
    case ProviderType::JSON:
        return std::make_unique<JsonTargetProvider>(param);
    }
    return nullptr;
}

std::unique_ptr<IConfigLoader> createLoader(
    LoaderType type,
    const std::string& configSource,
    const std::string& ammoSource)
{
    switch (type)
    {
    case LoaderType::FILE:
        return std::make_unique<FileConfigLoader>(
            configSource,
            ammoSource);
    }
    return nullptr;
}
