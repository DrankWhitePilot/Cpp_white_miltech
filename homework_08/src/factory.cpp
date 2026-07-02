#include "factory.hpp"

#include "analytical_solver.hpp"
#include "file_config_loader.hpp"
#include "json_target_provider.hpp"

IBallisticSolver* createSolver(SolverType type)
{
    switch (type)
    {
    case SolverType::ANALYTICAL:
        return new AnalyticalSolver();
    }

    return nullptr;
}

ITargetProvider* createProvider(ProviderType type, const std::string& param)
{
    switch (type)
    {
    case ProviderType::JSON:
        return new JsonTargetProvider(param);
    }
    return nullptr;
}

IConfigLoader* createLoader(LoaderType type, const std::string& configSource, const std::string& ammoSource)
{
    switch (type)
    {
    case LoaderType::FILE:
        return new FileConfigLoader(configSource, ammoSource);
    }

    return nullptr;
}
