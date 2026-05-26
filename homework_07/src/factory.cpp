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

ITargetProvider* createProvider(ProviderType type, const char* param)
{
    ITargetProvider* provider = nullptr;

    switch (type)
    {
    case ProviderType::JSON:
        provider = new JsonTargetProvider();
        break;
    }

    if (provider != nullptr && param != nullptr)
    {
        if (provider->load(param) != 0)
        {
            delete provider;
            return nullptr;
        }
    }

    return provider;
}

IConfigLoader* createLoader(LoaderType type)
{
    switch (type)
    {
    case LoaderType::FILE:
        return new FileConfigLoader();
    }

    return nullptr;
}
