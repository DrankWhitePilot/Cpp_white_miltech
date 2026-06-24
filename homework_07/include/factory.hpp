#pragma once

class IBallisticSolver;
class ITargetProvider;
class IConfigLoader;

enum class SolverType
{
    ANALYTICAL
};

enum class ProviderType
{
    JSON
};

enum class LoaderType
{
    FILE
};

IBallisticSolver* createSolver(SolverType type);
ITargetProvider* createProvider(ProviderType type, const char* param);
IConfigLoader* createLoader(LoaderType type);
