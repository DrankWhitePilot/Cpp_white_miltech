#pragma once

#include <string>
class IBallisticSolver;
class ITargetProvider;
class IConfigLoader;

enum class SolverType { ANALYTICAL };

enum class ProviderType { JSON };

enum class LoaderType { FILE };

IBallisticSolver* createSolver(SolverType type);
ITargetProvider* createProvider(ProviderType type, const std::string& param);
IConfigLoader* createLoader(LoaderType type, const std::string& configSource, const std::string& ammoSource);
