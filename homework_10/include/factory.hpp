#pragma once

#include <memory>
#include <string>

class IBallisticSolver;
class ITargetProvider;
class IConfigLoader;

enum class SolverType { ANALYTICAL, TABLE };
enum class ProviderType { JSON };
enum class LoaderType { FILE };

std::unique_ptr<IBallisticSolver> createSolver(
    SolverType type,
    const std::string& param = {});
std::unique_ptr<ITargetProvider> createProvider(
    ProviderType type,
    const std::string& param,
    double arrayTimeStep,
    double targetTimeStep,
    double timeScale);
std::unique_ptr<IConfigLoader> createLoader(
    LoaderType type,
    const std::string& configSource,
    const std::string& ammoSource);
