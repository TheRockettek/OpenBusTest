#pragma once

#include "VehicleConfigLoader.h"

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

class SimulationState;
class VehicleState;

class ScriptRuntime {
  public:
    ScriptRuntime(const VehicleConfig& configuration, VehicleState& localState,
                  SimulationState& sharedState);
    ~ScriptRuntime();

    ScriptRuntime(const ScriptRuntime&) = delete;
    ScriptRuntime& operator=(const ScriptRuntime&) = delete;

    bool valid() const;
    const std::vector<std::string>& errors() const;
    void initialize();
    void update(bool isAiVehicle);
    void invokeEntryPoint(const std::string& functionName);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
