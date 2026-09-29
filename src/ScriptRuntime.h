#pragma once

#include "Variables.h"
#include "VehicleConfigLoader.h"

#include <filesystem>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class SimulationState;

class ScriptRuntime {
  public:
    struct ScriptTextureSnapshot {
        int width = 0;
        int height = 0;
        std::vector<std::uint8_t> pixels;
    };

    ScriptRuntime(const VehicleConfig& configuration, Variables& localState,
                  SimulationState& sharedState);
    ~ScriptRuntime();

    ScriptRuntime(const ScriptRuntime&) = delete;
    ScriptRuntime& operator=(const ScriptRuntime&) = delete;

    bool valid() const;
    const std::vector<std::string>& errors() const;
    void initialize();
    void update(bool isAiVehicle);
    void invokeEntryPoint(const std::string& functionName);
    void invokeSystemTrigger(const std::string& triggerName);
    bool copyScriptTexture(int index, ScriptTextureSnapshot& snapshot) const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
