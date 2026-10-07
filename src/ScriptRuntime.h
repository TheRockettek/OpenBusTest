#pragma once

#include "Variables.h"
#include "VehicleConfigLoader.h"
#include "ModelConfigTypes.h"

#include <filesystem>
#include <cstdint>
#include <functional>
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
        bool filtered = false;
        std::uint64_t revision = 0;
    };

    ScriptRuntime(
        const VehicleConfig& configuration, Variables& localState, SimulationState& sharedState,
        std::function<void(const std::string&, const std::string&, float)> soundTrigger = {});
    ~ScriptRuntime();

    ScriptRuntime(const ScriptRuntime&) = delete;
    ScriptRuntime& operator=(const ScriptRuntime&) = delete;

    bool valid() const;
    const std::vector<std::string>& errors() const;
    void initialize();
    void update(bool isAiVehicle);
    void invokeEntryPoint(const std::string& functionName);
    void invokeSystemTrigger(const std::string& triggerName);
    void invokeInputEvent(const std::string& keyName, bool pressed);
    void invokeKeyBinding(const std::string& bindingName, bool pressed, bool logEvent = true);
    void invokeMouseEvent(const std::string& eventName);
    bool hasScriptEntryPoint(const std::string& functionName) const;
    void invokeMouseRelease(const std::string& eventName);
    void invokeMouseDrag(const std::string& eventName, float deltaX, float deltaY, float cursorX,
                         float cursorY);
    void configureScriptTextures(const std::vector<ModelScriptTexture>& definitions);
    void configureTextTextures(const std::vector<ModelTextTexture>& definitions);
    bool copyScriptTexture(int index, ScriptTextureSnapshot& snapshot) const;
    bool copyTextTexture(int index, ScriptTextureSnapshot& snapshot) const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
