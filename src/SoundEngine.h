#pragma once

#include "Viewpoint.h"

#include <array>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

struct SoundCurvePoint {
    float x = 0.0F;
    float y = 0.0F;
};

struct SoundVolumeCurve {
    std::string variable;
    std::vector<SoundCurvePoint> points;
};

struct SoundCondition {
    std::string variable;
    float referenceValue = 0.0F;
    int comparison = -1;
};

struct SoundTriggerDefinition {
    std::filesystem::path file;
    bool loop = false;
    bool loopDisabled = false;
    int viewpoint = 0;
    double maxDistance = 0.0;
    std::array<double, 3> position = {};
    std::string controlVariable;
    float controlCenter = 0.0F;
    double baseGain = 1.0;
    std::vector<SoundCurvePoint> volumeCurve;
    std::vector<SoundVolumeCurve> volumeCurves;
    std::vector<SoundCondition> conditions;
};

class Variables;

class SoundEngine {
  public:
    SoundEngine();
    ~SoundEngine();
    SoundEngine(const SoundEngine&) = delete;
    SoundEngine& operator=(const SoundEngine&) = delete;

    void load(const std::filesystem::path& configPath);
    void setViewpoint(openbus::rendering::ViewpointContext viewpoint);
    void setListenerPose(const std::array<double, 3>& position,
                         const std::array<double, 3>& forward, const std::array<double, 3>& up);
    void updateLoops(const Variables& variables);
    bool hasTrigger(const std::string& name) const;
    void trigger(const std::string& name, const std::filesystem::path& overrideFile = {},
                 float controlValue = 0.0F, const Variables* variables = nullptr);
    void stop(const std::string& name);

  private:
    friend struct SoundEngineProbeAccess;

    static bool conditionsAllow(const SoundTriggerDefinition& definition,
                                const Variables& variables);

    struct Backend;
    std::unordered_map<std::string, std::vector<SoundTriggerDefinition>> triggers_;
    std::vector<SoundTriggerDefinition> untriggeredLoopSounds_;
    std::filesystem::path basePath_;
    openbus::rendering::ViewpointContext viewpoint_ =
        openbus::rendering::ViewpointContext::PlayerExterior;
    std::unique_ptr<Backend> backend_;
};