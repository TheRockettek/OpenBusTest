#pragma once

#include "SoundPlayback.h"
#include "Viewpoint.h"

#include <array>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

class Variables;

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

class VehicleSoundBank {
  public:
    void load(const std::filesystem::path& configPath);
    bool hasTrigger(const std::string& name) const;
    void trigger(SoundPlayback& playback, const std::string& name,
                 const std::filesystem::path& overrideFile, float controlValue,
                 const Variables& variables, openbus::rendering::ViewpointContext viewpoint);
    void stop(SoundPlayback& playback, const std::string& name);
    void updateAmbient(SoundPlayback& playback, const Variables& variables,
                       openbus::rendering::ViewpointContext viewpoint);
    void stopAllLoops(SoundPlayback& playback);

  private:
    static bool conditionsAllow(const SoundTriggerDefinition& definition,
                                const Variables& variables);
    std::filesystem::path resolvedFile(const SoundTriggerDefinition& definition) const;
    void stopLoopFile(SoundPlayback& playback, const std::filesystem::path& path);
    void stopAmbientLoopFile(SoundPlayback& playback, const std::filesystem::path& path);

    std::unordered_map<std::string, std::vector<SoundTriggerDefinition>> triggers_;
    std::vector<SoundTriggerDefinition> untriggeredLoopSounds_;
    std::filesystem::path basePath_;
    std::unordered_map<std::string, SoundPlaybackHandle> activeTriggeredLoops_;
    std::unordered_map<std::string, SoundPlaybackHandle> activeAmbientLoops_;
};
