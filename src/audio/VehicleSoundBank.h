#pragma once

#include "audio/SoundPlayback.h"
#include "core/BusTypes.h"
#include "core/Viewpoint.h"

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class Variables;

using SoundDefinitionId = std::uint64_t;

// OpenBus policy choice, pending native OMSI same-file retrigger parity validation.
enum class SoundRetriggerPolicy { KeepPlaying, Restart };

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
    // Assigned once per configured [sound]/[loopsound] block, shared by its aliases.
    SoundDefinitionId id = 0;
    std::filesystem::path file;
    bool loop = false;
    bool loopDisabled = false;
    int viewpoint = 0;
    double maxDistance = 0.0;
    std::array<double, 3> position = {};
    std::string controlVariable;
    float controlCenter = 0.0F;
    double baseGain = 1.0;
    std::vector<SoundVolumeCurve> volumeCurves;
    std::vector<SoundCondition> conditions;
};

class VehicleSoundBank {
  public:
    // Reload resets ownership without a playback reference. Call stopAll(playback) first
    // when existing sources must be cancelled; no playback pointer is retained here.
    void load(const std::filesystem::path& configPath);
    void setPose(const BodyPose& pose);
    bool hasTrigger(const std::string& name) const;
    void trigger(SoundPlayback& playback, const std::string& name,
                 const std::filesystem::path& overrideFile, float controlValue,
                 const Variables& variables, openbus::rendering::ViewpointContext viewpoint,
                 SoundRetriggerPolicy retriggerPolicy = SoundRetriggerPolicy::KeepPlaying);
    void stop(SoundPlayback& playback, const std::string& name);
    void updateAmbient(SoundPlayback& playback, const Variables& variables,
                       openbus::rendering::ViewpointContext viewpoint);
    void updateActiveSounds(SoundPlayback& playback, const Variables& variables,
                            openbus::rendering::ViewpointContext viewpoint);
    void stopAllLoops(SoundPlayback& playback);
    void stopAll(SoundPlayback& playback);

  private:
    struct ActiveSound {
        std::filesystem::path file;
        SoundPlaybackHandle handle = 0;
        SoundTriggerDefinition definition;
        float controlValue = 0.0F;
        SoundPlaybackParameters parameters;
    };

    static bool conditionsAllow(const SoundTriggerDefinition& definition,
                                const Variables& variables);
    std::filesystem::path resolvedFile(const SoundTriggerDefinition& definition) const;
    SoundPlaybackParameters worldParameters(SoundPlaybackParameters parameters) const;
    void stopTriggeredLoop(SoundPlayback& playback, SoundDefinitionId id);
    void stopOneShot(SoundPlayback& playback, SoundDefinitionId id);
    void stopAmbientLoop(SoundPlayback& playback, SoundDefinitionId id);

    std::unordered_map<std::string, std::vector<SoundTriggerDefinition>> triggers_;
    std::unordered_set<std::string> warnedUnknownTriggers_;
    std::vector<SoundTriggerDefinition> untriggeredLoopSounds_;
    std::filesystem::path basePath_;
    std::unordered_map<SoundDefinitionId, ActiveSound> activeTriggeredLoops_;
    std::unordered_map<SoundDefinitionId, SoundPlaybackHandle> activeAmbientLoops_;
    std::unordered_map<SoundDefinitionId, ActiveSound> activeOneShots_;
    BodyPose pose_{{}, {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
};
