#pragma once

#include <array>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

struct SoundCurvePoint {
    double x = 0.0;
    double y = 0.0;
};

struct SoundTriggerDefinition {
    std::filesystem::path file;
    bool loop = false;
    int viewpoint = 0;
    double maxDistance = 0.0;
    std::array<double, 3> position = {};
    std::string controlVariable;
    double controlCenter = 0.0;
    std::vector<SoundCurvePoint> volumeCurve;
};

class SoundEngine {
  public:
    SoundEngine();
    ~SoundEngine();
    SoundEngine(const SoundEngine&) = delete;
    SoundEngine& operator=(const SoundEngine&) = delete;

    void load(const std::filesystem::path& configPath);
    void setListenerPose(const std::array<double, 3>& position,
                         const std::array<double, 3>& forward, const std::array<double, 3>& up);
    void trigger(const std::string& name, const std::filesystem::path& overrideFile = {},
                 double controlValue = 0.0);
    void stop(const std::string& name);

  private:
    struct Backend;
    std::unordered_map<std::string, std::vector<SoundTriggerDefinition>> triggers_;
    std::vector<SoundTriggerDefinition> untriggeredLoopSounds_;
    std::filesystem::path basePath_;
    std::unique_ptr<Backend> backend_;
};