#pragma once

#include <initializer_list>
#include <string>
#include <unordered_map>

struct VariableDefinition {
    std::string name;
    float value;
};

enum class ScriptObjectKind {
    Generic,
    System,
    Vehicle,
    Human,
    SceneryObject,
};

class Variables {
  public:
    explicit Variables(ScriptObjectKind objectKind = ScriptObjectKind::Generic);

    void setValues(std::initializer_list<VariableDefinition> values);

    void declare(const std::string& name);
    void declareString(const std::string& name);

    bool has(const std::string& name) const;
    float get(const std::string& name) const;
    void set(const std::string& name, float value);
    const std::unordered_map<std::string, float>& numericValues() const;

    bool hasString(const std::string& name) const;
    std::string getString(const std::string& name) const;
    void setString(const std::string& name, const std::string& value);
    const std::unordered_map<std::string, std::string>& stringValues() const;

    ScriptObjectKind objectKind() const;
    bool supportsSystemMacro(const std::string& name) const;
    bool supportsSystemTrigger(const std::string& name) const;

  private:
    // TODO: Add synchronization for shared/non-local variables if threaded access is introduced.
    std::unordered_map<std::string, float> values_;
    std::unordered_map<std::string, std::string> strings_;
    ScriptObjectKind objectKind_;
};

class SystemVariables : public Variables {
  public:
    SystemVariables();

    void configureSimulationClock(int year, int month, int day, double timeSeconds);
    void updateSimulationClock(double elapsedSeconds);
    void setPrecipitation(int type, double rate);
    void setWeather(int type, double rate, double temperatureCelsius,
                    double absoluteHumidityGramsPerCubicMeter);
    void setAutoClutch(bool enabled);
    void setSunAltitude(double altitudeDegrees);
    void setPaused(bool paused);
    void setSoundAvailable(bool available);

    // Cursor position is separate from handler-scoped OMSI mouse_x/y deltas.
    void updateFrame(float timegap, float getTime, float cursorX, float cursorY);

  private:
    int clockStartYear_ = 0;
    int clockStartMonth_ = 0;
    int clockStartDay_ = 0;
    double clockStartTimeSeconds_ = 0.0;
};

class SimulationState {
  public:
    SystemVariables& sharedVariables() {
        return variables_;
    }
    const SystemVariables& sharedVariables() const {
        return variables_;
    }

  private:
    SystemVariables variables_;
};

namespace openbus::scripting {

class Vehicle : public ::Variables {
  public:
    struct Environment {
        float precipitationType;
        float precipitationRate;
    };

    Vehicle();

    static Environment captureEnvironment(const SystemVariables& system);
    void updateFrame();
    void updateEnvironment(const SystemVariables& system);
    void updateEnvironment(const Environment& environment);
};

class Human : public ::Variables {
  public:
    Human();
};

class SceneryObject : public ::Variables {
  public:
    SceneryObject();
};

} // namespace openbus::scripting
