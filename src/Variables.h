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

    void updateFrame(float timegap, float getTime, float mouseX, float mouseY);
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
    Vehicle();

    void updateFrame();
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
