#pragma once

#include <shared_mutex>
#include <string>
#include <unordered_map>

class Variables {
  public:
    Variables();

    void declare(const std::string& name);
    double get(const std::string& name) const;
    void set(const std::string& name, double value);
    void updateFrame(double timegap, double getTime, double mouseX, double mouseY);

  private:
    mutable std::shared_mutex mutex_;
    std::unordered_map<std::string, double> values_;
};

class SimulationState {
  public:
    Variables& sharedVariables() {
        return variables_;
    }
    const Variables& sharedVariables() const {
        return variables_;
    }

  private:
    Variables variables_;
};

class VehicleState : public Variables {
  public:
    explicit VehicleState(SimulationState& simulation) : simulation_(simulation) {}

    Variables& sharedVariables() {
        return simulation_.sharedVariables();
    }
    const Variables& sharedVariables() const {
        return simulation_.sharedVariables();
    }

  private:
    SimulationState& simulation_;
};