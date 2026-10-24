#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

namespace openbus::map {

class MapTrafficSignalController;

struct MapTrafficSignalPhase {
    int state = 0;
    double durationSeconds = 0.0;
};

struct MapTrafficSignalControlPoint {
    std::size_t lightIndex = 0;
    double timeSeconds = 0.0;
    bool ifRequest = false;
};

struct MapTrafficSignalBusPriorityRule {
    int phaseToInterrupt = 0;
    double triggerDistanceMeters = 0.0;
    int targetPhase = 0;
    double minimumGreenSeconds = 0.0;
};

struct MapTrafficSignalProgram {
    double cycleDurationSeconds = 0.0;
    std::vector<std::vector<MapTrafficSignalPhase>> lights;
    std::vector<std::optional<double>> approachDistancesMeters;
    std::vector<MapTrafficSignalControlPoint> controlPoints;
    std::vector<MapTrafficSignalBusPriorityRule> busPriorityRules;
};

struct MapTrafficSignalBusPriorityRequest {
    std::shared_ptr<MapTrafficSignalController> controller;
    std::size_t lightIndex = 0;
};

inline std::optional<int>
mapTrafficSignalPhaseAtTime(double cycleDurationSeconds,
                            const std::vector<MapTrafficSignalPhase>& phases,
                            double elapsedSeconds) {
    if (!std::isfinite(cycleDurationSeconds) || cycleDurationSeconds <= 0.0 ||
        !std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0 || phases.empty()) {
        return std::nullopt;
    }

    double configuredDuration = 0.0;
    for (std::size_t index = 0; index < phases.size(); ++index) {
        const MapTrafficSignalPhase& phase = phases[index];
        if (phase.state < 0 || phase.state > 12 || !std::isfinite(phase.durationSeconds) ||
            phase.durationSeconds < 0.0 ||
            (phase.durationSeconds == 0.0 && index + 1 != phases.size())) {
            return std::nullopt;
        }
        configuredDuration += phase.durationSeconds;
    }
    const double durationTolerance = std::max(1.0, cycleDurationSeconds) * 1.0e-9;
    const bool finalPhaseExtendsToCycle = phases.back().durationSeconds == 0.0;
    if (configuredDuration > cycleDurationSeconds + durationTolerance ||
        (finalPhaseExtendsToCycle &&
         configuredDuration >= cycleDurationSeconds - durationTolerance)) {
        return std::nullopt;
    }

    const double cycleTime = std::fmod(elapsedSeconds, cycleDurationSeconds);
    double phaseStart = 0.0;
    for (std::size_t index = 0; index < phases.size(); ++index) {
        const MapTrafficSignalPhase& phase = phases[index];
        if (phase.durationSeconds == 0.0 && index + 1 == phases.size()) {
            return phase.state;
        }
        const double phaseEnd = phaseStart + phase.durationSeconds;
        if (cycleTime < phaseEnd) {
            return phase.state;
        }
        phaseStart = phaseEnd;
    }
    return 12;
}

inline bool mapTrafficSignalProgramValid(const MapTrafficSignalProgram& program) {
    if (!std::isfinite(program.cycleDurationSeconds) || program.cycleDurationSeconds <= 0.0 ||
        program.lights.empty() || program.approachDistancesMeters.size() != program.lights.size()) {
        return false;
    }
    for (const auto& phases : program.lights) {
        if (!mapTrafficSignalPhaseAtTime(program.cycleDurationSeconds, phases, 0.0)) {
            return false;
        }
    }
    for (const MapTrafficSignalControlPoint& point : program.controlPoints) {
        if (point.lightIndex >= program.lights.size() || !std::isfinite(point.timeSeconds) ||
            point.timeSeconds < 0.0) {
            return false;
        }
    }
    for (const MapTrafficSignalBusPriorityRule& rule : program.busPriorityRules) {
        if (rule.phaseToInterrupt < 0 || rule.phaseToInterrupt > 12 ||
            !std::isfinite(rule.triggerDistanceMeters) || rule.triggerDistanceMeters < 0.0 ||
            rule.targetPhase < 6 || rule.targetPhase > 8 ||
            !std::isfinite(rule.minimumGreenSeconds) || rule.minimumGreenSeconds < 0.0) {
            return false;
        }
    }
    return true;
}

class MapTrafficSignalController {
  public:
    explicit MapTrafficSignalController(std::shared_ptr<const MapTrafficSignalProgram> program)
        : program_(std::move(program)),
          requests_(program_ == nullptr ? 0 : program_->lights.size(), false),
          busPriorityRequests_(program_ == nullptr ? 0 : program_->lights.size(), false),
          servedBusPriorityRequests_(program_ == nullptr ? 0 : program_->lights.size(), false),
          passed_(program_ == nullptr ? 0 : program_->controlPoints.size(), false) {
        if (program_ == nullptr || !mapTrafficSignalProgramValid(*program_)) {
            throw std::invalid_argument("Traffic signal program is invalid");
        }
    }

    void clearRequests() noexcept {
        std::fill(requests_.begin(), requests_.end(), false);
        std::fill(busPriorityRequests_.begin(), busPriorityRequests_.end(), false);
    }

    void request(std::size_t lightIndex) noexcept {
        if (lightIndex < requests_.size()) {
            requests_[lightIndex] = true;
        }
    }

    void requestBusPriority(std::size_t lightIndex) noexcept {
        if (lightIndex < busPriorityRequests_.size()) {
            busPriorityRequests_[lightIndex] = true;
        }
    }

    void advance(double elapsedSeconds, double initialClockSeconds) {
        if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0 ||
            !std::isfinite(initialClockSeconds) || initialClockSeconds < 0.0) {
            throw std::invalid_argument("Traffic signal clock inputs are invalid");
        }
        const double cycle = program_->cycleDurationSeconds;
        if (!started_) {
            clockSeconds_ = std::fmod(initialClockSeconds, cycle);
            started_ = true;
        }

        for (std::size_t index = 0; index < busPriorityRequests_.size(); ++index) {
            if (!busPriorityRequests_[index]) {
                servedBusPriorityRequests_[index] = false;
            }
        }
        if (priorityOverrideLightIndex_.has_value()) {
            priorityOverrideRemainingSeconds_ -= elapsedSeconds;
            if (priorityOverrideRemainingSeconds_ <= 1.0e-9) {
                priorityOverrideLightIndex_.reset();
                priorityOverrideRemainingSeconds_ = 0.0;
            }
        }
        if (!priorityOverrideLightIndex_.has_value()) {
            activateBusPriorityIfSafe();
        }

        if (heldControl_.has_value()) {
            if (controlPointActive(*heldControl_)) {
                return;
            }
            passed_[*heldControl_] = true;
            heldControl_.reset();
        }

        double remaining = elapsedSeconds;
        const std::size_t iterationLimit = 16 + program_->controlPoints.size();
        for (std::size_t iteration = 0; iteration < iterationLimit; ++iteration) {
            std::optional<std::size_t> nextPoint;
            double distanceToPoint = remaining + 1.0e-9;
            for (std::size_t index = 0; index < program_->controlPoints.size(); ++index) {
                const double pointTime = normalizedTime(program_->controlPoints[index].timeSeconds);
                const double distance = normalizedTime(pointTime - clockSeconds_);
                if (distance <= 1.0e-9 && passed_[index]) {
                    continue;
                }
                if (distance <= remaining + 1.0e-9 && distance < distanceToPoint) {
                    nextPoint = index;
                    distanceToPoint = distance;
                }
            }
            if (!nextPoint.has_value()) {
                advanceClock(remaining);
                return;
            }

            if (distanceToPoint > 1.0e-9) {
                advanceClock(distanceToPoint);
                remaining = std::max(0.0, remaining - distanceToPoint);
            }
            const std::size_t pointIndex = *nextPoint;
            if (!controlPointActive(pointIndex)) {
                passed_[pointIndex] = true;
                continue;
            }

            if (inRedYellowChangeover()) {
                passed_[pointIndex] = true;
                continue;
            }
            heldControl_ = pointIndex;
            return;
        }
        advanceClock(remaining);
    }

    std::optional<int> state(std::size_t lightIndex) const {
        if (lightIndex >= program_->lights.size()) {
            return std::nullopt;
        }
        if (priorityOverrideLightIndex_.has_value() && priorityOverrideRemainingSeconds_ > 1.0e-9) {
            return lightIndex == *priorityOverrideLightIndex_ ? priorityOverrideState_ : 0;
        }
        return mapTrafficSignalPhaseAtTime(program_->cycleDurationSeconds,
                                           program_->lights[lightIndex], clockSeconds_);
    }

    double clockSeconds() const noexcept {
        return clockSeconds_;
    }

  private:
    void activateBusPriorityIfSafe() {
        for (std::size_t requestedLight = 0; requestedLight < busPriorityRequests_.size();
             ++requestedLight) {
            if (!busPriorityRequests_[requestedLight] ||
                servedBusPriorityRequests_[requestedLight]) {
                continue;
            }
            const std::optional<int> currentState = mapTrafficSignalPhaseAtTime(
                program_->cycleDurationSeconds, program_->lights[requestedLight], clockSeconds_);
            if (!currentState.has_value()) {
                continue;
            }
            for (const MapTrafficSignalBusPriorityRule& rule : program_->busPriorityRules) {
                if (*currentState != rule.phaseToInterrupt) {
                    continue;
                }
                bool conflictingGreen = false;
                for (std::size_t otherLight = 0; otherLight < program_->lights.size();
                     ++otherLight) {
                    if (otherLight == requestedLight) {
                        continue;
                    }
                    const std::optional<int> otherState =
                        mapTrafficSignalPhaseAtTime(program_->cycleDurationSeconds,
                                                    program_->lights[otherLight], clockSeconds_);
                    if (otherState.has_value() && *otherState >= 6 && *otherState <= 8) {
                        conflictingGreen = true;
                        break;
                    }
                }
                if (conflictingGreen) {
                    continue;
                }
                priorityOverrideLightIndex_ = requestedLight;
                priorityOverrideState_ = rule.targetPhase;
                priorityOverrideRemainingSeconds_ = rule.minimumGreenSeconds;
                servedBusPriorityRequests_[requestedLight] = true;
                if (priorityOverrideRemainingSeconds_ <= 1.0e-9) {
                    priorityOverrideRemainingSeconds_ = std::numeric_limits<double>::epsilon();
                }
                return;
            }
        }
    }

    double normalizedTime(double time) const noexcept {
        const double cycle = program_->cycleDurationSeconds;
        double normalized = std::fmod(time, cycle);
        if (normalized < 0.0) {
            normalized += cycle;
        }
        return normalized;
    }

    void advanceClock(double elapsedSeconds) noexcept {
        if (elapsedSeconds <= 1.0e-9) {
            return;
        }
        clockSeconds_ = normalizedTime(clockSeconds_ + elapsedSeconds);
        std::fill(passed_.begin(), passed_.end(), false);
    }

    bool controlPointActive(std::size_t pointIndex) const {
        const MapTrafficSignalControlPoint& point = program_->controlPoints[pointIndex];
        bool requested = requests_[point.lightIndex];
        if (point.ifRequest) {
            for (const MapTrafficSignalControlPoint& other : program_->controlPoints) {
                if (other.ifRequest && other.timeSeconds == point.timeSeconds &&
                    requests_[other.lightIndex]) {
                    requested = true;
                    break;
                }
            }
            return !requested;
        }
        return requested;
    }

    bool inRedYellowChangeover() const {
        for (std::size_t index = 0; index < program_->lights.size(); ++index) {
            const std::optional<int> lightState = state(index);
            if (lightState.has_value() && *lightState >= 3 && *lightState <= 5) {
                return true;
            }
        }
        return false;
    }

    std::shared_ptr<const MapTrafficSignalProgram> program_;
    std::vector<bool> requests_;
    std::vector<bool> busPriorityRequests_;
    std::vector<bool> servedBusPriorityRequests_;
    std::vector<bool> passed_;
    double clockSeconds_ = 0.0;
    double priorityOverrideRemainingSeconds_ = 0.0;
    int priorityOverrideState_ = 0;
    std::optional<std::size_t> priorityOverrideLightIndex_;
    bool started_ = false;
    std::optional<std::size_t> heldControl_;
};

struct MapTrafficSignalSchedule {
    double cycleDurationSeconds = 0.0;
    std::vector<MapTrafficSignalPhase> phases;
    bool hasConditionalOverrides = false;
    std::size_t lightIndex = 0;
    std::optional<double> approachDistanceMeters;
    bool hasBusPriority = false;
    std::shared_ptr<const MapTrafficSignalProgram> program;
    std::shared_ptr<MapTrafficSignalController> controller;
};

// Returns the active OMSI light phase, or no value when the schedule is invalid.
inline std::optional<int> mapTrafficSignalStateAtTime(const MapTrafficSignalSchedule& schedule,
                                                      double elapsedSeconds) {
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0) {
        return std::nullopt;
    }
    if (schedule.controller != nullptr &&
        (schedule.hasConditionalOverrides || schedule.hasBusPriority)) {
        return schedule.controller->state(schedule.lightIndex);
    }
    return mapTrafficSignalPhaseAtTime(schedule.cycleDurationSeconds, schedule.phases,
                                       elapsedSeconds);
}

} // namespace openbus::map
