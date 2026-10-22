#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

namespace openbus::map {

struct MapTrafficSignalPhase {
    int state = 0;
    double durationSeconds = 0.0;
};

struct MapTrafficSignalControlPoint {
    std::size_t lightIndex = 0;
    double timeSeconds = 0.0;
    bool ifRequest = false;
    std::optional<double> jumpToSeconds;
};

struct MapTrafficSignalProgram {
    double cycleDurationSeconds = 0.0;
    std::vector<std::vector<MapTrafficSignalPhase>> lights;
    std::vector<std::optional<double>> approachDistancesMeters;
    std::vector<MapTrafficSignalControlPoint> controlPoints;
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
        (!finalPhaseExtendsToCycle &&
         std::abs(configuredDuration - cycleDurationSeconds) > durationTolerance) ||
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
    return std::nullopt;
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
            point.timeSeconds < 0.0 ||
            (point.jumpToSeconds.has_value() &&
             (!std::isfinite(*point.jumpToSeconds) || *point.jumpToSeconds < 0.0))) {
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
          passed_(program_ == nullptr ? 0 : program_->controlPoints.size(), false) {
        if (program_ == nullptr || !mapTrafficSignalProgramValid(*program_)) {
            throw std::invalid_argument("Traffic signal program is invalid");
        }
    }

    void clearRequests() noexcept {
        std::fill(requests_.begin(), requests_.end(), false);
    }

    void request(std::size_t lightIndex) noexcept {
        if (lightIndex < requests_.size()) {
            requests_[lightIndex] = true;
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
                if (rewoundControl_ == index) {
                    continue;
                }
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
            const MapTrafficSignalControlPoint& point = program_->controlPoints[pointIndex];
            if (!controlPointActive(pointIndex)) {
                passed_[pointIndex] = true;
                continue;
            }

            if (!point.jumpToSeconds.has_value()) {
                if (inRedYellowChangeover()) {
                    passed_[pointIndex] = true;
                    continue;
                }
                heldControl_ = pointIndex;
                return;
            }

            const double sourceTime = normalizedTime(point.timeSeconds);
            const double targetTime =
                *point.jumpToSeconds < cycle ? normalizedTime(*point.jumpToSeconds) : 0.0;
            const bool noProgress = normalizedTime(targetTime - sourceTime) <= 1.0e-9;
            rewoundControl_ = targetTime > 1.0e-9 && targetTime < sourceTime - 1.0e-9
                                  ? pointIndex
                                  : std::optional<std::size_t>{};
            clockSeconds_ = targetTime;
            std::fill(passed_.begin(), passed_.end(), false);
            for (std::size_t index = 0; index < program_->controlPoints.size(); ++index) {
                if (index != pointIndex &&
                    normalizedTime(normalizedTime(program_->controlPoints[index].timeSeconds) -
                                   targetTime) <= 1.0e-9) {
                    passed_[index] = true;
                }
            }
            if (noProgress) {
                passed_[pointIndex] = true;
            }
            if (remaining <= 1.0e-9) {
                return;
            }
        }
        advanceClock(remaining);
    }

    std::optional<int> state(std::size_t lightIndex) const {
        if (lightIndex >= program_->lights.size()) {
            return std::nullopt;
        }
        return mapTrafficSignalPhaseAtTime(program_->cycleDurationSeconds,
                                           program_->lights[lightIndex], clockSeconds_);
    }

    double clockSeconds() const noexcept {
        return clockSeconds_;
    }

  private:
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
        if (rewoundControl_.has_value()) {
            const double sourceTime =
                normalizedTime(program_->controlPoints[*rewoundControl_].timeSeconds);
            const double distanceToSource = normalizedTime(sourceTime - clockSeconds_);
            if (distanceToSource <= elapsedSeconds + 1.0e-9) {
                rewoundControl_.reset();
            }
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
                    other.jumpToSeconds == point.jumpToSeconds && requests_[other.lightIndex]) {
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
    std::vector<bool> passed_;
    double clockSeconds_ = 0.0;
    bool started_ = false;
    std::optional<std::size_t> heldControl_;
    std::optional<std::size_t> rewoundControl_;
};

struct MapTrafficSignalSchedule {
    double cycleDurationSeconds = 0.0;
    std::vector<MapTrafficSignalPhase> phases;
    bool hasConditionalOverrides = false;
    std::size_t lightIndex = 0;
    std::optional<double> approachDistanceMeters;
    std::shared_ptr<const MapTrafficSignalProgram> program;
    std::shared_ptr<MapTrafficSignalController> controller;
};

// Returns the active OMSI light phase, or no value when the schedule is invalid.
inline std::optional<int> mapTrafficSignalStateAtTime(const MapTrafficSignalSchedule& schedule,
                                                      double elapsedSeconds) {
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0) {
        return std::nullopt;
    }
    if (schedule.controller != nullptr && schedule.hasConditionalOverrides) {
        return schedule.controller->state(schedule.lightIndex);
    }
    return mapTrafficSignalPhaseAtTime(schedule.cycleDurationSeconds, schedule.phases,
                                       elapsedSeconds);
}

} // namespace openbus::map
