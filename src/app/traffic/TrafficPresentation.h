#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace openbus::app {

inline double trafficPresentationTravel(double previousDistance, double currentDistance,
                                        double routeLength,
                                        std::optional<double> loopStart) {
    if (!std::isfinite(previousDistance) || !std::isfinite(currentDistance) ||
        !std::isfinite(routeLength) || routeLength <= 0.0 ||
        (loopStart && (!std::isfinite(*loopStart) || *loopStart < 0.0 ||
                       *loopStart >= routeLength))) {
        throw std::invalid_argument("Invalid traffic presentation travel");
    }
    const double delta = currentDistance - previousDistance;
    if (!std::isfinite(delta)) {
        throw std::invalid_argument("Traffic presentation travel is not finite");
    }
    if (delta >= 0.0) {
        return delta;
    }
    if (!loopStart) {
        return 0.0;
    }
    const double loopLength = routeLength - *loopStart;
    const double wrapped = std::fmod(delta, loopLength);
    const double tolerance = 32.0 * std::numeric_limits<double>::epsilon() *
                             std::max({1.0, std::abs(previousDistance),
                                       std::abs(currentDistance), loopLength});
    if (std::abs(wrapped) <= tolerance) {
        return 0.0;
    }
    return wrapped < 0.0 ? wrapped + loopLength : 0.0;
}

class TrafficPresentation {
  public:
    struct Sample {
        double distanceMeters;
        double lateralOffsetMeters;
    };

    void advance(double elapsedSeconds) {
        if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0) {
            throw std::invalid_argument("Invalid traffic presentation elapsed time");
        }
        elapsedSeconds_ = std::min(durationSeconds_, elapsedSeconds_ + elapsedSeconds);
        if (durationSeconds_ > 0.0 && elapsedSeconds_ >= durationSeconds_) {
            from_ = {lastTargetDistance_, to_.lateralOffsetMeters};
            to_ = from_;
            elapsedSeconds_ = 0.0;
            durationSeconds_ = 0.0;
        }
    }

    void reset(std::uint64_t generation, Sample sample) {
        validate(sample);
        generation_ = generation;
        from_ = sample;
        to_ = sample;
        lastTargetDistance_ = sample.distanceMeters;
        elapsedSeconds_ = 0.0;
        durationSeconds_ = 0.0;
    }

    bool retarget(std::uint64_t generation, Sample target, double durationSeconds,
                  double maximumTravelMeters, double routeLengthMeters,
                  std::optional<double> loopStartMeters) {
        validate(target);
        if (!std::isfinite(durationSeconds) || durationSeconds < 0.0 ||
            !std::isfinite(maximumTravelMeters) || maximumTravelMeters < 0.0 ||
            !std::isfinite(routeLengthMeters) || routeLengthMeters <= 0.0 ||
            (loopStartMeters && (!std::isfinite(*loopStartMeters) || *loopStartMeters < 0.0 ||
                                 *loopStartMeters >= routeLengthMeters))) {
            throw std::invalid_argument("Invalid traffic presentation target");
        }
        if (!generation_ || *generation_ != generation || durationSeconds == 0.0) {
            reset(generation, target);
            return true;
        }
        double travel = target.distanceMeters - lastTargetDistance_;
        if (travel < 0.0 && loopStartMeters) {
            travel += routeLengthMeters - *loopStartMeters;
        }
        if (travel < 0.0 || travel > maximumTravelMeters) {
            reset(generation, target);
            return true;
        }
        from_ = sample();
        to_ = {to_.distanceMeters + travel, target.lateralOffsetMeters};
        lastTargetDistance_ = target.distanceMeters;
        elapsedSeconds_ = 0.0;
        durationSeconds_ = durationSeconds;
        return false;
    }

    [[nodiscard]] bool initializedFor(std::uint64_t generation) const {
        return generation_ && *generation_ == generation;
    }

    [[nodiscard]] Sample sample() const {
        if (!generation_) {
            throw std::logic_error("Traffic presentation is not initialized");
        }
        const double factor = durationSeconds_ > 0.0 ? elapsedSeconds_ / durationSeconds_ : 1.0;
        return {std::lerp(from_.distanceMeters, to_.distanceMeters, factor),
                std::lerp(from_.lateralOffsetMeters, to_.lateralOffsetMeters, factor)};
    }

  private:
    static void validate(Sample sample) {
        if (!std::isfinite(sample.distanceMeters) || !std::isfinite(sample.lateralOffsetMeters)) {
            throw std::invalid_argument("Invalid traffic presentation sample");
        }
    }

    std::optional<std::uint64_t> generation_;
    Sample from_{};
    Sample to_{};
    double lastTargetDistance_ = 0.0;
    double elapsedSeconds_ = 0.0;
    double durationSeconds_ = 0.0;
};

} // namespace openbus::app
