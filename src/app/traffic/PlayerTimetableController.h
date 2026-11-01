#pragma once

#include "app/traffic/PassengerNetwork.h"
#include "map/MapConfigLoader.h"
#include "config/HofData.h"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace openbus::app {

bool passengerRouteMatchesHof(const PassengerStopRouteState& route,
                              const openbus::config::HofData& hof);
const openbus::config::HofRoute* passengerRouteHofRoute(const PassengerStopRouteState& route,
                                                        const openbus::config::HofData& hof);

enum class PassengerHofMatchStatus {
    Matched,
    TooFewStops,
    NoMatchingSequence,
    AmbiguousLines
};

struct PassengerHofMatch {
    const openbus::config::HofRoute* route = nullptr;
    PassengerHofMatchStatus status = PassengerHofMatchStatus::NoMatchingSequence;
    std::size_t matchingSequences = 0;
    std::size_t sameLineCandidates = 0;
    std::optional<std::size_t> exampleSameLineRoute;
};

PassengerHofMatch analyzePassengerRouteHof(const PassengerStopRouteState& route,
                                           const openbus::config::HofData& hof);

struct PlayerTimetableCatalogTrip {
    std::size_t tripIndex = 0;
    std::size_t routeIndex = 0;
};

struct PlayerTimetableCatalogCourse {
    std::size_t courseIndex = 0;
    std::vector<PlayerTimetableCatalogTrip> trips;
};

struct PlayerTimetableCatalogLine {
    std::string name;
    std::vector<PlayerTimetableCatalogCourse> courses;
};

struct PlayerTimetableCatalogRoute {
    PassengerHofMatchStatus status = PassengerHofMatchStatus::NoMatchingSequence;
    std::size_t matchingSequences = 0;
    std::size_t sameLineCandidates = 0;
    std::optional<std::size_t> exampleSameLineRoute;
    std::string hofLine;
    std::string hofCode;
};

class PlayerTimetableCatalog {
  public:
    void invalidate() noexcept { valid_ = false; }
    bool update(const std::vector<PassengerStopRouteState>& routes,
                const std::vector<openbus::map::MapAiTimetableCourse>& courses,
                const openbus::config::HofData& hof);

    std::vector<PlayerTimetableCatalogRoute> routes;
    std::vector<PlayerTimetableCatalogLine> lines;

  private:
    bool valid_ = false;
};

struct PlayerTimetableSelection {
    std::size_t passengerRouteIndex = 0;
    std::size_t courseIndex = 0;
    std::size_t courseTripIndex = 0;
    std::size_t firstStopIndex = 0;
    bool driveAsAdditionalBus = false;
};

struct PlayerTimetableState {
    PlayerTimetableSelection selection;
    std::size_t nextStopIndex = 0;
    std::string lineName;
    bool stopAfterCurrentTrip = false;
};

class PlayerTimetableController {
  public:
    std::optional<PlayerTimetableState> state;

    const openbus::map::MapAiTimetableCourseTrip*
    select(std::optional<PlayerTimetableSelection> selection,
           const std::vector<PassengerStopRouteState>& routes,
           const std::vector<openbus::map::MapAiTimetableCourse>& courses);

    const openbus::map::MapAiTimetableCourseTrip*
    advance(const std::vector<PassengerStopRouteState>& routes,
            const std::vector<openbus::map::MapAiTimetableCourse>& courses,
            const openbus::config::HofData* hof = nullptr);
};

} // namespace openbus::app
