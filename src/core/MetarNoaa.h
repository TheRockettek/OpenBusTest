#pragma once

#include "core/Metar.h"
#include "core/WeatherState.h"

#include <chrono>
#include <string>
#include <string_view>

namespace openbus {

struct NoaaMetarReport {
    MetarObservation observation;
    std::chrono::sys_time<std::chrono::minutes> observedAtUtc;
};

std::string noaaMetarStation(std::string_view station);
std::string fetchNoaaMetar(std::string_view station);
std::string fetchNoaaMetarDirectory();
NoaaMetarReport parseNoaaMetar(std::string_view station, std::string_view body,
                               std::chrono::system_clock::time_point now);
WeatherState weatherFromMetar(const MetarObservation& observation);

} // namespace openbus
