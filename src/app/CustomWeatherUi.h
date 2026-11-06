#pragma once

#include "core/WeatherState.h"

#include <string>

namespace openbus {

enum class CustomWeatherAction { None, Apply, Cancel };

CustomWeatherAction drawCustomWeatherEditor(WeatherState& draft, const std::string& error);
std::optional<SurfaceWeatherAction>
drawSurfaceWeatherControls(const RoadWeatherConditions& conditions);

} // namespace openbus
