#pragma once

#include "core/WeatherSelection.h"

#include <array>
#include <optional>
#include <vector>

namespace openbus {

struct MetarStationPicker {
    std::array<char, 128> search{};
    std::array<char, 32> code{};
    std::optional<std::size_t> selected;
};

void drawMetarStationPicker(MetarStationPicker& picker, const std::vector<MetarStation>& stations);
void remapMetarStationSelection(MetarStationPicker& picker,
                                const std::vector<MetarStation>& previous,
                                const std::vector<MetarStation>& updated);

} // namespace openbus
