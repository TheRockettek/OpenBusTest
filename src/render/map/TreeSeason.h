#pragma once

#include "map/MapConfigLoader.h"

#include <array>
#include <charconv>
#include <optional>
#include <string>
#include <vector>

namespace openbus::rendering::map_detail {

inline constexpr std::array<const char*, 5> treeSeasonDirectories = {"Summer", "Spring", "Fall",
                                                                     "Winter", "WinterSnow"};
inline constexpr double treeSnowThreshold = 1.0 / 255.0;

struct TreeSeasonPeriod {
    int season;
    int start;
    int end;
};

inline std::optional<TreeSeasonPeriod>
parseTreeSeason(const openbus::map::MapSeasonDefinition& definition) {
    std::array<int, 3> values = {};
    for (std::size_t index = 0; index < values.size(); ++index) {
        const std::string& field = definition.fields[index];
        const auto result =
            std::from_chars(field.data(), field.data() + field.size(), values[index]);
        if (result.ec != std::errc{} || result.ptr != field.data() + field.size()) {
            return std::nullopt;
        }
    }
    if (values[0] < 0 || values[0] > 4 || values[1] < 0 || values[2] > 370 ||
        values[1] >= values[2]) {
        return std::nullopt;
    }
    return TreeSeasonPeriod{values[0], values[1], values[2]};
}

inline std::size_t treeSeasonIndex(const std::vector<TreeSeasonPeriod>& periods, int dayOfYear,
                                   double groundSnow) {
    if (groundSnow >= treeSnowThreshold) {
        return 4;
    }
    const int day = dayOfYear - 1;
    for (const TreeSeasonPeriod& period : periods) {
        if (day >= period.start && day < period.end) {
            return static_cast<std::size_t>(period.season == 4 ? 3 : period.season);
        }
    }
    return 0;
}

} // namespace openbus::rendering::map_detail
