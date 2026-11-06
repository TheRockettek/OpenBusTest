#include "app/MetarStationUi.h"

#include <imgui.h>

#include <algorithm>
#include <cctype>
#include <string>

namespace openbus {
namespace {

std::string lowercase(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

std::string stationLabel(const MetarStation& station) {
    std::string label = station.code;
    if (!station.name.empty())
        label += " - " + station.name;
    if (!station.country.empty())
        label += " - " + station.country;
    return label;
}

} // namespace

void remapMetarStationSelection(MetarStationPicker& picker,
                                const std::vector<MetarStation>& previous,
                                const std::vector<MetarStation>& updated) {
    if (!picker.selected)
        return;
    if (*picker.selected >= previous.size()) {
        picker.selected.reset();
        return;
    }
    const auto& code = previous[*picker.selected].code;
    const auto found = std::find_if(updated.begin(), updated.end(),
                                    [&](const auto& station) { return station.code == code; });
    if (found == updated.end())
        picker.selected.reset();
    else
        picker.selected = static_cast<std::size_t>(found - updated.begin());
}

void drawMetarStationPicker(MetarStationPicker& picker, const std::vector<MetarStation>& stations) {
    ImGui::InputTextWithHint("Search stations", "ICAO code, name or country", picker.search.data(),
                             picker.search.size());
    const std::string search = lowercase(picker.search.data());
    const std::string selectedLabel = picker.selected && *picker.selected < stations.size()
                                          ? stationLabel(stations[*picker.selected])
                                          : "Select a station";
    if (ImGui::BeginCombo("Station", selectedLabel.c_str())) {
        std::vector<std::size_t> matches;
        for (std::size_t i = 0; i < stations.size(); ++i) {
            if (lowercase(stationLabel(stations[i])).find(search) != std::string::npos)
                matches.push_back(i);
        }
        ImGuiListClipper clipper;
        clipper.Begin(static_cast<int>(matches.size()));
        while (clipper.Step()) {
            for (int item = clipper.DisplayStart; item < clipper.DisplayEnd; ++item) {
                const auto i = matches[static_cast<std::size_t>(item)];
                const auto& station = stations[i];
                const std::string label = stationLabel(station);
                if (ImGui::Selectable(label.c_str(), picker.selected == i)) {
                    picker.selected = i;
                    std::copy_n(station.code.begin(), 4, picker.code.begin());
                    picker.code[4] = '\0';
                }
            }
        }
        if (matches.empty())
            ImGui::TextDisabled("No matching stations");
        ImGui::EndCombo();
    }
    if (ImGui::InputTextWithHint("Station ICAO", "e.g. EGLL or KJFK", picker.code.data(),
                                 picker.code.size(), ImGuiInputTextFlags_CharsUppercase))
        picker.selected.reset();
}

} // namespace openbus
