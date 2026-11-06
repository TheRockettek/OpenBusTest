#include "app/CustomWeatherUi.h"
#include "core/Metar.h"

#include <imgui.h>

#include <cmath>
#include <limits>
#include <optional>

namespace openbus {

CustomWeatherAction drawCustomWeatherEditor(WeatherState& draft, const std::string& error) {
    ImGui::TextWrapped("Start from the current weather. Enable an optional value to set "
                       "it; disabled values remain unspecified.");
    const auto optionalInput = [](const char* label, std::optional<double>& field,
                                  double defaultValue) {
        ImGui::PushID(label);
        bool enabled = field.has_value();
        if (ImGui::Checkbox("Set", &enabled)) {
            if (enabled)
                field = defaultValue;
            else
                field.reset();
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(!enabled);
        double value = field.value_or(defaultValue);
        ImGui::SetNextItemWidth(160.0F);
        if (ImGui::InputDouble(label, &value, 0.0, 0.0, "%.2f") && enabled)
            field = value;
        ImGui::EndDisabled();
        ImGui::PopID();
    };
    ImGui::SetNextItemWidth(160.0F);
    const char* precipitationTypes[] = {"Clear", "Rain", "Snow"};
    ImGui::Combo("Precipitation", &draft.precipitationType, precipitationTypes, 3);
    ImGui::SetNextItemWidth(160.0F);
    ImGui::InputDouble("Intensity (0 to 1)", &draft.precipitationRate, 0.0, 0.0, "%.2f");
    ImGui::SetNextItemWidth(160.0F);
    ImGui::InputDouble("Temperature (C)", &draft.temperatureCelsius, 0.0, 0.0, "%.2f");
    ImGui::SetNextItemWidth(160.0F);
    ImGui::InputDouble("Absolute humidity (g/m3)", &draft.absoluteHumidityGramsPerCubicMeter, 0.0,
                       0.0, "%.2f");
    optionalInput("Visibility (m)", draft.visibilityMeters, 10000.0);
    optionalInput("Brightness (0 to 1)", draft.brightness, 1.0);
    optionalInput("Wind from north (degrees, 0 to <360)", draft.windDirectionDegrees, 0.0);
    optionalInput("Wind speed (m/s)", draft.windSpeedMetersPerSecond, 0.0);
    optionalInput("Cloud base above ground (m)", draft.cloudBaseAboveGroundMeters, 1000.0);
    optionalInput("Fog top above ground (m)", draft.fogTopAboveGroundMeters, 100.0);
    optionalInput("Atmospheric pressure (hPa)", draft.atmosphericPressureHectopascals, 1013.25);
    const char* cloudTypes[] = {"None", "Cumulus 1", "Cumulus 2", "Cumulus 3", "Overcast 1"};
    int cloud = draft.cloudType ? static_cast<int>(*draft.cloudType) + 1 : 0;
    ImGui::SetNextItemWidth(160.0F);
    if (ImGui::Combo("Cloud type", &cloud, cloudTypes, 5)) {
        draft.cloudType =
            cloud == 0 ? std::nullopt : std::optional{static_cast<WeatherCloudType>(cloud - 1)};
    }
    if (std::isfinite(draft.temperatureCelsius) && draft.temperatureCelsius >= -40.0 &&
        draft.temperatureCelsius <= 50.0 &&
        std::isfinite(draft.absoluteHumidityGramsPerCubicMeter) &&
        draft.absoluteHumidityGramsPerCubicMeter >= 0.0 &&
        draft.absoluteHumidityGramsPerCubicMeter <= std::numeric_limits<float>::max()) {
        double relative = relativeHumidityFromAbsolute(draft.temperatureCelsius,
                                                       draft.absoluteHumidityGramsPerCubicMeter);
        const double minimum = 0.0;
        const double maximum = 100.0;
        ImGui::SetNextItemWidth(160.0F);
        if (ImGui::SliderScalar("Relative humidity (%)", ImGuiDataType_Double, &relative, &minimum,
                                &maximum, "%.1f", ImGuiSliderFlags_AlwaysClamp)) {
            draft.absoluteHumidityGramsPerCubicMeter =
                absoluteHumidityFromRelative(draft.temperatureCelsius, relative);
        }
        if (relative > 100.0)
            ImGui::TextWrapped("Above saturation; moving the slider sets humidity to 0-100%%.");
        const auto dewPoint = dewPointFromAbsoluteHumidity(
            draft.temperatureCelsius, draft.absoluteHumidityGramsPerCubicMeter);
        if (dewPoint)
            ImGui::Text("Estimated dew point: %.1f C", *dewPoint);
        else
            ImGui::TextDisabled("Dew point unavailable for this humidity.");
    } else {
        ImGui::TextWrapped("Relative humidity is unavailable for invalid or extremely large "
                           "humidity, or temperatures outside -40 to 50 C.");
    }
    ImGui::TextWrapped("Clear weather sets intensity to zero. Visibility may be reduced "
                       "by precipitation. Pressure is stored but does not yet affect "
                       "simulation. Existing surface water and snow are preserved.");
    if (!error.empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0F, 0.4F, 0.4F, 1.0F));
        ImGui::TextWrapped("%s", error.c_str());
        ImGui::PopStyleColor();
    }
    if (ImGui::Button("Apply custom weather")) {
        return CustomWeatherAction::Apply;
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
        return CustomWeatherAction::Cancel;
    }
    return CustomWeatherAction::None;
}

std::optional<SurfaceWeatherAction>
drawSurfaceWeatherControls(const RoadWeatherConditions& conditions) {
    ImGui::Text("Road water: %.0f%%", conditions.water * 100.0);
    ImGui::Text("Road snow: %.0f%%", conditions.snow * 100.0);
    ImGui::Text("Ground snow: %.0f%%", conditions.groundSnow * 100.0);
    ImGui::TextWrapped("These controls change global accumulated conditions, not the current "
                       "weather or authored snow surfaces. Precipitation can accumulate again; "
                       "remaining snow can melt into road water.");
    if (ImGui::Button("Dry accumulated road water"))
        return SurfaceWeatherAction::DryRoadWater;
    if (ImGui::Button("Clear accumulated road snow"))
        return SurfaceWeatherAction::ClearRoadSnow;
    if (ImGui::Button("Clear accumulated road and ground snow"))
        return SurfaceWeatherAction::ClearAllSnow;
    return std::nullopt;
}

} // namespace openbus
