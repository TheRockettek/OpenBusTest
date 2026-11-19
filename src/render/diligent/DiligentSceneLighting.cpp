#include "render/diligent/DiligentSceneLighting.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace openbus::rendering {

DiligentLightingParameters
buildDiligentSceneLighting(const Matrix4& view, std::optional<std::array<double, 3>> sunDirection,
                           std::optional<double> weatherBrightness, bool reflectionPass,
                           double reflectionAmbientFloor) {
    const auto isFinite = [](double value) { return std::isfinite(value); };
    if (!std::all_of(view.begin(), view.end(), isFinite)) {
        throw std::invalid_argument("Diligent scene-light view matrix must be finite");
    }

    const std::array<double, 3> worldDirection =
        sunDirection.value_or(std::array<double, 3>{-0.35, 0.5, 0.8});
    if (!std::all_of(worldDirection.begin(), worldDirection.end(), isFinite) ||
        std::hypot(worldDirection[0], worldDirection[1], worldDirection[2]) < 1.0e-12) {
        throw std::invalid_argument("Diligent scene sun direction must be finite and non-zero");
    }
    if (weatherBrightness &&
        (!isFinite(*weatherBrightness) || *weatherBrightness < 0.0 || *weatherBrightness > 1.0)) {
        throw std::invalid_argument("Diligent weather brightness must be between zero and one");
    }
    if (!isFinite(reflectionAmbientFloor) || reflectionAmbientFloor < 0.0 ||
        reflectionAmbientFloor > 1.0) {
        throw std::invalid_argument(
            "Diligent reflection ambient floor must be between zero and one");
    }

    const double brightness = weatherBrightness.value_or(1.0);
    DiligentLightingParameters lighting;
    lighting.directionalEnabled = true;
    lighting.ambientIntensity = static_cast<float>(
        std::max(0.58 * brightness, reflectionPass ? reflectionAmbientFloor : 0.0));
    lighting.diffuseIntensity = static_cast<float>(0.42 * brightness);
    lighting.weatherBrightness = static_cast<float>(brightness);
    for (std::size_t row = 0; row < lighting.lightDirectionView.size(); ++row) {
        const double component = view[row] * worldDirection[0] + view[row + 4] * worldDirection[1] +
                                 view[row + 8] * worldDirection[2];
        if (!isFinite(component) ||
            std::abs(component) > static_cast<double>(std::numeric_limits<float>::max())) {
            throw std::invalid_argument("Diligent view-space sun direction exceeds float range");
        }
        lighting.lightDirectionView[row] = static_cast<float>(component);
    }
    if (!std::all_of(lighting.lightDirectionView.begin(), lighting.lightDirectionView.end(),
                     [](float value) { return std::isfinite(value); }) ||
        std::hypot(lighting.lightDirectionView[0], lighting.lightDirectionView[1],
                   lighting.lightDirectionView[2]) < 1.0e-12F) {
        throw std::invalid_argument("Diligent view-space sun direction is invalid");
    }
    return lighting;
}

DiligentWeatherFog buildDiligentWeatherFog(const Matrix4& view, double cameraHeightAboveGround,
                                           std::optional<double> weatherBrightness,
                                           std::optional<double> visibilityMeters,
                                           std::optional<double> layerHeightMeters) {
    const auto isFinite = [](double value) { return std::isfinite(value); };
    const double brightness = weatherBrightness.value_or(1.0);
    if (!std::all_of(view.begin(), view.end(), isFinite) || !isFinite(cameraHeightAboveGround) ||
        std::abs(cameraHeightAboveGround) >
            static_cast<double>(std::numeric_limits<float>::max())) {
        throw std::invalid_argument("Diligent weather-fog view or camera height is invalid");
    }
    if (weatherBrightness && (!isFinite(brightness) || brightness < 0.0 || brightness > 1.0)) {
        throw std::invalid_argument("Diligent weather brightness must be between zero and one");
    }
    if (visibilityMeters && (!isFinite(*visibilityMeters) || *visibilityMeters < 0.0)) {
        throw std::invalid_argument("Diligent weather visibility must be finite and non-negative");
    }
    if (layerHeightMeters &&
        (!isFinite(*layerHeightMeters) || *layerHeightMeters < 0.0 ||
         *layerHeightMeters > static_cast<double>(std::numeric_limits<float>::max()))) {
        throw std::invalid_argument("Diligent weather-fog layer height is invalid");
    }

    DiligentWeatherFog fog;
    fog.color = {static_cast<float>(0.45 * brightness), static_cast<float>(0.65 * brightness),
                 static_cast<float>(0.88 * brightness)};
    fog.density = visibilityMeters
                      ? static_cast<float>(3.912023005428146 / std::max(*visibilityMeters, 1.0))
                      : 0.0F;
    fog.layerHeight = layerHeightMeters ? static_cast<float>(*layerHeightMeters) : -1.0F;
    fog.cameraHeight = static_cast<float>(cameraHeightAboveGround);
    for (std::size_t component = 0; component < fog.worldUpView.size(); ++component) {
        const double value = view[component + 8];
        if (std::abs(value) > static_cast<double>(std::numeric_limits<float>::max())) {
            throw std::invalid_argument("Diligent weather-fog up vector exceeds float range");
        }
        fog.worldUpView[component] = static_cast<float>(value);
    }
    return fog;
}

DiligentSceneEnvironment buildDiligentSceneEnvironment(
    const Matrix4& view, std::optional<std::array<double, 3>> sunDirection,
    std::optional<double> weatherBrightness, bool reflectionPass, double cameraHeightAboveGround,
    std::optional<double> visibilityMeters, std::optional<double> layerHeightMeters,
    double reflectionAmbientFloor) {
    DiligentSceneEnvironment environment;
    environment.lighting = buildDiligentSceneLighting(view, sunDirection, weatherBrightness,
                                                      reflectionPass, reflectionAmbientFloor);
    environment.fog = buildDiligentWeatherFog(view, cameraHeightAboveGround, weatherBrightness,
                                              visibilityMeters, layerHeightMeters);
    environment.lighting.weatherFog = environment.fog;
    environment.clearColor = {environment.fog.color[0], environment.fog.color[1],
                              environment.fog.color[2], 1.0F};
    return environment;
}

} // namespace openbus::rendering
