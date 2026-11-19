#pragma once

#include "render/CameraMath.h"
#include "render/diligent/DiligentTexturedMesh.h"

#include <array>
#include <optional>

namespace openbus::rendering {

// Matches the production directional-light policy for the selected weather and view.
// The caller supplies the configured reflection ambient floor (0.25 by default).
[[nodiscard]] DiligentLightingParameters
buildDiligentSceneLighting(const Matrix4& view, std::optional<std::array<double, 3>> sunDirection,
                           std::optional<double> weatherBrightness, bool reflectionPass,
                           double reflectionAmbientFloor = 0.25);

// Builds the production exponential weather fog parameters for one view.
[[nodiscard]] DiligentWeatherFog buildDiligentWeatherFog(const Matrix4& view,
                                                         double cameraHeightAboveGround,
                                                         std::optional<double> weatherBrightness,
                                                         std::optional<double> visibilityMeters,
                                                         std::optional<double> layerHeightMeters);

struct DiligentSceneEnvironment {
    DiligentLightingParameters lighting{};
    DiligentWeatherFog fog{};
    std::array<float, 4> clearColor{0.45F, 0.65F, 0.88F, 1.0F};
};

// Builds coherent lighting, fog, and background inputs for one scene view.
[[nodiscard]] DiligentSceneEnvironment buildDiligentSceneEnvironment(
    const Matrix4& view, std::optional<std::array<double, 3>> sunDirection,
    std::optional<double> weatherBrightness, bool reflectionPass, double cameraHeightAboveGround,
    std::optional<double> visibilityMeters, std::optional<double> layerHeightMeters,
    double reflectionAmbientFloor = 0.25);

} // namespace openbus::rendering
