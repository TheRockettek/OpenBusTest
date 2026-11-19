#pragma once

#include "app/DiligentCameraBridge.h"
#include "core/Variables.h"
#include "render/diligent/DiligentSceneLighting.h"

#include <stdexcept>

namespace openbus::app {

[[nodiscard]] inline openbus::rendering::DiligentSceneEnvironment buildDiligentSceneEnvironment(
    const DiligentCameraView& camera, const SystemVariables& weatherVariables,
    double cameraHeightAboveGround, bool reflectionPass, double reflectionAmbientFloor = 0.25) {
    if (camera.viewportWidth <= 0 || camera.viewportHeight <= 0) {
        throw std::invalid_argument("Diligent scene environment requires a valid viewport");
    }
    return openbus::rendering::buildDiligentSceneEnvironment(
        camera.view, weatherVariables.sunDirection(), weatherVariables.environmentBrightness(),
        reflectionPass, cameraHeightAboveGround,
        weatherVariables.effectiveWeatherVisibilityMeters(),
        weatherVariables.weatherFogTopAboveGroundMeters(), reflectionAmbientFloor);
}

} // namespace openbus::app
