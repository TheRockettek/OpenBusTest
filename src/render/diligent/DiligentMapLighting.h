#pragma once

#include "render/CameraMath.h"
#include "render/MapLighting.h"
#include "render/SpotlightShadowCamera.h"
#include "render/diligent/DiligentTexturedMesh.h"

#include <array>
#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace openbus::rendering {

struct DiligentSpotlightShadowBinding {
    std::optional<SpotlightShadowCamera> camera;
    Diligent::Uint32 layerIndex = 0;
    float bias = 0.001F;
};

// Selects the nearest active map lights and converts their world-space data
// to the view-space representation consumed by DiligentTexturedMeshRenderer.
std::vector<DiligentPointLight>
buildDiligentMapPointLights(std::span<const MapPointLight> activeLights, const Matrix4& view,
                            const std::array<double, 3>& cameraWorldPosition, double strength,
                            std::size_t maximumLights);

// Selects the nearest world-space spot lights and converts them to the view-space
// representation consumed by DiligentTexturedMeshRenderer.
std::vector<DiligentSpotLight>
buildDiligentSpotLights(std::span<const DiligentWorldSpotLight> activeLights, const Matrix4& view,
                        const std::array<double, 3>& cameraWorldPosition,
                        std::size_t maximumLights);

// Converts an existing world-space shadow camera into Diligent clip-from-view space.
// Returns no matrix when the active camera view is not invertible.
[[nodiscard]] std::optional<Matrix4>
buildDiligentSpotlightShadowClipFromView(const SpotlightShadowCamera& shadowCamera,
                                         const Matrix4& cameraView, float minimumNdcZ);

// Applies scheduler-owned shadow cameras/layers to already selected lights. Bindings
// must have the same ordering as lights; a missing camera disables shadowing for that light.
void applyDiligentSpotlightShadowBindings(std::span<DiligentSpotLight> lights,
                                          std::span<const DiligentSpotlightShadowBinding> bindings,
                                          const Matrix4& cameraView, float minimumNdcZ);

} // namespace openbus::rendering
