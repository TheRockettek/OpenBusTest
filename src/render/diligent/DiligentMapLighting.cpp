#include "render/diligent/DiligentMapLighting.h"

#include "render/MatrixMath.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace openbus::rendering {

std::vector<DiligentPointLight>
buildDiligentMapPointLights(std::span<const MapPointLight> activeLights, const Matrix4& view,
                            const std::array<double, 3>& cameraWorldPosition, double strength,
                            std::size_t maximumLights) {
    const auto isFinite = [](double value) { return std::isfinite(value); };
    if (!std::all_of(view.begin(), view.end(), isFinite) ||
        !std::all_of(cameraWorldPosition.begin(), cameraWorldPosition.end(), isFinite) ||
        !isFinite(strength) || strength < 0.0) {
        throw std::invalid_argument("Invalid Diligent map-light view parameters");
    }
    if (strength == 0.0 || maximumLights == 0 || activeLights.empty()) {
        return {};
    }
    const float intensity = static_cast<float>(strength);
    if (!std::isfinite(intensity)) {
        throw std::invalid_argument("Diligent map-light strength exceeds float range");
    }

    std::vector<std::pair<double, std::size_t>> nearest;
    nearest.reserve(activeLights.size());
    for (std::size_t index = 0; index < activeLights.size(); ++index) {
        const MapPointLight& light = activeLights[index];
        if (!std::all_of(light.position.begin(), light.position.end(), isFinite) ||
            !std::all_of(light.color.begin(), light.color.end(), isFinite) ||
            std::any_of(light.color.begin(), light.color.end(),
                        [](double value) { return value < 0.0; }) ||
            !isFinite(light.radius) || light.radius <= 0.0) {
            throw std::invalid_argument("Diligent map light contains invalid parameters");
        }
        const double dx = light.position[0] - cameraWorldPosition[0];
        const double dy = light.position[1] - cameraWorldPosition[1];
        const double dz = light.position[2] - cameraWorldPosition[2];
        const double distanceSquared = dx * dx + dy * dy + dz * dz;
        if (!isFinite(distanceSquared)) {
            throw std::invalid_argument("Diligent map-light distance is outside numeric range");
        }
        nearest.emplace_back(distanceSquared, index);
    }

    const std::size_t count = std::min(maximumLights, activeLights.size());
    if (count < nearest.size()) {
        std::partial_sort(nearest.begin(), nearest.begin() + count, nearest.end(),
                          [](const auto& lhs, const auto& rhs) {
                              return lhs.first < rhs.first ||
                                     (lhs.first == rhs.first && lhs.second < rhs.second);
                          });
    }

    std::vector<DiligentPointLight> result;
    result.reserve(count);
    for (std::size_t selected = 0; selected < count; ++selected) {
        const MapPointLight& source = activeLights[nearest[selected].second];
        DiligentPointLight light;
        for (std::size_t row = 0; row < 3; ++row) {
            const double positionView = view[row] * source.position[0] +
                                        view[row + 4] * source.position[1] +
                                        view[row + 8] * source.position[2] + view[row + 12];
            light.positionView[row] = static_cast<float>(positionView);
            light.color[row] = static_cast<float>(source.color[row]);
            if (!std::isfinite(light.positionView[row]) || !std::isfinite(light.color[row])) {
                throw std::invalid_argument("Diligent map light exceeds float range");
            }
        }
        light.radius = static_cast<float>(source.radius);
        light.intensity = intensity;
        if (!std::isfinite(light.radius) || light.radius <= 0.0F) {
            throw std::invalid_argument("Diligent map-light radius is outside float range");
        }
        result.push_back(light);
    }
    return result;
}

std::vector<DiligentSpotLight>
buildDiligentSpotLights(std::span<const DiligentWorldSpotLight> activeLights, const Matrix4& view,
                        const std::array<double, 3>& cameraWorldPosition,
                        std::size_t maximumLights) {
    const auto isFinite = [](double value) { return std::isfinite(value); };
    if (!std::all_of(view.begin(), view.end(), isFinite) ||
        !std::all_of(cameraWorldPosition.begin(), cameraWorldPosition.end(), isFinite)) {
        throw std::invalid_argument("Invalid Diligent spotlight view parameters");
    }

    std::vector<std::pair<double, std::size_t>> nearest;
    nearest.reserve(activeLights.size());
    for (std::size_t index = 0; index < activeLights.size(); ++index) {
        const DiligentWorldSpotLight& light = activeLights[index];
        if (!std::all_of(light.position.begin(), light.position.end(), isFinite) ||
            !std::all_of(light.direction.begin(), light.direction.end(), isFinite) ||
            !std::all_of(light.color.begin(), light.color.end(), isFinite) ||
            std::any_of(light.color.begin(), light.color.end(),
                        [](double value) { return value < 0.0; }) ||
            !isFinite(light.range) || light.range <= 0.0 || !isFinite(light.intensity) ||
            light.intensity < 0.0 || !isFinite(light.innerAngleDegrees) ||
            !isFinite(light.outerAngleDegrees) || light.innerAngleDegrees < 0.0 ||
            light.outerAngleDegrees < light.innerAngleDegrees || light.outerAngleDegrees > 180.0 ||
            std::hypot(light.direction[0], light.direction[1], light.direction[2]) < 1.0e-12) {
            throw std::invalid_argument("Diligent world spotlight contains invalid parameters");
        }

        const double dx = light.position[0] - cameraWorldPosition[0];
        const double dy = light.position[1] - cameraWorldPosition[1];
        const double dz = light.position[2] - cameraWorldPosition[2];
        const double distanceSquared = dx * dx + dy * dy + dz * dz;
        if (!isFinite(distanceSquared)) {
            throw std::invalid_argument("Diligent spotlight distance is outside numeric range");
        }
        nearest.emplace_back(distanceSquared, index);
    }

    const std::size_t count = std::min(maximumLights, activeLights.size());
    if (count < nearest.size()) {
        std::partial_sort(nearest.begin(), nearest.begin() + count, nearest.end(),
                          [](const auto& lhs, const auto& rhs) {
                              return lhs.first < rhs.first ||
                                     (lhs.first == rhs.first && lhs.second < rhs.second);
                          });
    }

    const auto toFloat = [](double value) {
        if (!std::isfinite(value) ||
            std::abs(value) > static_cast<double>(std::numeric_limits<float>::max())) {
            throw std::invalid_argument("Diligent spotlight exceeds float range");
        }
        return static_cast<float>(value);
    };
    std::vector<DiligentSpotLight> result;
    result.reserve(count);
    for (std::size_t selected = 0; selected < count; ++selected) {
        const DiligentWorldSpotLight& source = activeLights[nearest[selected].second];
        DiligentSpotLight light;
        for (std::size_t row = 0; row < 3; ++row) {
            const double positionView = view[row] * source.position[0] +
                                        view[row + 4] * source.position[1] +
                                        view[row + 8] * source.position[2] + view[row + 12];
            const double directionView = view[row] * source.direction[0] +
                                         view[row + 4] * source.direction[1] +
                                         view[row + 8] * source.direction[2];
            light.positionView[row] = toFloat(positionView);
            light.directionView[row] = toFloat(directionView);
            light.color[row] = toFloat(source.color[row]);
        }
        light.range = toFloat(source.range);
        light.intensity = toFloat(source.intensity);
        light.innerAngleDegrees = toFloat(source.innerAngleDegrees);
        light.outerAngleDegrees = toFloat(source.outerAngleDegrees);
        if (light.range <= 0.0F || std::hypot(light.directionView[0], light.directionView[1],
                                              light.directionView[2]) <= 0.000001F) {
            throw std::invalid_argument("Diligent spotlight is invalid in view space");
        }
        result.push_back(light);
    }
    return result;
}

std::optional<Matrix4>
buildDiligentSpotlightShadowClipFromView(const SpotlightShadowCamera& shadowCamera,
                                         const Matrix4& cameraView, float minimumNdcZ) {
    const auto isFinite = [](double value) { return std::isfinite(value); };
    if (!std::all_of(cameraView.begin(), cameraView.end(), isFinite) ||
        !std::all_of(shadowCamera.view.begin(), shadowCamera.view.end(), isFinite) ||
        !std::all_of(shadowCamera.projection.begin(), shadowCamera.projection.end(), isFinite) ||
        !isFinite(shadowCamera.nearPlane) || !isFinite(shadowCamera.farPlane) ||
        shadowCamera.nearPlane <= 0.0 || shadowCamera.farPlane <= shadowCamera.nearPlane ||
        !std::isfinite(minimumNdcZ) || minimumNdcZ < 0.0F || minimumNdcZ >= 1.0F) {
        throw std::invalid_argument("Invalid Diligent spotlight shadow view parameters");
    }

    Matrix4 worldFromView;
    if (!invertAffineMatrix(cameraView, worldFromView)) {
        return std::nullopt;
    }
    Matrix4 shadowClipFromView =
        multiplyMatrix4(multiplyMatrix4(shadowCamera.projection, shadowCamera.view), worldFromView);
    const double zScale = (1.0 - static_cast<double>(minimumNdcZ)) * 0.5;
    const double zBias = (1.0 + static_cast<double>(minimumNdcZ)) * 0.5;
    const Matrix4 clipAdjustment = {1.0, 0.0, 0.0,    0.0, 0.0, 1.0, 0.0,   0.0,
                                    0.0, 0.0, zScale, 0.0, 0.0, 0.0, zBias, 1.0};
    shadowClipFromView = multiplyMatrix4(clipAdjustment, shadowClipFromView);
    if (!std::all_of(shadowClipFromView.begin(), shadowClipFromView.end(), [&](double value) {
            return isFinite(value) &&
                   std::abs(value) <= static_cast<double>(std::numeric_limits<float>::max());
        })) {
        return std::nullopt;
    }
    return shadowClipFromView;
}

void applyDiligentSpotlightShadowBindings(std::span<DiligentSpotLight> lights,
                                          std::span<const DiligentSpotlightShadowBinding> bindings,
                                          const Matrix4& cameraView, float minimumNdcZ) {
    if (lights.size() != bindings.size()) {
        throw std::invalid_argument(
            "Diligent spotlight shadow bindings must match selected lights");
    }
    if (!std::all_of(cameraView.begin(), cameraView.end(),
                     [](double value) { return std::isfinite(value); }) ||
        !std::isfinite(minimumNdcZ) || minimumNdcZ < 0.0F || minimumNdcZ >= 1.0F) {
        throw std::invalid_argument("Invalid Diligent spotlight shadow binding view parameters");
    }
    for (const DiligentSpotlightShadowBinding& binding : bindings) {
        if (!std::isfinite(binding.bias) || binding.bias < 0.0F) {
            throw std::invalid_argument(
                "Diligent spotlight shadow bias must be finite and non-negative");
        }
    }

    std::vector<std::optional<Matrix4>> clipFromViews;
    clipFromViews.reserve(bindings.size());
    for (const DiligentSpotlightShadowBinding& binding : bindings) {
        clipFromViews.push_back(binding.camera ? buildDiligentSpotlightShadowClipFromView(
                                                     *binding.camera, cameraView, minimumNdcZ)
                                               : std::nullopt);
    }

    for (std::size_t index = 0; index < lights.size(); ++index) {
        DiligentSpotLight& light = lights[index];
        const DiligentSpotlightShadowBinding& binding = bindings[index];
        light.shadowValid = false;
        if (!clipFromViews[index]) {
            continue;
        }
        light.shadowClipFromView = *clipFromViews[index];
        light.shadowMapIndex = binding.layerIndex;
        light.shadowBias = binding.bias;
        light.shadowValid = true;
    }
}

} // namespace openbus::rendering
