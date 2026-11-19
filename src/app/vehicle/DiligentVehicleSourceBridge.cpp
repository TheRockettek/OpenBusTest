#include "app/vehicle/DiligentVehicleSourceBridge.h"
#include "app/vehicle/DiligentVehicleBridge.h"
#include "config/ScriptTextureLimits.h"
#include "render/InteriorLighting.h"
#include "render/MatrixMath.h"
#include "render/ReflectionTextureIndex.h"

#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <variant>

namespace openbus::app::vehicle {
namespace {

float toDiligentFloat(double value, const char* field) {
    if (!std::isfinite(value) ||
        std::abs(value) > static_cast<double>((std::numeric_limits<float>::max)())) {
        throw std::invalid_argument(std::string("Vehicle material ") + field +
                                    " exceeds Diligent float range");
    }
    return static_cast<float>(value);
}

bool hasBaseTexture(const openbus::rendering::DiligentVehicleTextureSource& texture) {
    return !std::holds_alternative<std::monostate>(texture);
}

struct TextureSnapshot {
    bool ready = true;
    openbus::rendering::DiligentVehicleTextureSource source;
};

TextureSnapshot snapshotTexture(const std::shared_ptr<Vehicle::TextureCacheEntry>& entry) {
    if (!entry || !entry->request) {
        return {false, {}};
    }

    const std::shared_ptr<Vehicle::TextureRequest>& request = entry->request;
    std::lock_guard<std::mutex> lock(request->mutex);
    if (!request->complete ||
        (request->fallbackDecodeStarted && !request->fallbackDecodeComplete)) {
        return {false, {}};
    }
    if (request->compressedTexture) {
        return {true, std::shared_ptr<const gli::texture>(request->compressedTexture)};
    }
    if (request->compressedDds) {
        return {true, std::shared_ptr<const CompressedDds>(request->compressedDds)};
    }
    if (request->image) {
        return {true, std::shared_ptr<const Image>(request->image)};
    }
    return {};
}

TextureSnapshot snapshotAuxiliary(const Vehicle::AuxiliaryTexture& auxiliary) {
    if (auxiliary.name.empty() ||
        openbus::scripting::scriptTextureReferenceIndex(auxiliary.name) >= 0) {
        return {};
    }
    return snapshotTexture(auxiliary.cacheEntry);
}

} // namespace

std::vector<openbus::rendering::DiligentVehicleGroupTransform>
buildDiligentVehicleViewGroupTransforms(std::span<const DiligentVehicleViewGroupInput> groups,
                                        const openbus::rendering::Matrix4& modelView,
                                        const openbus::rendering::Matrix4& projection,
                                        openbus::rendering::ViewpointContext context,
                                        int activeLod) {
    using namespace openbus::rendering;
    const auto matrixIsFinite = [](const Matrix4& matrix) {
        return std::all_of(matrix.begin(), matrix.end(),
                           [](double value) { return std::isfinite(value); });
    };
    if (!matrixIsFinite(modelView) || !matrixIsFinite(projection)) {
        throw std::invalid_argument("Diligent auxiliary vehicle view matrices must be finite");
    }

    const ViewFrustum frustum = buildViewFrustum(projection);
    std::vector<DiligentVehicleGroupTransform> transforms;
    transforms.reserve(groups.size());
    std::unordered_set<std::size_t> groupIds;
    groupIds.reserve(groups.size());
    for (const DiligentVehicleViewGroupInput& group : groups) {
        if (!groupIds.insert(group.animationGroupIndex).second) {
            throw std::invalid_argument(
                "Diligent auxiliary vehicle group identifiers must be unique");
        }
        if (!matrixIsFinite(group.localAnimationTransform) || !std::isfinite(group.localRadius) ||
            group.localRadius < 0.0 ||
            !std::all_of(
                group.localCenter.begin(), group.localCenter.end(),
                [](double value) { return std::isfinite(value); })) {
            throw std::invalid_argument(
                "Diligent auxiliary vehicle bounds or transform are invalid");
        }

        const Matrix4 groupModelView = multiplyMatrix4(modelView, group.localAnimationTransform);
        const std::array<double, 4> centerView =
            transformPoint(groupModelView,
                           {group.localCenter[0], group.localCenter[1], group.localCenter[2], 1.0});
        const double centerW = centerView[3];
        if (!matrixIsFinite(groupModelView) ||
            !std::all_of(centerView.begin(), centerView.end(),
                         [](double value) { return std::isfinite(value); }) ||
            std::abs(centerW) < 1.0e-12) {
            throw std::invalid_argument("Diligent auxiliary vehicle center is not finite");
        }
        const double scaleX = std::hypot(groupModelView[0], groupModelView[1], groupModelView[2]);
        const double scaleY = std::hypot(groupModelView[4], groupModelView[5], groupModelView[6]);
        const double scaleZ = std::hypot(groupModelView[8], groupModelView[9], groupModelView[10]);
        const double radius = group.localRadius * std::max({scaleX, scaleY, scaleZ});
        if (!std::isfinite(radius)) {
            throw std::invalid_argument("Diligent auxiliary vehicle radius is not finite");
        }
        const std::array<double, 4> viewCenter = {centerView[0] / centerW, centerView[1] / centerW,
                                                  centerView[2] / centerW, 1.0};
        const Matrix4 modelViewProjection = multiplyMatrix4(projection, groupModelView);
        if (!std::all_of(viewCenter.begin(), viewCenter.end(),
                         [](double value) { return std::isfinite(value); }) ||
            !matrixIsFinite(modelViewProjection)) {
            throw std::invalid_argument("Diligent auxiliary vehicle clip transform is not finite");
        }

        DiligentVehicleGroupTransform transform;
        transform.animationGroupIndex = group.animationGroupIndex;
        transform.modelView = groupModelView;
        transform.modelViewProjection = modelViewProjection;
        transform.visible = group.variableVisible &&
                            viewpointMatches(group.viewpointMask, context) &&
                            (activeLod < 0 || group.lodIndex < 0 || group.lodIndex == activeLod) &&
                            !sphereOutsideFrustum(frustum, viewCenter, radius);
        transforms.push_back(std::move(transform));
    }
    return transforms;
}

openbus::rendering::DiligentMaterialState
makeDiligentVehicleMaterialState(const Vehicle::Batch& batch, const Variables& variables,
                                 double alpha, double visibilityFade) {
    if (!std::isfinite(alpha) || alpha < 0.0 || alpha > 1.0 || !std::isfinite(visibilityFade) ||
        visibilityFade < 0.0 || visibilityFade > 1.0) {
        throw std::invalid_argument(
            "Vehicle Diligent alpha and visibility fade must be finite values in [0, 1]");
    }

    openbus::rendering::DiligentMaterialState material;
    material.alphaMode = toDiligentVehicleAlphaMode(batch.alphaMode);
    if (visibilityFade < 1.0) {
        material.alphaMode = openbus::rendering::DiligentAlphaMode::Blend;
    }
    material.baseColor = {
        toDiligentFloat(batch.color[0], "color"),
        toDiligentFloat(batch.color[1], "color"),
        toDiligentFloat(batch.color[2], "color"),
        toDiligentFloat(alpha, "alpha"),
    };
    for (std::size_t channel = 0; channel < 3; ++channel) {
        material.specularColor[channel] =
            toDiligentFloat(batch.specularColor[channel], "specular color");
        material.emissionColor[channel] =
            toDiligentFloat(batch.emissionColor[channel], "emission color");
    }
    material.specularPower = toDiligentFloat(batch.specularPower, "specular power");
    material.textureAlphaOnly = batch.textureAlphaOnly;
    material.normalMapStrength =
        toDiligentFloat(std::clamp(batch.bumpmapStrength, 0.0, 1.0), "normal strength");
    material.environmentStrength =
        toDiligentFloat(batch.environmentStrength, "environment strength");
    material.addressU = batch.textureAddressS;
    material.addressV = batch.textureAddressT;
    material.blendEnabled = batch.noZwrite;
    material.depthWrite =
        !batch.noZwrite && alpha > 0.0 && visibilityFade >= 1.0 && batch.alphaMode != 2;
    material.depthMode = batch.noZcheck ? openbus::rendering::DiligentDepthMode::Disabled
                                        : ((batch.alphaMode == 2 || visibilityFade < 1.0)
                                               ? openbus::rendering::DiligentDepthMode::LessEqual
                                               : openbus::rendering::DiligentDepthMode::Less);
    material.visibilityFade = toDiligentFloat(visibilityFade, "visibility fade");
    const double lightmapStrength =
        batch.lightmapStrengthVariable.empty()
            ? 1.0
            : std::clamp(static_cast<double>(variables.get(batch.lightmapStrengthVariable)), 0.0,
                         1.0);
    material.lightMapStrength = toDiligentFloat(lightmapStrength, "lightmap strength");
    const double nightlight =
        batch.selectedMaterialItem || batch.visibilitySwitched
            ? 1.0
            : std::max(static_cast<double>(variables.get("nightlighta")),
                       1.0 - static_cast<double>(variables.get("envir_brightness")));
    material.nightMapStrength =
        toDiligentFloat(std::clamp(nightlight, 0.0, 1.0), "nightmap strength");
    material.textureOffset = {
        toDiligentFloat(batch.texcoordTransXVariable.empty()
                            ? 0.0
                            : variables.get(batch.texcoordTransXVariable),
                        "texture U offset"),
        toDiligentFloat(batch.texcoordTransYVariable.empty()
                            ? 0.0
                            : variables.get(batch.texcoordTransYVariable),
                        "texture V offset"),
    };
    return material;
}

DiligentVehicleMaterialOverrideSet buildDiligentVehicleMaterialOverrides(
    std::span<const Vehicle::DisplayPart> displayParts, const Variables& variables,
    std::span<const openbus::rendering::DiligentVehicleMaterialOverride> runtimeTextures) {
    using BatchKey = std::pair<std::size_t, std::size_t>;
    std::map<BatchKey, const openbus::rendering::DiligentVehicleMaterialOverride*>
        runtimeTexturesByBatch;
    for (const auto& runtimeTexture : runtimeTextures) {
        if (!runtimeTexture.animationGroupIndex.has_value() ||
            !runtimeTexture.sourceBatchIndex.has_value() || runtimeTexture.material != nullptr ||
            runtimeTexture.baseTexture != nullptr || runtimeTexture.runtimeTexture != nullptr) {
            throw std::invalid_argument("Vehicle runtime texture overrides must target a batch and "
                                        "contain texture maps only");
        }
        if (!runtimeTexturesByBatch
                 .emplace(BatchKey{*runtimeTexture.animationGroupIndex,
                                   *runtimeTexture.sourceBatchIndex},
                          &runtimeTexture)
                 .second) {
            throw std::invalid_argument("Vehicle runtime texture override targets a batch twice");
        }
    }

    DiligentVehicleMaterialOverrideSet result;
    std::size_t batchCount = 0;
    for (const Vehicle::DisplayPart& part : displayParts) {
        if (part.batches.size() > (std::numeric_limits<std::size_t>::max)() - batchCount) {
            throw std::length_error("Vehicle material override count exceeds addressable size");
        }
        batchCount += part.batches.size();
    }
    result.materials.reserve(batchCount);
    result.overrides.reserve(batchCount);

    std::size_t runtimeTextureCount = 0;
    for (std::size_t displayPartIndex = 0; displayPartIndex < displayParts.size();
         ++displayPartIndex) {
        const Vehicle::DisplayPart& part = displayParts[displayPartIndex];
        for (std::size_t batchIndex = 0; batchIndex < part.batches.size(); ++batchIndex) {
            const Vehicle::Batch& batch = part.batches[batchIndex];
            const double alpha =
                batch.alphaScaleVariable.empty()
                    ? 1.0
                    : std::clamp(static_cast<double>(variables.get(batch.alphaScaleVariable)), 0.0,
                                 1.0);
            result.materials.push_back(
                makeDiligentVehicleMaterialState(batch, variables, alpha, part.visibilityFade));
            openbus::rendering::DiligentVehicleMaterialOverride override;
            override.materialIndex = batch.baseMaterial.materialIndex;
            override.animationGroupIndex = displayPartIndex;
            override.sourceBatchIndex = batchIndex;
            override.material = &result.materials.back();
            const auto runtimeTexture = runtimeTexturesByBatch.find({displayPartIndex, batchIndex});
            if (runtimeTexture != runtimeTexturesByBatch.end()) {
                override.freeMapTexture = runtimeTexture->second->freeMapTexture;
                override.textMapTexture = runtimeTexture->second->textMapTexture;
                ++runtimeTextureCount;
            }
            result.overrides.push_back(std::move(override));
        }
    }
    if (runtimeTextureCount != runtimeTexturesByBatch.size()) {
        throw std::invalid_argument(
            "Vehicle runtime texture override references a missing display-part batch");
    }
    return result;
}

openbus::rendering::DiligentVehicleBatchSource
makeDiligentVehicleBatchSource(const Vehicle::DisplayPart& part, std::size_t displayPartIndex,
                               std::size_t batchIndex,
                               const DiligentVehicleBatchTextures& textures) {
    if (batchIndex >= part.batches.size()) {
        throw std::out_of_range("Vehicle Diligent batch references a missing material batch");
    }
    const Vehicle::Batch& batch = part.batches[batchIndex];
    if (batch.vertices.empty() || batch.vertices.size() % 3 != 0) {
        throw std::invalid_argument("Vehicle batch geometry must contain complete triangles");
    }
    const bool hasArrayLayers = !textures.textureArrayLayers.empty();
    const bool hasCompressedArray = static_cast<bool>(textures.compressedTextureArray);
    const bool hasBase = hasBaseTexture(textures.base);
    if ((hasArrayLayers && hasCompressedArray) ||
        (hasBase && (hasArrayLayers || hasCompressedArray))) {
        throw std::invalid_argument("Vehicle batch has conflicting Diligent texture sources");
    }
    const bool needsArray =
        batch.textureArray || textures.baseIsTextureArray || !batch.materialTextures.empty();
    if (needsArray && !hasArrayLayers && !hasCompressedArray) {
        throw std::invalid_argument("Vehicle batch texture array source is not available");
    }
    if (batch.textured && !needsArray && !hasBase) {
        throw std::invalid_argument(
            "Textured vehicle batch requires a Diligent base texture source");
    }
    if (!needsArray && (hasArrayLayers || hasCompressedArray)) {
        throw std::invalid_argument(
            "Vehicle batch array textures are not configured for this batch");
    }

    openbus::rendering::DiligentVehicleBatchSource source;
    source.materialIndex = batch.baseMaterial.materialIndex;
    source.sourceBatchIndex = batchIndex;
    source.vertices = {batch.vertices.data(), batch.vertices.size()};
    source.texture = textures.base;
    source.textureArrayLayers = textures.textureArrayLayers;
    source.compressedTextureArray = textures.compressedTextureArray;
    source.auxiliaryTextures = textures.auxiliary;
    source.animationGroupIndex = displayPartIndex;

    const Variables defaultVariables;
    source.material = makeDiligentVehicleMaterialState(batch, defaultVariables, 1.0);
    source.material.cullBackFaces = part.backFaceCulling;
    return source;
}

void populateDiligentVehicleInteriorLights(
    openbus::rendering::DiligentMaterialState& material, const Vehicle::Batch& batch,
    std::span<const ModelInteriorLight> lights,
    std::span<const Vehicle::InteriorLightController> controllers, const Variables& variables,
    const openbus::rendering::Matrix4& vehicleRootModelView) {
    material.interiorLightCount = 0;
    material.interiorLightPositionsView = {};
    material.interiorLightColors = {};
    material.interiorLightStrengths = {};
    material.useInteriorLights = false;
    for (std::size_t lightIndex = 0; lightIndex < lights.size(); ++lightIndex) {
        if (lightIndex >= controllers.size() ||
            std::find(batch.interiorLightIndexes.begin(), batch.interiorLightIndexes.end(),
                      static_cast<int>(lightIndex)) == batch.interiorLightIndexes.end() ||
            material.interiorLightCount >= openbus::rendering::DILIGENT_MAX_INTERIOR_LIGHTS) {
            continue;
        }
        const ModelInteriorLight& light = lights[lightIndex];
        const Vehicle::InteriorLightController& controller = controllers[lightIndex];
        const double brightness = std::max(
            0.0, controller.isNumeric ? controller.numericValue : variables.get(light.controller));
        const double strength = brightness * std::max(0.0, light.intensity);
        if (strength <= 0.0) {
            continue;
        }
        const std::size_t outputIndex = material.interiorLightCount++;
        const auto viewPosition = openbus::rendering::interiorLightPositionInViewSpace(
            vehicleRootModelView, light.position);
        for (std::size_t channel = 0; channel < 3; ++channel) {
            material.interiorLightPositionsView[outputIndex][channel] =
                toDiligentFloat(viewPosition[channel], "interior light position");
            material.interiorLightColors[outputIndex][channel] = toDiligentFloat(
                std::clamp(light.color[channel] / 255.0, 0.0, 1.0), "interior light color");
        }
        material.interiorLightStrengths[outputIndex] =
            toDiligentFloat(strength, "interior light strength");
    }
    material.useInteriorLights = material.interiorLightCount > 0;
}

DiligentVehicleMaterialOverrideSet buildDiligentVehicleMaterialOverrides(
    Vehicle& vehicle, const openbus::rendering::Matrix4& vehicleRootModelView,
    std::span<const openbus::rendering::DiligentVehicleMaterialOverride> runtimeTextures) {
    DiligentVehicleMaterialOverrideSet result = buildDiligentVehicleMaterialOverrides(
        vehicle.displayLists, vehicle.variables, runtimeTextures);
    std::size_t materialIndex = 0;
    for (const Vehicle::DisplayPart& part : vehicle.displayLists) {
        for (const Vehicle::Batch& batch : part.batches) {
            openbus::rendering::DiligentMaterialState& material =
                result.materials.at(materialIndex++);
            populateDiligentVehicleInteriorLights(material, batch, vehicle.interiorLights,
                                                  vehicle.interiorLightControllers,
                                                  vehicle.variables, vehicleRootModelView);
        }
    }
    return result;
}

void bindDiligentVehicleReflectionTextures(
    std::span<const Vehicle::DisplayPart> displayParts,
    DiligentVehicleMaterialOverrideSet& overrides,
    std::span<Diligent::ITextureView* const> reflectionTextures) {
    if (overrides.materials.size() != overrides.overrides.size()) {
        throw std::invalid_argument(
            "Vehicle reflection texture binding requires aligned material overrides");
    }

    std::size_t overrideIndex = 0;
    for (std::size_t displayPartIndex = 0; displayPartIndex < displayParts.size();
         ++displayPartIndex) {
        const Vehicle::DisplayPart& part = displayParts[displayPartIndex];
        for (std::size_t batchIndex = 0; batchIndex < part.batches.size(); ++batchIndex) {
            if (overrideIndex >= overrides.overrides.size()) {
                throw std::invalid_argument(
                    "Vehicle reflection texture binding is missing a batch override");
            }
            auto& override = overrides.overrides[overrideIndex];
            auto& material = overrides.materials[overrideIndex];
            if (override.animationGroupIndex != displayPartIndex ||
                override.sourceBatchIndex != batchIndex) {
                throw std::invalid_argument(
                    "Vehicle reflection texture binding has mismatched batch identities");
            }

            const Vehicle::Batch& batch = part.batches[batchIndex];
            const auto bindReflectionTexture = [&](const std::string& textureName,
                                                   std::size_t materialLayer) {
                if (textureName.empty()) {
                    return;
                }
                const int reflectionIndex = openbus::rendering::reflectionTextureIndex(textureName);
                if (reflectionIndex < 0 ||
                    static_cast<std::size_t>(reflectionIndex) >= reflectionTextures.size()) {
                    return;
                }
                Diligent::ITextureView* reflectionTexture =
                    reflectionTextures[static_cast<std::size_t>(reflectionIndex)];
                if (reflectionTexture == nullptr) {
                    return;
                }
                if (override.baseTexture != nullptr) {
                    throw std::invalid_argument(
                        "Vehicle reflection texture conflicts with a base texture override");
                }
                if (materialLayer == 0 && batch.materialTextures.size() <= 1) {
                    override.baseTexture = reflectionTexture;
                    material.flipTextureY = true;
                    return;
                }
                if (materialLayer >= material.arrayReflectionTextures.size()) {
                    throw std::invalid_argument(
                        "Vehicle material-array reflection exceeds supported layer count");
                }
                material.arrayReflectionTextures[materialLayer] = reflectionTexture;
            };
            if (batch.materialTextures.empty()) {
                bindReflectionTexture(
                    batch.textureName.empty() ? batch.texturePath.string() : batch.textureName, 0);
            } else {
                for (std::size_t layer = 0; layer < batch.materialTextures.size(); ++layer) {
                    bindReflectionTexture(batch.materialTextures[layer].textureName, layer);
                }
            }
            ++overrideIndex;
        }
    }
    if (overrideIndex != overrides.overrides.size()) {
        throw std::invalid_argument("Vehicle reflection texture binding has extra batch overrides");
    }
}

bool makeDiligentVehicleBatchSources(const Vehicle::DisplayPart& part, std::size_t displayPartIndex,
                                     DiligentVehicleBatchSourceSet& result) {
    DiligentVehicleBatchSourceSet prepared;
    prepared.textureArrayLayerStorage.reserve(part.batches.size());
    prepared.batches.reserve(part.batches.size());

    for (std::size_t batchIndex = 0; batchIndex < part.batches.size(); ++batchIndex) {
        const Vehicle::Batch& batch = part.batches[batchIndex];
        prepared.textureArrayLayerStorage.emplace_back();
        std::vector<std::shared_ptr<const Image>>& arrayLayers =
            prepared.textureArrayLayerStorage.back();
        DiligentVehicleBatchTextures textures;

        if (batch.materialTextures.empty()) {
            if (batch.textureCacheEntry) {
                TextureSnapshot snapshot = snapshotTexture(batch.textureCacheEntry);
                if (!snapshot.ready) {
                    result = {};
                    return false;
                }
                if (batch.textureArray || [&] {
                        const auto* compressed =
                            std::get_if<std::shared_ptr<const gli::texture>>(&snapshot.source);
                        return compressed && *compressed && (*compressed)->layers() > 1;
                    }()) {
                    if (const auto* compressed =
                            std::get_if<std::shared_ptr<const gli::texture>>(&snapshot.source)) {
                        if (*compressed && (*compressed)->layers() != batch.textureArrayLayers) {
                            if (batch.textureArray) {
                                throw std::invalid_argument("Vehicle texture-array layer count "
                                                            "does not match its decoded source");
                            }
                        }
                        textures.compressedTextureArray = *compressed;
                        textures.baseIsTextureArray = true;
                    } else if (const auto* image =
                                   std::get_if<std::shared_ptr<const Image>>(&snapshot.source)) {
                        if (batch.textureArrayLayers != 1) {
                            throw std::invalid_argument(
                                "Decoded vehicle texture array has no source for every layer");
                        }
                        arrayLayers.push_back(*image);
                    } else if (hasBaseTexture(snapshot.source)) {
                        throw std::invalid_argument(
                            "Vehicle texture-array source format is unsupported");
                    }
                } else {
                    textures.base = std::move(snapshot.source);
                }
            } else if (batch.textured || batch.textureArray || !batch.textureName.empty() ||
                       !batch.texturePath.empty()) {
                result = {};
                return false;
            }
        } else {
            if (batch.textureArray) {
                throw std::invalid_argument(
                    "Vehicle material textures and a base texture array cannot be combined");
            }
            for (const Vehicle::MaterialTextureSource& materialTexture : batch.materialTextures) {
                TextureSnapshot snapshot = snapshotTexture(materialTexture.textureCacheEntry);
                if (!snapshot.ready) {
                    result = {};
                    return false;
                }
                if (const auto* image =
                        std::get_if<std::shared_ptr<const Image>>(&snapshot.source)) {
                    arrayLayers.push_back(*image);
                } else if (const auto* compressed =
                               std::get_if<std::shared_ptr<const gli::texture>>(&snapshot.source);
                           compressed && *compressed && batch.materialTextures.size() == 1 &&
                           (*compressed)->layers() > 1) {
                    textures.compressedTextureArray = *compressed;
                } else if (hasBaseTexture(snapshot.source)) {
                    throw std::invalid_argument("Vehicle material texture arrays require decoded "
                                                "layers or one compressed array");
                } else {
                    throw std::invalid_argument(
                        "Vehicle material texture source did not decode to a usable image");
                }
            }
        }

        if (batch.lightmap.cacheEntry || !batch.lightmap.name.empty()) {
            TextureSnapshot snapshot = snapshotAuxiliary(batch.lightmap);
            if (!snapshot.ready) {
                result = {};
                return false;
            }
            textures.auxiliary.lightMap = std::move(snapshot.source);
        }
        if (batch.nightmap.cacheEntry || !batch.nightmap.name.empty()) {
            TextureSnapshot snapshot = snapshotAuxiliary(batch.nightmap);
            if (!snapshot.ready) {
                result = {};
                return false;
            }
            textures.auxiliary.nightMap = std::move(snapshot.source);
        }
        if (batch.transmap.cacheEntry || !batch.transmap.name.empty()) {
            TextureSnapshot snapshot = snapshotAuxiliary(batch.transmap);
            if (!snapshot.ready) {
                result = {};
                return false;
            }
            textures.auxiliary.transMap = std::move(snapshot.source);
        }
        if (batch.bumpmap.cacheEntry || !batch.bumpmap.name.empty()) {
            TextureSnapshot snapshot = snapshotAuxiliary(batch.bumpmap);
            if (!snapshot.ready) {
                result = {};
                return false;
            }
            textures.auxiliary.normalMap = std::move(snapshot.source);
        }
        if (batch.environmentStrength > 0.0 && !batch.environmentTextureName.empty()) {
            TextureSnapshot snapshot = snapshotTexture(batch.environmentTextureCacheEntry);
            if (!snapshot.ready) {
                result = {};
                return false;
            }
            textures.auxiliary.environmentMap = std::move(snapshot.source);
        }
        if (batch.freeTextureAsset.cacheEntry) {
            TextureSnapshot snapshot = snapshotTexture(batch.freeTextureAsset.cacheEntry);
            if (!snapshot.ready) {
                result = {};
                return false;
            }
            textures.auxiliary.freeMap = std::move(snapshot.source);
        }

        textures.textureArrayLayers = arrayLayers;
        prepared.batches.push_back(
            makeDiligentVehicleBatchSource(part, displayPartIndex, batchIndex, textures));
    }

    result = std::move(prepared);
    return true;
}

bool makeDiligentVehicleBatchSources(std::span<const Vehicle::DisplayPart> displayParts,
                                     DiligentVehicleBatchSourceSet& result) {
    DiligentVehicleBatchSourceSet prepared;
    for (std::size_t displayPartIndex = 0; displayPartIndex < displayParts.size();
         ++displayPartIndex) {
        DiligentVehicleBatchSourceSet partSources;
        if (!makeDiligentVehicleBatchSources(displayParts[displayPartIndex], displayPartIndex,
                                             partSources)) {
            result = {};
            return false;
        }
        for (auto& layers : partSources.textureArrayLayerStorage) {
            prepared.textureArrayLayerStorage.push_back(std::move(layers));
        }
        for (auto& batch : partSources.batches) {
            prepared.batches.push_back(std::move(batch));
        }
    }

    result = std::move(prepared);
    return true;
}

} // namespace openbus::app::vehicle
