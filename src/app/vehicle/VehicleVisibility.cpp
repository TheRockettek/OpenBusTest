// Per-frame part visibility, LOD and reflection texture requirements.

#include "app/vehicle/Vehicle.h"

const std::vector<int>& Vehicle::reflectionTextureIndicesForPart(const DisplayPart& part) const {
    return part.reflectionTextureIndices;
}

void Vehicle::cacheReflectionTextureIndices(DisplayPart& part) const {
    std::unordered_set<int> uniqueReflectionIndices;
    for (const Batch& batch : part.batches) {
        const auto registerReflection = [&](const std::string& textureName) {
            const int reflectionIndex = openbus::rendering::reflectionTextureIndex(textureName);
            if (reflectionIndex >= 0) {
                uniqueReflectionIndices.insert(reflectionIndex);
            }
        };
        registerReflection(batch.baseTextureName);
        registerReflection(batch.textureName);
        for (const MaterialTextureSource& source : batch.materialTextures) {
            registerReflection(source.textureName);
        }
        for (const MaterialState::TextureChange& change : batch.textureChanges) {
            registerReflection(change.textureName);
        }
    }
    part.reflectionTextureIndices.assign(uniqueReflectionIndices.begin(),
                                         uniqueReflectionIndices.end());
}

std::vector<std::uint32_t> Vehicle::computePartVisibility(bool selectLod) const {
    using namespace openbus::rendering;
    std::vector<std::uint32_t> result;
    if (!computeCullingAvailable())
        return result;
    std::vector<CullingItem> items;
    std::vector<float> boundaries;
    appendCullingLodBoundaries(boundaries, lodThresholds);
    items.reserve(displayLists.size());
    for (const DisplayPart& part : displayLists) {
        auto item = makeCullingItem(modelViewMatrix(), animatedPartCenter(part), part.radius);
        item.metadata[0] = frustumCulling ? CullFrustum : 0U;
        if (selectLod && part.lodIndex >= 0 && !boundaries.empty()) {
            item.metadata[0] |= CullLod;
            item.metadata[1] = static_cast<std::uint32_t>(part.lodIndex);
            item.metadata[3] = static_cast<std::uint32_t>(boundaries.size());
        }
        items.push_back(item);
    }
    computeCull(items, makeCullingPlanes(projectionMatrix()), boundaries, result);
    return result;
}

void Vehicle::prepareFrameVisibility(RenderViewContext context, int viewportWidth,
                                     int viewportHeight) {
    TraceScope trace("render", "Vehicle::prepareFrameVisibility");
    ++materialSelectionGeneration;
    {
        TraceScope phase("render", "Vehicle::prepareFrameVisibility.reset");
        variableVisibleParts.resize(displayLists.size());
        visibleClickablePartIndices.clear();
        visibleOpaquePickPartIndices.clear();
        visibleReflectionTextureIndices.clear();
        reflectionRequiredSizes.clear();
    }
    const auto& modelView = openbus::rendering::modelViewMatrix();
    const auto& projection = openbus::rendering::projectionMatrix();
    const double viewportWidthPixels = static_cast<double>(std::max(viewportWidth, 1));
    const double viewportHeightPixels = static_cast<double>(std::max(viewportHeight, 1));
    ViewFrustum frustum;
    {
        TraceScope phase("render", "Vehicle::prepareFrameVisibility.frustum");
        if (frustumCulling) {
            frustum = buildViewFrustum(projection);
        }
    }
    {
        TraceScope phase("render", "Vehicle::prepareFrameVisibility.scanParts");
        for (std::size_t partIndex = 0; partIndex < displayLists.size(); ++partIndex) {
            const DisplayPart& part = displayLists[partIndex];
            const bool visible =
                part.visibleVariable.empty() ||
                variables.get(part.visibleVariable) == static_cast<double>(part.visibleValue);
            variableVisibleParts[partIndex] = visible;
            if (!visible || !openbus::rendering::viewpointMatches(part.viewpoint, context)) {
                continue;
            }
            const bool lodMatches =
                activeLod < 0 || part.lodIndex < 0 || part.lodIndex == activeLod;
            if (lodMatches && !part.mouseEvent.empty()) {
                visibleClickablePartIndices.push_back(partIndex);
            }
            if (lodMatches && part.pickOccluder) {
                visibleOpaquePickPartIndices.push_back(partIndex);
            }
            const std::vector<int>& partReflectionIndices = reflectionTextureIndicesForPart(part);
            if (partReflectionIndices.empty()) {
                continue;
            }
            const std::array<double, 3> center = animatedPartCenter(part);
            const std::array<double, 4> local = {center[0], center[1], center[2], 1.0};
            const std::array<double, 4> eye = transformPoint(modelView, local);
            if (frustumCulling && sphereOutsideFrustum(frustum, eye, part.radius)) {
                continue;
            }
            const Matrix4 animation = animationTransformForPart(part);
            const std::array<double, 3> halfSize = {std::max(part.size[0] * 0.5, 0.0),
                                                    std::max(part.size[1] * 0.5, 0.0),
                                                    std::max(part.size[2] * 0.5, 0.0)};
            double minimumNdcX = std::numeric_limits<double>::max();
            double maximumNdcX = std::numeric_limits<double>::lowest();
            double minimumNdcY = std::numeric_limits<double>::max();
            double maximumNdcY = std::numeric_limits<double>::lowest();
            bool hasProjectedCorner = false;
            bool intersectsNearPlane = false;

            for (int corner = 0; corner < 8; ++corner) {
                const std::array<double, 4> cornerLocal = {
                    part.center[0] + ((corner & 1) == 0 ? -halfSize[0] : halfSize[0]),
                    part.center[1] + ((corner & 2) == 0 ? -halfSize[1] : halfSize[1]),
                    part.center[2] + ((corner & 4) == 0 ? -halfSize[2] : halfSize[2]), 1.0};
                std::array<double, 4> animated = transformPoint(animation, cornerLocal);
                animated[2] += modelOffsetZ;
                const std::array<double, 4> cornerEye = transformPoint(modelView, animated);
                const double depth = -cornerEye[2];
                if (depth <= openbus::rendering::kReflectionNearPlane) {
                    intersectsNearPlane = true;
                    continue;
                }
                const double clipX = projection[0] * cornerEye[0] + projection[4] * cornerEye[1] +
                                     projection[8] * cornerEye[2] + projection[12] * cornerEye[3];
                const double clipY = projection[1] * cornerEye[0] + projection[5] * cornerEye[1] +
                                     projection[9] * cornerEye[2] + projection[13] * cornerEye[3];
                minimumNdcX = std::min(minimumNdcX, clipX / depth);
                maximumNdcX = std::max(maximumNdcX, clipX / depth);
                minimumNdcY = std::min(minimumNdcY, clipY / depth);
                maximumNdcY = std::max(maximumNdcY, clipY / depth);
                hasProjectedCorner = true;
            }
            if (!hasProjectedCorner) {
                continue;
            }
            const double projectedWidth =
                intersectsNearPlane
                    ? viewportWidthPixels
                    : std::clamp((maximumNdcX - minimumNdcX) * 0.5 * viewportWidthPixels, 0.0,
                                 viewportWidthPixels);
            const double projectedHeight =
                intersectsNearPlane
                    ? viewportHeightPixels
                    : std::clamp((maximumNdcY - minimumNdcY) * 0.5 * viewportHeightPixels, 0.0,
                                 viewportHeightPixels);
            const double screenBoundedDiameter = std::max(projectedWidth, projectedHeight);
            const int requiredSize = std::max(openbus::rendering::kMinReflectionTargetSize,
                                              static_cast<int>(std::ceil(screenBoundedDiameter)));

            for (const int reflectionIndex : partReflectionIndices) {
                visibleReflectionTextureIndices.insert(reflectionIndex);
                reflectionRequiredSizes[reflectionIndex] =
                    std::max(reflectionRequiredSizes[reflectionIndex], requiredSize);
            }
        }
    }
}

bool Vehicle::needsReflectionTexture(std::size_t reflectionIndex) const {
    return visibleReflectionTextureIndices.find(static_cast<int>(reflectionIndex)) !=
           visibleReflectionTextureIndices.end();
}

int Vehicle::requiredReflectionSize(std::size_t reflectionIndex) const {
    const auto required = reflectionRequiredSizes.find(static_cast<int>(reflectionIndex));
    return required == reflectionRequiredSizes.end() ? openbus::rendering::kMinReflectionTargetSize
                                                     : required->second;
}

int Vehicle::lodForDistance(double distance) const {
    if (lodThresholds.empty()) {
        return -1;
    }
    for (std::size_t index = 0; index < lodThresholds.size(); ++index) {
        const double boundary = 25.0 / std::max(lodThresholds[index], 0.001);
        if (distance <= boundary) {
            return static_cast<int>(index);
        }
    }
    return static_cast<int>(lodThresholds.size() - 1);
}

void Vehicle::rebuildDisplayOrder() {
    std::stable_sort(displayLists.begin(), displayLists.end(),
                     [](const DisplayPart& first, const DisplayPart& second) {
                         return first.transparent < second.transparent;
                     });
    ++clickableStateRevision;
    for (DisplayPart& part : displayLists) {
        part.pickOccluder =
            std::any_of(part.batches.begin(), part.batches.end(), [](const Batch& batch) {
                return batch.alphaMode == 0 && !batch.noZwrite && !batch.noZcheck;
            });
    }
    opaqueDisplayCount = 0;
    while (opaqueDisplayCount < displayLists.size() &&
           !displayLists[opaqueDisplayCount].transparent) {
        ++opaqueDisplayCount;
    }
}
