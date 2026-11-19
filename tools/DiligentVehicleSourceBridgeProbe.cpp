#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "app/vehicle/DiligentVehicleSourceBridge.h"
#include "app/vehicle/DiligentVehicleBridge.h"

#include <array>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::shared_ptr<Vehicle::TextureCacheEntry>
makeTextureEntry(const std::shared_ptr<Image>& image, bool complete = true) {
    auto request = std::make_shared<Vehicle::TextureRequest>();
    request->resolvedPath = "probe.png";
    request->image = image;
    request->complete = complete;
    auto entry = std::make_shared<Vehicle::TextureCacheEntry>();
    entry->request = std::move(request);
    return entry;
}

} // namespace

void runDiligentVehicleSourceBridgeProbe(Diligent::ITextureView& reflectionTexture) {
    using namespace openbus::app::vehicle;
    using namespace openbus::rendering;

    const Matrix4 identity = identityMatrix();
    DiligentVehicleViewGroupInput visibleViewGroup;
    visibleViewGroup.localAnimationTransform = identity;
    visibleViewGroup.localCenter = {0.0, 0.0, 0.5};
    visibleViewGroup.localRadius = 0.1;
    DiligentVehicleViewGroupInput offscreenViewGroup = visibleViewGroup;
    offscreenViewGroup.animationGroupIndex = 1;
    offscreenViewGroup.localAnimationTransform = translationMatrix({4.0, 0.0, 0.0});
    DiligentVehicleViewGroupInput wrongViewpointGroup = visibleViewGroup;
    wrongViewpointGroup.animationGroupIndex = 2;
    wrongViewpointGroup.viewpointMask =
        static_cast<int>(openbus::rendering::ViewpointContext::PlayerInterior);
    DiligentVehicleViewGroupInput wrongLodGroup = visibleViewGroup;
    wrongLodGroup.animationGroupIndex = 3;
    wrongLodGroup.lodIndex = 1;
    const std::array<DiligentVehicleViewGroupInput, 4> viewGroups = {
        visibleViewGroup, offscreenViewGroup, wrongViewpointGroup, wrongLodGroup};
    const auto viewTransforms = buildDiligentVehicleViewGroupTransforms(
        viewGroups, identity, identity, openbus::rendering::ViewpointContext::PlayerExterior, 2);
    require(viewTransforms.size() == 4 && viewTransforms[0].visible &&
                !viewTransforms[0].motion.has_value() && !viewTransforms[1].visible &&
                !viewTransforms[2].visible && !viewTransforms[3].visible &&
                viewTransforms[1].modelView[12] == 4.0,
            "Diligent auxiliary vehicle transforms did not apply view, visibility, or LOD state");

    Vehicle::DisplayPart part{};
    part.backFaceCulling = true;
    Vehicle::Batch batch;
    batch.vertices = std::vector<ModelVertex>{
        {-0.5F, -0.5F, 0.5F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        {0.5F, -0.5F, 0.5F, 1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        {0.0F, 0.5F, 0.5F, 0.5F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F},
    };
    batch.baseMaterial.materialIndex = 5;
    batch.alphaMode = 2;
    batch.noZwrite = true;
    batch.bumpmapStrength = 0.5;
    batch.color = {0.25, 0.5, 0.75};
    part.batches.push_back(std::move(batch));

    const DiligentVehicleBatchSource source =
        makeDiligentVehicleBatchSource(part, 7, 0);
    require(source.materialIndex == 5 && source.animationGroupIndex == 7 &&
                source.vertices.size() == 3 &&
                source.material.alphaMode == DiligentAlphaMode::Blend &&
                !source.material.depthWrite && source.material.normalMapStrength == 0.5F &&
                source.material.baseColor ==
                    std::array<float, 4>{0.25F, 0.5F, 0.75F, 1.0F} &&
                source.material.cullBackFaces,
            "Production vehicle batch bridge lost geometry or material state");

    Vehicle::Batch runtimeMaterialBatch;
    runtimeMaterialBatch.alphaMode = 0;
    runtimeMaterialBatch.color = {0.25, 0.5, 0.75};
    runtimeMaterialBatch.specularColor = {0.1, 0.2, 0.3};
    runtimeMaterialBatch.emissionColor = {0.4, 0.5, 0.6};
    runtimeMaterialBatch.specularPower = 12.0;
    runtimeMaterialBatch.bumpmapStrength = 1.5;
    runtimeMaterialBatch.environmentStrength = 0.75;
    runtimeMaterialBatch.textureAlphaOnly = true;
    runtimeMaterialBatch.textureAddressS = TextureAddressMode::Clamp;
    runtimeMaterialBatch.textureAddressT = TextureAddressMode::Mirror;
    runtimeMaterialBatch.lightmapStrengthVariable = "light_strength";
    runtimeMaterialBatch.texcoordTransXVariable = "texture_u";
    runtimeMaterialBatch.texcoordTransYVariable = "texture_v";
    Variables vehicleVariables(ScriptObjectKind::Vehicle);
    vehicleVariables.set("light_strength", 0.25F);
    vehicleVariables.set("nightlighta", 0.2F);
    vehicleVariables.set("envir_brightness", 0.75F);
    vehicleVariables.set("texture_u", 0.125F);
    vehicleVariables.set("texture_v", -0.25F);
    runtimeMaterialBatch.alphaScaleVariable = "batch_alpha";
    vehicleVariables.set("batch_alpha", 0.5F);
    const DiligentMaterialState runtimeMaterial =
        makeDiligentVehicleMaterialState(runtimeMaterialBatch, vehicleVariables, 0.5, 0.75);
    require(runtimeMaterial.alphaMode == DiligentAlphaMode::Blend &&
                runtimeMaterial.baseColor ==
                    std::array<float, 4>{0.25F, 0.5F, 0.75F, 0.5F} &&
                runtimeMaterial.specularColor ==
                    std::array<float, 3>{0.1F, 0.2F, 0.3F} &&
                runtimeMaterial.emissionColor ==
                    std::array<float, 3>{0.4F, 0.5F, 0.6F} &&
                runtimeMaterial.specularPower == 12.0F &&
                runtimeMaterial.normalMapStrength == 1.0F &&
                runtimeMaterial.textureAlphaOnly &&
                runtimeMaterial.environmentStrength == 0.75F &&
                runtimeMaterial.addressU == TextureAddressMode::Clamp &&
                runtimeMaterial.addressV == TextureAddressMode::Mirror &&
                runtimeMaterial.lightMapStrength == 0.25F &&
                runtimeMaterial.nightMapStrength == 0.25F &&
                runtimeMaterial.textureOffset == std::array<float, 2>{0.125F, -0.25F} &&
                runtimeMaterial.visibilityFade == 0.75F && !runtimeMaterial.depthWrite &&
                runtimeMaterial.depthMode == DiligentDepthMode::LessEqual,
            "Production vehicle live material state did not map to Diligent overrides");
    runtimeMaterialBatch.alphaMode = 1;
    const DiligentMaterialState cutoutMaterial =
        makeDiligentVehicleMaterialState(runtimeMaterialBatch, vehicleVariables, 1.0);
    require(cutoutMaterial.alphaMode == DiligentAlphaMode::Cutout &&
                cutoutMaterial.depthWrite &&
                cutoutMaterial.depthMode == DiligentDepthMode::Less,
            "Production vehicle cutout material did not retain opaque depth writes");
    runtimeMaterialBatch.noZwrite = true;
    const DiligentMaterialState noZwriteCutoutMaterial =
        makeDiligentVehicleMaterialState(runtimeMaterialBatch, vehicleVariables, 1.0);
    require(noZwriteCutoutMaterial.alphaMode == DiligentAlphaMode::Cutout &&
                noZwriteCutoutMaterial.blendEnabled &&
                !noZwriteCutoutMaterial.depthWrite,
            "Production vehicle no-Z-write cutout did not retain cutout blending behavior");
    runtimeMaterialBatch.noZwrite = false;
    runtimeMaterialBatch.alphaMode = 3;
    const DiligentMaterialState nearOpaqueMaterial =
        makeDiligentVehicleMaterialState(runtimeMaterialBatch, vehicleVariables, 1.0);
    require(nearOpaqueMaterial.alphaMode == DiligentAlphaMode::NearOpaque &&
                nearOpaqueMaterial.depthWrite,
            "Production vehicle near-opaque material did not retain opaque depth writes");
    runtimeMaterialBatch.alphaMode = 2;
    require(!makeDiligentVehicleMaterialState(runtimeMaterialBatch, vehicleVariables, 1.0)
                 .depthWrite,
            "Production vehicle blended material unexpectedly writes depth");
    runtimeMaterialBatch.alphaMode = 0;
    runtimeMaterialBatch.noZcheck = true;
    require(makeDiligentVehicleMaterialState(runtimeMaterialBatch, vehicleVariables, 1.0)
                    .depthMode == DiligentDepthMode::Disabled,
            "Production vehicle no-depth-check state was not preserved");
    bool invalidRuntimeAlphaRejected = false;
    try {
        (void)makeDiligentVehicleMaterialState(runtimeMaterialBatch, vehicleVariables, 1.1);
    } catch (const std::invalid_argument&) {
        invalidRuntimeAlphaRejected = true;
    }
    require(invalidRuntimeAlphaRejected,
            "Production vehicle material adapter accepted an invalid alpha value");

    Vehicle::DisplayPart liveMaterialPart{};
    liveMaterialPart.visibilityFade = 0.75;
    liveMaterialPart.batches.push_back(runtimeMaterialBatch);
    const std::array<Vehicle::DisplayPart, 1> liveMaterialParts = {
        std::move(liveMaterialPart)};
    DiligentVehicleMaterialOverrideSet liveMaterialOverrides =
        buildDiligentVehicleMaterialOverrides(liveMaterialParts, vehicleVariables);
    require(liveMaterialOverrides.overrides.size() == 1 &&
                liveMaterialOverrides.overrides[0].animationGroupIndex == 0 &&
                liveMaterialOverrides.overrides[0].sourceBatchIndex == 0 &&
                liveMaterialOverrides.overrides[0].material ==
                    &liveMaterialOverrides.materials[0] &&
                liveMaterialOverrides.materials[0].baseColor[3] == 0.5F &&
                liveMaterialOverrides.materials[0].visibilityFade == 0.75F,
            "Production vehicle material adapter lost batch identity or variable alpha state");

    Vehicle::Batch interiorLightBatch;
    interiorLightBatch.interiorLightIndexes[0] = 0;
    ModelInteriorLight interiorLight;
    interiorLight.intensity = 3.0;
    interiorLight.color = {255.0, 128.0, 0.0};
    interiorLight.position = {1.0, 2.0, 3.0};
    Vehicle::InteriorLightController interiorLightController;
    interiorLightController.isNumeric = true;
    interiorLightController.numericValue = 2.0;
    const std::array<ModelInteriorLight, 1> interiorLights = {interiorLight};
    const std::array<Vehicle::InteriorLightController, 1> interiorLightControllers = {
        interiorLightController};
    DiligentMaterialState interiorLightMaterial;
    populateDiligentVehicleInteriorLights(interiorLightMaterial, interiorLightBatch,
                                          interiorLights, interiorLightControllers,
                                          vehicleVariables, identity);
    require(interiorLightMaterial.useInteriorLights &&
                interiorLightMaterial.interiorLightCount == 1 &&
                interiorLightMaterial.interiorLightPositionsView[0] ==
                    std::array<float, 3>{2.0F, -1.0F, 3.0F} &&
                interiorLightMaterial.interiorLightColors[0][0] == 1.0F &&
                interiorLightMaterial.interiorLightColors[0][1] > 0.5F &&
                interiorLightMaterial.interiorLightStrengths[0] == 6.0F,
            "Production vehicle interior-light adapter lost transform, color, or controller state");

    Vehicle::DisplayPart reflectionPart{};
    Vehicle::Batch reflectionBatch;
    reflectionBatch.textureName = "reflexion0.bmp";
    reflectionPart.batches.push_back(std::move(reflectionBatch));
    Vehicle::Batch materialArrayBatch;
    Vehicle::MaterialTextureSource materialReflectionTexture;
    materialReflectionTexture.textureName = "reflexion0.bmp";
    materialArrayBatch.materialTextures.push_back(std::move(materialReflectionTexture));
    reflectionPart.batches.push_back(std::move(materialArrayBatch));
    Vehicle::Batch multiMaterialArrayBatch;
    Vehicle::MaterialTextureSource firstMaterialReflection;
    firstMaterialReflection.textureName = "reflexion0.bmp";
    Vehicle::MaterialTextureSource secondMaterial;
    secondMaterial.textureName = "body.png";
    multiMaterialArrayBatch.materialTextures.push_back(std::move(firstMaterialReflection));
    multiMaterialArrayBatch.materialTextures.push_back(std::move(secondMaterial));
    Vehicle::MaterialTextureSource thirdMaterialReflection;
    thirdMaterialReflection.textureName = "reflexion0.bmp";
    multiMaterialArrayBatch.materialTextures.push_back(std::move(thirdMaterialReflection));
    reflectionPart.batches.push_back(std::move(multiMaterialArrayBatch));
    const std::array<Vehicle::DisplayPart, 1> reflectionParts = {std::move(reflectionPart)};
    DiligentVehicleMaterialOverrideSet reflectionOverrides =
        buildDiligentVehicleMaterialOverrides(reflectionParts, vehicleVariables);
    const std::array<Diligent::ITextureView*, 1> reflectionTextures = {&reflectionTexture};
    bindDiligentVehicleReflectionTextures(reflectionParts, reflectionOverrides,
                                          reflectionTextures);
    require(reflectionOverrides.overrides.size() == 3 &&
                reflectionOverrides.overrides[0].baseTexture == &reflectionTexture &&
                reflectionOverrides.materials[0].flipTextureY &&
                reflectionOverrides.overrides[1].baseTexture == &reflectionTexture &&
                reflectionOverrides.materials[1].flipTextureY &&
                reflectionOverrides.overrides[2].baseTexture == nullptr &&
                !reflectionOverrides.materials[2].flipTextureY &&
                reflectionOverrides.materials[2].arrayReflectionTextures[0] ==
                    &reflectionTexture &&
                reflectionOverrides.materials[2].arrayReflectionTextures[1] == nullptr &&
                reflectionOverrides.materials[2].arrayReflectionTextures[2] ==
                    &reflectionTexture,
            "Diligent reflection binding did not preserve and override selected array layers");
    DiligentVehicleMaterialOverrideSet unavailableReflectionOverrides =
        buildDiligentVehicleMaterialOverrides(reflectionParts, vehicleVariables);
    const std::array<Diligent::ITextureView*, 1> unavailableReflectionTextures = {nullptr};
    bindDiligentVehicleReflectionTextures(reflectionParts, unavailableReflectionOverrides,
                                          unavailableReflectionTextures);
    require(unavailableReflectionOverrides.overrides[0].baseTexture == nullptr &&
                !unavailableReflectionOverrides.materials[0].flipTextureY &&
                unavailableReflectionOverrides.materials[2].arrayReflectionTextures[0] == nullptr,
            "Diligent reflection binding enabled a texture when its target was unavailable");

    auto baseImage = std::make_shared<Image>();
    baseImage->width = 1;
    baseImage->height = 1;
    baseImage->rgba = {255, 255, 255, 255};
    auto lightImage = std::make_shared<Image>(*baseImage);
    Vehicle::DisplayPart texturedPart{};
    Vehicle::Batch texturedBatch;
    texturedBatch.vertices = std::vector<ModelVertex>{
        {-0.5F, -0.5F, 0.5F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        {0.5F, -0.5F, 0.5F, 1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        {0.0F, 0.5F, 0.5F, 0.5F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F},
    };
    texturedBatch.textured = true;
    texturedBatch.textureName = "body.png";
    texturedBatch.textureCacheEntry = makeTextureEntry(baseImage);
    texturedBatch.lightmap.name = "light.png";
    texturedBatch.lightmap.cacheEntry = makeTextureEntry(lightImage);
    texturedPart.batches.push_back(std::move(texturedBatch));

    DiligentVehicleBatchSourceSet sourceSet;
    require(makeDiligentVehicleBatchSources(texturedPart, 7, sourceSet) &&
                sourceSet.batches.size() == 1 &&
                sourceSet.batches[0].animationGroupIndex == 7 &&
                std::get<std::shared_ptr<const Image>>(sourceSet.batches[0].texture) ==
                    baseImage &&
                std::get<std::shared_ptr<const Image>>(
                    sourceSet.batches[0].auxiliaryTextures.lightMap) == lightImage,
            "Production vehicle texture snapshot lost cached base or lightmap pixels");

    texturedPart.batches[0].textureCacheEntry = makeTextureEntry(baseImage, false);
    require(!makeDiligentVehicleBatchSources(texturedPart, 7, sourceSet) &&
                sourceSet.batches.empty(),
            "Production vehicle texture snapshot published a partially decoded batch");

    Vehicle::DisplayPart materialPart{};
    Vehicle::Batch materialBatch;
    materialBatch.vertices = std::vector<ModelVertex>{
        {-0.5F, -0.5F, 0.5F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        {0.5F, -0.5F, 0.5F, 1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F},
        {0.0F, 0.5F, 0.5F, 0.5F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F},
    };
    Vehicle::MaterialTextureSource materialTexture;
    materialTexture.textureCacheEntry = makeTextureEntry(lightImage);
    materialBatch.materialTextures.push_back(std::move(materialTexture));
    materialPart.batches.push_back(std::move(materialBatch));
    require(makeDiligentVehicleBatchSources(materialPart, 4, sourceSet) &&
                sourceSet.batches.size() == 1 &&
                sourceSet.batches[0].animationGroupIndex == 4 &&
                sourceSet.batches[0].textureArrayLayers.size() == 1 &&
                sourceSet.batches[0].textureArrayLayers[0] == lightImage,
            "Production vehicle material source was not retained as a Diligent array layer");

    texturedPart.batches[0].textureCacheEntry = makeTextureEntry(baseImage);
    std::array<Vehicle::DisplayPart, 2> displayParts = {
        std::move(texturedPart), std::move(materialPart)};
    DiligentVehicleBatchSourceSet combinedSources;
    require(makeDiligentVehicleBatchSources(displayParts, combinedSources) &&
                combinedSources.batches.size() == 2 &&
                combinedSources.batches[0].animationGroupIndex == 0 &&
                combinedSources.batches[0].sourceBatchIndex == 0 &&
                combinedSources.batches[1].animationGroupIndex == 1 &&
                combinedSources.batches[1].sourceBatchIndex == 0 &&
                combinedSources.batches[1].textureArrayLayers[0] == lightImage,
            "Production vehicle source aggregation lost display-part or batch identity");

    part.batches[0].alphaMode = 4;
    bool invalidAlphaRejected = false;
    try {
        (void)makeDiligentVehicleBatchSource(part, 7, 0);
    } catch (const std::invalid_argument&) {
        invalidAlphaRejected = true;
    }
    require(invalidAlphaRejected,
            "Production vehicle batch bridge accepted an unsupported alpha mode");

    part.batches[0].alphaMode = 0;
    part.batches[0].textured = true;
    bool missingTextureRejected = false;
    try {
        (void)makeDiligentVehicleBatchSource(part, 7, 0);
    } catch (const std::invalid_argument&) {
        missingTextureRejected = true;
    }
    require(missingTextureRejected,
            "Production vehicle batch bridge silently omitted a required base texture");

    part.batches[0].textured = false;
    part.batches[0].textureArray = true;
    bool missingArrayRejected = false;
    try {
        (void)makeDiligentVehicleBatchSource(part, 7, 0);
    } catch (const std::invalid_argument&) {
        missingArrayRejected = true;
    }
    require(missingArrayRejected,
            "Production vehicle batch bridge silently omitted a required texture array");
}
