// Opaque/transparent batch submission for vehicle meshes.

#include "app/vehicle/Vehicle.h"

void Vehicle::setOneSidedFrontFaceConvention() {
    glFrontFace(GL_CW);
    // Face selection is conditional in the model shader so generated and
    // imported geometry can use different sidedness per draw.
    glDisable(GL_CULL_FACE);
}

void Vehicle::drawBatch(Batch& batch, double alpha, bool forceUntextured,
                        const std::array<double, 3>* overrideColor, int alphaModeOverride,
                        bool forceDepthTest) {
    if (alpha <= 0.0) {
        return;
    }
    const int alphaMode = alphaModeOverride >= 0 ? alphaModeOverride : batch.alphaMode;
    const bool noZwrite = alphaModeOverride >= 0 ? false : batch.noZwrite;
    if (alphaMode == 2 || noZwrite) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
    } else if (alphaMode == 1) {
        glDisable(GL_BLEND);
        // Alpha mode 1 is a cutout: the shader discards transparent texels,
        // while surviving fragments are opaque and must occlude later draws.
        glDepthMask(GL_TRUE);
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(-1.0f, -1.0f);
    } else {
        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
    }
    // Ground-projected fake shadows must still be occluded by vehicle
    // geometry; keep their noZwrite behavior but honor the depth test.
    const bool noZcheck = batch.noZcheck && !forceDepthTest;
    if (noZcheck) {
        glDisable(GL_DEPTH_TEST);
    } else if (forceDepthTest) {
        glEnable(GL_DEPTH_TEST);
    }
    const bool materialBatch = !batch.materialTextures.empty() && !forceUntextured;
    const std::vector<GLuint>* materialTextureIds = nullptr;
    {
        TraceScope phase("render", "Vehicle::drawBatch.resolveTextures");
        if (materialBatch) {
            batch.materialTextureIds.resize(batch.materialTextures.size());
            batch.materialTextureFlips.resize(batch.materialTextures.size());
            std::fill(batch.materialTextureFlips.begin(), batch.materialTextureFlips.end(), false);
            for (std::size_t sourceIndex = 0; sourceIndex < batch.materialTextures.size();
                 ++sourceIndex) {
                MaterialTextureSource& source = batch.materialTextures[sourceIndex];
                ensureMaterialTexture(source);
                if (!openbus::rendering::reflectionPassActive()) {
                    const int reflectionIndex =
                        openbus::rendering::reflectionTextureIndex(source.textureName);
                    const unsigned int reflectionTexture =
                        openbus::rendering::reflectionTextureForIndex(reflectionIndex);
                    if (reflectionTexture != 0) {
                        source.texture = reflectionTexture;
                        source.textureArray = false;
                        batch.materialTextureFlips[sourceIndex] = true;
                    }
                }
                batch.materialTextureIds[sourceIndex] = source.texture;
            }
            materialTextureIds = &batch.materialTextureIds;
        } else {
            ensureTexture(batch);
        }
        ensureAuxiliaryTexture(batch, batch.lightmap);
        ensureAuxiliaryTexture(batch, batch.nightmap);
        ensureAuxiliaryTexture(batch, batch.transmap);
        ensureAuxiliaryTexture(batch, batch.bumpmap);
        updateFreeTexture(batch);
    }
    const std::array<double, 3>& color = overrideColor == nullptr ? batch.color : *overrideColor;
    TraceScope phase("render", "Vehicle::drawBatch.prepareAndSubmit");
    openbus::rendering::ModelMaterial material;
    for (std::size_t index = 0; index < 3; ++index) {
        material.specularColor[index] = static_cast<float>(batch.specularColor[index]);
        material.emissionColor[index] = static_cast<float>(batch.emissionColor[index]);
    }
    material.specularPower = static_cast<float>(batch.specularPower);
    material.texture = materialBatch ? 0 : batch.texture;
    material.textureArray = materialBatch ? false : batch.textureArray;
    material.textured = materialBatch ? false : batch.textured && !forceUntextured;
    material.textureWrapS = batch.textureWrapS;
    material.textureWrapT = batch.textureWrapT;
    material.lightmap = batch.lightmap.texture;
    material.nightmap = batch.nightmap.texture;
    material.transmap = batch.transmap.texture;
    material.bumpmap = batch.bumpmap.texture;
    material.freeTexture = batch.freeTexture;
    // Text textures use their own shader sampler. They share the dynamic
    // upload storage with script/free textures, but must not be left only
    // in the free-texture sampler or the shader will never display them.
    material.textTexture = batch.textTextureIndex >= 0 ? batch.freeTexture : 0;
    material.useLightmap = !forceUntextured && material.lightmap != 0;
    material.useNightmap = !forceUntextured && material.nightmap != 0;
    material.useTransmap = !forceUntextured && material.transmap != 0;
    material.useBumpmap = !forceUntextured && material.bumpmap != 0;
    material.useFreeTexture =
        !forceUntextured && batch.textTextureIndex < 0 && material.freeTexture != 0;
    material.useTextTexture =
        !forceUntextured && batch.textTextureIndex >= 0 && material.textTexture != 0;
    if (!forceUntextured) {
        applyInteriorLightMaterial(batch, material);
    }
    material.lightmapStrength = static_cast<float>(
        batch.lightmapStrengthVariable.empty()
            ? 1.0F
            : std::clamp(variables.get(batch.lightmapStrengthVariable), 0.0F, 1.0F));
    // Script-selected emissive items and [visible]-switched lamp meshes illuminate
    // even in daytime. Ordinary base nightmaps still follow environmental lighting.
    const float nightlight =
        batch.selectedMaterialItem || batch.visibilitySwitched
            ? 1.0F
            : std::max(variables.get("nightlighta"), 1.0F - variables.get("envir_brightness"));
    material.nightmapStrength = std::clamp(nightlight, 0.0F, 1.0F);
    material.bumpmapStrength = static_cast<float>(std::clamp(batch.bumpmapStrength, 0.0, 1.0));
    material.texcoordOffsetX = static_cast<float>(
        batch.texcoordTransXVariable.empty() ? 0.0 : variables.get(batch.texcoordTransXVariable));
    material.texcoordOffsetY = static_cast<float>(
        batch.texcoordTransYVariable.empty() ? 0.0 : variables.get(batch.texcoordTransYVariable));
    if (!openbus::rendering::reflectionPassActive() && !materialBatch) {
        const int reflectionIndex = openbus::rendering::reflectionTextureIndex(
            batch.textureName.empty() ? batch.texturePath.string() : batch.textureName);
        const unsigned int reflectionTexture =
            openbus::rendering::reflectionTextureForIndex(reflectionIndex);
        if (reflectionTexture != 0) {
            material.texture = reflectionTexture;
            material.textureArray = false;
            material.textured = true;
            material.flipTextureY = true;
        }
    }
    if (materialBatch) {
        drawMaterialBatch(batch.buffer, batch.vertexCount, batch.materialColors,
                          *materialTextureIds, batch.materialTextureFlips);
    } else {
        drawModelBatch(batch.buffer, batch.vertexCount, material, color, alpha,
                       forceUntextured ? 0 : alphaMode, true);
    }
    if (noZcheck) {
        glEnable(GL_DEPTH_TEST);
    }
}

void Vehicle::sortTransparentBatch(Batch& batch) {
    if (batch.vertices.size() < 6 || batch.vertices.size() % 3 != 0) {
        return;
    }
    struct TriangleDepth {
        std::size_t index;
        double depth;
    };
    const Matrix4& modelView = openbus::rendering::modelViewMatrix();
    std::vector<TriangleDepth> triangles;
    triangles.reserve(batch.vertices.size() / 3);
    for (std::size_t index = 0; index < batch.vertices.size(); index += 3) {
        const auto& first = batch.vertices[index];
        const auto& second = batch.vertices[index + 1];
        const auto& third = batch.vertices[index + 2];
        const std::array<double, 4> center = {
            (static_cast<double>(first.x) + second.x + third.x) / 3.0,
            (static_cast<double>(first.y) + second.y + third.y) / 3.0,
            (static_cast<double>(first.z) + second.z + third.z) / 3.0, 1.0};
        const std::array<double, 4> viewCenter = transformPoint(modelView, center);
        triangles.push_back({index, -viewCenter[2]});
    }
    std::stable_sort(triangles.begin(), triangles.end(),
                     [](const TriangleDepth& first, const TriangleDepth& second) {
                         return first.depth > second.depth;
                     });
    std::vector<Vertex> sortedVertices;
    sortedVertices.reserve(batch.vertices.size());
    for (const TriangleDepth& triangle : triangles) {
        sortedVertices.insert(sortedVertices.end(), batch.vertices.begin() + triangle.index,
                              batch.vertices.begin() + triangle.index + 3);
    }
    if (batch.sharedGeometryBuffer) {
        GLuint privateBuffer = 0;
        pglGenBuffers(1, &privateBuffer);
        if (privateBuffer == 0) {
            return;
        }
        batch.buffer = privateBuffer;
        batch.sharedGeometryBuffer = false;
        batch.bufferTracked = true;
        assets->trackBuffer(batch.buffer);
    }
    pglBindBuffer(GL_ARRAY_BUFFER, batch.buffer);
    pglBufferData(GL_ARRAY_BUFFER,
                  static_cast<std::ptrdiff_t>(sortedVertices.size() * sizeof(Vertex)),
                  sortedVertices.data(), GL_STATIC_DRAW);
}

void Vehicle::draw(RenderViewContext context, VehicleRenderPass renderPass,
                   const Matrix4* groundShadowTransform) {
    TraceScope trace("render", "Vehicle::draw");
    openbus::rendering::IndirectCullingBatch indirectBatch;
    openbus::rendering::IndirectCullingScope indirectScope;
    std::unordered_map<const Batch*, std::size_t> indirectSlots;
    bool indirect = false;
    if (!loaded) {
        return;
    }
    const bool reflectionPass = openbus::rendering::reflectionPassActive();
    const bool renderOpaque = renderPass != VehicleRenderPass::Transparent;
    const bool classifyTransparent =
        !reflectionPass || openbus::rendering::reflectionTransparentEnabled();
    const bool renderTransparent = renderPass != VehicleRenderPass::Opaque && classifyTransparent;
    if (!reflectionPass && renderPass != VehicleRenderPass::Transparent) {
        updateAnimationStates();
    }
    // Texture uploads must happen on the OpenGL thread, so decoding and GL
    // upload are deliberately split between the worker and draw paths.
    textureUploadStart = std::chrono::steady_clock::now();

    const auto& modelView = openbus::rendering::modelViewMatrix();
    const auto& projection = openbus::rendering::projectionMatrix();
    const bool reusePreparedDraw =
        !openbus::rendering::indirectCullingAvailable() &&
        renderPass == VehicleRenderPass::Transparent && preparedDrawListsValid &&
        preparedDrawContext == context && preparedDrawReflection == reflectionPass &&
        preparedDrawTransparent == classifyTransparent &&
        preparedDrawAnimationGeneration == animationGeneration &&
        preparedDrawMaterialGeneration == materialSelectionGeneration &&
        preparedDrawModelView == modelView && preparedDrawProjection == projection;
    ViewFrustum frustum;

    {
        TraceScope phase("render", "Vehicle::draw.setup");

        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
        setOneSidedFrontFaceConvention();
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(-1.0f, -1.0f);
    }

    {
        if (!reusePreparedDraw && frustumCulling) {
            TraceScope phase("render", "Vehicle::draw.frustumSetup");
            frustum = buildViewFrustum(openbus::rendering::projectionMatrix());
        }
    }

    if (openbus::rendering::indirectCullingAvailable()) {
        using namespace openbus::rendering;
        std::vector<CullingItem> items;
        std::vector<float> boundaries;
        appendCullingLodBoundaries(boundaries, lodThresholds);
        for (const DisplayPart& part : displayLists) {
            const std::size_t partIndex = static_cast<std::size_t>(&part - displayLists.data());
            if (partIndex >= variableVisibleParts.size() || !variableVisibleParts[partIndex] ||
                !openbus::rendering::viewpointMatches(part.viewpoint, context))
                continue;
            auto item = makeCullingItem(modelView, animatedPartCenter(part), part.radius);
            item.metadata[0] = frustumCulling ? CullFrustum : 0U;
            if (part.lodIndex >= 0 && !boundaries.empty()) {
                item.metadata[0] |= CullLod;
                item.metadata[1] = static_cast<std::uint32_t>(part.lodIndex);
                item.metadata[3] = static_cast<std::uint32_t>(boundaries.size());
            }
            for (const Batch& batch : part.batches) {
                item.draw[0] = batch.vertexCount <=
                                       static_cast<std::size_t>(std::numeric_limits<GLsizei>::max())
                                   ? static_cast<std::uint32_t>(batch.vertexCount)
                                   : std::numeric_limits<std::uint32_t>::max();
                indirectSlots.emplace(&batch, items.size());
                items.push_back(item);
            }
        }
        indirect = indirectBatch.dispatch(items, makeCullingPlanes(projection), boundaries);
    }
    const auto selectIndirect = [&](const Batch& batch) {
        if (indirect)
            indirectBatch.select(indirectSlots.at(&batch));
    };
    const auto computeVisibility =
        reusePreparedDraw ? std::vector<std::uint32_t>{} : computePartVisibility(true);
    const auto visible = [&](const DisplayPart& part, std::size_t partIndex) {
        // TraceScope phase("render", "Vehicle::draw.visible");
        if (partIndex >= variableVisibleParts.size() || !variableVisibleParts[partIndex]) {
            return false;
        }
        if (indirect)
            return true;
        if (!computeVisibility.empty()) {
            return computeVisibility[partIndex] != 0;
        }
        const std::array<double, 3> center = animatedPartCenter(part);
        const std::array<double, 4> local = {center[0], center[1], center[2], 1.0};
        const std::array<double, 4> eye = transformPoint(modelView, local);
        part.cachedViewDepth = -eye[2];
        part.viewDepthCacheGeneration = viewDepthGeneration;
        const double distance = std::sqrt(eye[0] * eye[0] + eye[1] * eye[1] + eye[2] * eye[2]);
        if (!lodThresholds.empty() && part.lodIndex >= 0) {
            std::size_t selectedLod = lodThresholds.size() - 1;
            for (std::size_t index = 0; index < lodThresholds.size(); ++index) {
                const double boundary = 25.0 / std::max(lodThresholds[index], 0.001);
                if (distance <= boundary) {
                    selectedLod = index;
                    break;
                }
            }
            if (static_cast<std::size_t>(part.lodIndex) != selectedLod) {
                return false;
            }
        }
        if (!frustumCulling) {
            return true;
        }
        if (sphereOutsideFrustum(frustum, eye, part.radius)) {
            return false;
        }
        return true;
    };
    const auto viewDepth = [&](const DisplayPart& part) {
        // TraceScope phase("render", "Vehicle::draw.viewDepth");
        if (part.viewDepthCacheGeneration == viewDepthGeneration) {
            return part.cachedViewDepth;
        }
        const std::array<double, 3> center = animatedPartCenter(part);
        const std::array<double, 4> local = {center[0], center[1], center[2], 1.0};
        const double depth = -transformPoint(modelView, local)[2];
        part.cachedViewDepth = depth;
        part.viewDepthCacheGeneration = viewDepthGeneration;
        return depth;
    };
    std::vector<DisplayPart*>& opaqueParts = opaquePartsScratch;
    std::vector<TransparentBatch>& noDepthOpaqueBatches = noDepthOpaqueBatchesScratch;
    std::vector<TransparentBatch>& transparentBatches = transparentBatchesScratch;
    if (!reusePreparedDraw) {
        ++viewDepthGeneration;
        opaqueParts.clear();
        noDepthOpaqueBatches.clear();
        transparentBatches.clear();
        opaqueParts.reserve(displayLists.size());
        if (classifyTransparent) {
            transparentBatches.reserve(displayLists.size());
            noDepthOpaqueBatches.reserve(displayLists.size());
        }
        {
            TraceScope phase("render", "Vehicle::draw.classifyParts");
            for (std::size_t partIndex = 0; partIndex < displayLists.size(); ++partIndex) {
                DisplayPart& part = displayLists[partIndex];
                const bool viewpointMatches =
                    openbus::rendering::viewpointMatches(part.viewpoint, context);
                if (!viewpointMatches || !visible(part, partIndex)) {
                    continue;
                }
                bool hasOpaqueBatch = false;
                for (Batch& batch : part.batches) {
                    updateMaterialChange(batch);
                    const bool textTexture = batch.textTextureIndex >= 0;
                    hasOpaqueBatch =
                        hasOpaqueBatch || (batch.alphaMode == 0 && !batch.noZwrite && !textTexture);
                    if (classifyTransparent &&
                        (batch.alphaMode != 0 || batch.noZwrite || textTexture)) {
                        transparentBatches.push_back(
                            {&batch, &part, viewDepth(part), part.renderType});
                    }
                }
                if (renderOpaque && hasOpaqueBatch) {
                    opaqueParts.push_back(&part);
                }
            }
        }
        {
            TraceScope phase("render", "Vehicle::draw.sortBatches");
            std::sort(opaqueParts.begin(), opaqueParts.end(),
                      [&](const DisplayPart* first, const DisplayPart* second) {
                          return viewDepth(*first) < viewDepth(*second);
                      });
            std::stable_sort(transparentBatches.begin(), transparentBatches.end(),
                             [](const TransparentBatch& first, const TransparentBatch& second) {
                                 if (first.depth != second.depth) {
                                     return first.depth > second.depth;
                                 }
                                 if (first.renderType != second.renderType) {
                                     return first.renderType < second.renderType;
                                 }
                                 return false;
                             });
            std::stable_sort(noDepthOpaqueBatches.begin(), noDepthOpaqueBatches.end(),
                             [](const TransparentBatch& first, const TransparentBatch& second) {
                                 if (first.renderType != second.renderType) {
                                     return first.renderType < second.renderType;
                                 }
                                 return first.depth > second.depth;
                             });
        }
        preparedDrawListsValid = renderPass == VehicleRenderPass::Opaque && !indirect;
        if (preparedDrawListsValid) {
            preparedDrawContext = context;
            preparedDrawReflection = reflectionPass;
            preparedDrawTransparent = classifyTransparent;
            preparedDrawAnimationGeneration = animationGeneration;
            preparedDrawMaterialGeneration = materialSelectionGeneration;
            preparedDrawModelView = modelView;
            preparedDrawProjection = projection;
        }
    }
    pushMatrix();
    translate(0.0, 0.0, modelOffsetZ);
    interiorLightRootModelView = openbus::rendering::modelViewMatrix();
    if (renderOpaque) {
        TraceScope phase("render", "Vehicle::draw.opaquePass");
        for (DisplayPart* part : opaqueParts) {
            pushMatrix();
            applyDrawTransform(*part, groundShadowTransform);
            for (Batch& batch : part->batches) {
                const double alpha = alphaScale(batch);
                if (alpha <= 0.0) {
                    continue;
                }
                if (batch.alphaMode != 0 || batch.noZwrite || batch.textTextureIndex >= 0) {
                    continue;
                }
                selectIndirect(batch);
                drawBatch(batch, alpha);
                if (!reflectionPass) {
                    drawEnvironmentMap(batch, alpha);
                }
            }
            popMatrix();
        }
    }
    if (renderTransparent) {
        {
            TraceScope phase("render", "Vehicle::draw.transparentDepthPrepass");
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
            glDisable(GL_POLYGON_OFFSET_FILL);
            glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
            for (const TransparentBatch& transparent : transparentBatches) {
                Batch& batch = *transparent.batch;
                if (batch.textTextureIndex >= 0 || batch.alphaMode != 2 ||
                    batch.transmap.name.empty() || batch.noZwrite || batch.noZcheck) {
                    continue;
                }
                pushMatrix();
                applyDrawTransform(*transparent.part, groundShadowTransform);
                const double alpha = alphaScale(batch);
                if (alpha > 0.0) {
                    selectIndirect(batch);
                    drawBatch(batch, alpha, false, nullptr, 3);
                }
                popMatrix();
            }
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        }
        glDepthMask(GL_FALSE);
        glDisable(GL_BLEND);
        {
            TraceScope phase("render", "Vehicle::draw.noDepthPass");
            for (const TransparentBatch& noDepthOpaque : noDepthOpaqueBatches) {
                Batch& batch = *noDepthOpaque.batch;
                pushMatrix();
                applyDrawTransform(*noDepthOpaque.part, groundShadowTransform);
                const double alpha = alphaScale(batch);
                if (alpha <= 0.0) {
                    popMatrix();
                    continue;
                }
                selectIndirect(batch);
                drawBatch(batch, alpha);
                if (!reflectionPass) {
                    drawEnvironmentMap(batch, alpha);
                }
                popMatrix();
            }
        }
        glDepthMask(GL_FALSE);
        {
            TraceScope phase("render", "Vehicle::draw.transparentPass");
            for (const TransparentBatch& transparent : transparentBatches) {
                Batch& batch = *transparent.batch;
                const int alphaMode = batch.textTextureIndex >= 0 ? 2 : batch.alphaMode;
                pushMatrix();
                applyDrawTransform(*transparent.part, groundShadowTransform);
                const double alpha = alphaScale(batch);
                if (alpha <= 0.0) {
                    popMatrix();
                    continue;
                }
                if (alphaMode == 1) {
                    glDisable(GL_BLEND);
                    glEnable(GL_POLYGON_OFFSET_FILL);
                    glPolygonOffset(-1.0f, -1.0f);
                } else if (alphaMode == 2) {
                    glEnable(GL_BLEND);
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                    glDepthFunc(GL_LEQUAL);
                    glEnable(GL_POLYGON_OFFSET_FILL);
                    glPolygonOffset(-1.0f, -1.0f);
                } else if (batch.noZwrite) {
                    glEnable(GL_BLEND);
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                    glDisable(GL_POLYGON_OFFSET_FILL);
                } else {
                    glDisable(GL_BLEND);
                    glDisable(GL_POLYGON_OFFSET_FILL);
                }
                if (alphaMode == 2 && batch.textTextureIndex < 0 && !batch.transmap.name.empty()) {
                    sortTransparentBatch(batch);
                }
                selectIndirect(batch);
                drawBatch(batch, alpha, false, nullptr, batch.textTextureIndex >= 0 ? 2 : -1,
                          transparent.part->isShadow);
                if (!reflectionPass) {
                    drawEnvironmentMap(batch, alpha);
                }
                popMatrix();
            }
        }
    }
    {
        TraceScope phase("render", "Vehicle::draw.cleanup");
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LESS);
        glDisable(GL_BLEND);
        glDisable(GL_POLYGON_OFFSET_FILL);
        setOneSidedFrontFaceConvention();
        glDepthMask(GL_TRUE);
    }
    popMatrix();
}

void Vehicle::drawEnvironmentMap(Batch& batch, double alpha) {
    TraceScope trace("render", "Vehicle::drawEnvironmentMap");
    ensureEnvironmentTexture(batch);
    if (batch.environmentTexture == 0 || !batch.hasNormals || alpha <= 0.0) {
        return;
    }
    const double reflectionStrength =
        std::clamp(alpha * batch.environmentStrength * ENVIRONMENT_MAP_OPACITY, 0.0, 1.0);
    if (reflectionStrength <= 0.0) {
        return;
    }

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    drawEnvironmentBatch(batch.buffer, batch.vertexCount, batch.environmentTexture,
                         reflectionStrength);
    glDisable(GL_BLEND);
}
