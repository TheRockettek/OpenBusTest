#pragma once

#include "render/CameraMath.h"
#include "render/GlCompat.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace openbus::rendering {

constexpr std::size_t MAX_INTERIOR_LIGHTS = 4;

struct PrimitiveVertex {
    float x;
    float y;
    float z;
    float r;
    float g;
    float b;
};

// Renderer-owned GL storage for cached primitive vertices; shutdown resets the cache.
struct StaticPrimitiveBuffer {
    GLuint buffer = 0;
    std::size_t vertexCount = 0;
    std::uint64_t contentRevision = 0;

    bool requiresUpload(std::size_t requestedVertexCount,
                        std::uint64_t requestedContentRevision) const noexcept {
        return buffer == 0 || vertexCount != requestedVertexCount ||
               contentRevision != requestedContentRevision;
    }
};

struct ModelMaterial {
    GLuint texture = 0;
    GLuint detailTexture = 0;
    bool textureArray = false;
    bool textured = false;
    bool useDetailTexture = false;
    std::array<float, 2> textureScale = {1.0F, 1.0F};
    std::array<float, 2> detailTextureScale = {1.0F, 1.0F};
    GLenum textureWrapS = GL_REPEAT;
    GLenum textureWrapT = GL_REPEAT;
    GLuint lightmap = 0;
    GLuint nightmap = 0;
    GLuint transmap = 0;
    std::array<float, 2> transmapScale = {1.0F, 1.0F};
    std::array<float, 3> specularColor = {};
    std::array<float, 3> emissionColor = {};
    float specularPower = 8.0F;
    bool flipTransmapY = false;
    GLuint bumpmap = 0;
    GLuint freeTexture = 0;
    GLuint textTexture = 0;
    bool useLightmap = false;
    bool useNightmap = false;
    bool useTransmap = false;
    bool useBumpmap = false;
    bool useFreeTexture = false;
    bool useTextTexture = false;
    bool useInteriorLight = false;
    bool flipTextureY = false;
    float lightmapStrength = 0.0f;
    float nightmapStrength = 0.0f;
    float bumpmapStrength = 0.0f;
    int interiorLightCount = 0;
    // Positions are in view space, transformed from the vehicle root (not the
    // receiving mesh's per-part animation transform).
    std::array<std::array<float, 3>, MAX_INTERIOR_LIGHTS> interiorLightViewPositions = {};
    std::array<std::array<float, 3>, MAX_INTERIOR_LIGHTS> interiorLightColors = {};
    std::array<float, MAX_INTERIOR_LIGHTS> interiorLightStrengths = {};
    float texcoordOffsetX = 0.0f;
    float texcoordOffsetY = 0.0f;
};

bool initializeCoreRenderer();
void shutdownCoreRenderer();
void invalidateTextureBindings();
// Frame-local total of triangle primitives submitted through the renderer.
void resetFrameTriangleCount();
void resetFrameReflectionTriangleCount();
std::size_t frameTriangleCount();
std::size_t frameReflectionTriangleCount();
void setFrameTriangleCountReflectionPass(bool reflectionPass);

void drawModelBatch(GLuint buffer, std::size_t vertexCount, const ModelMaterial& material,
                    const std::array<double, 3>& color, double alpha, int alphaMode,
                    bool cullBackFaces = false);
void drawIndexedModelBatch(GLuint buffer, GLuint indexBuffer, std::size_t indexCount,
                           const ModelMaterial& material, const std::array<double, 3>& color,
                           double alpha, int alphaMode, bool cullBackFaces = false);
void drawMaterialBatch(GLuint buffer, std::size_t vertexCount,
                       const std::vector<std::array<float, 4>>& colors,
                       const std::vector<GLuint>& textures, const std::vector<bool>& flipTextureY);
void drawEnvironmentBatch(GLuint buffer, std::size_t vertexCount, GLuint texture, double alpha);
void drawPrimitives(const std::vector<PrimitiveVertex>& vertices, GLenum primitive,
                    float lineWidth = 1.0f);
void drawStaticPrimitives(StaticPrimitiveBuffer& cache,
                          const std::vector<PrimitiveVertex>& vertices, GLenum primitive,
                          float lineWidth = 1.0f, std::uint64_t contentRevision = 0);
void drawTextureQuad(GLuint texture, float x, float y, float width, float height,
                     bool flipTextureY = false);

} // namespace openbus::rendering
