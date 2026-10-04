#pragma once

#include "CameraMath.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif
#include <GL/gl.h>
#include <array>
#include <cstddef>
#include <vector>

#ifndef GL_TEXTURE0
#define GL_TEXTURE0 0x84C0
#endif
#ifndef GL_TEXTURE1
#define GL_TEXTURE1 0x84C1
#define GL_TEXTURE2 0x84C2
#define GL_TEXTURE3 0x84C3
#define GL_TEXTURE4 0x84C4
#endif
#ifndef GL_TEXTURE_2D_ARRAY
#define GL_TEXTURE_2D_ARRAY 0x8C1A
#endif
#ifndef GL_ARRAY_BUFFER
#define GL_ARRAY_BUFFER 0x8892
#endif
#ifndef GL_STATIC_DRAW
#define GL_STATIC_DRAW 0x88E4
#endif
#ifndef GL_STREAM_DRAW
#define GL_STREAM_DRAW 0x88E0
#endif

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

// Renderer-owned GL storage for immutable primitive vertices; shutdown resets the cache.
struct StaticPrimitiveBuffer {
    GLuint buffer = 0;
    std::size_t vertexCount = 0;
};

struct ModelMaterial {
    GLuint texture = 0;
    bool textureArray = false;
    bool textured = false;
    GLenum textureWrapS = GL_REPEAT;
    GLenum textureWrapT = GL_REPEAT;
    GLuint lightmap = 0;
    GLuint nightmap = 0;
    GLuint transmap = 0;
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
    std::array<std::array<float, 3>, MAX_INTERIOR_LIGHTS> interiorLightPositions = {};
    std::array<std::array<float, 3>, MAX_INTERIOR_LIGHTS> interiorLightColors = {};
    std::array<float, MAX_INTERIOR_LIGHTS> interiorLightStrengths = {};
    float texcoordOffsetX = 0.0f;
    float texcoordOffsetY = 0.0f;
};

bool initializeCoreRenderer();
void shutdownCoreRenderer();
void invalidateTextureBindings();

void drawModelBatch(GLuint buffer, std::size_t vertexCount, const ModelMaterial& material,
                    const std::array<double, 3>& color, double alpha, int alphaMode);
void drawMaterialBatch(GLuint buffer, std::size_t vertexCount,
                       const std::vector<std::array<float, 4>>& colors,
                       const std::vector<GLuint>& textures, const std::vector<bool>& flipTextureY);
void drawEnvironmentBatch(GLuint buffer, std::size_t vertexCount, GLuint texture, double alpha);
void drawPrimitives(const std::vector<PrimitiveVertex>& vertices, GLenum primitive,
                    float lineWidth = 1.0f);
void drawStaticPrimitives(StaticPrimitiveBuffer& cache,
                          const std::vector<PrimitiveVertex>& vertices, GLenum primitive,
                          float lineWidth = 1.0f);
void drawTextureQuad(GLuint texture, float x, float y, float width, float height,
                     bool flipTextureY = false);

} // namespace openbus::rendering
