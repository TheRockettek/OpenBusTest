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

struct PrimitiveVertex {
    float x;
    float y;
    float z;
    float r;
    float g;
    float b;
};

struct ModelMaterial {
    GLuint texture = 0;
    bool textureArray = false;
    bool textured = false;
    GLuint lightmap = 0;
    GLuint nightmap = 0;
    GLuint transmap = 0;
    GLuint freeTexture = 0;
    bool useLightmap = false;
    bool useNightmap = false;
    bool useTransmap = false;
    bool useFreeTexture = false;
    float lightmapStrength = 0.0f;
    float nightmapStrength = 0.0f;
    float texcoordOffsetX = 0.0f;
    float texcoordOffsetY = 0.0f;
};

bool initializeCoreRenderer();
void shutdownCoreRenderer();
void invalidateTextureBindings();

void drawModelBatch(GLuint buffer, std::size_t vertexCount, const ModelMaterial& material,
                    const std::array<double, 3>& color, double alpha, int alphaMode);
void drawEnvironmentBatch(GLuint buffer, std::size_t vertexCount, GLuint texture, double alpha);
void drawPrimitives(const std::vector<PrimitiveVertex>& vertices, GLenum primitive,
                    float lineWidth = 1.0f);

} // namespace openbus::rendering
