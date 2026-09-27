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

bool initializeCoreRenderer();
void shutdownCoreRenderer();
void invalidateTextureBindings();

void drawModelBatch(GLuint buffer, std::size_t vertexCount, GLuint texture, bool textureArray,
                    bool textured, const std::array<double, 3>& color, double alpha, int alphaMode);
void drawEnvironmentBatch(GLuint buffer, std::size_t vertexCount, GLuint texture, double alpha);
void drawPrimitives(const std::vector<PrimitiveVertex>& vertices, GLenum primitive,
                    float lineWidth = 1.0f);

} // namespace openbus::rendering
