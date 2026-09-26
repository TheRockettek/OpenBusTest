#include "OpenGLFunctions.h"

#include "Logger.h"

#ifndef GLFW_INCLUDE_NONE
#define GLFW_INCLUDE_NONE
#endif
#include <GLFW/glfw3.h>

extern Logger gameLog;

GenBuffersProc pglGenBuffers = nullptr;
BindBufferProc pglBindBuffer = nullptr;
BufferDataProc pglBufferData = nullptr;
DeleteBuffersProc pglDeleteBuffers = nullptr;
CompressedTexImage2DProc pglCompressedTexImage2D = nullptr;
CompressedTexImage3DProc pglCompressedTexImage3D = nullptr;
ActiveTextureProc pglActiveTexture = nullptr;

bool loadOpenGLFunctions() {
    // Load extension entry points after the GLFW context exists.
    pglGenBuffers = reinterpret_cast<GenBuffersProc>(glfwGetProcAddress("glGenBuffers"));
    pglBindBuffer = reinterpret_cast<BindBufferProc>(glfwGetProcAddress("glBindBuffer"));
    pglBufferData = reinterpret_cast<BufferDataProc>(glfwGetProcAddress("glBufferData"));
    pglDeleteBuffers = reinterpret_cast<DeleteBuffersProc>(glfwGetProcAddress("glDeleteBuffers"));
    pglCompressedTexImage2D = reinterpret_cast<CompressedTexImage2DProc>(
        glfwGetProcAddress("glCompressedTexImage2D"));
    pglCompressedTexImage3D = reinterpret_cast<CompressedTexImage3DProc>(
        glfwGetProcAddress("glCompressedTexImage3D"));
    pglActiveTexture = reinterpret_cast<ActiveTextureProc>(glfwGetProcAddress("glActiveTexture"));
    const bool available = pglGenBuffers && pglBindBuffer && pglBufferData && pglDeleteBuffers &&
                           pglCompressedTexImage2D && pglActiveTexture;
    if (!available) {
        gameLog.Log("Failed to load required OpenGL VBO functions");
    }
    return available;
}
