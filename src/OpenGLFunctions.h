#pragma once

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include <cstddef>
#include <GL/gl.h>

using GenBuffersProc = void (*)(GLsizei, GLuint*);
using BindBufferProc = void (*)(GLenum, GLuint);
using BufferDataProc = void (*)(GLenum, std::ptrdiff_t, const void*, GLenum);
using DeleteBuffersProc = void (*)(GLsizei, const GLuint*);
using CompressedTexImage2DProc = void (*)(GLenum, GLint, GLenum, GLsizei, GLsizei, GLint, GLsizei,
                                          const void*);
using CompressedTexImage3DProc = void (*)(GLenum, GLint, GLenum, GLsizei, GLsizei, GLsizei, GLint,
                                          GLsizei, const void*);
using ActiveTextureProc = void (*)(GLenum);

extern GenBuffersProc pglGenBuffers;
extern BindBufferProc pglBindBuffer;
extern BufferDataProc pglBufferData;
extern DeleteBuffersProc pglDeleteBuffers;
extern CompressedTexImage2DProc pglCompressedTexImage2D;
extern CompressedTexImage3DProc pglCompressedTexImage3D;
extern ActiveTextureProc pglActiveTexture;

bool loadOpenGLFunctions();
