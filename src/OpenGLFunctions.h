#pragma once

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include <cstddef>
#include <GL/gl.h>

#ifndef GL_VERTEX_SHADER
#define GL_VERTEX_SHADER 0x8B31
#define GL_FRAGMENT_SHADER 0x8B30
#define GL_COMPILE_STATUS 0x8B81
#define GL_LINK_STATUS 0x8B82
#define GL_INFO_LOG_LENGTH 0x8B84
#endif

using GenBuffersProc = void (*)(GLsizei, GLuint*);
using BindBufferProc = void (*)(GLenum, GLuint);
using BufferDataProc = void (*)(GLenum, std::ptrdiff_t, const void*, GLenum);
using DeleteBuffersProc = void (*)(GLsizei, const GLuint*);
using CompressedTexImage2DProc = void (*)(GLenum, GLint, GLenum, GLsizei, GLsizei, GLint, GLsizei,
                                          const void*);
using CompressedTexImage3DProc = void (*)(GLenum, GLint, GLenum, GLsizei, GLsizei, GLsizei, GLint,
                                          GLsizei, const void*);
using ActiveTextureProc = void (*)(GLenum);
using GenVertexArraysProc = void (*)(GLsizei, GLuint*);
using BindVertexArrayProc = void (*)(GLuint);
using DeleteVertexArraysProc = void (*)(GLsizei, const GLuint*);
using EnableVertexAttribArrayProc = void (*)(GLuint);
using DisableVertexAttribArrayProc = void (*)(GLuint);
using VertexAttribPointerProc = void (*)(GLuint, GLint, GLenum, GLboolean, GLsizei, const void*);
using CreateShaderProc = GLuint (*)(GLenum);
using ShaderSourceProc = void (*)(GLuint, GLsizei, const char* const*, const GLint*);
using CompileShaderProc = void (*)(GLuint);
using GetShaderivProc = void (*)(GLuint, GLenum, GLint*);
using GetShaderInfoLogProc = void (*)(GLuint, GLsizei, GLsizei*, char*);
using DeleteShaderProc = void (*)(GLuint);
using CreateProgramProc = GLuint (*)();
using AttachShaderProc = void (*)(GLuint, GLuint);
using LinkProgramProc = void (*)(GLuint);
using GetProgramivProc = void (*)(GLuint, GLenum, GLint*);
using GetProgramInfoLogProc = void (*)(GLuint, GLsizei, GLsizei*, char*);
using DeleteProgramProc = void (*)(GLuint);
using UseProgramProc = void (*)(GLuint);
using GetUniformLocationProc = GLint (*)(GLuint, const char*);
using UniformMatrix4fvProc = void (*)(GLint, GLsizei, GLboolean, const GLfloat*);
using Uniform1iProc = void (*)(GLint, GLint);
using Uniform1fProc = void (*)(GLint, GLfloat);
using Uniform2fProc = void (*)(GLint, GLfloat, GLfloat);
using Uniform3fProc = void (*)(GLint, GLfloat, GLfloat, GLfloat);
using Uniform4fProc = void (*)(GLint, GLfloat, GLfloat, GLfloat, GLfloat);
using GenFramebuffersProc = void (*)(GLsizei, GLuint*);
using BindFramebufferProc = void (*)(GLenum, GLuint);
using DeleteFramebuffersProc = void (*)(GLsizei, const GLuint*);
using FramebufferTexture2DProc = void (*)(GLenum, GLenum, GLenum, GLuint, GLint);
using CheckFramebufferStatusProc = GLenum (*)(GLenum);

extern GenBuffersProc pglGenBuffers;
extern BindBufferProc pglBindBuffer;
extern BufferDataProc pglBufferData;
extern DeleteBuffersProc pglDeleteBuffers;
extern CompressedTexImage2DProc pglCompressedTexImage2D;
extern CompressedTexImage3DProc pglCompressedTexImage3D;
extern ActiveTextureProc pglActiveTexture;
extern GenVertexArraysProc pglGenVertexArrays;
extern BindVertexArrayProc pglBindVertexArray;
extern DeleteVertexArraysProc pglDeleteVertexArrays;
extern EnableVertexAttribArrayProc pglEnableVertexAttribArray;
extern DisableVertexAttribArrayProc pglDisableVertexAttribArray;
extern VertexAttribPointerProc pglVertexAttribPointer;
extern CreateShaderProc pglCreateShader;
extern ShaderSourceProc pglShaderSource;
extern CompileShaderProc pglCompileShader;
extern GetShaderivProc pglGetShaderiv;
extern GetShaderInfoLogProc pglGetShaderInfoLog;
extern DeleteShaderProc pglDeleteShader;
extern CreateProgramProc pglCreateProgram;
extern AttachShaderProc pglAttachShader;
extern LinkProgramProc pglLinkProgram;
extern GetProgramivProc pglGetProgramiv;
extern GetProgramInfoLogProc pglGetProgramInfoLog;
extern DeleteProgramProc pglDeleteProgram;
extern UseProgramProc pglUseProgram;
extern GetUniformLocationProc pglGetUniformLocation;
extern UniformMatrix4fvProc pglUniformMatrix4fv;
extern Uniform1iProc pglUniform1i;
extern Uniform1fProc pglUniform1f;
extern Uniform2fProc pglUniform2f;
extern Uniform3fProc pglUniform3f;
extern Uniform4fProc pglUniform4f;
extern GenFramebuffersProc pglGenFramebuffers;
extern BindFramebufferProc pglBindFramebuffer;
extern DeleteFramebuffersProc pglDeleteFramebuffers;
extern FramebufferTexture2DProc pglFramebufferTexture2D;
extern CheckFramebufferStatusProc pglCheckFramebufferStatus;

bool loadOpenGLFunctions();
