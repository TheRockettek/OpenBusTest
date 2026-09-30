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
GenVertexArraysProc pglGenVertexArrays = nullptr;
BindVertexArrayProc pglBindVertexArray = nullptr;
DeleteVertexArraysProc pglDeleteVertexArrays = nullptr;
EnableVertexAttribArrayProc pglEnableVertexAttribArray = nullptr;
DisableVertexAttribArrayProc pglDisableVertexAttribArray = nullptr;
VertexAttribPointerProc pglVertexAttribPointer = nullptr;
CreateShaderProc pglCreateShader = nullptr;
ShaderSourceProc pglShaderSource = nullptr;
CompileShaderProc pglCompileShader = nullptr;
GetShaderivProc pglGetShaderiv = nullptr;
GetShaderInfoLogProc pglGetShaderInfoLog = nullptr;
DeleteShaderProc pglDeleteShader = nullptr;
CreateProgramProc pglCreateProgram = nullptr;
AttachShaderProc pglAttachShader = nullptr;
LinkProgramProc pglLinkProgram = nullptr;
GetProgramivProc pglGetProgramiv = nullptr;
GetProgramInfoLogProc pglGetProgramInfoLog = nullptr;
DeleteProgramProc pglDeleteProgram = nullptr;
UseProgramProc pglUseProgram = nullptr;
GetUniformLocationProc pglGetUniformLocation = nullptr;
UniformMatrix4fvProc pglUniformMatrix4fv = nullptr;
Uniform1iProc pglUniform1i = nullptr;
Uniform1fProc pglUniform1f = nullptr;
Uniform2fProc pglUniform2f = nullptr;
Uniform3fProc pglUniform3f = nullptr;
Uniform4fProc pglUniform4f = nullptr;
Uniform4fvProc pglUniform4fv = nullptr;
GenFramebuffersProc pglGenFramebuffers = nullptr;
BindFramebufferProc pglBindFramebuffer = nullptr;
DeleteFramebuffersProc pglDeleteFramebuffers = nullptr;
FramebufferTexture2DProc pglFramebufferTexture2D = nullptr;
CheckFramebufferStatusProc pglCheckFramebufferStatus = nullptr;

bool loadOpenGLFunctions() {
    // Load extension entry points after the GLFW context exists.
    pglGenBuffers = reinterpret_cast<GenBuffersProc>(glfwGetProcAddress("glGenBuffers"));
    pglBindBuffer = reinterpret_cast<BindBufferProc>(glfwGetProcAddress("glBindBuffer"));
    pglBufferData = reinterpret_cast<BufferDataProc>(glfwGetProcAddress("glBufferData"));
    pglDeleteBuffers = reinterpret_cast<DeleteBuffersProc>(glfwGetProcAddress("glDeleteBuffers"));
    pglCompressedTexImage2D =
        reinterpret_cast<CompressedTexImage2DProc>(glfwGetProcAddress("glCompressedTexImage2D"));
    pglCompressedTexImage3D =
        reinterpret_cast<CompressedTexImage3DProc>(glfwGetProcAddress("glCompressedTexImage3D"));
    pglActiveTexture = reinterpret_cast<ActiveTextureProc>(glfwGetProcAddress("glActiveTexture"));
    pglGenVertexArrays =
        reinterpret_cast<GenVertexArraysProc>(glfwGetProcAddress("glGenVertexArrays"));
    pglBindVertexArray =
        reinterpret_cast<BindVertexArrayProc>(glfwGetProcAddress("glBindVertexArray"));
    pglDeleteVertexArrays =
        reinterpret_cast<DeleteVertexArraysProc>(glfwGetProcAddress("glDeleteVertexArrays"));
    pglEnableVertexAttribArray = reinterpret_cast<EnableVertexAttribArrayProc>(
        glfwGetProcAddress("glEnableVertexAttribArray"));
    pglDisableVertexAttribArray = reinterpret_cast<DisableVertexAttribArrayProc>(
        glfwGetProcAddress("glDisableVertexAttribArray"));
    pglVertexAttribPointer =
        reinterpret_cast<VertexAttribPointerProc>(glfwGetProcAddress("glVertexAttribPointer"));
    pglCreateShader = reinterpret_cast<CreateShaderProc>(glfwGetProcAddress("glCreateShader"));
    pglShaderSource = reinterpret_cast<ShaderSourceProc>(glfwGetProcAddress("glShaderSource"));
    pglCompileShader = reinterpret_cast<CompileShaderProc>(glfwGetProcAddress("glCompileShader"));
    pglGetShaderiv = reinterpret_cast<GetShaderivProc>(glfwGetProcAddress("glGetShaderiv"));
    pglGetShaderInfoLog =
        reinterpret_cast<GetShaderInfoLogProc>(glfwGetProcAddress("glGetShaderInfoLog"));
    pglDeleteShader = reinterpret_cast<DeleteShaderProc>(glfwGetProcAddress("glDeleteShader"));
    pglCreateProgram = reinterpret_cast<CreateProgramProc>(glfwGetProcAddress("glCreateProgram"));
    pglAttachShader = reinterpret_cast<AttachShaderProc>(glfwGetProcAddress("glAttachShader"));
    pglLinkProgram = reinterpret_cast<LinkProgramProc>(glfwGetProcAddress("glLinkProgram"));
    pglGetProgramiv = reinterpret_cast<GetProgramivProc>(glfwGetProcAddress("glGetProgramiv"));
    pglGetProgramInfoLog =
        reinterpret_cast<GetProgramInfoLogProc>(glfwGetProcAddress("glGetProgramInfoLog"));
    pglDeleteProgram = reinterpret_cast<DeleteProgramProc>(glfwGetProcAddress("glDeleteProgram"));
    pglUseProgram = reinterpret_cast<UseProgramProc>(glfwGetProcAddress("glUseProgram"));
    pglGetUniformLocation =
        reinterpret_cast<GetUniformLocationProc>(glfwGetProcAddress("glGetUniformLocation"));
    pglUniformMatrix4fv =
        reinterpret_cast<UniformMatrix4fvProc>(glfwGetProcAddress("glUniformMatrix4fv"));
    pglUniform1i = reinterpret_cast<Uniform1iProc>(glfwGetProcAddress("glUniform1i"));
    pglUniform1f = reinterpret_cast<Uniform1fProc>(glfwGetProcAddress("glUniform1f"));
    pglUniform2f = reinterpret_cast<Uniform2fProc>(glfwGetProcAddress("glUniform2f"));
    pglUniform3f = reinterpret_cast<Uniform3fProc>(glfwGetProcAddress("glUniform3f"));
    pglUniform4f = reinterpret_cast<Uniform4fProc>(glfwGetProcAddress("glUniform4f"));
    pglUniform4fv = reinterpret_cast<Uniform4fvProc>(glfwGetProcAddress("glUniform4fv"));
    pglGenFramebuffers =
        reinterpret_cast<GenFramebuffersProc>(glfwGetProcAddress("glGenFramebuffers"));
    pglBindFramebuffer =
        reinterpret_cast<BindFramebufferProc>(glfwGetProcAddress("glBindFramebuffer"));
    pglDeleteFramebuffers =
        reinterpret_cast<DeleteFramebuffersProc>(glfwGetProcAddress("glDeleteFramebuffers"));
    pglFramebufferTexture2D =
        reinterpret_cast<FramebufferTexture2DProc>(glfwGetProcAddress("glFramebufferTexture2D"));
    pglCheckFramebufferStatus = reinterpret_cast<CheckFramebufferStatusProc>(
        glfwGetProcAddress("glCheckFramebufferStatus"));
    const bool available =
        pglGenBuffers && pglBindBuffer && pglBufferData && pglDeleteBuffers &&
        pglCompressedTexImage2D && pglActiveTexture && pglGenVertexArrays && pglBindVertexArray &&
        pglDeleteVertexArrays && pglEnableVertexAttribArray && pglDisableVertexAttribArray &&
        pglVertexAttribPointer && pglCreateShader && pglShaderSource && pglCompileShader &&
        pglGetShaderiv && pglGetShaderInfoLog && pglDeleteShader && pglCreateProgram &&
        pglAttachShader && pglLinkProgram && pglGetProgramiv && pglGetProgramInfoLog &&
        pglDeleteProgram && pglUseProgram && pglGetUniformLocation && pglUniformMatrix4fv &&
        pglUniform1i && pglUniform1f && pglUniform2f && pglUniform3f && pglUniform4f &&
        pglUniform4fv && pglGenFramebuffers && pglBindFramebuffer && pglDeleteFramebuffers &&
        pglFramebufferTexture2D && pglCheckFramebufferStatus;
    if (!available) {
        gameLog.Log("Failed to load required OpenGL VBO functions");
    }
    return available;
}
