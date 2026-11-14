#include "render/OpenGLFunctions.h"

#include "core/Logger.h"

#ifndef GLFW_INCLUDE_NONE
#define GLFW_INCLUDE_NONE
#endif
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>

GenBuffersProc pglGenBuffers = nullptr;
BindBufferProc pglBindBuffer = nullptr;
BufferDataProc pglBufferData = nullptr;
TexBufferProc pglTexBuffer = nullptr;
DeleteBuffersProc pglDeleteBuffers = nullptr;
CompressedTexImage2DProc pglCompressedTexImage2D = nullptr;
CompressedTexImage3DProc pglCompressedTexImage3D = nullptr;
TexImage2DMultisampleProc pglTexImage2DMultisample = nullptr;
GenerateMipmapProc pglGenerateMipmap = nullptr;
GetStringiProc pglGetStringi = nullptr;
ActiveTextureProc pglActiveTexture = nullptr;
BlendFuncSeparateProc pglBlendFuncSeparate = nullptr;
BlendEquationSeparateProc pglBlendEquationSeparate = nullptr;
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
GetUniformBlockIndexProc pglGetUniformBlockIndex = nullptr;
UniformBlockBindingProc pglUniformBlockBinding = nullptr;
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
DrawBuffersProc pglDrawBuffers = nullptr;
BlitFramebufferProc pglBlitFramebuffer = nullptr;
ClearBufferfvProc pglClearBufferfv = nullptr;
GenQueriesProc pglGenQueries = nullptr;
DeleteQueriesProc pglDeleteQueries = nullptr;
BeginQueryProc pglBeginQuery = nullptr;
EndQueryProc pglEndQuery = nullptr;
GetQueryObjectuivProc pglGetQueryObjectuiv = nullptr;
GetQueryObjectui64vProc pglGetQueryObjectui64v = nullptr;
BeginConditionalRenderProc pglBeginConditionalRender = nullptr;
EndConditionalRenderProc pglEndConditionalRender = nullptr;
DebugMessageCallbackProc pglDebugMessageCallback = nullptr;
DispatchComputeProc pglDispatchCompute = nullptr;
MemoryBarrierProc pglMemoryBarrier = nullptr;
BindBufferBaseProc pglBindBufferBase = nullptr;
GetBufferSubDataProc pglGetBufferSubData = nullptr;
GetIntegeriProc pglGetIntegeri = nullptr;
DrawArraysIndirectProc pglDrawArraysIndirect = nullptr;
DrawElementsIndirectProc pglDrawElementsIndirect = nullptr;

namespace {

constexpr GLenum GL_TEXTURE_MAX_ANISOTROPY_EXT = 0x84FE;
constexpr GLenum GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT = 0x84FF;
TextureAnisotropyCapability textureAnisotropy;
bool textureMipmapsEnabled = false;
double configuredMapTextureScale = 1.0;

} // namespace

bool loadOpenGLFunctions() {
    pglGetStringi = reinterpret_cast<GetStringiProc>(glfwGetProcAddress("glGetStringi"));
    pglDrawArraysIndirect =
        reinterpret_cast<DrawArraysIndirectProc>(glfwGetProcAddress("glDrawArraysIndirect"));
    pglDrawElementsIndirect =
        reinterpret_cast<DrawElementsIndirectProc>(glfwGetProcAddress("glDrawElementsIndirect"));
    pglDispatchCompute =
        reinterpret_cast<DispatchComputeProc>(glfwGetProcAddress("glDispatchCompute"));
    pglMemoryBarrier = reinterpret_cast<MemoryBarrierProc>(glfwGetProcAddress("glMemoryBarrier"));
    pglBindBufferBase =
        reinterpret_cast<BindBufferBaseProc>(glfwGetProcAddress("glBindBufferBase"));
    pglGetBufferSubData =
        reinterpret_cast<GetBufferSubDataProc>(glfwGetProcAddress("glGetBufferSubData"));
    pglGetIntegeri = reinterpret_cast<GetIntegeriProc>(glfwGetProcAddress("glGetIntegeri_v"));
    // Load extension entry points after the GLFW context exists.
    pglGenBuffers = reinterpret_cast<GenBuffersProc>(glfwGetProcAddress("glGenBuffers"));
    pglBindBuffer = reinterpret_cast<BindBufferProc>(glfwGetProcAddress("glBindBuffer"));
    pglBufferData = reinterpret_cast<BufferDataProc>(glfwGetProcAddress("glBufferData"));
    pglTexBuffer = reinterpret_cast<TexBufferProc>(glfwGetProcAddress("glTexBuffer"));
    pglDeleteBuffers = reinterpret_cast<DeleteBuffersProc>(glfwGetProcAddress("glDeleteBuffers"));
    pglCompressedTexImage2D =
        reinterpret_cast<CompressedTexImage2DProc>(glfwGetProcAddress("glCompressedTexImage2D"));
    pglCompressedTexImage3D =
        reinterpret_cast<CompressedTexImage3DProc>(glfwGetProcAddress("glCompressedTexImage3D"));
    pglTexImage2DMultisample = reinterpret_cast<TexImage2DMultisampleProc>(
        glfwGetProcAddress("glTexImage2DMultisample"));
    pglGenerateMipmap =
        reinterpret_cast<GenerateMipmapProc>(glfwGetProcAddress("glGenerateMipmap"));
    pglActiveTexture = reinterpret_cast<ActiveTextureProc>(glfwGetProcAddress("glActiveTexture"));
    pglBlendFuncSeparate =
        reinterpret_cast<BlendFuncSeparateProc>(glfwGetProcAddress("glBlendFuncSeparate"));
    pglBlendEquationSeparate =
        reinterpret_cast<BlendEquationSeparateProc>(glfwGetProcAddress("glBlendEquationSeparate"));
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
    pglGetUniformBlockIndex =
        reinterpret_cast<GetUniformBlockIndexProc>(glfwGetProcAddress("glGetUniformBlockIndex"));
    pglUniformBlockBinding =
        reinterpret_cast<UniformBlockBindingProc>(glfwGetProcAddress("glUniformBlockBinding"));
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
    pglDrawBuffers = reinterpret_cast<DrawBuffersProc>(glfwGetProcAddress("glDrawBuffers"));
    pglBlitFramebuffer =
        reinterpret_cast<BlitFramebufferProc>(glfwGetProcAddress("glBlitFramebuffer"));
    pglClearBufferfv = reinterpret_cast<ClearBufferfvProc>(glfwGetProcAddress("glClearBufferfv"));
    pglGenQueries = reinterpret_cast<GenQueriesProc>(glfwGetProcAddress("glGenQueries"));
    pglDeleteQueries = reinterpret_cast<DeleteQueriesProc>(glfwGetProcAddress("glDeleteQueries"));
    pglBeginQuery = reinterpret_cast<BeginQueryProc>(glfwGetProcAddress("glBeginQuery"));
    pglEndQuery = reinterpret_cast<EndQueryProc>(glfwGetProcAddress("glEndQuery"));
    pglGetQueryObjectuiv =
        reinterpret_cast<GetQueryObjectuivProc>(glfwGetProcAddress("glGetQueryObjectuiv"));
    pglGetQueryObjectui64v =
        reinterpret_cast<GetQueryObjectui64vProc>(glfwGetProcAddress("glGetQueryObjectui64v"));
    pglBeginConditionalRender = reinterpret_cast<BeginConditionalRenderProc>(
        glfwGetProcAddress("glBeginConditionalRender"));
    pglEndConditionalRender =
        reinterpret_cast<EndConditionalRenderProc>(glfwGetProcAddress("glEndConditionalRender"));
    pglDebugMessageCallback =
        reinterpret_cast<DebugMessageCallbackProc>(glfwGetProcAddress("glDebugMessageCallback"));
    if (pglDebugMessageCallback == nullptr) {
        pglDebugMessageCallback = reinterpret_cast<DebugMessageCallbackProc>(
            glfwGetProcAddress("glDebugMessageCallbackKHR"));
    }
    const bool available =
        pglGenBuffers && pglBindBuffer && pglBufferData && pglTexBuffer && pglDeleteBuffers &&
        pglCompressedTexImage2D && pglGetStringi && pglActiveTexture && pglBlendFuncSeparate &&
        pglBlendEquationSeparate && pglGenVertexArrays && pglBindVertexArray &&
        pglDeleteVertexArrays && pglEnableVertexAttribArray && pglDisableVertexAttribArray &&
        pglVertexAttribPointer && pglCreateShader && pglShaderSource && pglCompileShader &&
        pglGetShaderiv && pglGetShaderInfoLog && pglDeleteShader && pglCreateProgram &&
        pglAttachShader && pglLinkProgram && pglGetProgramiv && pglGetProgramInfoLog &&
        pglDeleteProgram && pglUseProgram && pglGetUniformLocation && pglGetUniformBlockIndex &&
        pglUniformBlockBinding && pglBindBufferBase && pglUniformMatrix4fv && pglUniform1i &&
        pglUniform1f && pglUniform2f && pglUniform3f && pglUniform4f && pglUniform4fv &&
        pglGenFramebuffers && pglBindFramebuffer && pglDeleteFramebuffers &&
        pglFramebufferTexture2D && pglCheckFramebufferStatus && pglDrawBuffers &&
        pglBlitFramebuffer && pglClearBufferfv;
    if (!available) {
        gameLog.Error("Failed to load required OpenGL VBO functions");
    }
    return available;
}

bool hasOpenGLExtension(std::string_view extension) {
    if (extension.empty() || extension.find(' ') != std::string_view::npos ||
        pglGetStringi == nullptr) {
        return false;
    }

    GLint extensionCount = 0;
    glGetIntegerv(GL_NUM_EXTENSIONS, &extensionCount);
    if (extensionCount <= 0) {
        return false;
    }
    for (GLint index = 0; index < extensionCount; ++index) {
        const auto* value = pglGetStringi(GL_EXTENSIONS, static_cast<GLuint>(index));
        if (value != nullptr && extension == reinterpret_cast<const char*>(value)) {
            return true;
        }
    }
    return false;
}

TextureAnisotropyCapability configureMaterialTextureAnisotropy(float requested) {
    textureAnisotropy = {};
    if (!std::isfinite(requested) || requested < 1.0F) {
        return textureAnisotropy;
    }
    if (!hasOpenGLExtension("GL_EXT_texture_filter_anisotropic") &&
        !hasOpenGLExtension("GL_ARB_texture_filter_anisotropic")) {
        return textureAnisotropy;
    }
    GLfloat maximum = 1.0F;
    glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &maximum);
    if (!std::isfinite(maximum) || maximum < 1.0F) {
        return textureAnisotropy;
    }
    textureAnisotropy.supported = true;
    textureAnisotropy.maximum = maximum;
    textureAnisotropy.applied = std::clamp(requested, 1.0F, maximum);
    return textureAnisotropy;
}

bool materialTextureMipmapsEnabled() {
    return pglGenerateMipmap != nullptr &&
           (textureMipmapsEnabled ||
            (textureAnisotropy.supported && textureAnisotropy.applied > 1.0F));
}

void configureMaterialTextureMipmaps(bool enabled) {
    textureMipmapsEnabled = enabled;
}

void configureMapTextureScale(double scale) {
    configuredMapTextureScale = scale;
}

double mapTextureScale() noexcept {
    return configuredMapTextureScale;
}

void applyMaterialTextureAnisotropy(GLenum target) {
    if (textureAnisotropy.supported && textureAnisotropy.applied > 1.0F) {
        glTexParameterf(target, GL_TEXTURE_MAX_ANISOTROPY_EXT, textureAnisotropy.applied);
    }
}
