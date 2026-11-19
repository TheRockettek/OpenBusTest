#include "SpotlightShadowAtlas.h"

#include "CoreRenderer.h"
#include "OpenGLFunctions.h"
#include "core/Logger.h"

#include <cmath>
#include <stdexcept>

namespace openbus::rendering {
namespace {

constexpr GLenum DRAW_FRAMEBUFFER_BINDING = 0x8CA6;
constexpr GLenum READ_FRAMEBUFFER_BINDING = 0x8CAA;
constexpr GLenum PIXEL_UNPACK_BUFFER = 0x88EC;
constexpr GLenum PIXEL_UNPACK_BUFFER_BINDING = 0x88EF;

void restoreFramebuffers(GLint draw, GLint read) {
    pglBindFramebuffer(GL_DRAW_FRAMEBUFFER, static_cast<GLuint>(draw));
    pglBindFramebuffer(GL_READ_FRAMEBUFFER, static_cast<GLuint>(read));
}

void restoreCapability(GLenum capability, GLboolean enabled) {
    if (enabled)
        glEnable(capability);
    else
        glDisable(capability);
}

} // namespace

SpotlightShadowAtlas::~SpotlightShadowAtlas() {
    shutdown();
}

void SpotlightShadowAtlas::initialize(int tileSize, int capacity) {
    if (texture_ != 0 || framebuffer_ != 0)
        throw std::logic_error("Spotlight shadow atlas is already initialized");
    if (tileSize <= 0 || capacity <= 0 || capacity > MAX_LIGHTS)
        throw std::invalid_argument("Invalid spotlight shadow atlas dimensions");
    if (pglGenFramebuffers == nullptr || pglBindFramebuffer == nullptr ||
        pglFramebufferTexture2D == nullptr || pglCheckFramebufferStatus == nullptr ||
        pglDeleteFramebuffers == nullptr || pglBindBuffer == nullptr)
        throw std::runtime_error("Spotlight shadow framebuffer functions are unavailable");
    if (glGetError() != GL_NO_ERROR)
        throw std::runtime_error("OpenGL error before spotlight shadow atlas initialization");
    const int columns = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(capacity))));
    const int rows = (capacity + columns - 1) / columns;
    GLint maximumTextureSize = 0;
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maximumTextureSize);
    if (maximumTextureSize <= 0 || tileSize > maximumTextureSize / columns ||
        tileSize > maximumTextureSize / rows)
        throw std::length_error("Spotlight shadow atlas exceeds maximum texture dimensions");

    GLint previousTexture = 0;
    GLint previousDraw = 0;
    GLint previousRead = 0;
    GLint previousUnpackBuffer = 0;
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &previousTexture);
    glGetIntegerv(DRAW_FRAMEBUFFER_BINDING, &previousDraw);
    glGetIntegerv(READ_FRAMEBUFFER_BINDING, &previousRead);
    glGetIntegerv(PIXEL_UNPACK_BUFFER_BINDING, &previousUnpackBuffer);
    if (glGetError() != GL_NO_ERROR)
        throw std::runtime_error("Could not save spotlight shadow atlas state");
    pglBindBuffer(PIXEL_UNPACK_BUFFER, 0);
    GLuint texture = 0;
    GLuint framebuffer = 0;
    GLenum error = GL_NO_ERROR;
    GLenum status = 0;
    glGenTextures(1, &texture);
    error = glGetError();
    if (texture != 0 && error == GL_NO_ERROR) {
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, tileSize * columns, tileSize * rows, 0,
                     GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, nullptr);
        error = glGetError();
    }
    if (texture != 0 && error == GL_NO_ERROR) {
        pglGenFramebuffers(1, &framebuffer);
        error = glGetError();
    }
    if (framebuffer != 0 && error == GL_NO_ERROR) {
        pglBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
        pglFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, texture, 0);
        glDrawBuffer(GL_NONE);
        glReadBuffer(GL_NONE);
        status = pglCheckFramebufferStatus(GL_FRAMEBUFFER);
        error = glGetError();
    }
    glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(previousTexture));
    pglBindBuffer(PIXEL_UNPACK_BUFFER, static_cast<GLuint>(previousUnpackBuffer));
    restoreFramebuffers(previousDraw, previousRead);
    const GLenum restoreError = glGetError();
    if (texture == 0 || framebuffer == 0 || status != GL_FRAMEBUFFER_COMPLETE ||
        error != GL_NO_ERROR || restoreError != GL_NO_ERROR) {
        if (framebuffer != 0)
            pglDeleteFramebuffers(1, &framebuffer);
        if (texture != 0)
            glDeleteTextures(1, &texture);
        throw std::runtime_error("Could not allocate or restore spotlight shadow atlas state");
    }
    texture_ = texture;
    framebuffer_ = framebuffer;
    tileSize_ = tileSize;
    columns_ = columns;
    rows_ = rows;
    capacity_ = capacity;
}

void SpotlightShadowAtlas::shutdown() {
    if (framebuffer_ != 0) {
        pglDeleteFramebuffers(1, &framebuffer_);
        framebuffer_ = 0;
    }
    if (texture_ != 0) {
        glDeleteTextures(1, &texture_);
        texture_ = 0;
        invalidateTextureBindings();
    }
    tileSize_ = 0;
    columns_ = 0;
    rows_ = 0;
    capacity_ = 0;
}

SpotlightShadowAtlas::TileScope::TileScope(const SpotlightShadowAtlas& atlas, int index) {
    if (atlas.framebuffer_ == 0 || index < 0 || index >= atlas.capacity_)
        throw std::invalid_argument("Invalid spotlight shadow atlas tile");
    if (glGetError() != GL_NO_ERROR)
        throw std::runtime_error("OpenGL error before spotlight shadow tile");
    glGetIntegerv(DRAW_FRAMEBUFFER_BINDING, &drawFramebuffer_);
    glGetIntegerv(READ_FRAMEBUFFER_BINDING, &readFramebuffer_);
    glGetIntegerv(GL_VIEWPORT, viewport_.data());
    glGetIntegerv(GL_SCISSOR_BOX, scissor_.data());
    glGetBooleanv(GL_COLOR_WRITEMASK, colorMask_.data());
    glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask_);
    glGetIntegerv(GL_DEPTH_FUNC, &depthFunction_);
    glGetDoublev(GL_DEPTH_CLEAR_VALUE, &clearDepth_);
    depthTest_ = glIsEnabled(GL_DEPTH_TEST);
    scissorTest_ = glIsEnabled(GL_SCISSOR_TEST);
    if (glGetError() != GL_NO_ERROR)
        throw std::runtime_error("Could not save spotlight shadow tile state");

    const int x = (index % atlas.columns_) * atlas.tileSize_;
    const int y = (index / atlas.columns_) * atlas.tileSize_;
    pglBindFramebuffer(GL_FRAMEBUFFER, atlas.framebuffer_);
    glViewport(x, y, atlas.tileSize_, atlas.tileSize_);
    glEnable(GL_SCISSOR_TEST);
    glScissor(x, y, atlas.tileSize_, atlas.tileSize_);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
    glClearDepth(1.0);
    glClear(GL_DEPTH_BUFFER_BIT);
    if (glGetError() != GL_NO_ERROR) {
        restore();
        if (glGetError() != GL_NO_ERROR)
            throw std::runtime_error("Spotlight shadow tile setup and state restoration failed");
        throw std::runtime_error("Could not set up spotlight shadow tile");
    }
}

void SpotlightShadowAtlas::TileScope::restore() const noexcept {
    restoreFramebuffers(drawFramebuffer_, readFramebuffer_);
    glViewport(viewport_[0], viewport_[1], viewport_[2], viewport_[3]);
    glScissor(scissor_[0], scissor_[1], scissor_[2], scissor_[3]);
    restoreCapability(GL_SCISSOR_TEST, scissorTest_);
    glColorMask(colorMask_[0], colorMask_[1], colorMask_[2], colorMask_[3]);
    glDepthMask(depthMask_);
    glDepthFunc(static_cast<GLenum>(depthFunction_));
    glClearDepth(clearDepth_);
    restoreCapability(GL_DEPTH_TEST, depthTest_);
}

SpotlightShadowAtlas::TileScope::~TileScope() {
    if (!active_)
        return;
    restore();
    if (glGetError() != GL_NO_ERROR)
        gameLog.Error("OpenGL error during spotlight shadow tile or state restoration");
}

void SpotlightShadowAtlas::TileScope::finish() {
    if (!active_)
        throw std::logic_error("Spotlight shadow tile already finished");
    restore();
    active_ = false;
    if (glGetError() != GL_NO_ERROR)
        throw std::runtime_error("OpenGL error during spotlight shadow tile or state restoration");
}

} // namespace openbus::rendering
