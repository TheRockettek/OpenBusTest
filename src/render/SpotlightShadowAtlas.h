#pragma once

#include "GlCompat.h"

#include <array>

namespace openbus::rendering {

class SpotlightShadowAtlas {
  public:
    static constexpr int MAX_LIGHTS = 64;

    // The owning GL context must be current through shutdown and tile scopes.
    SpotlightShadowAtlas() = default;
    ~SpotlightShadowAtlas();
    SpotlightShadowAtlas(const SpotlightShadowAtlas&) = delete;
    SpotlightShadowAtlas& operator=(const SpotlightShadowAtlas&) = delete;

    void initialize(int tileSize, int capacity);
    void shutdown();
    [[nodiscard]] GLuint texture() const noexcept {
        return texture_;
    }
    [[nodiscard]] GLuint framebuffer() const noexcept {
        return framebuffer_;
    }
    [[nodiscard]] int tileSize() const noexcept {
        return tileSize_;
    }
    [[nodiscard]] int columns() const noexcept {
        return columns_;
    }
    [[nodiscard]] int rows() const noexcept {
        return rows_;
    }
    [[nodiscard]] int capacity() const noexcept {
        return capacity_;
    }

    class TileScope {
      public:
        TileScope(const SpotlightShadowAtlas& atlas, int index);
        ~TileScope();
        TileScope(const TileScope&) = delete;
        TileScope& operator=(const TileScope&) = delete;
        // A failed pass must not be sampled; finish reports GL errors to the caller.
        void finish();

      private:
        void restore() const noexcept;
        GLint drawFramebuffer_ = 0;
        GLint readFramebuffer_ = 0;
        std::array<GLint, 4> viewport_ = {};
        std::array<GLint, 4> scissor_ = {};
        std::array<GLboolean, 4> colorMask_ = {};
        GLboolean depthMask_ = GL_FALSE;
        GLboolean depthTest_ = GL_FALSE;
        GLboolean scissorTest_ = GL_FALSE;
        GLint depthFunction_ = 0;
        GLdouble clearDepth_ = 1.0;
        bool active_ = true;
    };

  private:
    GLuint texture_ = 0;
    GLuint framebuffer_ = 0;
    int tileSize_ = 0;
    int columns_ = 0;
    int rows_ = 0;
    int capacity_ = 0;
};

} // namespace openbus::rendering
