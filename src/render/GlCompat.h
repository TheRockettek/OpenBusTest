#pragma once

// OpenGL enum values newer than the GL 1.1 headers shipped with some platforms.
// Each constant is guarded individually so this header composes with any
// platform or loader header that already provides a subset of them.

#include "config/ModelConfigTypes.h"
#include "render/OpenGLFunctions.h"

#ifndef GL_ARRAY_BUFFER
#define GL_ARRAY_BUFFER 0x8892
#endif
#ifndef GL_STATIC_DRAW
#define GL_STATIC_DRAW 0x88E4
#endif
#ifndef GL_COMPRESSED_RGBA_S3TC_DXT5_EXT
#define GL_COMPRESSED_RGBA_S3TC_DXT5_EXT 0x83F3
#endif
#ifndef GL_TEXTURE_2D_ARRAY
#define GL_TEXTURE_2D_ARRAY 0x8C1A
#endif
#ifndef GL_FRAMEBUFFER
#define GL_FRAMEBUFFER 0x8D40
#endif
#ifndef GL_COLOR_ATTACHMENT0
#define GL_COLOR_ATTACHMENT0 0x8CE0
#endif
#ifndef GL_DEPTH_ATTACHMENT
#define GL_DEPTH_ATTACHMENT 0x8D00
#endif
#ifndef GL_FRAMEBUFFER_COMPLETE
#define GL_FRAMEBUFFER_COMPLETE 0x8CD5
#endif
#ifndef GL_DEPTH_COMPONENT24
#define GL_DEPTH_COMPONENT24 0x81A6
#endif
#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F
#endif
#ifndef GL_CLAMP_TO_BORDER
#define GL_CLAMP_TO_BORDER 0x812D
#endif
#ifndef GL_MIRRORED_REPEAT
#define GL_MIRRORED_REPEAT 0x8370
#endif
#ifndef GL_MIRROR_CLAMP_TO_EDGE
#define GL_MIRROR_CLAMP_TO_EDGE 0x8743
#endif
#ifndef GL_TEXTURE0
#define GL_TEXTURE0 0x84C0
#endif
#ifndef GL_TEXTURE1
#define GL_TEXTURE1 0x84C1
#endif
#ifndef GL_TEXTURE_ENV
#define GL_TEXTURE_ENV 0x2300
#endif
#ifndef GL_TEXTURE_ENV_MODE
#define GL_TEXTURE_ENV_MODE 0x2200
#endif
#ifndef GL_COMBINE
#define GL_COMBINE 0x8570
#endif
#ifndef GL_REPLACE
#define GL_REPLACE 0x1E01
#endif
#ifndef GL_MODULATE
#define GL_MODULATE 0x2100
#endif
#ifndef GL_COMBINE_RGB
#define GL_COMBINE_RGB 0x8571
#endif
#ifndef GL_COMBINE_ALPHA
#define GL_COMBINE_ALPHA 0x8572
#endif
#ifndef GL_SOURCE0_RGB
#define GL_SOURCE0_RGB 0x8580
#endif
#ifndef GL_SOURCE0_ALPHA
#define GL_SOURCE0_ALPHA 0x8588
#endif
#ifndef GL_SOURCE1_ALPHA
#define GL_SOURCE1_ALPHA 0x8589
#endif
#ifndef GL_OPERAND0_RGB
#define GL_OPERAND0_RGB 0x8590
#endif
#ifndef GL_OPERAND0_ALPHA
#define GL_OPERAND0_ALPHA 0x8598
#endif
#ifndef GL_OPERAND1_ALPHA
#define GL_OPERAND1_ALPHA 0x8599
#endif
#ifndef GL_PREVIOUS
#define GL_PREVIOUS 0x8578
#endif
#ifndef GL_SRC_ALPHA
#define GL_SRC_ALPHA 0x0302
#endif
#ifndef GL_TEXTURE
#define GL_TEXTURE 0x1702
#endif
#ifndef GL_RGBA8
#define GL_RGBA8 0x8058
#endif
#ifndef GL_TEXTURE2
#define GL_TEXTURE2 0x84C2
#endif
#ifndef GL_TEXTURE3
#define GL_TEXTURE3 0x84C3
#endif
#ifndef GL_TEXTURE4
#define GL_TEXTURE4 0x84C4
#endif
#ifndef GL_TEXTURE8
#define GL_TEXTURE8 0x84C8
#endif
#ifndef GL_ELEMENT_ARRAY_BUFFER
#define GL_ELEMENT_ARRAY_BUFFER 0x8893
#endif
#ifndef GL_STREAM_DRAW
#define GL_STREAM_DRAW 0x88E0
#endif

namespace openbus::rendering {

[[nodiscard]] inline GLenum textureAddressModeToGl(TextureAddressMode mode) {
    switch (mode) {
    case TextureAddressMode::Clamp:
        return GL_CLAMP_TO_EDGE;
    case TextureAddressMode::Border:
        return GL_CLAMP_TO_BORDER;
    case TextureAddressMode::Mirror:
        return GL_MIRRORED_REPEAT;
    case TextureAddressMode::MirrorOnce:
        return GL_MIRROR_CLAMP_TO_EDGE;
    case TextureAddressMode::Repeat:
    default:
        return GL_REPEAT;
    }
}

} // namespace openbus::rendering
