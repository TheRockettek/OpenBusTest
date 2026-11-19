#pragma once

#include "render/CameraMath.h"
#include "render/PrimitiveVertex.h"
#include "render/diligent/DiligentMultisampling.h"

#include <Graphics/GraphicsEngine/interface/GraphicsTypes.h>

#include <memory>
#include <span>

namespace Diligent {
struct IDeviceContext;
struct IRenderDevice;
} // namespace Diligent

namespace openbus::rendering {

// Draws opaque colored triangles into the active scene's color and temporal targets.
// The supplied transforms use OpenBus's column-major OpenGL matrix convention.
// Primitive motion is marked invalid; linear depth is still written when enabled.
// The device, context and bound targets must belong to the same device.
// Calling draw changes the context's pipeline, vertex buffer and shader bindings.
class DiligentPrimitiveRenderer {
  public:
    DiligentPrimitiveRenderer(Diligent::IRenderDevice& device, Diligent::TEXTURE_FORMAT colorFormat,
                              Diligent::TEXTURE_FORMAT depthFormat,
                              DiligentMultisampling multisampling = {},
                              bool includeLinearDepth = true);
    ~DiligentPrimitiveRenderer();

    DiligentPrimitiveRenderer(const DiligentPrimitiveRenderer&) = delete;
    DiligentPrimitiveRenderer& operator=(const DiligentPrimitiveRenderer&) = delete;

    void draw(Diligent::IDeviceContext& context, std::span<const PrimitiveVertex> vertices,
              const Matrix4& modelViewProjection, const Matrix4& modelView);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace openbus::rendering
