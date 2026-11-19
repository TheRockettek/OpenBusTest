#pragma once

#include <filesystem>

namespace Diligent {
struct IRenderDevice;
struct IDeviceContext;
} // namespace Diligent

void verifyDiligentPrimitiveRendering(Diligent::IRenderDevice& device,
                                      Diligent::IDeviceContext& context,
                                      const std::filesystem::path* meshPath);
