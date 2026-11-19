#pragma once

namespace Diligent {
struct IDeviceContext;
struct IRenderDevice;
} // namespace Diligent

void verifyDiligentImGuiRendering(Diligent::IRenderDevice& device,
                                  Diligent::IDeviceContext& context);
