#pragma once

namespace Diligent {
struct IDeviceContext;
struct IRenderDevice;
} // namespace Diligent

void verifyDiligentMotionDataRendering(Diligent::IRenderDevice& device,
                                       Diligent::IDeviceContext& context);
