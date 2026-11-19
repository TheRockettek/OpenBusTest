#pragma once

namespace Diligent {
struct IDeviceContext;
struct IRenderDevice;
}

void verifyDiligentTexturedMeshRendering(Diligent::IRenderDevice& device,
                                         Diligent::IDeviceContext& context);
