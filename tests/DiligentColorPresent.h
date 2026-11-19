#pragma once

namespace Diligent {
struct IDeviceContext;
struct IRenderDevice;
}

void verifyDiligentColorPresent(Diligent::IRenderDevice& device,
                                Diligent::IDeviceContext& context);
