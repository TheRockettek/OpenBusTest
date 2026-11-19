#pragma once

#include "app/DiligentScriptTexture.h"
#include "app/vehicle/DiligentVehicleSourceBridge.h"

#include <map>
#include <memory>
#include <span>
#include <vector>

class ScriptRuntime;

namespace Diligent {
struct IDeviceContext;
struct IRenderDevice;
} // namespace Diligent

namespace openbus::app::vehicle {

// Per-vehicle runtime texture storage. Keep this object alive with the vehicle's
// Diligent asset so script slot revisions and GPU allocations remain reusable.
class DiligentVehicleRuntimeTextures final {
  public:
    explicit DiligentVehicleRuntimeTextures(Diligent::IRenderDevice& device);

    // Produces batch-addressed free/text map overrides. False means at least one referenced
    // slot does not yet have a drawable snapshot; the output is cleared in that case.
    bool synchronize(const Vehicle::DisplayPart& part, std::size_t displayPartIndex,
                     ScriptRuntime& scripts, Diligent::IDeviceContext& immediateContext,
                     std::vector<openbus::rendering::DiligentVehicleMaterialOverride>& overrides);
    bool synchronize(std::span<const Vehicle::DisplayPart> displayParts, ScriptRuntime& scripts,
                     Diligent::IDeviceContext& immediateContext,
                     std::vector<openbus::rendering::DiligentVehicleMaterialOverride>& overrides);

  private:
    using TextureKey = std::pair<openbus::app::DiligentScriptTextureKind, int>;
    Diligent::IRenderDevice* device_ = nullptr;
    std::map<TextureKey, std::unique_ptr<openbus::app::DiligentScriptTexture>> textures_;
};

} // namespace openbus::app::vehicle
