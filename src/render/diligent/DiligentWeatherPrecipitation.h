#pragma once

#include "render/CameraMath.h"
#include "render/diligent/DiligentMultisampling.h"
#include "render/diligent/DiligentTexturedMesh.h"

#include <Graphics/GraphicsEngine/interface/GraphicsTypes.h>

#include <memory>
#include <optional>

namespace Diligent {
struct IDeviceContext;
struct IRenderDevice;
} // namespace Diligent

namespace openbus::rendering {

class DiligentWeatherPrecipitation final {
  public:
    DiligentWeatherPrecipitation(Diligent::IRenderDevice& device,
                                 Diligent::TEXTURE_FORMAT colorFormat,
                                 DiligentMultisampling multisampling, bool includeLinearDepth);
    ~DiligentWeatherPrecipitation();
    DiligentWeatherPrecipitation(const DiligentWeatherPrecipitation&) = delete;
    DiligentWeatherPrecipitation& operator=(const DiligentWeatherPrecipitation&) = delete;

    bool draw(Diligent::IDeviceContext& context, const Matrix4& view, const Matrix4& projection,
              Diligent::Uint32 targetWidth, Diligent::Uint32 targetHeight,
              const DiligentWeatherFog& fog, int type, double rate, double simulationSeconds,
              std::optional<double> windDirectionDegrees,
              std::optional<double> windSpeedMetersPerSecond, std::optional<double> brightness);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace openbus::rendering
