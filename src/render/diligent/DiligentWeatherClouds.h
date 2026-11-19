#pragma once

#include "core/WeatherState.h"
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

class DiligentWeatherClouds final {
  public:
    DiligentWeatherClouds(Diligent::IRenderDevice& device, Diligent::TEXTURE_FORMAT colorFormat,
                          DiligentMultisampling multisampling, bool includeLinearDepth,
                          bool colorOnly = false);
    ~DiligentWeatherClouds();
    DiligentWeatherClouds(const DiligentWeatherClouds&) = delete;
    DiligentWeatherClouds& operator=(const DiligentWeatherClouds&) = delete;

    bool draw(Diligent::IDeviceContext& context, const Matrix4& view, const Matrix4& projection,
              const DiligentWeatherFog& fog, std::optional<WeatherCloudType> type,
              std::optional<double> baseAboveGroundMeters,
              std::optional<double> windDirectionDegrees,
              std::optional<double> windSpeedMetersPerSecond, std::optional<double> brightness,
              double groundHeightMeters, double simulationSeconds);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace openbus::rendering
