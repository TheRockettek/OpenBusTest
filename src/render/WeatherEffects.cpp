#include "render/WeatherEffects.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace openbus::rendering {
namespace {

std::uint32_t mixBits(std::uint32_t value) {
    value ^= value >> 16U;
    value *= 0x7feb352dU;
    value ^= value >> 15U;
    value *= 0x846ca68bU;
    value ^= value >> 16U;
    return value;
}

double unitValue(std::uint32_t seed) {
    return static_cast<double>(mixBits(seed)) /
           static_cast<double>(std::numeric_limits<std::uint32_t>::max());
}

PrimitiveVertex vertex(double x, double y, double z, const std::array<float, 3>& color) {
    return {static_cast<float>(x),
            static_cast<float>(y),
            static_cast<float>(z),
            color[0],
            color[1],
            color[2]};
}

} // namespace

std::vector<PrimitiveVertex> makePrecipitationVertices(PrecipitationType type, double rate,
                                                       double elapsedSeconds) {
    if (!std::isfinite(rate) || rate < 0.0 || rate > 1.0 || !std::isfinite(elapsedSeconds) ||
        elapsedSeconds < 0.0) {
        throw std::invalid_argument("Precipitation effect input is outside its valid range");
    }
    if (type == PrecipitationType::None || rate == 0.0) {
        return {};
    }
    if (type != PrecipitationType::Rain && type != PrecipitationType::Snow) {
        throw std::invalid_argument("Precipitation effect type is unsupported");
    }

    constexpr std::size_t maxRainParticles = 320;
    constexpr std::size_t maxSnowParticles = 180;
    constexpr double particleHeight = 2.0;
    const std::size_t maxParticles =
        type == PrecipitationType::Rain ? maxRainParticles : maxSnowParticles;
    const std::size_t particleCount =
        static_cast<std::size_t>(std::lround(rate * static_cast<double>(maxParticles)));
    std::vector<PrimitiveVertex> vertices;
    vertices.reserve(particleCount * (type == PrecipitationType::Rain ? 2U : 1U));

    const bool rain = type == PrecipitationType::Rain;
    const std::array<float, 3> color =
        rain ? std::array<float, 3>{0.52F, 0.66F, 0.82F} : std::array<float, 3>{0.92F, 0.96F, 1.0F};
    const double fallSpeed = rain ? 2.2 : 0.28;
    for (std::size_t index = 0; index < particleCount; ++index) {
        const std::uint32_t seed = static_cast<std::uint32_t>(index + 1);
        const double xOffset = unitValue(seed * 3U + 1U) * 2.0 - 1.0;
        const double phase = unitValue(seed * 3U + 2U) * particleHeight;
        const double fallOffset = std::fmod(elapsedSeconds * fallSpeed + phase, particleHeight);
        const double sway = rain ? 0.0 : std::sin(elapsedSeconds * 0.8 + phase) * 0.025;
        const double x = xOffset + sway;
        const double y = 1.0 - fallOffset;
        if (rain) {
            vertices.push_back(vertex(x, y, 0.0, color));
            vertices.push_back(vertex(x + 0.008, y - 0.035, 0.0, color));
        } else {
            vertices.push_back(vertex(x, y, 0.0, color));
        }
    }
    return vertices;
}

} // namespace openbus::rendering
