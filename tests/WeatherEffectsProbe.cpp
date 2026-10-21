#include "render/WeatherEffects.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool finiteVertices(const std::vector<openbus::rendering::PrimitiveVertex>& vertices) {
    for (const auto& vertex : vertices) {
        if (!std::isfinite(vertex.x) || !std::isfinite(vertex.y) || !std::isfinite(vertex.z) ||
            !std::isfinite(vertex.r) || !std::isfinite(vertex.g) || !std::isfinite(vertex.b)) {
            return false;
        }
    }
    return true;
}

} // namespace

int main() {
    using openbus::rendering::PrecipitationType;
    using openbus::rendering::makePrecipitationVertices;
    try {
        require(makePrecipitationVertices(PrecipitationType::None, 1.0, 0.0).empty(),
                "clear weather produces no precipitation vertices");
        const auto rain = makePrecipitationVertices(PrecipitationType::Rain, 0.5, 2.0);
        require(rain.size() == 320 && finiteVertices(rain),
                "rain rate scales a bounded set of finite streak vertices");
        const auto repeatedRain =
            makePrecipitationVertices(PrecipitationType::Rain, 0.5, 2.0);
        require(rain.front().x == repeatedRain.front().x &&
                    rain.front().y == repeatedRain.front().y &&
                    rain.front().z == repeatedRain.front().z,
                "rain particles are deterministic for a given simulation time");
        const auto laterRain =
            makePrecipitationVertices(PrecipitationType::Rain, 0.5, 3.0);
        require(rain.front().y != laterRain.front().y,
                "rain streak positions advance with simulation time");
        const auto snow = makePrecipitationVertices(PrecipitationType::Snow, 0.5, 2.0);
        require(snow.size() == 90 && finiteVertices(snow),
                "snow rate scales a bounded set of finite flake vertices");
        bool invalidRateRejected = false;
        try {
            makePrecipitationVertices(PrecipitationType::Rain, 1.1, 0.0);
        } catch (const std::invalid_argument&) {
            invalidRateRejected = true;
        }
        require(invalidRateRejected, "precipitation rates outside [0, 1] are rejected");
        std::cout << "Verified bounded deterministic rain and snow precipitation geometry.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Weather effects probe failed: " << error.what() << '\n';
        return 1;
    }
}
