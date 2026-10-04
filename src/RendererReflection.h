#pragma once

#include "BusTypes.h"
#include "VehicleConfigLoader.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

struct GLFWwindow;
class BusSimulation;

namespace openbus::rendering {

inline constexpr int kMinReflectionTargetSize = 64;
inline constexpr double kDefaultFieldOfView = 60.0;
inline constexpr double kMinFieldOfView = 20.0;
inline constexpr double kMaxFieldOfView = 120.0;
inline constexpr double kReflectionNearPlane = 0.1;

struct ReflectionRequirement {
    bool needed = false;
    int size = kMinReflectionTargetSize;
};

class ReflectionRenderer {
  public:
    using VisibilityCallback = std::function<ReflectionRequirement(std::size_t)>;
    using DrawCallback = std::function<void(const BusSimulation&, std::size_t, int, int)>;
    using RestoreCallback = std::function<void(int, int)>;

    ReflectionRenderer();
    ~ReflectionRenderer();

    ReflectionRenderer(const ReflectionRenderer&) = delete;
    ReflectionRenderer& operator=(const ReflectionRenderer&) = delete;

    void initialize(const std::vector<VehicleCamera>& cameras);
    void destroy();
    bool empty() const;
    void render(const BusSimulation& simulation, const std::vector<VehicleCamera>& cameras,
                const std::array<int, 4>& viewport, const VisibilityCallback& visibility,
                const DrawCallback& draw, const RestoreCallback& restore);
    void renderDebugOverlay(GLFWwindow* window, bool enabled) const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

bool reflectionPassActive();
unsigned int reflectionTextureForIndex(int index);
int reflectionTextureIndex(const std::string& textureName);
bool reflectionTransparentEnabled();

} // namespace openbus::rendering
