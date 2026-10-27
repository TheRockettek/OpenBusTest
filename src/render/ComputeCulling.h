#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace openbus::rendering {

enum CullingFlags : std::uint32_t {
    CullFrustum = 1,
    CullMapDistance = 2,
    CullLod = 4,
    CullReject = 8,
    CullProjectedSize = 16,
};

// std430 layout shared with the compute shader. Centers are transformed in
// double precision before packing, avoiding cancellation at large map origins.
struct alignas(16) CullingItem {
    std::array<float, 4> eyeSphere = {};
    // Camera-relative XY AABB for the existing map visibility-radius rule.
    std::array<float, 4> distanceBounds = {};
    // Flags, desired LOD, first LOD boundary, boundary count.
    std::array<std::uint32_t, 4> metadata = {};
    // Geometry count for GPU-produced indirect commands; remaining words reserved.
    std::array<std::uint32_t, 4> draw = {};
};
static_assert(sizeof(CullingItem) == 64);

using CullingPlanes = std::array<std::array<float, 4>, 6>;
struct CullingProjectedSize {
    float projectionScale = 0.0F;
    float minimumFraction = 0.0F;
};
CullingProjectedSize makeCullingProjectedSize(const std::array<double, 16>& projection,
                                              double minimumFraction);
CullingPlanes makeCullingPlanes(const std::array<double, 16>& projection);
CullingItem makeCullingItem(const std::array<double, 16>& modelView,
                            const std::array<double, 3>& center, double radius);
bool sphereBelowMinimumViewportFraction(const std::array<double, 16>& modelView,
                                        const std::array<double, 16>& projection,
                                        const std::array<double, 3>& center, double radius,
                                        double minimumFraction);
std::uint32_t appendCullingLodBoundaries(std::vector<float>& boundaries,
                                         std::span<const double> thresholds);
bool cullingItemVisible(const CullingItem& item, const CullingPlanes& planes,
                        std::span<const float> lodBoundaries,
                        CullingProjectedSize projectedSize = {});
// Conservative CPU broad phase; false guarantees that the GPU cannot select
// this LOD. The compute shader still makes the final visibility decision.
bool cullingLodMayMatch(const CullingItem& item, std::span<const float> lodBoundaries);
// Retain uncertain or boundary candidates for the GPU's final frustum test.
bool cullingFrustumMayIntersect(const CullingItem& item, const CullingPlanes& planes);

constexpr bool indirectCullingRequested(std::string_view setting) {
    return setting == "compute-indirect";
}

// Single render-context service. Initialize/shutdown while that context is current.
// Disabled initialization performs no GL work and allocates no compute resources.
// Readback is reserved for diagnostic probes and is not an application backend.
void initializeComputeCulling(bool enableReadback = false, bool enableIndirect = false);
void shutdownComputeCulling();
bool computeCullingAvailable();
// False means the caller must use CPU tests. No previous-view results are reused.
bool computeCull(std::span<const CullingItem> items, const CullingPlanes& planes,
                 std::span<const float> lodBoundaries, std::vector<std::uint32_t>& visibility,
                 CullingProjectedSize projectedSize = {});

bool indirectCullingAvailable();
// Render-thread/context-owned, non-copyable batch. Every nonempty dispatch
// replaces its output store so queued draws retain their earlier view's data.
class IndirectCullingBatch {
  public:
    IndirectCullingBatch() = default;
    ~IndirectCullingBatch();
    IndirectCullingBatch(const IndirectCullingBatch&) = delete;
    IndirectCullingBatch& operator=(const IndirectCullingBatch&) = delete;
    bool dispatch(std::span<const CullingItem>, const CullingPlanes&, std::span<const float>,
                  CullingProjectedSize projectedSize = {}, bool occlusionProxies = false);
    void select(std::size_t slot) const;
    std::uint32_t commandBuffer() const {
        return buffer_;
    }

  private:
    std::uint32_t buffer_ = 0;
    std::size_t count_ = 0;
    bool occlusionProxies_ = false;
};

// Selection applies only to model/material/environment submissions, not UI/debug
// primitives. Nested scopes restore both selection and GL indirect-buffer binding.
// The indirect target and shader-storage bindings must not be changed directly
// while a scope is active; renderer buffer deletion invalidates the cached state.
class IndirectCullingScope {
  public:
    IndirectCullingScope();
    ~IndirectCullingScope();
    IndirectCullingScope(const IndirectCullingScope&) = delete;
    IndirectCullingScope& operator=(const IndirectCullingScope&) = delete;

  private:
    std::uint32_t buffer_ = 0;
    std::size_t slot_ = 0;
    bool occlusionProxies_ = false;
    int binding_ = 0;
    bool previousBindingTracked_ = false;
    bool previousStorageTracked_ = false;
    bool active_ = false;
};
void invalidateCullingStorageBinding(std::uint32_t buffer);
// Returns true if the selected GPU command was submitted; otherwise draw normally.
bool drawSelectedIndirect(bool indexed);
// Draw a 36-vertex occlusion proxy only when the selected candidate is visible.
bool drawSelectedIndirectProxy();

} // namespace openbus::rendering