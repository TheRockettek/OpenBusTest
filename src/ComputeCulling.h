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
CullingPlanes makeCullingPlanes(const std::array<double, 16>& projection);
CullingItem makeCullingItem(const std::array<double, 16>& modelView,
                            const std::array<double, 3>& center, double radius);
std::uint32_t appendCullingLodBoundaries(std::vector<float>& boundaries,
                                        std::span<const double> thresholds);
bool cullingItemVisible(const CullingItem& item, const CullingPlanes& planes,
                        std::span<const float> lodBoundaries);

// Readback compute is an explicit diagnostic opt-in, not the default backend.
// A supported GPU alone is not evidence that dispatch + readback is worthwhile.
constexpr bool computeCullingRequested(std::string_view setting) {
    return setting == "compute-readback";
}

// Single render-context service. Initialize/shutdown while that context is current.
// Disabled initialization performs no GL work and allocates no compute resources.
void initializeComputeCulling(bool enableReadback = false, bool enableIndirect = false);
void shutdownComputeCulling();
bool computeCullingAvailable();
// False means the caller must use CPU tests. No previous-view results are reused.
bool computeCull(std::span<const CullingItem> items, const CullingPlanes& planes,
                  std::span<const float> lodBoundaries, std::vector<std::uint32_t>& visibility);

bool indirectCullingAvailable();
// Render-thread/context-owned, non-copyable batch. A separate output allocation
// per invocation keeps queued views independent. GL deletion defers actual release
// until queued draws finish; no CPU fence or readback is needed.
class IndirectCullingBatch {
    public:
        IndirectCullingBatch() = default;
        ~IndirectCullingBatch();
        IndirectCullingBatch(const IndirectCullingBatch&) = delete;
        IndirectCullingBatch& operator=(const IndirectCullingBatch&) = delete;
        bool dispatch(std::span<const CullingItem>, const CullingPlanes&, std::span<const float>);
        void select(std::size_t slot) const;
        std::uint32_t commandBuffer() const { return buffer_; }
    private:
        std::uint32_t buffer_ = 0;
        std::size_t count_ = 0;
};

// Selection applies only to model/material/environment submissions, not UI/debug
// primitives. Nested scopes restore both selection and GL indirect-buffer binding.
class IndirectCullingScope {
    public:
        IndirectCullingScope();
        ~IndirectCullingScope();
        IndirectCullingScope(const IndirectCullingScope&) = delete;
        IndirectCullingScope& operator=(const IndirectCullingScope&) = delete;
    private:
        std::uint32_t buffer_ = 0;
        std::size_t slot_ = 0;
        int binding_ = 0;
        bool active_ = false;
};
// Returns true if the selected GPU command was submitted; otherwise draw normally.
bool drawSelectedIndirect(bool indexed);

} // namespace openbus::rendering