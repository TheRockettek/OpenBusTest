#pragma once

#include "render/diligent/DiligentSceneryAsset.h"
#include "render/map/SharedMapTextureImages.h"

#include <cstddef>
#include <deque>
#include <filesystem>
#include <future>
#include <memory>
#include <span>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace Diligent {
struct IRenderDevice;
}

namespace openbus::rendering {

class DiligentTexture2D;

class DiligentSceneryAssetCache final {
  public:
    explicit DiligentSceneryAssetCache(Diligent::IRenderDevice& device,
                                       bool generateMipmaps = false);
    ~DiligentSceneryAssetCache();
    DiligentSceneryAssetCache(const DiligentSceneryAssetCache&) = delete;
    DiligentSceneryAssetCache& operator=(const DiligentSceneryAssetCache&) = delete;

    // Queues CPU-only asset preparation and uploads completed assets on the calling thread.
    // Returns false when this configuration is already cached, pending, or queued.
    bool request(const std::filesystem::path& configPath, const std::filesystem::path& omsiRoot);
    bool requestSeasonalTexture(const std::filesystem::path& texturePath);
    // Publishes prepared scenery assets and decoded seasonal textures on the caller's thread.
    std::size_t publishReady();
    // Sets the exact retained asset/texture set. Call between scene frames before requesting its
    // members; pending stale work completes on its worker but is discarded instead of uploaded.
    std::size_t
    retain(std::span<const std::pair<std::filesystem::path, std::filesystem::path>> assets,
           std::span<const std::filesystem::path> seasonalTextures);

    [[nodiscard]] const DiligentSceneryAsset* find(const std::filesystem::path& configPath,
                                                   const std::filesystem::path& omsiRoot) const;
    [[nodiscard]] const DiligentTexture2D*
    seasonalTexture(const std::filesystem::path& texturePath) const;
    [[nodiscard]] std::size_t assetCount() const noexcept;

  private:
    struct Request {
        std::string key;
        std::filesystem::path configPath;
        std::filesystem::path omsiRoot;
    };

    struct SeasonalTextureRequest {
        std::string key;
        std::filesystem::path path;
    };

    [[nodiscard]] static std::string makeKey(const std::filesystem::path& configPath,
                                             const std::filesystem::path& omsiRoot);
    [[nodiscard]] static std::string makeTextureKey(const std::filesystem::path& texturePath);
    std::size_t scheduleQueued();
    std::size_t scheduleSeasonalTextures();

    Diligent::IRenderDevice& device_;
    bool generateMipmaps_;
    SharedMapTextureImages textureImages_;
    std::unordered_map<std::string, std::unique_ptr<DiligentSceneryAsset>> assets_;
    std::unordered_map<std::string, std::shared_ptr<DiligentTexture2D>> seasonalTextures_;
    std::unordered_map<std::string, std::future<PreparedSceneryAsset>> pending_;
    std::unordered_map<std::string, std::future<std::shared_ptr<Image>>> pendingSeasonalTextures_;
    std::deque<Request> queued_;
    std::unordered_set<std::string> queuedKeys_;
    std::deque<SeasonalTextureRequest> queuedSeasonalTextures_;
    std::unordered_set<std::string> queuedSeasonalTextureKeys_;
    bool retentionEnabled_ = false;
    std::unordered_set<std::string> retainedAssetKeys_;
    std::unordered_set<std::string> retainedSeasonalTextureKeys_;
};

} // namespace openbus::rendering
