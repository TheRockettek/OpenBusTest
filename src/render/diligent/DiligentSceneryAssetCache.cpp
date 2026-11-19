#include "render/diligent/DiligentSceneryAssetCache.h"

#include "core/Utf8Path.h"
#include "render/map/PreparedMapScenery.h"
#include "render/diligent/DiligentTexture2D.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <future>
#include <stdexcept>
#include <utility>

namespace openbus::rendering {

DiligentSceneryAssetCache::DiligentSceneryAssetCache(Diligent::IRenderDevice& device,
                                                     bool generateMipmaps)
    : device_(device), generateMipmaps_(generateMipmaps) {}

DiligentSceneryAssetCache::~DiligentSceneryAssetCache() = default;

std::string DiligentSceneryAssetCache::makeKey(const std::filesystem::path& configPath,
                                               const std::filesystem::path& omsiRoot) {
    std::string config = openbus::pathToUtf8(configPath.lexically_normal());
    std::string root = openbus::pathToUtf8(omsiRoot.lexically_normal());
#if defined(_WIN32) || defined(WIN32)
    const auto normalizeCase = [](std::string& value) {
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
    };
    normalizeCase(config);
    normalizeCase(root);
#endif
    return std::to_string(config.size()) + ":" + config + std::to_string(root.size()) + ":" + root;
}

std::string DiligentSceneryAssetCache::makeTextureKey(const std::filesystem::path& texturePath) {
    std::string key = openbus::pathToUtf8(texturePath.lexically_normal());
#if defined(_WIN32) || defined(WIN32)
    std::transform(key.begin(), key.end(), key.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
#endif
    return key;
}

bool DiligentSceneryAssetCache::request(const std::filesystem::path& configPath,
                                        const std::filesystem::path& omsiRoot) {
    const std::string key = makeKey(configPath, omsiRoot);
    if (retentionEnabled_ && !retainedAssetKeys_.contains(key)) {
        throw std::logic_error("Diligent scenery asset was not included in the retained set");
    }
    if (assets_.contains(key) || pending_.contains(key) || queuedKeys_.contains(key)) {
        return false;
    }
    queued_.push_back({key, configPath, omsiRoot});
    queuedKeys_.insert(key);
    scheduleQueued();
    return true;
}

bool DiligentSceneryAssetCache::requestSeasonalTexture(const std::filesystem::path& texturePath) {
    const std::string key = makeTextureKey(texturePath);
    if (retentionEnabled_ && !retainedSeasonalTextureKeys_.contains(key)) {
        throw std::logic_error("Diligent seasonal texture was not included in the retained set");
    }
    if (seasonalTextures_.contains(key) || pendingSeasonalTextures_.contains(key) ||
        queuedSeasonalTextureKeys_.contains(key)) {
        return false;
    }
    queuedSeasonalTextures_.push_back({key, texturePath});
    queuedSeasonalTextureKeys_.insert(key);
    scheduleSeasonalTextures();
    return true;
}

std::size_t DiligentSceneryAssetCache::scheduleQueued() {
    constexpr std::size_t maxConcurrentPreparations = 4;
    std::size_t scheduled = 0;
    while (!queued_.empty() && pending_.size() < maxConcurrentPreparations) {
        const Request request = queued_.front();
        std::future<PreparedSceneryAsset> preparation =
            std::async(std::launch::async, [this, configPath = request.configPath,
                                            omsiRoot = request.omsiRoot] {
                if (!std::filesystem::is_regular_file(configPath)) {
                    throw std::runtime_error("Diligent scenery config is not a regular file: " +
                                             openbus::pathToUtf8(configPath));
                }
                return prepareSceneryAsset(configPath, omsiRoot, textureImages_);
            });
        pending_.emplace(request.key, std::move(preparation));
        queuedKeys_.erase(request.key);
        queued_.pop_front();
        ++scheduled;
    }
    return scheduled;
}

std::size_t DiligentSceneryAssetCache::scheduleSeasonalTextures() {
    constexpr std::size_t maxConcurrentDecodes = 4;
    std::size_t scheduled = 0;
    while (!queuedSeasonalTextures_.empty() &&
           pendingSeasonalTextures_.size() < maxConcurrentDecodes) {
        const SeasonalTextureRequest request = queuedSeasonalTextures_.front();
        std::future<std::shared_ptr<Image>> decoded =
            std::async(std::launch::async, [this, path = request.path] {
                if (!std::filesystem::is_regular_file(path)) {
                    throw std::runtime_error("Diligent seasonal texture is not a regular file: " +
                                             openbus::pathToUtf8(path));
                }
                const std::string imageKey = openbus::pathToUtf8(path.lexically_normal());
                return textureImages_.getOrDecode(imageKey, [&path] {
                    auto image = std::make_shared<Image>();
                    if (!TextureLoader::readImage(path, *image)) {
                        throw std::runtime_error("Could not decode Diligent seasonal texture: " +
                                                 openbus::pathToUtf8(path));
                    }
                    return image;
                });
            });
        pendingSeasonalTextures_.emplace(request.key, std::move(decoded));
        queuedSeasonalTextureKeys_.erase(request.key);
        queuedSeasonalTextures_.pop_front();
        ++scheduled;
    }
    return scheduled;
}

std::size_t DiligentSceneryAssetCache::publishReady() {
    std::size_t published = 0;
    for (auto pending = pending_.begin(); pending != pending_.end();) {
        if (pending->second.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            ++pending;
            continue;
        }
        const std::string key = pending->first;
        std::future<PreparedSceneryAsset> preparation = std::move(pending->second);
        pending = pending_.erase(pending);
        PreparedSceneryAsset prepared = preparation.get();
        if (!retentionEnabled_ || retainedAssetKeys_.contains(key)) {
            auto asset =
                std::make_unique<DiligentSceneryAsset>(device_, prepared, generateMipmaps_);
            assets_.emplace(key, std::move(asset));
            ++published;
        }
    }
    for (auto pending = pendingSeasonalTextures_.begin();
         pending != pendingSeasonalTextures_.end();) {
        if (pending->second.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            ++pending;
            continue;
        }
        const std::string key = pending->first;
        std::future<std::shared_ptr<Image>> decoded = std::move(pending->second);
        pending = pendingSeasonalTextures_.erase(pending);
        std::shared_ptr<Image> image = decoded.get();
        if (!retentionEnabled_ || retainedSeasonalTextureKeys_.contains(key)) {
            seasonalTextures_.emplace(
                key, std::make_shared<DiligentTexture2D>(device_, *image, generateMipmaps_));
            ++published;
        }
    }
    scheduleQueued();
    scheduleSeasonalTextures();
    return published;
}

std::size_t DiligentSceneryAssetCache::retain(
    std::span<const std::pair<std::filesystem::path, std::filesystem::path>> assets,
    std::span<const std::filesystem::path> seasonalTextures) {
    std::unordered_set<std::string> retainedAssets;
    retainedAssets.reserve(assets.size());
    for (const auto& [configPath, omsiRoot] : assets) {
        retainedAssets.insert(makeKey(configPath, omsiRoot));
    }
    std::unordered_set<std::string> retainedTextures;
    retainedTextures.reserve(seasonalTextures.size());
    for (const std::filesystem::path& path : seasonalTextures) {
        retainedTextures.insert(makeTextureKey(path));
    }

    retentionEnabled_ = true;
    retainedAssetKeys_ = std::move(retainedAssets);
    retainedSeasonalTextureKeys_ = std::move(retainedTextures);
    std::size_t removed = 0;
    for (auto asset = assets_.begin(); asset != assets_.end();) {
        if (retainedAssetKeys_.contains(asset->first)) {
            ++asset;
        } else {
            asset = assets_.erase(asset);
            ++removed;
        }
    }
    for (auto texture = seasonalTextures_.begin(); texture != seasonalTextures_.end();) {
        if (retainedSeasonalTextureKeys_.contains(texture->first)) {
            ++texture;
        } else {
            texture = seasonalTextures_.erase(texture);
            ++removed;
        }
    }
    for (auto request = queued_.begin(); request != queued_.end();) {
        if (retainedAssetKeys_.contains(request->key)) {
            ++request;
        } else {
            queuedKeys_.erase(request->key);
            request = queued_.erase(request);
        }
    }
    for (auto request = queuedSeasonalTextures_.begin();
         request != queuedSeasonalTextures_.end();) {
        if (retainedSeasonalTextureKeys_.contains(request->key)) {
            ++request;
        } else {
            queuedSeasonalTextureKeys_.erase(request->key);
            request = queuedSeasonalTextures_.erase(request);
        }
    }
    scheduleQueued();
    scheduleSeasonalTextures();
    return removed;
}

const DiligentSceneryAsset*
DiligentSceneryAssetCache::find(const std::filesystem::path& configPath,
                                const std::filesystem::path& omsiRoot) const {
    const auto asset = assets_.find(makeKey(configPath, omsiRoot));
    return asset == assets_.end() ? nullptr : asset->second.get();
}

const DiligentTexture2D*
DiligentSceneryAssetCache::seasonalTexture(const std::filesystem::path& texturePath) const {
    const auto texture = seasonalTextures_.find(makeTextureKey(texturePath));
    return texture == seasonalTextures_.end() ? nullptr : texture->second.get();
}

std::size_t DiligentSceneryAssetCache::assetCount() const noexcept {
    return assets_.size();
}

} // namespace openbus::rendering
