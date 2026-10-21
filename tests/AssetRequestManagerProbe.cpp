#include "render/AssetRequestManager.h"
#include "ProbeTempDirectory.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <future>
#include <iostream>
#include <mutex>

namespace {

bool writeTinyTga(const std::filesystem::path& path) {
    std::array<std::uint8_t, 22> data = {};
    data[2] = 2;
    data[12] = 1;
    data[14] = 1;
    data[16] = 32;
    data[17] = 8;
    data[18] = 20;
    data[19] = 40;
    data[20] = 60;
    data[21] = 255;
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(data.data()),
                 static_cast<std::streamsize>(data.size()));
    return output.good();
}

bool waitForCompletion(const std::shared_ptr<openbus::rendering::AssetRequestManager::
                                                TextureCacheEntry>& entry) {
    using namespace std::chrono_literals;
    return entry->request->task.valid() &&
           entry->request->task.wait_for(5s) == std::future_status::ready;
}

} // namespace

int main() {
    using namespace std::chrono_literals;
    const std::filesystem::path directory =
        openbus::probe::makeProbeDirectory("openbus_asset_request_probe");
    const std::filesystem::path imagePath = directory / "recovery.tga";
    if (!writeTinyTga(imagePath)) {
        std::cerr << "could not create async texture fixture\n";
        openbus::probe::removeProbeDirectory(directory);
        return 1;
    }

    bool passed = true;
    {
        openbus::rendering::AssetRequestManager assets;
        std::promise<void> resolverEntered;
        std::promise<void> releaseResolver;
        const std::shared_future<void> release = releaseResolver.get_future().share();
        auto delayed = assets.requestTexture(
            directory, {}, "delayed.tga",
            [&](const auto&, const auto&, const auto&) {
                resolverEntered.set_value();
                release.wait();
                return imagePath;
            });
        assets.startTextureRequest(delayed);
        if (resolverEntered.get_future().wait_for(5s) != std::future_status::ready) {
            std::cerr << "texture worker did not enter the delayed resolver\n";
            passed = false;
        } else {
            std::lock_guard<std::mutex> lock(delayed->request->mutex);
            if (!delayed->request->started || delayed->request->complete) {
                std::cerr << "delayed texture request did not remain pending\n";
                passed = false;
            }
        }
        releaseResolver.set_value();
        if (!waitForCompletion(delayed)) {
            std::cerr << "delayed texture request did not complete\n";
            passed = false;
        } else {
            std::lock_guard<std::mutex> lock(delayed->request->mutex);
            if (!delayed->request->complete || delayed->request->resolvedPath != imagePath ||
                delayed->request->image == nullptr) {
                std::cerr << "delayed texture request did not publish its decoded image\n";
                passed = false;
            }
        }

        auto failed = assets.requestTexture(
            directory, {}, "missing.tga",
            [&](const auto&, const auto&, const auto&) { return directory / "missing.tga"; });
        assets.startTextureRequest(failed);
        if (!waitForCompletion(failed)) {
            std::cerr << "failed texture request did not reach a terminal state\n";
            passed = false;
        } else {
            std::lock_guard<std::mutex> lock(failed->request->mutex);
            if (!failed->request->complete || !failed->request->resolvedPath.empty() ||
                failed->request->image != nullptr) {
                std::cerr << "failed texture request published invalid decoded state\n";
                passed = false;
            }
        }

        auto recovered = assets.requestTexture(
            directory, {}, "recovery.tga",
            [&](const auto&, const auto&, const auto&) { return imagePath; });
        assets.startTextureRequest(recovered);
        if (!waitForCompletion(recovered)) {
            std::cerr << "valid texture request after a failure did not complete\n";
            passed = false;
        } else {
            std::lock_guard<std::mutex> lock(recovered->request->mutex);
            if (!recovered->request->complete ||
                recovered->request->resolvedPath != imagePath ||
                recovered->request->image == nullptr) {
                std::cerr << "valid texture request after a failure did not recover\n";
                passed = false;
            }
        }
    }
    openbus::probe::removeProbeDirectory(directory);
    return passed ? 0 : 1;
}
