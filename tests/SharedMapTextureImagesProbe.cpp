#include "render/map/SharedMapTextureImages.h"

#include <array>
#include <atomic>
#include <barrier>
#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

using openbus::rendering::Image;
using openbus::rendering::SharedMapTextureImages;
using namespace std::chrono_literals;

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::shared_ptr<Image> makeImage() {
    auto image = std::make_shared<Image>();
    image->width = 1024;
    image->height = 1024;
    image->rgba.resize(1024 * 1024 * 4);
    return image;
}

void checkSharedImage() {
    SharedMapTextureImages cache;
    constexpr std::size_t count = 8;
    std::barrier start(static_cast<std::ptrdiff_t>(count + 1));
    std::atomic<int> decodes = 0;
    std::promise<void> decoderEntered;
    std::promise<void> releaseDecoder;
    const auto release = releaseDecoder.get_future().share();
    std::vector<std::future<std::shared_ptr<Image>>> tasks;
    for (std::size_t index = 0; index < count; ++index) {
        tasks.push_back(std::async(std::launch::async, [&] {
            start.arrive_and_wait();
            return cache.getOrDecode("shared", [&] {
                if (decodes.fetch_add(1) == 0) {
                    decoderEntered.set_value();
                    release.wait();
                }
                return makeImage();
            });
        }));
    }
    start.arrive_and_wait();
    const bool entered = decoderEntered.get_future().wait_for(5s) == std::future_status::ready;
    releaseDecoder.set_value();
    std::vector<std::shared_ptr<Image>> images;
    for (auto& task : tasks) {
        images.push_back(task.get());
    }
    require(entered && decodes == 1, "Concurrent requests did not share one decode");
    const auto stats = cache.stats();
    require(stats.decodeAttempts == 1 && stats.sharedImageUses == count - 1,
            "Shared image statistics were incorrect");
    for (const auto& image : images) {
        require(image == images.front(), "Concurrent requests received different images");
    }
    require(images.front()->rgba.size() == 4 * 1024 * 1024,
            "Shared image had an unexpected size");
    std::weak_ptr<Image> lifetime = images.front();
    images.clear();
    require(lifetime.expired(), "Sharing index retained an unused image");
    auto reloaded = cache.getOrDecode("shared", [&] {
        ++decodes;
        return makeImage();
    });
    require(reloaded && decodes == 2, "Expired image was not decoded again");
    std::cout << "Eight requests: one 4194304-byte image instead of eight copies\n";
}

void checkIndependentDecodes() {
    SharedMapTextureImages cache;
    std::atomic<int> entered = 0;
    std::promise<void> releaseDecoders;
    const auto release = releaseDecoders.get_future().share();
    std::array<std::future<std::shared_ptr<Image>>, 2> tasks;
    for (std::size_t index = 0; index < tasks.size(); ++index) {
        tasks[index] = std::async(std::launch::async, [&, index] {
            return cache.getOrDecode(std::to_string(index), [&] {
                ++entered;
                release.wait();
                return makeImage();
            });
        });
    }
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (entered.load() < 2 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(1ms);
    }
    const bool concurrent = entered.load() == 2;
    releaseDecoders.set_value();
    const auto first = tasks[0].get();
    const auto second = tasks[1].get();
    require(concurrent && first != second, "Different texture decodes did not run independently");
}

void checkFailureRetry() {
    SharedMapTextureImages cache;
    try {
        cache.getOrDecode("missing", []() -> std::shared_ptr<Image> {
            throw std::runtime_error("decode failed");
        });
        throw std::runtime_error("Decode failure was not propagated");
    } catch (const std::runtime_error& error) {
        require(std::string(error.what()) == "decode failed", "Wrong decode failure");
    }
    try {
        cache.getOrDecode("missing", [] { return std::shared_ptr<Image>{}; });
        throw std::runtime_error("Null decode was not rejected");
    } catch (const std::runtime_error& error) {
        require(std::string(error.what()).find("Texture decoder returned no image") == 0,
                "Wrong null decode failure");
    }
    auto recovered = cache.getOrDecode("missing", [] { return makeImage(); });
    require(recovered != nullptr, "Texture did not recover after decode failure");
}

void checkCleanupPreservesLiveImages() {
    SharedMapTextureImages cache;
    auto live = cache.getOrDecode("live", [] { return makeImage(); });
    for (int index = 0; index < 256; ++index) {
        cache.getOrDecode(std::to_string(index), [] { return std::make_shared<Image>(); });
    }
    bool decodedAgain = false;
    auto reused = cache.getOrDecode("live", [&] {
        decodedAgain = true;
        return makeImage();
    });
    require(!decodedAgain && reused == live, "Cleanup removed a live image");
}

} // namespace

int main() {
    try {
        checkSharedImage();
        checkIndependentDecodes();
        checkFailureRetry();
        checkCleanupPreservesLiveImages();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
