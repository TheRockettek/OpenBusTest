#include "render/map/PreparedMapTextures.h"

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

using openbus::rendering::Image;

namespace {

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

} // namespace

int main() {
    try {
        openbus::rendering::PreparedMapTextures cache;
        auto road = makeImage();
        auto scenery = makeImage();
        std::weak_ptr<Image> roadLifetime = road;
        std::weak_ptr<Image> sceneryLifetime = scenery;
        cache.remember("road", std::move(road));
        cache.remember("scenery", std::move(scenery));
        const auto initialBytes = cache.bytes();
        require(cache.count() == 2 && initialBytes >= 8 * 1024 * 1024,
                "Prepared images were not retained");

        try {
            cache.upload("road", [](const Image* image) -> int {
                require(image != nullptr, "Deferred road image was lost");
                throw std::runtime_error("upload failed");
            });
            throw std::runtime_error("Upload failure was not propagated");
        } catch (const std::runtime_error& error) {
            require(std::string(error.what()) == "upload failed", "Wrong upload failure");
        }
        require(!roadLifetime.expired() && cache.bytes() == initialBytes,
                "Failed upload released its image");

        const int roadTexture = cache.upload("road", [](const Image* image) {
            require(image != nullptr, "Retry did not receive the prepared road image");
            return 1;
        });
        require(roadTexture == 1 && roadLifetime.expired() && cache.count() == 1,
                "Successful road upload did not release its image");
        require(!sceneryLifetime.expired(), "Unrelated pending image was released");

        auto duplicate = makeImage();
        std::weak_ptr<Image> duplicateLifetime = duplicate;
        cache.remember("road", std::move(duplicate), true);
        require(duplicateLifetime.expired() && !cache.contains("road"),
                "Already-uploaded texture retained a new prepared image");

        bool loadedFromDisk = false;
        const int clampTexture = cache.upload("road", [&](const Image* image) {
            require(image == nullptr, "A second address mode should use the fallback");
            loadedFromDisk = true;
            return 2;
        });
        require(loadedFromDisk && clampTexture != roadTexture,
                "Second texture address mode was not independently uploaded");

        try {
            cache.upload("missing", [](const Image* image) -> int {
                require(image == nullptr, "Missing texture unexpectedly had an image");
                throw std::runtime_error("decode failed");
            });
            throw std::runtime_error("Decode failure was not propagated");
        } catch (const std::runtime_error& error) {
            require(std::string(error.what()) == "decode failed", "Wrong decode failure");
        }

        cache.upload("scenery", [](const Image* image) {
            require(image != nullptr, "Pending scenery image was lost");
            return 3;
        });
        require(sceneryLifetime.expired() && cache.bytes() == 0 && cache.count() == 0,
                "Successful scenery upload did not release its image");
        std::cout << "Prepared image bytes: " << initialBytes << " before, " << cache.bytes()
                  << " after upload\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
