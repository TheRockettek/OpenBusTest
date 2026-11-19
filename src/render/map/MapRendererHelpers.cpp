// Free helpers shared by the map renderer translation units.

#include "render/map/MapRendererInternal.h"

#include "core/Utf8Path.h"

namespace openbus::rendering::map_detail {

double squaredDistanceToBounds(double x, double y, double minX, double minY, double maxX,
                               double maxY) {
    const double dx = x < minX ? minX - x : (x > maxX ? x - maxX : 0.0);
    const double dy = y < minY ? minY - y : (y > maxY ? y - maxY : 0.0);
    return dx * dx + dy * dy;
}

long long elapsedMilliseconds(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                 start)
        .count();
}

std::string tileLabel(int x, int y) {
    return "tile(" + std::to_string(x) + "," + std::to_string(y) + ")";
}

void recordSkipped(SkipSummaries& summaries, const std::string& reason, const std::string& example,
                   std::size_t count, LogLevel level) {
    SkipSummary& summary = summaries[reason];
    summary.count += count;
    if (level == LogLevel::Error) {
        summary.level = LogLevel::Error;
    } else if (level == LogLevel::Warning && summary.level != LogLevel::Error) {
        summary.level = LogLevel::Warning;
    }
    if (!example.empty() && summary.examples.size() < MAX_SKIP_EXAMPLES) {
        summary.examples.push_back(example);
        gameLog.Log(level, "Map skip encountered: " + reason + "; example: " + example);
    }
}

void logSkipped(const SkipSummaries& summaries) {
    for (const auto& [reason, summary] : summaries) {
        std::string message =
            "Map skip summary: skipped " + std::to_string(summary.count) + " " + reason;
        if (!summary.examples.empty()) {
            message += " (context logged when encountered)";
        }
        gameLog.Log(summary.level, message);
    }
}

std::filesystem::path groundTexturePath(const openbus::map::MapDefinition& map,
                                        const std::filesystem::path& omsiRoot,
                                        const std::string& configuredPath) {
    std::string normalized = configuredPath;
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    const std::filesystem::path relative(normalized);
    const std::array<std::filesystem::path, 2> candidates = {map.rootPath / relative,
                                                             omsiRoot / relative};
    for (const std::filesystem::path& candidate : candidates) {
        const std::filesystem::path resolved = openbus::assets::resolveTexturePath(candidate);
        if (std::filesystem::is_regular_file(resolved)) {
            return resolved;
        }
    }
    throw std::runtime_error("Could not resolve map ground texture: " + configuredPath);
}

GLuint loadTexture(const std::filesystem::path& path, bool repeat) {
    Image image;
    if (!TextureLoader::readImage(path, image)) {
        throw std::runtime_error("Could not decode map ground texture: " +
                                 openbus::pathToUtf8(path));
    }
    try {
        return uploadTextureImage(image, repeat);
    } catch (const std::exception& error) {
        throw std::runtime_error("Could not upload map ground texture " +
                                 openbus::pathToUtf8(path) + ": " + error.what());
    }
}

GLuint uploadTextureImage(const Image& image, bool repeat) {
    TraceScope trace("texture", "MapRenderer::uploadTextureImage");
    Image scaledImage;
    const Image* uploadImage = &image;
    if (mapTextureScale() < 0.999) {
        if (!TextureLoader::scaleImage(image, mapTextureScale(), scaledImage)) {
            throw std::runtime_error("Could not scale decoded map texture");
        }
        uploadImage = &scaledImage;
    }
    if (uploadImage->width <= 0 || uploadImage->height <= 0 ||
        uploadImage->rgba.size() < static_cast<std::size_t>(uploadImage->width) *
                                       static_cast<std::size_t>(uploadImage->height) * 4U) {
        throw std::runtime_error("Texture image dimensions or pixel data are invalid");
    }
    GLuint texture = 0;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    const bool generateMipmaps = materialTextureMipmapsEnabled();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                    generateMipmaps ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    const GLint wrapMode = repeat ? GL_REPEAT : GL_CLAMP_TO_EDGE;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapMode);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapMode);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, uploadImage->width, uploadImage->height, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, uploadImage->rgba.data());
    if (generateMipmaps) {
        pglGenerateMipmap(GL_TEXTURE_2D);
        applyMaterialTextureAnisotropy(GL_TEXTURE_2D);
    }
    openbus::rendering::invalidateTextureBindings();
    if (glGetError() != GL_NO_ERROR) {
        glDeleteTextures(1, &texture);
        throw std::runtime_error("OpenGL failed to upload texture image");
    }
    return texture;
}

std::filesystem::path normalizedAssetPath(const std::string& value) {
    std::string normalized = value;
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    return std::filesystem::path(normalized);
}

std::string lowercasePathComponent(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

std::filesystem::path findCaseInsensitiveFile(const std::filesystem::path& path) {
    if (std::filesystem::is_regular_file(path)) {
        return path;
    }
    std::filesystem::path resolved = path.root_path();
    for (const std::filesystem::path& component : path.relative_path()) {
        const std::filesystem::path exact = resolved / component;
        if (std::filesystem::exists(exact)) {
            resolved = exact;
            continue;
        }
        std::error_code error;
        if (!std::filesystem::is_directory(resolved, error) || error) {
            return {};
        }
        const std::string targetName = lowercasePathComponent(openbus::pathToUtf8(component));
        bool matched = false;
        for (std::filesystem::directory_iterator entry(resolved, error), end;
             !error && entry != end; entry.increment(error)) {
            if (lowercasePathComponent(openbus::pathToUtf8(entry->path().filename())) ==
                targetName) {
                resolved = entry->path();
                matched = true;
                break;
            }
        }
        if (!matched) {
            return {};
        }
    }
    return std::filesystem::is_regular_file(resolved) ? resolved : std::filesystem::path{};
}

std::filesystem::path findSceneryTexture(const std::filesystem::path& objectRoot,
                                         const std::filesystem::path& modelRoot,
                                         const std::filesystem::path& omsiRoot,
                                         const std::string& textureName) {
    return openbus::assets::findSceneryTexture(objectRoot, modelRoot, omsiRoot, textureName);
}

std::uint64_t tileKey(int x, int y) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32U) |
           static_cast<std::uint32_t>(y);
}

bool outsideMapFrustum(const ViewFrustum& frustum, const Matrix4& modelView,
                       const std::array<double, 3>& center, double radius) {
    return sphereOutsideFrustum(
        frustum, transformPoint(modelView, {center[0], center[1], center[2], 1.0}), radius);
}

std::filesystem::path resolveSplineProfile(const openbus::map::MapDefinition& map,
                                           const std::filesystem::path& omsiRoot,
                                           const std::string& assetPath) {
    const std::filesystem::path relative = normalizedAssetPath(assetPath);
    const std::array<std::filesystem::path, 2> candidates = {omsiRoot / relative,
                                                             map.rootPath / relative};
    for (const std::filesystem::path& candidate : candidates) {
        if (std::filesystem::is_regular_file(candidate)) {
            return candidate;
        }
    }
    return {};
}

Matrix4 makeSceneryInstanceTransform(double x, double y, double z,
                                     const std::array<double, 6>& rotationDegrees,
                                     std::size_t rotationCount,
                                     const std::array<double, 9>* orientationMatrix) {
    // Uses the shared matrix-stack operations so the result matches the
    // per-draw translate/rotate sequence this replaces.
    pushMatrix();
    setModelViewMatrix(
        {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    translate(x, y, z);
    if (orientationMatrix != nullptr) {
        const std::array<double, 9>& m = *orientationMatrix;
        multiplyMatrix({m[0], m[1], m[2], 0.0, m[3], m[4], m[5], 0.0, m[6], m[7], m[8], 0.0, 0.0,
                        0.0, 0.0, 1.0});
    } else {
        std::size_t ownRotationOffset = 0;
        if (rotationCount >= 6) {
            rotate(-rotationDegrees[0], 0.0, 0.0, 1.0);
            rotate(rotationDegrees[2], 0.0, 1.0, 0.0);
            rotate(rotationDegrees[1], 1.0, 0.0, 0.0);
            ownRotationOffset = 3;
        }
        rotate(-rotationDegrees[ownRotationOffset], 0.0, 0.0, 1.0);
        rotate(-rotationDegrees[ownRotationOffset + 2], 0.0, 1.0, 0.0);
        rotate(-rotationDegrees[ownRotationOffset + 1], 1.0, 0.0, 0.0);
    }
    const Matrix4 transform = modelViewMatrix();
    popMatrix();
    return transform;
}

} // namespace openbus::rendering::map_detail
