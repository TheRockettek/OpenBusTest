// Free helpers shared by the map renderer translation units.

#include "render/map/MapRendererInternal.h"

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
                   std::size_t count) {
    SkipSummary& summary = summaries[reason];
    summary.count += count;
    if (!example.empty() && summary.examples.size() < MAX_SKIP_EXAMPLES) {
        summary.examples.push_back(example);
        gameLog.Log("Map skip encountered: " + reason + "; example: " + example);
    }
}

void logSkipped(const SkipSummaries& summaries) {
    for (const auto& [reason, summary] : summaries) {
        std::string message =
            "Map skip summary: skipped " + std::to_string(summary.count) + " " + reason;
        if (!summary.examples.empty()) {
            message += " (context logged when encountered)";
        }
        gameLog.Log(message);
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
        const std::filesystem::path resolved =
            openbus::assets::resolveCaseInsensitivePath(candidate);
        if (std::filesystem::is_regular_file(resolved)) {
            return resolved;
        }
    }
    throw std::runtime_error("Could not resolve map ground texture: " + configuredPath);
}

GLuint loadTexture(const std::filesystem::path& path, bool repeat) {
    Image image;
    if (!TextureLoader::readImage(path, image)) {
        throw std::runtime_error("Could not decode map ground texture: " + path.string());
    }
    try {
        return uploadTextureImage(image, repeat);
    } catch (const std::exception& error) {
        throw std::runtime_error("Could not upload map ground texture " + path.string() + ": " +
                                 error.what());
    }
}

GLuint uploadTextureImage(const Image& image, bool repeat) {
    if (image.width <= 0 || image.height <= 0 ||
        image.rgba.size() <
            static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height) * 4U) {
        throw std::runtime_error("Texture image dimensions or pixel data are invalid");
    }
    GLuint texture = 0;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    const GLint wrapMode = repeat ? GL_REPEAT : GL_CLAMP_TO_EDGE;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapMode);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapMode);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.width, image.height, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 image.rgba.data());
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
        const std::string targetName = lowercasePathComponent(component.string());
        bool matched = false;
        for (std::filesystem::directory_iterator entry(resolved, error), end;
             !error && entry != end; entry.increment(error)) {
            if (lowercasePathComponent(entry->path().filename().string()) == targetName) {
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
    const std::filesystem::path relative = normalizedAssetPath(textureName);
    const std::filesystem::path packageRoot = objectRoot.parent_path();
    const std::filesystem::path modelPackageRoot = modelRoot.parent_path();
    const std::array<std::filesystem::path, 9> candidates = {
        objectRoot / "texture" / relative,
        objectRoot / relative,
        modelRoot / relative,
        packageRoot / "texture" / relative,
        packageRoot / relative,
        modelPackageRoot / "texture" / relative,
        modelPackageRoot / relative,
        omsiRoot / relative,
        omsiRoot / "Texture" / relative};
    for (const std::filesystem::path& candidate : candidates) {
        const std::filesystem::path exact = findCaseInsensitiveFile(candidate);
        if (!exact.empty()) {
            return exact;
        }
        std::string extension = lowercasePathComponent(candidate.extension().string());
        constexpr std::array<const char*, 6> textureExtensions = {".dds", ".bmp", ".tga",
                                                                  ".png", ".jpg", ".jpeg"};
        const bool supportedExtension =
            std::any_of(textureExtensions.begin(), textureExtensions.end(),
                        [&extension](const char* supported) { return extension == supported; });
        if (supportedExtension) {
            for (const char* alternativeExtension : textureExtensions) {
                if (extension == alternativeExtension) {
                    continue;
                }
                std::filesystem::path alternate = candidate;
                alternate.replace_extension(alternativeExtension);
                const std::filesystem::path alternateMatch = findCaseInsensitiveFile(alternate);
                if (!alternateMatch.empty()) {
                    return alternateMatch;
                }
            }
        }
    }
    return {};
}

std::uint64_t tileKey(int x, int y) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32U) |
           static_cast<std::uint32_t>(y);
}

double vertexBoundingRadius(const std::vector<ModelVertex>& vertices) {
    double radiusSquared = 0.0;
    for (const ModelVertex& vertex : vertices) {
        const double x = vertex.x;
        const double y = vertex.y;
        const double z = vertex.z;
        radiusSquared = std::max(radiusSquared, x * x + y * y + z * z);
    }
    return std::sqrt(radiusSquared);
}

bool outsideMapFrustum(const ViewFrustum& frustum, const Matrix4& modelView,
                       const std::array<double, 3>& center, double radius) {
    return sphereOutsideFrustum(
        frustum, transformPoint(modelView, {center[0], center[1], center[2], 1.0}), radius);
}

std::vector<ModelVertex> makeTreePreview(double height, double width) {
    // OMSI's SCO supplies a texture and height/ratio ranges; this preview uses
    // crossed cutout planes because its editor-only helper mesh is often .x.
    const float halfWidth = static_cast<float>(width * 0.5);
    const float top = static_cast<float>(height);
    constexpr float low = 0.0F;
    constexpr float high = 1.0F;
    const auto vertex = [](float x, float y, float z, float u, float v) {
        return ModelVertex{x, y, z, u, v, 0.0F, 0.0F, 0.0F, 1.0F};
    };
    const ModelVertex xLow = vertex(-halfWidth, 0.0F, low, 0.0F, 0.0F);
    const ModelVertex xHigh = vertex(halfWidth, 0.0F, low, 1.0F, 0.0F);
    const ModelVertex xTopHigh = vertex(halfWidth, 0.0F, top, 1.0F, high);
    const ModelVertex xTopLow = vertex(-halfWidth, 0.0F, top, 0.0F, high);
    const ModelVertex yLow = vertex(0.0F, -halfWidth, low, 0.0F, 0.0F);
    const ModelVertex yHigh = vertex(0.0F, halfWidth, low, 1.0F, 0.0F);
    const ModelVertex yTopHigh = vertex(0.0F, halfWidth, top, 1.0F, high);
    const ModelVertex yTopLow = vertex(0.0F, -halfWidth, top, 0.0F, high);
    return {xLow, xHigh, xTopHigh, xLow, xTopHigh, xTopLow,
            yLow, yHigh, yTopHigh, yLow, yTopHigh, yTopLow};
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

std::filesystem::path findSplineTexture(const std::filesystem::path& profilePath,
                                        const std::filesystem::path& omsiRoot,
                                        const std::string& textureName) {
    const std::filesystem::path relative = normalizedAssetPath(textureName);
    const std::filesystem::path profileRoot = profilePath.parent_path();
    const std::array<std::filesystem::path, 5> candidates = {
        profileRoot / "texture" / relative, profileRoot / relative, omsiRoot / "Texture" / relative,
        omsiRoot / "texture" / relative, omsiRoot / relative};
    for (const std::filesystem::path& candidate : candidates) {
        if (std::filesystem::is_regular_file(candidate)) {
            return candidate;
        }
    }
    return {};
}

void appendRoadProfile(RoadMeshGeometry& road, const openbus::map::MapSplinePlacement& spline,
                       const std::vector<openbus::map::MapSplineSample>& centerline,
                       const openbus::map::MapSplineProfileSection& section,
                       const openbus::map::MapRoadSplineJoins& joins,
                       std::optional<double> cantHalfWidth) {
    const openbus::map::MapRoadSectionGeometry geometry = openbus::map::buildMapRoadSectionGeometry(
        spline, centerline, section, joins, cantHalfWidth);
    const std::vector<std::array<double, 3>> normals =
        openbus::map::computeMapRoadVertexNormals(geometry);
    const std::uint32_t baseVertex = static_cast<std::uint32_t>(road.vertices.size());
    for (std::size_t vertexIndex = 0; vertexIndex < geometry.vertices.size(); ++vertexIndex) {
        const openbus::map::MapRoadVertex& vertex = geometry.vertices[vertexIndex];
        const std::array<double, 3>& normal = normals[vertexIndex];
        const ModelVertex generated{
            static_cast<float>(vertex.x),        static_cast<float>(vertex.y),
            static_cast<float>(vertex.z),        static_cast<float>(vertex.textureU),
            static_cast<float>(vertex.textureV), 0.0F,
            static_cast<float>(normal[0]),       static_cast<float>(normal[1]),
            static_cast<float>(normal[2])};
        road.minX = std::min(road.minX, vertex.x);
        road.minY = std::min(road.minY, vertex.y);
        road.maxX = std::max(road.maxX, vertex.x);
        road.maxY = std::max(road.maxY, vertex.y);
        road.minZ = std::min(road.minZ, vertex.z);
        road.maxZ = std::max(road.maxZ, vertex.z);
        road.vertices.push_back(generated);
    }
    for (const int index : geometry.indices) {
        road.indices.push_back(baseVertex + static_cast<std::uint32_t>(index));
    }
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
