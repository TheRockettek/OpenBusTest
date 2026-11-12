#include "assets/O3DLoader.h"
#include "core/Environment.h"
#include "core/Utf8Path.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace {

void appendU16(std::vector<std::uint8_t>& data, std::uint16_t value) {
    data.push_back(static_cast<std::uint8_t>(value & 0xFFu));
    data.push_back(static_cast<std::uint8_t>(value >> 8u));
}

void appendFloat(std::vector<std::uint8_t>& data, float value) {
    std::uint32_t raw = 0;
    std::memcpy(&raw, &value, sizeof(raw));
    for (unsigned int shift = 0; shift < 32; shift += 8) {
        data.push_back(static_cast<std::uint8_t>(raw >> shift));
    }
}

void appendString(std::vector<std::uint8_t>& data, const std::string& value) {
    data.push_back(static_cast<std::uint8_t>(value.size()));
    data.insert(data.end(), value.begin(), value.end());
}

bool testO3DTextureNameEncoding() {
    std::vector<std::uint8_t> data = {0x84, 0x19, 0x03, 0x49};
    appendU16(data, 1);
    appendU16(data, 0);
    appendU16(data, 1);
    appendU16(data, 2);
    appendU16(data, 0);
    data.push_back(0x17);
    appendU16(data, 3);
    for (const std::array<float, 3>& position :
         {std::array<float, 3>{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f},
          {0.0f, 1.0f, 0.0f}}) {
        for (const float value : {position[0], position[1], position[2], 0.0f, 1.0f, 0.0f,
                                  0.0f, 0.0f}) {
            appendFloat(data, value);
        }
    }
    data.push_back(0x26);
    appendU16(data, 2);
    std::string legacyTextureName = "Ww_ge-A9-Rebu-La-DrMaul_li-B13-W";
    legacyTextureName.push_back(static_cast<char>(0xFC));
    legacyTextureName += "-An-EI-ND-Klinikum.png";
    std::string utf8TextureName = "sign-";
    utf8TextureName += "\xC3\xBC";
    utf8TextureName += ".png";
    for (const std::string& textureName : {legacyTextureName, utf8TextureName}) {
        for (int field = 0; field < 11; ++field) {
            appendFloat(data, 0.0f);
        }
        appendString(data, textureName);
    }

    const auto fixturePath =
        std::filesystem::temp_directory_path() /
        ("OpenBusO3DUnicodeProbe_" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".o3d");
    struct TemporaryFile {
        std::filesystem::path path;
        ~TemporaryFile() {
            std::error_code error;
            std::filesystem::remove(path, error);
        }
    } cleanup{fixturePath};
    {
        std::ofstream output(fixturePath, std::ios::binary);
        if (!output) {
            return false;
        }
        output.write(reinterpret_cast<const char*>(data.data()),
                     static_cast<std::streamsize>(data.size()));
        if (!output) {
            return false;
        }
    }
    const auto parsed = openbus::rendering::O3DLoader::parse(fixturePath);
    const std::string expectedLegacyTexture =
        "Ww_ge-A9-Rebu-La-DrMaul_li-B13-W\xC3\xBC-An-EI-ND-Klinikum.png";
    return parsed && parsed->materials.at("matl_0").textureName == expectedLegacyTexture &&
           openbus::pathToUtf8(openbus::pathFromUtf8(parsed->materials.at("matl_0").textureName)) ==
               expectedLegacyTexture &&
           parsed->materials.at("matl_1").textureName == "sign-\xC3\xBC.png";
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "expected an O3D path\n";
        return 1;
    }
    if (openbus::rendering::convertO3DPositionToMapAxes(1.0, 0.0, 0.0) !=
            std::array<double, 3>{1.0, 0.0, 0.0} ||
        openbus::rendering::convertO3DPositionToMapAxes(0.0, 1.0, 0.0) !=
            std::array<double, 3>{0.0, 0.0, 1.0} ||
        openbus::rendering::convertO3DPositionToMapAxes(0.0, 0.0, 1.0) !=
            std::array<double, 3>{0.0, 1.0, 0.0}) {
        std::cerr << "O3D-to-map axis conversion is incorrect\n";
        return 1;
    }
    if (openbus::rendering::convertO3DMapPositionToVehicleAxes(1.0, 0.0, 0.0) !=
            std::array<double, 3>{0.0, -1.0, 0.0} ||
        openbus::rendering::convertO3DMapPositionToVehicleAxes(0.0, 1.0, 0.0) !=
            std::array<double, 3>{1.0, 0.0, 0.0} ||
        openbus::rendering::convertO3DMapPositionToVehicleAxes(0.0, 0.0, 1.0) !=
            std::array<double, 3>{0.0, 0.0, 1.0}) {
        std::cerr << "O3D map-to-vehicle axis conversion is incorrect\n";
        return 1;
    }
    const auto parsed = openbus::rendering::O3DLoader::parse(argv[1]);
    if (!parsed || parsed->positions.empty() || parsed->triangles.empty() ||
        parsed->materials.empty()) {
        std::cerr << "failed to parse O3D fixture\n";
        return 1;
    }
    if (!testO3DTextureNameEncoding()) {
        std::cerr << "O3D texture names were not normalized to UTF-8\n";
        return 1;
    }
    std::array<double, 3> boundsMinimum = {std::numeric_limits<double>::max(),
                                           std::numeric_limits<double>::max(),
                                           std::numeric_limits<double>::max()};
    std::array<double, 3> boundsMaximum = {std::numeric_limits<double>::lowest(),
                                           std::numeric_limits<double>::lowest(),
                                           std::numeric_limits<double>::lowest()};
    std::vector<std::array<double, 3>> vehiclePositions;
    vehiclePositions.reserve(parsed->positions.size());
    for (const auto& position : parsed->positions) {
        const auto vehiclePosition = openbus::rendering::convertO3DMapPositionToVehicleAxes(
            position.x, position.y, position.z);
        vehiclePositions.push_back(vehiclePosition);
        for (std::size_t axis = 0; axis < vehiclePosition.size(); ++axis) {
            boundsMinimum[axis] = std::min(boundsMinimum[axis], vehiclePosition[axis]);
            boundsMaximum[axis] = std::max(boundsMaximum[axis], vehiclePosition[axis]);
        }
    }
    for (std::size_t axis = 0; axis < boundsMinimum.size(); ++axis) {
        const double expectedCenter = (boundsMinimum[axis] + boundsMaximum[axis]) * 0.5;
        const double expectedSize = boundsMaximum[axis] - boundsMinimum[axis];
        if (std::abs(parsed->boundsCenter[axis] - expectedCenter) > 1.0e-8 ||
            std::abs(parsed->boundsSize[axis] - expectedSize) > 1.0e-8) {
            std::cerr << "O3D culling bounds do not enclose vertices in vehicle render axes\n";
            return 1;
        }
    }
    double expectedRadius = 0.0;
    for (const auto& position : vehiclePositions) {
        const double x = position[0] - parsed->boundsCenter[0];
        const double y = position[1] - parsed->boundsCenter[1];
        const double z = position[2] - parsed->boundsCenter[2];
        expectedRadius = std::max(expectedRadius, std::sqrt(x * x + y * y + z * z));
    }
    if (std::abs(parsed->boundsRadius - expectedRadius) > 1.0e-8) {
        std::cerr << "O3D culling sphere is not centered in vehicle render axes\n";
        return 1;
    }
    for (const auto& triangle : parsed->triangles) {
        for (const auto& index : triangle.indices) {
            if (index.position <= 0 || index.position != index.texCoord ||
                index.position != index.normal) {
                std::cerr << "O3D vertex index mapping is inconsistent\n";
                return 1;
            }
        }
    }
    if (openbus::getEnvironment("OPENBUS_O3D_PROBE_VERBOSE") != nullptr) {
        double minimumEdge = std::numeric_limits<double>::max();
        double maximumEdge = 0.0;
        std::vector<std::pair<double, std::size_t>> longestEdges;
        std::map<std::string, std::size_t> trianglesByMaterial;
        for (const auto& triangle : parsed->triangles) {
            ++trianglesByMaterial[triangle.material];
            const auto distance = [&](int first, int second) {
                const auto& left = parsed->positions[static_cast<std::size_t>(first - 1)];
                const auto& right = parsed->positions[static_cast<std::size_t>(second - 1)];
                const double x = left.x - right.x;
                const double y = left.y - right.y;
                const double z = left.z - right.z;
                return std::sqrt(x * x + y * y + z * z);
            };
            for (int edge = 0; edge < 3; ++edge) {
                const double length =
                    distance(triangle.indices[edge].position,
                             triangle.indices[(edge + 1) % 3].position);
                minimumEdge = std::min(minimumEdge, length);
                maximumEdge = std::max(maximumEdge, length);
            }
            longestEdges.push_back(
                {std::max({distance(triangle.indices[0].position, triangle.indices[1].position),
                           distance(triangle.indices[1].position, triangle.indices[2].position),
                           distance(triangle.indices[2].position, triangle.indices[0].position)}),
                 static_cast<std::size_t>(&triangle - parsed->triangles.data())});
        }
        std::sort(longestEdges.begin(), longestEdges.end(),
                  [](const auto& left, const auto& right) { return left.first > right.first; });
        std::cout << "positions=" << parsed->positions.size()
                  << " triangles=" << parsed->triangles.size() << " boundsSize="
                  << parsed->boundsSize[0] << ',' << parsed->boundsSize[1] << ','
                  << parsed->boundsSize[2] << " boundsCenter=" << parsed->boundsCenter[0] << ','
                  << parsed->boundsCenter[1] << ',' << parsed->boundsCenter[2]
                  << " edgeRange=" << minimumEdge << ','
                  << maximumEdge << '\n';
        for (const auto& [material, triangleCount] : trianglesByMaterial) {
            const auto sourceMaterial = parsed->materials.find(material);
            std::cout << "material=" << material << " triangles=" << triangleCount
                      << " texture=" << (sourceMaterial == parsed->materials.end()
                                              ? std::string{}
                                              : sourceMaterial->second.textureName)
                      << '\n';
        }
        for (std::size_t index = 0; index < std::min<std::size_t>(10, parsed->triangles.size());
             ++index) {
            const auto& triangle = parsed->triangles[index];
            std::cout << triangle.indices[0].position << ',' << triangle.indices[1].position
                      << ',' << triangle.indices[2].position << '\n';
        }
        for (std::size_t index = 0; index < std::min<std::size_t>(5, longestEdges.size()); ++index) {
            const auto& [length, triangleIndex] = longestEdges[index];
            const auto& triangle = parsed->triangles[triangleIndex];
            std::cout << "longest " << length << " triangle=" << triangleIndex << " indices="
                      << triangle.indices[0].position << ',' << triangle.indices[1].position << ','
                      << triangle.indices[2].position << '\n';
            for (const auto& vertex : triangle.indices) {
                const auto& position =
                    parsed->positions[static_cast<std::size_t>(vertex.position - 1)];
                std::cout << "  vertex " << vertex.position << "=" << position.x << ','
                          << position.y << ',' << position.z << '\n';
            }
        }
    }
    return 0;
}