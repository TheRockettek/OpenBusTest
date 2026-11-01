#include "ProbeTempDirectory.h"
#include "assets/O3DLoader.h"

#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void appendInteger(std::vector<std::uint8_t>& bytes, std::uint32_t value, int width) {
    for (int index = 0; index < width; ++index) {
        bytes.push_back(static_cast<std::uint8_t>(value >> (index * 8)));
    }
}

void appendFloat(std::vector<std::uint8_t>& bytes, float value) {
    appendInteger(bytes, std::bit_cast<std::uint32_t>(value), 4);
}

void appendString(std::vector<std::uint8_t>& bytes, const std::string& value) {
    bytes.push_back(static_cast<std::uint8_t>(value.size()));
    bytes.insert(bytes.end(), value.begin(), value.end());
}

std::vector<std::uint8_t> makeFixture(bool longIndices, std::uint32_t boneVertex = 2,
                                      float boneWeight = 0.25F) {
    std::vector<std::uint8_t> bytes = {0x84, 0x19,
                                       static_cast<std::uint8_t>(longIndices ? 4 : 3)};
    if (longIndices) {
        bytes.push_back(1);
        appendInteger(bytes, 0xffffffffU, 4);
    }
    bytes.push_back(0x17);
    appendInteger(bytes, 3, longIndices ? 4 : 2);
    for (int vertex = 0; vertex < 3; ++vertex) {
        for (int component = 0; component < 8; ++component) {
            const int normalComponent = vertex == 0 ? 4 : vertex == 1 ? 3 : 5;
            appendFloat(bytes, component == 0 ? static_cast<float>(vertex)
                                             : component == normalComponent ? 1.0F : 0.0F);
        }
    }
    bytes.push_back(0x49);
    appendInteger(bytes, 1, longIndices ? 4 : 2);
    for (std::uint32_t vertex = 0; vertex < 3; ++vertex) {
        appendInteger(bytes, vertex, longIndices ? 4 : 2);
    }
    appendInteger(bytes, 0, 2);
    bytes.push_back(0x54);
    appendInteger(bytes, 2, 2);
    appendString(bytes, "Head");
    appendInteger(bytes, 3, 2);
    appendInteger(bytes, boneVertex, longIndices ? 4 : 2);
    appendFloat(bytes, boneWeight);
    appendInteger(bytes, boneVertex, longIndices ? 4 : 2);
    appendFloat(bytes, boneWeight);
    appendInteger(bytes, 0, longIndices ? 4 : 2);
    appendFloat(bytes, 0.0F);
    appendString(bytes, "Main");
    appendInteger(bytes, 1, 2);
    appendInteger(bytes, 1, longIndices ? 4 : 2);
    appendFloat(bytes, 1.0F);
    return bytes;
}

void writeFixture(const std::filesystem::path& path, const std::vector<std::uint8_t>& bytes) {
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(bytes.data()),
                 static_cast<std::streamsize>(bytes.size()));
    require(static_cast<bool>(output), "could not write O3D bone fixture");
}

} // namespace

int main() {
    const auto root = std::filesystem::temp_directory_path() /
                      ("openbus-o3d-bones-" +
                       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
                       "-" + std::to_string(std::random_device{}()));
    if (!std::filesystem::create_directory(root)) {
        std::cerr << "could not create O3D bone probe directory\n";
        return 1;
    }
    int result = 0;
    try {
        for (const bool longIndices : {false, true}) {
            const auto path = root / (longIndices ? "long.o3d" : "short.o3d");
            writeFixture(path, makeFixture(longIndices));
            const auto mesh = openbus::rendering::O3DLoader::parse(path);
            require(mesh && mesh->bones.size() == 2, "O3D bone list was not retained");
            require(mesh->normals.size() == 3 && mesh->normals[0].x == 0.0 &&
                        mesh->normals[0].y == 0.0 && mesh->normals[0].z == 1.0 &&
                        mesh->normals[1].x == 1.0 && mesh->normals[1].y == 0.0 &&
                        mesh->normals[1].z == 0.0 && mesh->normals[2].x == 0.0 &&
                        mesh->normals[2].y == 1.0 && mesh->normals[2].z == 0.0,
                    "O3D normal axes or direction were changed");
            const auto objPath = root / "converted.obj";
            {
                std::ofstream obj(objPath);
                obj << "# openbus_transform 1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1\n"
                       "v 0 0 0\nv 1 0 0\nv 0 0 1\n"
                       "vn 0 1 0\nvn 1 0 0\nvn 0 0 1\n"
                       "f 1//1 2//1 3//1\nf 1//1 2//1 3//1\n";
                require(static_cast<bool>(obj), "could not write converted OBJ fixture");
            }
            const auto converted = openbus::rendering::ObjLoader::parse(objPath);
            require(converted && converted->normals.size() == mesh->normals.size(),
                    "converted OBJ normals were not retained");
            for (std::size_t index = 0; index < mesh->normals.size(); ++index) {
                require(converted->normals[index].x == mesh->normals[index].x &&
                            converted->normals[index].y == mesh->normals[index].y &&
                            converted->normals[index].z == mesh->normals[index].z,
                        "converted OBJ and O3D normals did not agree");
            }
            require(converted->backFaceCulling && converted->triangles.size() == 2 &&
                        converted->triangles[0].indices[1].position == 3 &&
                        converted->triangles[0].indices[2].position == 2,
                    "converted OBJ mirrored-winding behavior changed");
            require(mesh->bones[0].name == "Head" && mesh->bones[0].weights.size() == 3 &&
                        mesh->bones[0].weights[0].sourceIndex == 2 &&
                        mesh->bones[0].weights[0].weight == 0.25F &&
                        mesh->bones[0].weights[1].sourceIndex == 2 &&
                        mesh->bones[0].weights[1].weight == 0.25F &&
                        mesh->bones[0].weights[2].sourceIndex == 0 &&
                        mesh->bones[0].weights[2].weight == 0.0F &&
                        mesh->bones[1].name == "Main" &&
                        mesh->bones[1].weights[0].sourceIndex == 1 &&
                        mesh->bones[1].weights[0].weight == 1.0F,
                    "O3D bone names, raw indices or weights were changed");
            writeFixture(path, makeFixture(longIndices, 3));
            const auto externalIndex = openbus::rendering::O3DLoader::parse(path);
            require(externalIndex && externalIndex->bones[0].weights[0].sourceIndex == 3,
                    "bone index outside the decoded vertex list was not preserved");
            writeFixture(path, makeFixture(longIndices, 2, -0.5F));
            const auto negativeWeight = openbus::rendering::O3DLoader::parse(path);
            require(negativeWeight && negativeWeight->bones[0].weights[0].weight == -0.5F,
                    "unusual bone weight was not preserved");
            writeFixture(path, makeFixture(longIndices, 2,
                                           std::numeric_limits<float>::quiet_NaN()));
            const auto nanWeight = openbus::rendering::O3DLoader::parse(path);
            require(nanWeight && std::isnan(nanWeight->bones[0].weights[0].weight),
                    "non-finite bone weight was not preserved");
            auto truncated = makeFixture(longIndices);
            truncated.pop_back();
            writeFixture(path, truncated);
            require(!openbus::rendering::O3DLoader::parse(path),
                    "truncated bone weight was accepted");
        }
    } catch (const std::exception& error) {
        std::cerr << "O3D bone probe failed: " << error.what() << '\n';
        result = 1;
    }
    openbus::probe::removeProbeDirectory(root);
    return result;
}
