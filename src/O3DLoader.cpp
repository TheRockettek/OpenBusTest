#include "O3DLoader.h"

#include "PerfTrace.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace openbus::rendering {
namespace {

constexpr std::uint8_t VertexList = 0x17;
constexpr std::uint8_t TriangleList = 0x49;
constexpr std::uint8_t MaterialList = 0x26;
constexpr std::uint8_t BoneList = 0x54;
constexpr std::uint8_t Transform = 0x79;

class Reader {
  public:
    explicit Reader(std::vector<std::uint8_t> data) : data_(std::move(data)) {}

    std::size_t remaining() const { return data_.size() - offset_; }

    std::uint8_t u8() {
        ensure(1);
        return data_[offset_++];
    }

    std::uint16_t u16() {
        ensure(2);
        const std::uint16_t value = static_cast<std::uint16_t>(data_[offset_]) |
                                    (static_cast<std::uint16_t>(data_[offset_ + 1]) << 8);
        offset_ += 2;
        return value;
    }

    std::uint32_t u32() {
        ensure(4);
        const std::uint32_t value = static_cast<std::uint32_t>(data_[offset_]) |
                                    (static_cast<std::uint32_t>(data_[offset_ + 1]) << 8) |
                                    (static_cast<std::uint32_t>(data_[offset_ + 2]) << 16) |
                                    (static_cast<std::uint32_t>(data_[offset_ + 3]) << 24);
        offset_ += 4;
        return value;
    }

    float f32() {
        const std::uint32_t raw = u32();
        float value = 0.0f;
        std::memcpy(&value, &raw, sizeof(value));
        return value;
    }

    std::string string() {
        const std::uint8_t length = u8();
        ensure(length);
        const std::string value(reinterpret_cast<const char*>(data_.data() + offset_), length);
        offset_ += length;
        return value;
    }

  private:
    void ensure(std::size_t length) const {
        if (length > data_.size() - offset_) {
            throw std::runtime_error("Unexpected end of O3D file");
        }
    }

    std::vector<std::uint8_t> data_;
    std::size_t offset_ = 0;
};

std::uint32_t readCount(Reader& reader, bool longHeader) {
    return longHeader ? reader.u32() : static_cast<std::uint32_t>(reader.u16());
}

std::array<double, 16> convertTransform(const std::array<float, 16>& source) {
    std::array<double, 16> result = {};
    const auto sourceAxis = [](int axis) { return axis == 1 ? 2 : axis == 2 ? 1 : axis; };
    for (int column = 0; column < 3; ++column) {
        for (int row = 0; row < 3; ++row) {
            result[row + column * 4] = source[sourceAxis(row) + sourceAxis(column) * 4];
        }
    }
    result[3] = source[3];
    result[7] = source[7];
    result[11] = source[11];
    result[12] = source[12];
    result[13] = source[14];
    result[14] = source[13];
    result[15] = source[15];
    return result;
}

double determinant(const std::array<double, 16>& transform) {
    const double a = transform[0];
    const double b = transform[4];
    const double c = transform[8];
    const double d = transform[1];
    const double e = transform[5];
    const double f = transform[9];
    const double g = transform[2];
    const double h = transform[6];
    const double i = transform[10];
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
}

void calculateBounds(ParsedObj& result) {
    std::array<double, 3> minimum = {std::numeric_limits<double>::max(),
                                      std::numeric_limits<double>::max(),
                                      std::numeric_limits<double>::max()};
    std::array<double, 3> maximum = {std::numeric_limits<double>::lowest(),
                                      std::numeric_limits<double>::lowest(),
                                      std::numeric_limits<double>::lowest()};
    for (const ObjPosition& position : result.positions) {
        const std::array<double, 3> converted = {position.y, -position.x, position.z};
        for (int axis = 0; axis < 3; ++axis) {
            minimum[axis] = std::min(minimum[axis], converted[axis]);
            maximum[axis] = std::max(maximum[axis], converted[axis]);
        }
    }
    for (int axis = 0; axis < 3; ++axis) {
        result.boundsCenter[axis] = (minimum[axis] + maximum[axis]) * 0.5;
        result.boundsSize[axis] = maximum[axis] - minimum[axis];
    }
    for (const ObjPosition& position : result.positions) {
        const double x = position.y - result.boundsCenter[0];
        const double y = -position.x - result.boundsCenter[1];
        const double z = position.z - result.boundsCenter[2];
        result.boundsRadius = std::max(result.boundsRadius, std::sqrt(x * x + y * y + z * z));
    }
}

} // namespace

std::shared_ptr<ParsedObj> O3DLoader::parse(const std::filesystem::path& path) {
    TraceScope trace("o3d", "parseO3D");
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        return {};
    }
    std::vector<std::uint8_t> data((std::istreambuf_iterator<char>(input)),
                                   std::istreambuf_iterator<char>());
    if (data.size() < 3) {
        return {};
    }

    try {
        Reader reader(std::move(data));
        if (reader.u8() != 0x84 || reader.u8() != 0x19) {
            return {};
        }
        const std::uint8_t version = reader.u8();
        const bool longHeader = version > 3;
        bool longTriangleIndices = false;
        if (longHeader) {
            const std::uint8_t options = reader.u8();
            const std::uint32_t encryptionKey = reader.u32();
            longTriangleIndices = (options & 0x1u) != 0;
            if (encryptionKey != 0xFFFFFFFFu) {
                return {};
            }
        }

        auto result = std::make_shared<ParsedObj>();
        while (reader.remaining() > 0) {
            switch (reader.u8()) {
            case VertexList: {
                const std::uint32_t count = readCount(reader, longHeader);
                result->positions.reserve(count);
                result->normals.reserve(count);
                result->texCoords.reserve(count);
                for (std::uint32_t index = 0; index < count; ++index) {
                    const double x = reader.f32();
                    const double y = reader.f32();
                    const double z = reader.f32();
                    result->positions.push_back({x, z, y});
                    const double normalX = reader.f32();
                    const double normalY = reader.f32();
                    const double normalZ = reader.f32();
                    result->normals.push_back({-normalX, -normalZ, -normalY});
                    result->texCoords.push_back({reader.f32(), 1.0 - reader.f32()});
                }
                break;
            }
            case TriangleList: {
                const std::uint32_t count = readCount(reader, longHeader);
                result->triangles.reserve(count);
                for (std::uint32_t index = 0; index < count; ++index) {
                    const auto readIndex = [&] {
                        return longTriangleIndices ? reader.u32() : reader.u16();
                    };
                    const std::uint32_t first = readIndex();
                    const std::uint32_t second = readIndex();
                    const std::uint32_t third = readIndex();
                    const std::uint16_t material = reader.u16();
                                        ObjTriangle triangle;
                                        triangle.indices = {{{static_cast<int>(first + 1), static_cast<int>(first + 1),
                                                                                    static_cast<int>(first + 1)},
                                                                                 {static_cast<int>(second + 1),
                                                                                    static_cast<int>(second + 1),
                                                                                    static_cast<int>(second + 1)},
                                                                                 {static_cast<int>(third + 1), static_cast<int>(third + 1),
                                                                                    static_cast<int>(third + 1)}}};
                                        triangle.material = "matl_" + std::to_string(material);
                                        result->triangles.push_back(std::move(triangle));
                }
                break;
            }
            case MaterialList: {
                const std::uint16_t count = reader.u16();
                for (std::uint16_t index = 0; index < count; ++index) {
                    ObjMaterial material;
                    material.materialIndex = index;
                    material.color = {reader.f32(), reader.f32(), reader.f32()};
                    material.alpha = reader.f32();
                    material.specular = {reader.f32(), reader.f32(), reader.f32()};
                    material.emission = {reader.f32(), reader.f32(), reader.f32()};
                    material.specularPower = reader.f32();
                    material.textureName = reader.string();
                    result->materials.emplace("matl_" + std::to_string(index),
                                              std::move(material));
                }
                break;
            }
            case BoneList: {
                const std::uint16_t count = reader.u16();
                for (std::uint16_t index = 0; index < count; ++index) {
                    reader.string();
                    const std::uint16_t weights = reader.u16();
                    for (std::uint16_t weight = 0; weight < weights; ++weight) {
                        if (longTriangleIndices) {
                            reader.u32();
                        } else {
                            reader.u16();
                        }
                        reader.f32();
                    }
                }
                break;
            }
            case Transform: {
                std::array<float, 16> transform = {};
                for (float& value : transform) {
                    value = reader.f32();
                }
                result->transform = convertTransform(transform);
                result->hasTransform = true;
                break;
            }
            default:
                return {};
            }
        }
        if (result->triangles.empty() || result->positions.empty()) {
            return {};
        }
        result->backFaceCulling = determinant(result->transform) < 0.0;
        calculateBounds(*result);
        return result;
    } catch (const std::exception&) {
        return {};
    }
}

} // namespace openbus::rendering