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

    std::size_t remaining() const {
        return data_.size() - offset_;
    }

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

struct O3DVertexDecodeState {
    bool enabled = false;
    bool alternateSeed = false;
    std::uint32_t productId = 0;
    std::uint16_t productIdSalt = 0;
    std::uint16_t vertexCount = 0;
    std::uint8_t salt = 0;
};

O3DVertexDecodeState makeVertexDecodeState(std::uint8_t version, std::uint8_t options,
                                           std::uint32_t encryptionKey, std::uint32_t vertexCount) {
    O3DVertexDecodeState state;
    if (version <= 3 || encryptionKey == 0xFFFFFFFFu || encryptionKey == 0xFFFFu) {
        return state;
    }

    state.enabled = true;
    state.alternateSeed = (options & 0x2u) != 0;
    state.productId = encryptionKey;
    state.vertexCount = static_cast<std::uint16_t>(vertexCount % 65000u);
    state.productIdSalt = static_cast<std::uint16_t>(encryptionKey + version - 4u);
    state.productIdSalt = static_cast<std::uint16_t>(
        (static_cast<std::uint32_t>(state.productIdSalt) + (state.alternateSeed ? 381u : 0u)) %
        65000u);
    return state;
}

void mixVertexSalt(O3DVertexDecodeState& state) {
    const std::uint32_t mixed =
        (static_cast<std::uint32_t>(state.salt) * state.vertexCount +
         static_cast<std::uint32_t>(state.vertexCount) * state.productIdSalt) %
        8000u;
    state.productIdSalt = static_cast<std::uint16_t>(mixed);
    state.salt = static_cast<std::uint8_t>(mixed / 8000u);
}

void decodeVertex(O3DVertexDecodeState& state, float& x, float& y, float& z, float& normalX,
                  float& normalY, float& normalZ, float& u, float& v) {
    if (!state.enabled) {
        return;
    }

    if (state.productId == 0u) {
        state.productIdSalt = state.alternateSeed ? 304u : 0u;
    }
    mixVertexSalt(state);

    float integral = 0.0f;
    const float fractionalX = std::modf(x, &integral);
    const float fractionalY = std::modf(y, &integral);
    const float fractionalZ = std::modf(z, &integral);
    const float fractionalProduct = std::fabs(fractionalX * fractionalY * fractionalZ) * 600.0f;
    state.salt = static_cast<std::uint8_t>(static_cast<unsigned int>(fractionalProduct));

    if (state.productIdSalt >= 1000u) {
        if (state.productIdSalt < 3000u) {
            std::swap(x, z);
        } else if (state.productIdSalt > 7000u) {
            std::swap(y, z);
        }
    } else {
        std::swap(x, y);
    }

    if ((state.productIdSalt & 3u) == 0u) {
        normalX = -normalX;
    }
    if ((state.productIdSalt % 6u) == 0u) {
        normalY = -normalY;
    }
    if ((state.productIdSalt % 7u) == 0u) {
        normalZ = -normalZ;
    }
    if (state.productIdSalt >= 600u) {
        if (state.productIdSalt > 4500u) {
            std::swap(normalX, normalY);
        }
    } else {
        std::swap(normalY, normalZ);
    }

    if ((state.productIdSalt % 5u) == 0u) {
        const float offset = static_cast<float>(state.productIdSalt % 0x64u);
        u -= offset * offset / 10000.0f;
    }
    if ((state.productIdSalt % 3u) == 0u) {
        const float offset = static_cast<float>(state.productIdSalt % 0x32u);
        v -= offset * offset / 2500.0f;
    }
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
        O3DVertexDecodeState vertexDecodeState;
        if (longHeader) {
            const std::uint8_t options = reader.u8();
            const std::uint32_t encryptionKey = reader.u32();
            longTriangleIndices = (options & 0x1u) != 0;
            vertexDecodeState = makeVertexDecodeState(version, options, encryptionKey, 0);
        }

        auto result = std::make_shared<ParsedObj>();
        while (reader.remaining() > 0) {
            switch (reader.u8()) {
            case VertexList: {
                const std::uint32_t count = readCount(reader, longHeader);
                if (longHeader && vertexDecodeState.enabled) {
                    vertexDecodeState.vertexCount = static_cast<std::uint16_t>(count % 65000u);
                }
                result->positions.reserve(count);
                result->normals.reserve(count);
                result->texCoords.reserve(count);
                for (std::uint32_t index = 0; index < count; ++index) {
                    float x = reader.f32();
                    float y = reader.f32();
                    float z = reader.f32();
                    float normalX = reader.f32();
                    float normalY = reader.f32();
                    float normalZ = reader.f32();
                    float u = reader.f32();
                    float v = reader.f32();
                    decodeVertex(vertexDecodeState, x, y, z, normalX, normalY, normalZ, u, v);
                    result->positions.push_back({x, z, y});
                    result->normals.push_back({-normalX, -normalZ, -normalY});
                    result->texCoords.push_back({u, 1.0 - v});
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
                    const int firstIndex = static_cast<int>(first + 1);
                    const int secondIndex = static_cast<int>(second + 1);
                    const int thirdIndex = static_cast<int>(third + 1);
                    ObjTriangle triangle;
                    triangle.indices = {{{firstIndex, firstIndex, firstIndex},
                                         {secondIndex, secondIndex, secondIndex},
                                         {thirdIndex, thirdIndex, thirdIndex}}};
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
                    result->materials.emplace("matl_" + std::to_string(index), std::move(material));
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
        for (const ObjTriangle& triangle : result->triangles) {
            for (const ObjIndex& index : triangle.indices) {
                if (index.position <= 0 ||
                    index.position > static_cast<int>(result->positions.size())) {
                    return {};
                }
            }
        }
        result->backFaceCulling = determinant(result->transform) < 0.0;
        calculateBounds(*result);
        return result;
    } catch (const std::exception&) {
        return {};
    }
}

} // namespace openbus::rendering