#include "XLoader.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <vector>

#include <zlib.h>

using namespace openbus::rendering;

static bool checkBounds(const ParsedObj& mesh) {
    if (!(mesh.boundsRadius > 0.0)) {
        std::cerr << "X mesh has no culling radius\n";
        return false;
    }
    std::array<double, 3> minimum = {
        std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max()};
    std::array<double, 3> maximum = {
        std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest()};
    double radius = 0.0;
    for (const ObjPosition& position : mesh.positions) {
        const std::array<double, 3> vehiclePosition = {position.y, -position.x, position.z};
        double distanceSquared = 0.0;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            minimum[axis] = std::min(minimum[axis], vehiclePosition[axis]);
            maximum[axis] = std::max(maximum[axis], vehiclePosition[axis]);
            const double delta = vehiclePosition[axis] - mesh.boundsCenter[axis];
            distanceSquared += delta * delta;
        }
        radius = std::max(radius, std::sqrt(distanceSquared));
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (std::abs(mesh.boundsCenter[axis] - (minimum[axis] + maximum[axis]) * 0.5) > 1.0e-8 ||
            std::abs(mesh.boundsSize[axis] - (maximum[axis] - minimum[axis])) > 1.0e-8) {
            std::cerr << "X bounds must enclose transformed vertices in vehicle axes\n";
            return false;
        }
    }
    if (std::abs(mesh.boundsRadius - radius) > 1.0e-8) {
        std::cerr << "X culling sphere does not enclose its transformed vertices\n";
        return false;
    }
    return true;
}

namespace {

void appendU16(std::vector<std::uint8_t>& bytes, std::uint16_t value) {
    bytes.push_back(static_cast<std::uint8_t>(value & 0xffU));
    bytes.push_back(static_cast<std::uint8_t>((value >> 8) & 0xffU));
}

void appendU32(std::vector<std::uint8_t>& bytes, std::uint32_t value) {
    for (std::size_t byte = 0; byte < 4; ++byte) {
        bytes.push_back(static_cast<std::uint8_t>((value >> (byte * 8)) & 0xffU));
    }
}

void appendToken(std::vector<std::uint8_t>& bytes, std::uint16_t code) {
    appendU16(bytes, code);
}

void appendInteger(std::vector<std::uint8_t>& bytes, std::int32_t value) {
    appendToken(bytes, 3);
    appendU32(bytes, static_cast<std::uint32_t>(value));
}

void appendName(std::vector<std::uint8_t>& bytes, const std::string& value) {
    appendToken(bytes, 1);
    appendU32(bytes, static_cast<std::uint32_t>(value.size()));
    bytes.insert(bytes.end(), value.begin(), value.end());
}

void appendFloatList(std::vector<std::uint8_t>& bytes,
                     const std::vector<float>& values) {
    appendToken(bytes, 7);
    appendU32(bytes, static_cast<std::uint32_t>(values.size()));
    for (const float value : values) {
        appendU32(bytes, std::bit_cast<std::uint32_t>(value));
    }
}

void appendIntegerList(std::vector<std::uint8_t>& bytes,
                       const std::vector<std::uint32_t>& values) {
    appendToken(bytes, 6);
    appendU32(bytes, static_cast<std::uint32_t>(values.size()));
    for (const std::uint32_t value : values) {
        appendU32(bytes, value);
    }
}

std::vector<std::uint8_t> makeCompressedX(std::string_view format, std::string_view payload) {
    std::vector<std::uint8_t> file = {'x', 'o', 'f', ' ', '0', '3', '0', '3'};
    file.insert(file.end(), format.begin(), format.end());
    file.insert(file.end(), {'0', '0', '3', '2'});
    file.insert(file.end(), 6, 0); // Reserved checksum/flags header.
    std::string previousBlock;
    for (std::size_t start = 0; start < payload.size(); start += 32768) {
        const std::string_view block =
            payload.substr(start, (std::min<std::size_t>)(32768, payload.size() - start));
        z_stream stream = {};
        if (deflateInit2(&stream, Z_DEFAULT_COMPRESSION, Z_DEFLATED, -MAX_WBITS, 8,
                         Z_DEFAULT_STRATEGY) != Z_OK) {
            throw std::runtime_error("unable to initialize compressed X fixture writer");
        }
        if (!previousBlock.empty() &&
            deflateSetDictionary(&stream,
                                 reinterpret_cast<const Bytef*>(previousBlock.data()),
                                 static_cast<uInt>(previousBlock.size())) != Z_OK) {
            deflateEnd(&stream);
            throw std::runtime_error("unable to set compressed X fixture dictionary");
        }
        std::vector<std::uint8_t> compressed(deflateBound(&stream, block.size()));
        stream.next_in = reinterpret_cast<Bytef*>(const_cast<char*>(block.data()));
        stream.avail_in = static_cast<uInt>(block.size());
        stream.next_out = compressed.data();
        stream.avail_out = static_cast<uInt>(compressed.size());
        const int result = deflate(&stream, Z_SYNC_FLUSH);
        const std::size_t compressedSize = stream.total_out;
        deflateEnd(&stream);
        if (result != Z_OK || compressedSize >= 32786 ||
            compressedSize > std::numeric_limits<std::uint16_t>::max()) {
            throw std::runtime_error("unable to compress X fixture block");
        }
        compressed.resize(compressedSize);
        appendU16(file, static_cast<std::uint16_t>(compressed.size()));
        file.push_back('C');
        file.push_back('K');
        file.insert(file.end(), compressed.begin(), compressed.end());
        previousBlock.assign(block);
        if (previousBlock.size() > 32768) {
            previousBlock.erase(0, previousBlock.size() - 32768);
        }
    }
    return file;
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "expected a text-mode X mesh path\n";
        return 1;
    }
    const std::shared_ptr<ParsedObj> parsed = XLoader::parse(argv[1]);
    if (!parsed || parsed->positions.empty() || parsed->triangles.empty()) {
        std::cerr << "text X mesh did not yield static geometry\n";
        return 1;
    }
    if (!checkBounds(*parsed)) {
        return 1;
    }
    for (const ObjTriangle& triangle : parsed->triangles) {
        if (parsed->materials.find(triangle.material) == parsed->materials.end()) {
            std::cerr << "X triangle refers to a missing material\n";
            return 1;
        }
        for (const ObjIndex& index : triangle.indices) {
            if (index.position <= 0 ||
                static_cast<std::size_t>(index.position) > parsed->positions.size() ||
                (index.texCoord > 0 &&
                 static_cast<std::size_t>(index.texCoord) > parsed->texCoords.size()) ||
                (index.normal > 0 &&
                 static_cast<std::size_t>(index.normal) > parsed->normals.size())) {
                std::cerr << "X triangle contains an invalid output index\n";
                return 1;
            }
        }
    }
    if (std::filesystem::path(argv[1]).filename() == "scenery_text_mesh.x") {
        if (parsed->positions.size() != 4 || parsed->triangles.size() != 2 ||
            parsed->texCoords.size() != 4 || parsed->normals.empty() ||
            parsed->materials.size() != 1 ||
            std::abs(parsed->positions[0].x - 5.0) > 1.0e-8 ||
            std::abs(parsed->positions[0].y) > 1.0e-8 ||
            std::abs(parsed->positions[0].z) > 1.0e-8 ||
            std::abs(parsed->positions[2].x - 6.0) > 1.0e-8 ||
            std::abs(parsed->positions[2].y - 1.0) > 1.0e-8 ||
            std::abs(parsed->normals[0].z + 1.0) > 1.0e-8) {
            std::cerr << "frame and map-axis transforms were not applied to X vertices/normals\n";
            return 1;
        }
        if (parsed->triangles[0].material != "matl_0" ||
            parsed->triangles[1].indices[2].position != 4 ||
            parsed->triangles[0].indices[0].texCoord != 1 ||
            std::abs(parsed->texCoords[0].u + 0.25) > 1.0e-8 ||
            std::abs(parsed->texCoords[0].v + 0.5) > 1.0e-8 ||
            std::abs(parsed->texCoords[2].v - 1.5) > 1.0e-8) {
            std::cerr << "X triangulation, material indices, or normalized repeat UVs are incorrect\n";
            return 1;
        }
        const ObjMaterial& material = parsed->materials.at("matl_0");
        if (material.materialIndex != 0 || material.textureName != "leaf.tga" ||
            material.color != std::array<double, 3>{0.25, 0.5, 0.75} ||
            std::abs(material.alpha - 0.4) > 1.0e-8) {
            std::cerr << "X inline material color, alpha, or texture was not retained\n";
            return 1;
        }
    }

    const std::filesystem::path malformed =
        std::filesystem::temp_directory_path() / "openbus_invalid_x_mesh.x";
    {
        std::ofstream output(malformed);
        output << "xof 0303txt 0032\nMesh { 3; 0;0;0;, 1;0;0;, 0;1;0;; 1; 3; 0,1,3;; }\n";
    }
    const bool malformedRejected = !XLoader::parse(malformed);
    std::filesystem::remove(malformed);
    if (!malformedRejected) {
        std::cerr << "out-of-range X face index was accepted\n";
        return 1;
    }

    const std::filesystem::path binaryMesh =
        std::filesystem::temp_directory_path() / "openbus_binary_x_mesh.x";
    std::vector<std::uint8_t> binary = {
        'x', 'o', 'f', ' ', '0', '3', '0', '3', 'b', 'i', 'n', ' ', '0', '0', '3', '2'};
    appendToken(binary, 31); // TEMPLATE
    appendName(binary, "Header");
    appendToken(binary, 10);
    appendToken(binary, 5); // GUID
    binary.insert(binary.end(), 16, 0);
    appendToken(binary, 48); // VOID template field type
    appendToken(binary, 20);
    appendToken(binary, 11);
    appendName(binary, "Mesh");
    appendToken(binary, 10); // {
    appendInteger(binary, 3);
    appendToken(binary, 20); // ;
    appendFloatList(binary, {0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F});
    appendToken(binary, 20);
    appendInteger(binary, 1);
    appendToken(binary, 20);
    appendInteger(binary, 3);
    appendToken(binary, 20);
    appendIntegerList(binary, {0, 1, 2});
    appendToken(binary, 20);
    appendToken(binary, 11); // }
    {
        std::ofstream output(binaryMesh, std::ios::binary);
        output.write(reinterpret_cast<const char*>(binary.data()),
                     static_cast<std::streamsize>(binary.size()));
    }
    const std::shared_ptr<ParsedObj> parsedBinary = XLoader::parse(binaryMesh);
    std::filesystem::remove(binaryMesh);
    if (!parsedBinary || parsedBinary->positions.size() != 3 ||
        parsedBinary->triangles.size() != 1 ||
        parsedBinary->triangles.front().indices[2].position != 3 ||
        !checkBounds(*parsedBinary)) {
        std::cerr << "binary X token stream did not produce the expected triangle\n";
        return 1;
    }

    const std::string textMesh = std::string(40000, ' ') +
                                 "Mesh { 3; 0;0;0;, 1;0;0;, 0;1;0;; 1; 3; 0,1,2;; }\n";
    const std::filesystem::path compressedTextMesh =
        std::filesystem::temp_directory_path() / "openbus_compressed_text_x_mesh.x";
    const std::vector<std::uint8_t> compressedText = makeCompressedX("tzip", textMesh);
    {
        std::ofstream output(compressedTextMesh, std::ios::binary);
        output.write(reinterpret_cast<const char*>(compressedText.data()),
                     static_cast<std::streamsize>(compressedText.size()));
    }
    const std::shared_ptr<ParsedObj> parsedCompressedText = XLoader::parse(compressedTextMesh);
    std::filesystem::remove(compressedTextMesh);
    if (!parsedCompressedText || parsedCompressedText->triangles.size() != 1 ||
        !checkBounds(*parsedCompressedText)) {
        std::cerr << "MSZIP-compressed text X mesh did not parse\n";
        return 1;
    }

    const std::filesystem::path compressedBinaryMesh =
        std::filesystem::temp_directory_path() / "openbus_compressed_binary_x_mesh.x";
    const std::string binaryPayload(reinterpret_cast<const char*>(binary.data() + 16),
                                    binary.size() - 16);
    const std::vector<std::uint8_t> compressedBinary = makeCompressedX("bzip", binaryPayload);
    {
        std::ofstream output(compressedBinaryMesh, std::ios::binary);
        output.write(reinterpret_cast<const char*>(compressedBinary.data()),
                     static_cast<std::streamsize>(compressedBinary.size()));
    }
    const std::shared_ptr<ParsedObj> parsedCompressedBinary = XLoader::parse(compressedBinaryMesh);
    std::filesystem::remove(compressedBinaryMesh);
    if (!parsedCompressedBinary || parsedCompressedBinary->triangles.size() != 1 ||
        !checkBounds(*parsedCompressedBinary)) {
        std::cerr << "MSZIP-compressed binary X mesh did not parse\n";
        return 1;
    }
    return 0;
}
