#include "O3DLoader.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <vector>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "expected an O3D path\n";
        return 1;
    }
    const auto parsed = openbus::rendering::O3DLoader::parse(argv[1]);
    if (!parsed || parsed->positions.empty() || parsed->triangles.empty() ||
        parsed->materials.empty()) {
        std::cerr << "failed to parse O3D fixture\n";
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
    if (std::getenv("OPENBUS_O3D_PROBE_VERBOSE") != nullptr) {
        double minimumEdge = std::numeric_limits<double>::max();
        double maximumEdge = 0.0;
        std::vector<std::pair<double, std::size_t>> longestEdges;
        for (const auto& triangle : parsed->triangles) {
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
                  << parsed->boundsSize[2] << " edgeRange=" << minimumEdge << ','
                  << maximumEdge << '\n';
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