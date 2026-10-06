#include "XLoader.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

using namespace openbus::rendering;

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
            std::abs(parsed->texCoords[0].v - 1.5) > 1.0e-8) {
            std::cerr << "X triangulation, material indices, or repeat UVs were not preserved\n";
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
    return 0;
}
