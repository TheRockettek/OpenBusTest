#include "O3DLoader.h"

#include <filesystem>
#include <iostream>

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
    return 0;
}