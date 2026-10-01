#include "ObjLoader.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    if (argc < 2 || argc > 3) {
        std::cerr << "Usage: obj_bundle <model-root> [output.obx]\n";
        return 2;
    }
    const std::filesystem::path root = std::filesystem::path(argv[1]);
    const std::filesystem::path output =
        argc == 3 ? std::filesystem::path(argv[2]) : root / "openbus.obx";
    std::vector<std::filesystem::path> objects;
    std::error_code error;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(root, error)) {
        if (error) {
            std::cerr << "Failed to scan " << root << ": " << error.message() << "\n";
            return 1;
        }
        if (entry.is_regular_file(error) && entry.path().extension() == ".obj") {
            objects.push_back(entry.path());
        }
    }
    std::sort(objects.begin(), objects.end());
    if (objects.empty()) {
        std::cerr << "No OBJ files found under " << root << "\n";
        return 1;
    }
    if (!openbus::rendering::ObjLoader::writeBundle(output, root, objects)) {
        std::cerr << "Failed to write bundle " << output << "\n";
        return 1;
    }
    std::filesystem::path firstEntry = objects.front().lexically_relative(root);
    firstEntry.replace_extension(".obj");
    std::string entryName = firstEntry.generic_string();
    std::transform(entryName.begin(), entryName.end(), entryName.begin(), [](unsigned char value) {
        return static_cast<char>(std::tolower(value));
    });
    if (!openbus::rendering::ObjLoader::parse(output, entryName)) {
        std::cerr << "Bundle readback failed for " << entryName << "\n";
        return 1;
    }
    std::cout << "Wrote " << objects.size() << " objects to " << output << "\n";
    return 0;
}
