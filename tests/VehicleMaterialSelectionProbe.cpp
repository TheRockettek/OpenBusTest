#include "app/vehicle/VehicleMaterialSelection.h"

#include <iostream>
#include <unordered_map>

int main() {
    const std::unordered_map<std::string, std::string> replacements = {
        {"body.dds", "custom_body.dds"}, {"window.bmp", "custom_window.bmp"}};
    if (openbus::app::vehicle::resolveCtcTextureName("BODY.DDS", replacements) !=
            "custom_body.dds" ||
        openbus::app::vehicle::resolveCtcTextureName("unmapped.dds", replacements) !=
            "unmapped.dds") {
        std::cerr << "CTC texture replacement was not resolved case-insensitively\n";
        return 1;
    }

    struct LayerCase {
        bool textureArray;
        int requested;
        std::size_t available;
        int expected;
    };
    constexpr LayerCase cases[] = {
        {false, 2, 4, 0}, {true, -1, 4, 0}, {true, 0, 4, 0}, {true, 1, 4, 1},
        {true, 3, 4, 3},  {true, 8, 4, 3},  {true, 8, 0, 0}};
    for (const LayerCase& test : cases) {
        const int actual = openbus::app::vehicle::resolveTextureArrayLayer(
            test.textureArray, test.requested, test.available);
        if (actual != test.expected) {
            std::cerr << "texture-array layer selection did not clamp as expected\n";
            return 1;
        }
    }
    return 0;
}
