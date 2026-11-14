#include "core/GraphicsPreferences.h"
#include "ProbeTempDirectory.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void write(const std::filesystem::path& path, const std::string& value) {
    std::ofstream output(path);
    output << value;
    if (!output) throw std::runtime_error("Unable to write test INI");
}

std::string read(const std::filesystem::path& path) {
    std::ifstream input(path);
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

} // namespace

int main() {
    const auto root = openbus::probe::makeProbeDirectory("openbus_graphics_preferences", false);
    try {
        using namespace openbus::core;
        require(parseGraphicsFpsCapOverride("0") == 0.0 &&
                    parseGraphicsFpsCapOverride("60") == 60.0 &&
                    parseGraphicsFpsCapOverride("2000") == 1000.0,
                "FPS environment override parsing is incorrect");
        bool invalidOverride = false;
        try {
            static_cast<void>(parseGraphicsFpsCapOverride("-1"));
        } catch (const std::invalid_argument&) {
            invalidOverride = true;
        }
        require(invalidOverride, "negative FPS override must be rejected");
        const auto path = root / "graphics.ini";
        const auto empty = loadGraphicsPreferences(path);
        require(empty.values == GraphicsPreferences{}, "missing INI must use defaults");
        write(path, "\xEF\xBB\xBF; retained comment\n[display]\nmode = exclusive\nwidth = 2560\n"
                    "height = 1440\nrefresh_rate = 144\nmonitor = 1\nvsync = false\nfps_cap = 60\n"
                    "future_option = keep\n[unknown]\nkey = value\n"
                    "[textures]\nanisotropic_filtering = 8\ngenerate_mipmaps = true\n"
                    "vehicle_texture_scale = 0.5\nmap_texture_scale = 0.75\n"
                    "[antialiasing]\nmode = fxaa\n"
                    "[rendering]\nscale = 0.75\n"
                    "[advanced]\nvehicle_frustum_culling = false\n");
        auto file = loadGraphicsPreferences(path);
        require(file.values.windowMode == "exclusive" && file.values.width == 2560 &&
                    file.values.height == 1440 && file.values.refreshRate == 144 &&
                    file.values.monitor == 1 && file.values.anisotropicFiltering == 8 &&
                    file.values.generateTextureMipmaps &&
                    file.values.mapTextureScale == 0.75 &&
                    file.values.vehicleTextureScale == 0.5 &&
                    file.values.antiAliasingMode == "fxaa" &&
                    file.values.renderScale == 0.75 &&
                    !file.values.vehicleFrustumCulling &&
                    !file.values.vsync && file.values.fpsCap == 60.0,
                "known settings were not loaded");
        file.values.reflectionSize = 512;
        file.values.reflectionInterval = 2;
        file.values.reflectionMaxFps = 30.0;
        file.values.minimumObjectSizeMainPercent = 0.25;
        file.values.minimumObjectSizeMirrorsPercent = 0.5;
        saveGraphicsPreferences(path, file);
        const auto saved = read(path);
        require(saved.starts_with("\xEF\xBB\xBF; retained comment") &&
                    saved.find("future_option = keep") != std::string::npos &&
                    saved.find("[unknown]\nkey = value") != std::string::npos,
                "unknown keys and comments must survive saving");
        require(loadGraphicsPreferences(path).values == file.values,
                "graphics INI settings did not round-trip");
        auto externalValues = file.values;
        auto editedValues = file.values;
        externalValues.reflectionSize = 256;
        externalValues.windowMode = "borderless";
        externalValues.anisotropicFiltering = 4;
        externalValues.generateTextureMipmaps = false;
        externalValues.mapTextureScale = 1.0;
        externalValues.vehicleTextureScale = 1.0;
        externalValues.antiAliasingMode = "off";
        externalValues.renderScale = 0.5;
        externalValues.vehicleFrustumCulling = false;
        editedValues.width = 1280;
        editedValues.fpsCap = 75.0;
        editedValues.vehicleTextureScale = 0.75;
        editedValues.renderScale = 0.85;
        editedValues.vehicleFrustumCulling = true;
        const auto merged = mergeGraphicsPreferences(file.values, editedValues, externalValues);
        require(merged.reflectionSize == 256 && merged.windowMode == "borderless" &&
                    merged.anisotropicFiltering == 4 && !merged.generateTextureMipmaps &&
                    merged.mapTextureScale == 1.0 &&
                    merged.width == 1280 &&
                    merged.vehicleTextureScale == 0.75 &&
                    merged.antiAliasingMode == "off" && merged.fpsCap == 75.0 &&
                    merged.renderScale == 0.85 && merged.vehicleFrustumCulling,
                "saving an edit must retain unrelated external changes");
        file.values.fpsCap = std::numeric_limits<double>::quiet_NaN();
        bool rejected = false;
        try {
            saveGraphicsPreferences(path, file);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        require(rejected && read(path) == saved,
                "invalid settings must not replace the original INI");
        for (const std::string invalid : {"[display]\nvsync = maybe\n",
                                          "[display]\nfps_cap = -1\n",
                                          "[display]\nmode = fake\n",
                                          "[display]\nwidth = 320\n",
                                          "[display]\nrefresh_rate = 60.5\n",
                                          "[display]\nmonitor = -1\n",
                                          "[textures]\nanisotropic_filtering = 3\n",
                                          "[textures]\ngenerate_mipmaps = perhaps\n",
                                          "[textures]\nmap_texture_scale = 0.1\n",
                                          "[textures]\nvehicle_texture_scale = 0.1\n",
                                          "[advanced]\nvehicle_frustum_culling = maybe\n",
                                          "[antialiasing]\nmode = msaa\n",
                                          "[rendering]\nscale = 1.5\n",
                                          "[display]\nvsync = true\nvsync = false\n",
                                          "[reflections]\nsize = 400\n",
                                          "[visibility]\nmain_minimum_object_percent = 101\n",
                                          "[display\nfps_cap = 60\n"}) {
            write(path, invalid);
            rejected = false;
            try {
                static_cast<void>(loadGraphicsPreferences(path));
            } catch (const std::runtime_error&) {
                rejected = true;
            }
            require(rejected && read(path) == invalid,
                    "malformed settings must be reported without modifying the INI");
        }
        openbus::probe::removeProbeDirectory(root);
        std::cout << "Graphics preferences checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Graphics preferences probe failed: " << error.what() << '\n';
        openbus::probe::removeProbeDirectory(root);
        return 1;
    }
}
