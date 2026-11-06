#include "config/TextureSurfaceConfigLoader.h"

#include "config/ConfigurationParser.h"
#include "core/AssetPath.h"
#include "core/Utf8Path.h"

#include <utility>

namespace openbus::config {

namespace {

void skipSectionPayload(Reader& reader) {
    Line line;
    while (reader.nextIncludingBlank(line)) {
        if (line.isKeyword()) {
            reader.pushBack(std::move(line));
            return;
        }
    }
}

} // namespace

TextureSurfaceConfiguration
loadTextureSurfaceConfiguration(const std::filesystem::path& texturePath) {
    TextureSurfaceConfiguration result;
    std::filesystem::path configPath = texturePath;
    configPath += ".cfg";
    configPath = openbus::assets::resolveCaseInsensitivePath(configPath);
    std::error_code fileError;
    if (!std::filesystem::exists(configPath, fileError)) {
        if (fileError) {
            result.diagnostics.error(0, "file",
                                     "unable to inspect " + openbus::pathToUtf8(configPath));
        }
        return result;
    }
    if (!std::filesystem::is_regular_file(configPath, fileError) || fileError) {
        result.diagnostics.error(0, "file",
                                 "texture surface CFG is not a regular file: " +
                                     openbus::pathToUtf8(configPath));
        return result;
    }

    Reader reader(configPath);
    if (!reader.isOpen()) {
        result.diagnostics.error(0, "file", "unable to open " + openbus::pathToUtf8(configPath));
        return result;
    }
    result.configFound = true;

    Line line;
    while (reader.next(line)) {
        if (!line.isKeyword()) {
            result.diagnostics.warning(line.number, "section",
                                       "unexpected data outside a texture surface section");
            continue;
        }
        const std::string keyword = lower(line.keyword());
        const bool known = keyword == "puddles" || keyword == "moisture" || keyword == "surface" ||
                           keyword == "nightmapmode";
        const bool roadPhysics =
            keyword == "puddles" || keyword == "moisture" || keyword == "surface";
        logConfigurationTagStatus(
            "texture surface CFG", configPath, line.number, keyword,
            roadPhysics
                ? ConfigurationTagStatus::Active
                : (known ? ConfigurationTagStatus::Ignored : ConfigurationTagStatus::Unknown),
            roadPhysics ? "used for approximate spline-road wheel grip and moisture shading; "
                          "other surfaces are not yet supported"
                        : (known ? "texture night-map mode is parsed but not applied"
                                 : "unknown texture surface configuration section"));
        if (keyword == "puddles") {
            result.supportsPuddles = true;
            result.supportsMoisture = true;
        } else if (keyword == "moisture") {
            result.supportsMoisture = true;
        } else if (keyword == "surface" || keyword == "nightmapmode") {
            Line valueLine;
            int value = 0;
            if (!reader.readPayload(valueLine, result.diagnostics, keyword)) {
                continue;
            }
            if (!parseInt(valueLine.text, value)) {
                result.diagnostics.error(valueLine.number, keyword, "expected an integer value");
                continue;
            }
            if (keyword == "surface") {
                if (value < 0 || value > 255) {
                    result.diagnostics.error(valueLine.number, keyword,
                                             "surface type must be between 0 and 255");
                } else {
                    result.surfaceType = value;
                }
            } else {
                result.nightMapMode = value;
            }
        } else {
            result.diagnostics.warning(line.number, keyword,
                                       "texture surface configuration section is not implemented");
            skipSectionPayload(reader);
        }
    }
    return result;
}

} // namespace openbus::config
