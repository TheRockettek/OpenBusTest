#include "config/SceneryObjectConfigLoader.h"

#include "core/AssetPath.h"
#include "config/ConfigurationParser.h"
#include "config/ModelConfigLoader.h"
#include "core/Variables.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace {

using openbus::config::Line;
using openbus::config::lower;
using openbus::config::parseDouble;
using openbus::config::parseInt;
using openbus::config::Reader;

bool readList(Reader& reader, const std::string& keyword, std::vector<std::string>& destination,
              ConfigurationDiagnostics& diagnostics) {
    Line countLine;
    int count = 0;
    if (!reader.readPayload(countLine, diagnostics, keyword) || !parseInt(countLine.text, count) ||
        count < 0) {
        diagnostics.error(countLine.number, keyword, "expected a non-negative entry count");
        return false;
    }
    return reader.readPayloads(static_cast<std::size_t>(count), destination, diagnostics, keyword);
}

bool readDoubles(Reader& reader, const std::string& keyword, std::size_t count,
                 std::vector<double>& values, ConfigurationDiagnostics& diagnostics) {
    std::vector<std::string> raw;
    if (!reader.readPayloads(count, raw, diagnostics, keyword)) {
        return false;
    }
    values.clear();
    values.reserve(raw.size());
    for (const std::string& text : raw) {
        double value = 0.0;
        if (!parseDouble(text, value)) {
            diagnostics.error(0, keyword, "expected numeric values");
            return false;
        }
        values.push_back(value);
    }
    return true;
}

} // namespace

SceneryObjectConfig loadSceneryObjectFile(const std::filesystem::path& configPath) {
    SceneryObjectConfig result;
    result.sourcePath = configPath;
    Reader reader(configPath);
    if (!reader.isOpen()) {
        result.diagnostics.error(0, "file", "unable to open " + configPath.string());
        return result;
    }

    Line line;
    bool hasInlineModel = false;
    while (reader.next(line)) {
        if (!line.isKeyword()) {
            continue;
        }
        const std::string keyword = lower(line.keyword());
        if (keyword == "end") {
            continue;
        }
        if (keyword == "description") {
            bool terminated = false;
            while (reader.next(line)) {
                if (line.isKeyword() && lower(line.keyword()) == "end") {
                    terminated = true;
                    break;
                }
            }
            if (!terminated) {
                result.diagnostics.error(line.number, "description", "missing [end]");
            }
            continue;
        }
        if (keyword == "helparrow") {
            result.helpArrow = true;
            continue;
        }
        if (keyword == "groups" || keyword == "friendlyname") {
            // These SCO metadata/editor fields do not affect static model rendering. Their
            // payload lines are intentionally left for the line-oriented parser to ignore.
            openbus::config::logUnhandledConfigurationTag(
                "scenery object (.sco)", configPath, line.number, line.keyword(),
                "metadata is recognized but not used by the renderer");
            continue;
        }
        if (keyword == "model") {
            Line value;
            if (reader.readPayload(value, result.diagnostics, keyword)) {
                result.modelPath = value.text;
            }
            continue;
        }
        if (keyword == "mesh") {
            hasInlineModel = true;
            continue;
        }
        static const std::unordered_set<std::string> inlineModelKeywords = {"mesh_ident",
                                                                            "lod",
                                                                            "matl",
                                                                            "matl_alpha",
                                                                            "matl_bumpmap",
                                                                            "matl_change",
                                                                            "matl_envmap",
                                                                            "matl_item",
                                                                            "matl_freetex",
                                                                            "matl_lightmap",
                                                                            "matl_nightmap",
                                                                            "matl_nozcheck",
                                                                            "matl_nozwrite",
                                                                            "matl_transmap",
                                                                            "scripttexture",
                                                                            "texttexture",
                                                                            "texttexture_enh",
                                                                            "usescripttexture",
                                                                            "usetexttexture",
                                                                            "visible",
                                                                            "fixed",
                                                                            "isshadow",
                                                                            "viewpoint",
                                                                            "shadow"};
        if (inlineModelKeywords.contains(keyword)) {
            continue;
        }
        if (keyword == "new_attachment") {
            ModelAttachmentPoint attachment;
            bool sawOperation = false;
            Line entry;
            while (reader.next(entry)) {
                if (entry.isKeyword()) {
                    reader.pushBack(std::move(entry));
                    break;
                }
                const std::string operation = lower(entry.text);
                std::size_t valueCount = 0;
                if (operation == "attach_trans") {
                    valueCount = 3;
                    attachment.hasTranslation = true;
                } else if (operation == "attach_rot_x" || operation == "attach_rot_y" ||
                           operation == "attach_rot_z") {
                    valueCount = 1;
                    attachment.hasRotation = true;
                } else {
                    if (attachment.name.empty() && !sawOperation) {
                        attachment.name = entry.text;
                    } else {
                        openbus::config::logUnhandledConfigurationTag(
                            "scenery object attachment", configPath, entry.number, entry.text,
                            "attachment operation is not implemented");
                    }
                    continue;
                }
                sawOperation = true;
                for (std::size_t index = 0; index < valueCount; ++index) {
                    Line value;
                    if (!reader.readPayload(value, result.diagnostics, keyword)) {
                        break;
                    }
                    double parsed = 0.0;
                    if (!parseDouble(value.text, parsed)) {
                        result.diagnostics.error(value.number, keyword,
                                                 "attachment transform must be numeric");
                        continue;
                    }
                    if (operation == "attach_trans") {
                        attachment.translation[index] = parsed;
                    } else {
                        const std::size_t axis = operation == "attach_rot_x"   ? 0
                                                 : operation == "attach_rot_y" ? 1
                                                                               : 2;
                        attachment.rotationDegrees[axis] = parsed;
                    }
                }
            }
            result.attachmentPoints.push_back(std::move(attachment));
            continue;
        }
        if (keyword == "path") {
            std::vector<double> values;
            if (readDoubles(reader, keyword, 12, values, result.diagnostics) &&
                values.size() == 12) {
                SceneryObjectTrafficPath path;
                std::copy_n(values.begin(), 3, path.startPosition.begin());
                path.headingDegrees = values[3];
                path.radius = values[4];
                path.length = values[5];
                path.gradientStartPercent = values[6];
                path.gradientEndPercent = values[7];
                path.pathTypeCode = values[8];
                path.widthMeters = values[9];
                result.trafficPaths.push_back(path);
            }
            continue;
        }
        if (keyword == "traffic_lights_group") {
            Line value;
            double cycleDuration = 0.0;
            if (reader.readPayload(value, result.diagnostics, keyword)) {
                if (parseDouble(value.text, cycleDuration) && std::isfinite(cycleDuration) &&
                    cycleDuration > 0.0) {
                    result.trafficLightsGroupDurationSeconds = cycleDuration;
                } else {
                    result.diagnostics.warning(value.number, keyword,
                                               "expected a positive cycle duration");
                }
            }
            continue;
        }
        if (keyword == "traffic_light") {
            Line value;
            if (reader.readPayload(value, result.diagnostics, keyword)) {
                result.trafficLights.push_back({value.text, {}, std::nullopt});
            }
            continue;
        }
        if (keyword == "phase") {
            Line stateLine;
            Line durationLine;
            int state = 0;
            double duration = 0.0;
            if (reader.readPayload(stateLine, result.diagnostics, keyword) &&
                reader.readPayload(durationLine, result.diagnostics, keyword)) {
                if (!result.trafficLights.empty() && parseInt(stateLine.text, state) &&
                    state >= 0 && state <= 12 && parseDouble(durationLine.text, duration) &&
                    std::isfinite(duration) && duration >= 0.0) {
                    result.trafficLights.back().phases.push_back({state, duration});
                } else {
                    result.diagnostics.warning(
                        stateLine.number, keyword,
                        "expected a phase state in [0, 12], a non-negative duration, and a "
                        "preceding [traffic_light]");
                }
            }
            continue;
        }
        if (keyword == "approachdist") {
            Line value;
            double distance = 0.0;
            if (reader.readPayload(value, result.diagnostics, keyword)) {
                if (!result.trafficLights.empty() && parseDouble(value.text, distance) &&
                    std::isfinite(distance) && distance >= 0.0) {
                    result.trafficLights.back().approachDistanceMeters = distance;
                } else {
                    result.diagnostics.warning(
                        value.number, keyword,
                        "expected a non-negative distance after a [traffic_light]");
                }
            }
            continue;
        }
        if (keyword == "use_traffic_light") {
            Line value;
            int lightIndex = -1;
            if (reader.readPayload(value, result.diagnostics, keyword)) {
                if (result.trafficPaths.empty() || !parseInt(value.text, lightIndex) ||
                    lightIndex < -1) {
                    result.diagnostics.warning(value.number, keyword,
                                               "expected a light index for a preceding [path]");
                } else {
                    result.trafficPaths.back().trafficLightIndex = lightIndex;
                }
            }
            continue;
        }
        if (keyword == "traffic_light_stop" || keyword == "traffic_light_jump") {
            const std::size_t valueCount = keyword == "traffic_light_stop" ? 3 : 4;
            std::vector<double> values;
            if (readDoubles(reader, keyword, valueCount, values, result.diagnostics) &&
                values.size() == valueCount) {
                if (keyword == "traffic_light_stop") {
                    std::array<double, 3> stop{};
                    std::copy(values.begin(), values.end(), stop.begin());
                    result.trafficLightStops.push_back(stop);
                } else {
                    std::array<double, 4> jump{};
                    std::copy(values.begin(), values.end(), jump.begin());
                    result.trafficLightJumps.push_back(jump);
                }
            }
            continue;
        }
        if (keyword == "tree") {
            std::vector<std::string> values;
            if (reader.readPayloads(5, values, result.diagnostics, keyword) && values.size() == 5) {
                SceneryTreeDefinition tree;
                tree.texturePath = values[0];
                if (parseDouble(values[1], tree.minimumHeight) &&
                    parseDouble(values[2], tree.maximumHeight) &&
                    parseDouble(values[3], tree.minimumRatio) &&
                    parseDouble(values[4], tree.maximumRatio) && tree.minimumHeight > 0.0 &&
                    tree.maximumHeight >= tree.minimumHeight && tree.minimumRatio > 0.0 &&
                    tree.maximumRatio >= tree.minimumRatio) {
                    result.trees.push_back(std::move(tree));
                } else {
                    result.diagnostics.error(line.number, keyword,
                                             "expected valid height and ratio ranges");
                }
            }
            continue;
        }
        if (keyword == "boundingbox") {
            std::vector<double> values;
            if (readDoubles(reader, keyword, 6, values, result.diagnostics)) {
                for (std::size_t index = 0; index < values.size(); ++index) {
                    result.boundingBox[index] = values[index];
                }
                result.hasBoundingBox = true;
            }
            continue;
        }
        if (keyword == "collision_mesh") {
            Line value;
            if (reader.readPayload(value, result.diagnostics, keyword)) {
                result.collisionMesh = value.text;
            }
            continue;
        }
        if (keyword == "nocollision") {
            result.noCollision = true;
            continue;
        }
        if (keyword == "onlyeditor") {
            result.onlyEditor = true;
            continue;
        }
        if (keyword == "absheight") {
            result.absoluteHeight = true;
            continue;
        }
        if (keyword == "surface") {
            result.surface = true;
            continue;
        }
        if (keyword == "crashmode_pole") {
            std::vector<double> values;
            readDoubles(reader, keyword, 2, values, result.diagnostics);
            continue;
        }
        if (keyword == "maplight") {
            std::vector<double> values;
            readDoubles(reader, keyword, 7, values, result.diagnostics);
            continue;
        }
        if (keyword == "nightmapmode") {
            Line value;
            int mode = 0;
            if (reader.readPayload(value, result.diagnostics, keyword) &&
                !parseInt(value.text, mode)) {
                result.diagnostics.error(value.number, keyword, "expected a numeric mode");
            }
            continue;
        }
        if (keyword == "rendertype") {
            Line value;
            if (reader.readPayload(value, result.diagnostics, keyword)) {
                const std::string type = lower(value.text);
                if (type == "presurface") {
                    result.renderType = -1;
                } else if (type == "surface") {
                    result.renderType = -2;
                } else if (type == "on_surface") {
                    result.renderType = -3;
                } else if (!parseInt(type, result.renderType)) {
                    result.diagnostics.error(value.number, keyword,
                                             "expected a render type or named render category");
                }
            }
            continue;
        }
        if (keyword == "script" || keyword == "varnamelist" || keyword == "stringvarnamelist" ||
            keyword == "constfile") {
            std::vector<std::string> values;
            if (readList(reader, keyword, values, result.diagnostics)) {
                if (keyword == "script") {
                    result.scripts = std::move(values);
                } else if (keyword == "varnamelist") {
                    result.variableLists = std::move(values);
                } else if (keyword == "stringvarnamelist") {
                    result.stringVariableLists = std::move(values);
                } else {
                    result.constantFiles = std::move(values);
                }
            }
            continue;
        }
        result.diagnostics.warning(line.number, line.keyword(), "unknown SceneryObject keyword");
        openbus::config::logUnhandledConfigurationTag("scenery object (.sco)", configPath,
                                                      line.number, line.keyword(),
                                                      "unknown scenery-object keyword");
    }

    bool hasSignalizedTrafficPath = false;
    for (SceneryObjectTrafficPath& path : result.trafficPaths) {
        if (path.trafficLightIndex < 0) {
            continue;
        }
        if (static_cast<std::size_t>(path.trafficLightIndex) >= result.trafficLights.size()) {
            result.diagnostics.warning(0, "use_traffic_light",
                                       "path references a missing traffic light");
            continue;
        }
        if (result.trafficLightsGroupDurationSeconds <= 0.0) {
            result.diagnostics.warning(0, "traffic_lights_group",
                                       "signalized path has no valid group cycle duration");
            continue;
        }
        const SceneryObjectTrafficLight& light =
            result.trafficLights[static_cast<std::size_t>(path.trafficLightIndex)];
        openbus::map::MapTrafficSignalSchedule schedule;
        schedule.cycleDurationSeconds = result.trafficLightsGroupDurationSeconds;
        schedule.phases = light.phases;
        schedule.hasConditionalOverrides =
            !result.trafficLightStops.empty() || !result.trafficLightJumps.empty();
        if (!openbus::map::mapTrafficSignalStateAtTime(schedule, 0.0).has_value()) {
            result.diagnostics.warning(0, "traffic_light",
                                       "signalized path has an invalid phase schedule");
            continue;
        }
        path.trafficSignal = std::move(schedule);
        hasSignalizedTrafficPath = true;
    }
    if (hasSignalizedTrafficPath &&
        (!result.trafficLightStops.empty() || !result.trafficLightJumps.empty())) {
        result.diagnostics.warning(
            0, "traffic_light_jump",
            "AI traffic uses the base phase cycle; request-driven stop/jump timing is not "
            "simulated");
    }

    if (result.modelPath.empty() && !hasInlineModel && result.trees.empty()) {
        result.diagnostics.error(0, "model", "SceneryObject configuration has no [model] entry");
    }
    result.scriptConfiguration.sourcePath = configPath;
    result.scriptConfiguration.scripts = result.scripts;
    result.scriptConfiguration.variableLists = result.variableLists;
    result.scriptConfiguration.stringVariableLists = result.stringVariableLists;
    result.scriptConfiguration.constantFiles = result.constantFiles;
    prepareScriptConfiguration(result.scriptConfiguration);
    result.diagnostics.entries.insert(result.diagnostics.entries.end(),
                                      result.scriptConfiguration.diagnostics.entries.begin(),
                                      result.scriptConfiguration.diagnostics.entries.end());
    return result;
}

std::filesystem::path
resolveSceneryObjectModelConfigPath(const SceneryObjectConfig& configuration) {
    if (configuration.modelPath.empty()) {
        return configuration.sourcePath.lexically_normal();
    }

    std::string normalized = configuration.modelPath.generic_string();
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    const std::filesystem::path modelPath(normalized);
    if (modelPath.is_absolute()) {
        return openbus::assets::resolveCaseInsensitivePath(modelPath);
    }
    return openbus::assets::resolveCaseInsensitivePath(configuration.sourcePath.parent_path() /
                                                       modelPath);
}

ModelConfig loadSceneryObjectConfig(const std::filesystem::path& configPath,
                                    const std::filesystem::path& modelRoot,
                                    openbus::scripting::SceneryObject& variables) {
    return loadModelConfig(configPath, modelRoot, ModelConfigKind::SceneryObject, variables);
}
