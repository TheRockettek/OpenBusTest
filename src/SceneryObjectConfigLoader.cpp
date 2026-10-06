#include "SceneryObjectConfigLoader.h"

#include "ConfigurationParser.h"
#include "ModelConfigLoader.h"
#include "Variables.h"

#include <algorithm>

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
        if (keyword == "friendlyname") {
            std::vector<std::string> values;
            reader.readPayloads(3, values, result.diagnostics, keyword);
            continue;
        }
        if (keyword == "model") {
            Line value;
            if (reader.readPayload(value, result.diagnostics, keyword)) {
                result.modelPath = value.text;
            }
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
            reader.readPayload(value, result.diagnostics, keyword);
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
    }

    if (result.modelPath.empty() && result.trees.empty()) {
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
        return modelPath.lexically_normal();
    }
    return (configuration.sourcePath.parent_path() / modelPath).lexically_normal();
}

ModelConfig loadSceneryObjectConfig(const std::filesystem::path& configPath,
                                    const std::filesystem::path& modelRoot,
                                    openbus::scripting::SceneryObject& variables) {
    return loadModelConfig(configPath, modelRoot, ModelConfigKind::SceneryObject, variables);
}
