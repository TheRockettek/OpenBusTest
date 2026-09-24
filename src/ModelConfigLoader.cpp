#include "ModelConfigLoader.h"

#include "ConfigurationParser.h"
#include "Variables.h"

#include <algorithm>
#include <array>
#include <filesystem>
#include <iomanip>
#include <ostream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

using openbus::config::Line;
using openbus::config::Reader;
using openbus::config::lower;
using openbus::config::parseDouble;
using openbus::config::parseInt;
using openbus::config::trim;

std::string kindName(ModelConfigKind kind) {
    switch (kind) {
        case ModelConfigKind::Vehicle:
            return "Vehicle";
        case ModelConfigKind::Bus:
            return "Bus";
        case ModelConfigKind::SceneryObject:
            return "SceneryObject";
    }
    return "Model";
}

bool isKnownKeyword(const std::string& keyword) {
    static const std::unordered_set<std::string> keywords = {
        "absheight",          "alphascale",       "animparent",          "boundingbox",
        "collision_mesh",     "ctc",              "ctctexture",          "fixed",
        "illumination_interior", "interiorlight", "isshadow",            "light_enh",
        "light_enh_2",        "lod",              "matl",                "matl_alpha",
        "matl_bumpmap",       "matl_change",      "matl_envmap",         "matl_freetex",
        "matl_item",          "matl_lightmap",    "matl_nightmap",       "matl_nozcheck",
        "matl_nozwrite",      "matl_texadress_border", "matl_texadress_clamp",
        "matl_texadress_mirror", "matl_texadress_mirroronce", "matl_transmap", "mesh",
        "mesh_ident",         "mouseevent",       "newanim",              "nocollision",
        "rendertype",         "scripttexture",    "spotlight",            "tcoordtransx",
        "tcoordtransy",       "tex_detail_factor", "texcoordtransx",      "texcoordtransy",
        "texttexture",        "texttexture_enh", "usescripttexture",     "usetexttexture",
        "vfdmaxmin",          "viewpoint",        "visible"};
    return keywords.find(keyword) != keywords.end();
}

bool validForKind(const std::string& keyword, ModelConfigKind kind) {
    if ((keyword == "animparent" || keyword == "mesh_ident") &&
        kind != ModelConfigKind::Bus) {
        return false;
    }
    if ((keyword == "illumination_interior" || keyword == "interiorlight" ||
         keyword == "spotlight") &&
        kind != ModelConfigKind::Bus) {
        return false;
    }
    if (keyword == "rendertype" && kind != ModelConfigKind::SceneryObject) {
        return false;
    }
    if ((keyword == "isshadow" || keyword == "viewpoint") &&
        kind == ModelConfigKind::SceneryObject) {
        return false;
    }
    if ((keyword == "collision_mesh" || keyword == "nocollision") &&
        kind != ModelConfigKind::SceneryObject) {
        return false;
    }
    if (keyword == "mouseevent" && kind == ModelConfigKind::Vehicle) {
        return false;
    }
    return true;
}

bool readValues(Reader& reader, std::size_t line, const std::string& keyword, std::size_t count,
                std::vector<std::string>& values, ConfigurationDiagnostics& diagnostics) {
    if (!reader.readPayloads(count, values, diagnostics, keyword)) {
        return false;
    }
    if (values.size() != count) {
        diagnostics.error(line, keyword, "incomplete record");
        return false;
    }
    return true;
}

bool readDoubleValue(Reader& reader, const std::string& keyword, double& value,
                     ConfigurationDiagnostics& diagnostics) {
    Line valueLine;
    if (!reader.readPayload(valueLine, diagnostics, keyword)) {
        return false;
    }
    if (!parseDouble(valueLine.text, value)) {
        diagnostics.error(valueLine.number, keyword, "expected a numeric value");
        return false;
    }
    return true;
}

bool readIntValue(Reader& reader, const std::string& keyword, int& value,
                  ConfigurationDiagnostics& diagnostics) {
    Line valueLine;
    if (!reader.readPayload(valueLine, diagnostics, keyword)) {
        return false;
    }
    if (!parseInt(valueLine.text, value)) {
        diagnostics.error(valueLine.number, keyword, "expected an integer value");
        return false;
    }
    return true;
}

void declareIfVariable(const std::string& value, Variables& variables) {
    double numericValue = 0.0;
    if (!value.empty() && !parseDouble(value, numericValue)) {
        variables.declare(value);
    }
}

void writeJsonString(std::ostream& output, const std::string& value) {
    output << '"';
    for (const char character : value) {
        switch (character) {
            case '"':
                output << "\\\"";
                break;
            case '\\':
                output << "\\\\";
                break;
            case '\n':
                output << "\\n";
                break;
            case '\r':
                output << "\\r";
                break;
            case '\t':
                output << "\\t";
                break;
            default:
                output << character;
                break;
        }
    }
    output << '"';
}

void writeModelAnimationsJson(std::ostream& output,
                              const std::vector<ModelAnimation>& animations) {
    output << '[';
    for (std::size_t index = 0; index < animations.size(); ++index) {
        if (index != 0) {
            output << ',';
        }
        const ModelAnimation& animation = animations[index];
        output << "{\"type\":";
        writeJsonString(output, animation.type);
        output << ",\"variable\":";
        writeJsonString(output, animation.variable);
        output << ",\"scale\":" << animation.scale << '}';
    }
    output << ']';
}

std::filesystem::path resolveMesh(const std::filesystem::path& modelRoot,
                                  const std::string& meshValue) {
    std::string normalized = trim(meshValue);
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    std::filesystem::path relative = normalized;
    relative.replace_extension(".obj");
    std::filesystem::path candidate = modelRoot / relative;
    if (std::filesystem::exists(candidate)) {
        return candidate;
    }
    candidate = modelRoot / relative.filename();
    if (std::filesystem::exists(candidate)) {
        return candidate;
    }
    return {};
}

bool parseNewAnimation(Reader& reader, const Line& keywordLine, ModelPart& part,
                       Variables& variables, ConfigurationDiagnostics& diagnostics) {
    ModelWheelAnimation animation;
    std::string animationVariable;
    bool hasTransform = false;
    Line field;
    while (reader.next(field)) {
        if (field.isKeyword()) {
            reader.pushBack(std::move(field));
            break;
        }
        if (field.text == "--") {
            break;
        }
        if (!field.text.empty() && (field.text.front() == '#' || field.text.front() == '\'')) {
            break;
        }
        const std::string name = lower(field.text);
        if (name == "origin_from_mesh") {
            Line possibleOrigin;
            if (reader.next(possibleOrigin)) {
                double firstCoordinate = 0.0;
                if (possibleOrigin.isKeyword() ||
                    !parseDouble(possibleOrigin.text, firstCoordinate)) {
                    reader.pushBack(std::move(possibleOrigin));
                } else {
                    std::vector<std::string> remainingCoordinates;
                    if (!readValues(reader, possibleOrigin.number, "origin_from_mesh", 2,
                                    remainingCoordinates, diagnostics)) {
                        return false;
                    }
                    double secondCoordinate = 0.0;
                    double thirdCoordinate = 0.0;
                    if (!parseDouble(remainingCoordinates[0], secondCoordinate) ||
                        !parseDouble(remainingCoordinates[1], thirdCoordinate)) {
                        diagnostics.error(possibleOrigin.number, "origin_from_mesh",
                                           "expected numeric coordinates");
                        return false;
                    }
                    animation.origin = {secondCoordinate, -firstCoordinate, thirdCoordinate};
                    animation.hasOrigin = true;
                }
            }
            continue;
        }
        if (name == "origin_trans") {
            std::vector<std::string> values;
            if (!readValues(reader, field.number, "origin_trans", 3, values, diagnostics)) {
                return false;
            }
            std::array<double, 3> sourceOrigin = {};
            for (std::size_t index = 0; index < sourceOrigin.size(); ++index) {
                if (!parseDouble(values[index], sourceOrigin[index])) {
                    diagnostics.error(field.number, "origin_trans", "expected numeric coordinates");
                    return false;
                }
            }
            animation.origin = {sourceOrigin[1], -sourceOrigin[0], sourceOrigin[2]};
            animation.hasOrigin = true;
            continue;
        }
        if (name == "origin_rot_x" || name == "origin_rot_y" || name == "origin_rot_z" ||
            name == "delay" || name == "maxspeed" || name == "offset") {
            Line value;
            if (!reader.readPayload(value, diagnostics, name)) {
                return false;
            }
            double ignored = 0.0;
            if (!parseDouble(value.text, ignored)) {
                diagnostics.error(value.number, name, "expected a numeric value");
                return false;
            }
            continue;
        }
        if (name == "anim_rot" || name == "anim_trans") {
            std::vector<std::string> values;
            if (!readValues(reader, field.number, name, 2, values, diagnostics)) {
                return false;
            }
            animationVariable = trim(values[0]);
            if (animationVariable.empty()) {
                diagnostics.error(field.number, name, "animation variable cannot be empty");
                return false;
            }
            double scale = 0.0;
            if (!parseDouble(values[1], scale)) {
                diagnostics.error(field.number, name, "expected a numeric scale");
                return false;
            }
            part.animations.push_back({name, animationVariable, scale});
            variables.declare(animationVariable);
            hasTransform = true;
            continue;
        }
        diagnostics.warning(field.number, "newanim", "unrecognized animation field: " + field.text);
    }
    if (!hasTransform) {
        diagnostics.error(keywordLine.number, "newanim", "animation has no anim_rot or anim_trans");
        return false;
    }

    const std::string variable = lower(animationVariable);
    if (variable.rfind("wheel_rotation_", 0) == 0) {
        part.wheelAnimation.rotationVariable = animationVariable;
        if (animation.hasOrigin) {
            part.wheelAnimation.origin = animation.origin;
            part.wheelAnimation.hasOrigin = true;
        }
    } else if (variable.rfind("axle_suspension_", 0) == 0) {
        part.wheelAnimation.suspensionVariable = animationVariable;
    } else if (variable.rfind("axle_steering_", 0) == 0) {
        part.wheelAnimation.steeringVariable = animationVariable;
    }
    return true;
}

}  // namespace

ModelConfig loadModelConfig(const std::filesystem::path& configPath,
                            const std::filesystem::path& modelRoot, ModelConfigKind kind,
                            Variables& variables) {
    ModelConfig result;
    openbus::config::Reader reader(configPath);
    if (!reader.isOpen()) {
        result.diagnostics.error(0, "file", "unable to open " + configPath.string());
        return result;
    }

    std::size_t currentPartIndex = static_cast<std::size_t>(-1);
    std::string currentMaterialKey;
    int currentLodIndex = -1;
    std::unordered_set<std::string> meshIdentifiers;
    Line line;
    while (reader.next(line)) {
        if (!line.isKeyword()) {
            continue;
        }
        const std::string keyword = lower(line.keyword());
        if (!isKnownKeyword(keyword)) {
            result.diagnostics.warning(line.number, line.keyword(), "unknown keyword");
            continue;
        }
        if (!validForKind(keyword, kind)) {
            result.diagnostics.error(line.number, line.keyword(),
                                     "keyword is not valid in a " + kindName(kind) + " model");
        }

        const auto part = [&]() -> ModelPart* {
            if (currentPartIndex >= result.parts.size()) {
                return nullptr;
            }
            return &result.parts[currentPartIndex];
        };
        const auto requirePart = [&]() -> ModelPart* {
            ModelPart* current = part();
            if (current == nullptr) {
                result.diagnostics.error(line.number, line.keyword(),
                                         "keyword must follow [mesh]");
            }
            return current;
        };
        const auto material = [&]() -> ModelMaterialState* {
            ModelPart* current = part();
            if (current == nullptr || currentMaterialKey.empty()) {
                return nullptr;
            }
            const auto found = current->materialStates.find(currentMaterialKey);
            return found == current->materialStates.end() ? nullptr : &found->second;
        };
        const auto requireMaterial = [&]() -> ModelMaterialState* {
            ModelMaterialState* current = material();
            if (current == nullptr) {
                result.diagnostics.error(line.number, line.keyword(),
                                         "material keyword must follow [matl]");
            }
            return current;
        };

        if (keyword == "lod") {
            double threshold = 0.0;
            if (!readDoubleValue(reader, "LOD", threshold, result.diagnostics)) {
                continue;
            }
            if (threshold <= 0.0) {
                result.diagnostics.error(line.number, "LOD", "threshold must be greater than zero");
                currentLodIndex = -1;
            } else {
                if (!result.lodThresholds.empty() &&
                    threshold <= result.lodThresholds.back()) {
                    result.diagnostics.error(line.number, "LOD",
                                             "thresholds must increase in file order");
                }
                currentLodIndex = static_cast<int>(result.lodThresholds.size());
                result.lodThresholds.push_back(threshold);
            }
            currentPartIndex = static_cast<std::size_t>(-1);
            currentMaterialKey.clear();
            continue;
        }
        if (keyword == "mesh") {
            Line meshLine;
            if (!reader.readPayload(meshLine, result.diagnostics, "mesh")) {
                continue;
            }
            const std::string meshValue = trim(meshLine.text);
            const std::filesystem::path objPath = resolveMesh(modelRoot, meshValue);
            if (objPath.empty()) {
                result.diagnostics.warning(meshLine.number, "mesh",
                                           "converted mesh was not found: " + meshValue);
            }
            ModelPart newPart;
            newPart.objPath = objPath;
            newPart.sourceMeshPath = meshValue;
            newPart.lodIndex = currentLodIndex;
            result.parts.push_back(std::move(newPart));
            currentPartIndex = result.parts.size() - 1;
            currentMaterialKey.clear();
            continue;
        }
        if (keyword == "mesh_ident") {
            ModelPart* current = requirePart();
            Line value;
            if (current != nullptr && reader.readPayload(value, result.diagnostics, "mesh_ident")) {
                const std::string identifier = trim(value.text);
                if (identifier.empty()) {
                    result.diagnostics.error(value.number, "mesh_ident", "identifier cannot be empty");
                } else if (!meshIdentifiers.insert(identifier).second) {
                    result.diagnostics.error(value.number, "mesh_ident",
                                             "duplicate mesh identifier: " + identifier);
                } else {
                    current->meshIdentifier = identifier;
                }
            }
            continue;
        }
        if (keyword == "animparent") {
            ModelPart* current = requirePart();
            Line value;
            if (current != nullptr && reader.readPayload(value, result.diagnostics, "animparent")) {
                current->animationParent = trim(value.text);
                if (current->animationParent.empty()) {
                    result.diagnostics.error(value.number, "animparent", "parent identifier cannot be empty");
                }
            }
            continue;
        }
        if (keyword == "viewpoint") {
            ModelPart* current = requirePart();
            int viewpoint = 0;
            if (current != nullptr && readIntValue(reader, "viewpoint", viewpoint, result.diagnostics)) {
                if (viewpoint < 0 || viewpoint > 7) {
                    result.diagnostics.error(line.number, "viewpoint", "value must be between 0 and 7");
                }
                current->viewpoint = viewpoint;
            }
            continue;
        }
        if (keyword == "rendertype") {
            ModelPart* current = requirePart();
            Line value;
            if (current != nullptr && reader.readPayload(value, result.diagnostics, "rendertype")) {
                int renderType = 2;
                if (lower(value.text) == "surface") {
                    renderType = 2;
                } else if (!parseInt(value.text, renderType)) {
                    result.diagnostics.error(value.number, "rendertype", "expected a render type");
                }
                current->renderType = renderType;
            }
            continue;
        }
        if (keyword == "visible") {
            ModelPart* current = requirePart();
            std::vector<std::string> values;
            if (current != nullptr && readValues(reader, line.number, "visible", 2, values,
                                                 result.diagnostics)) {
                int value = 0;
                if (!parseInt(values[1], value)) {
                    result.diagnostics.error(line.number, "visible", "expected an integer value");
                } else {
                    current->visibleVariable = trim(values[0]);
                    current->visibleValue = value;
                    variables.declare(current->visibleVariable);
                }
            }
            continue;
        }
        if (keyword == "illumination_interior") {
            ModelPart* current = requirePart();
            std::vector<std::string> values;
            if (current != nullptr && readValues(reader, line.number, keyword, 4, values,
                                                 result.diagnostics)) {
                for (std::size_t index = 0; index < values.size(); ++index) {
                    int value = -1;
                    if (!parseInt(values[index], value)) {
                        result.diagnostics.error(line.number, keyword, "expected integer light indexes");
                        break;
                    }
                    if (value < -1) {
                        result.diagnostics.error(line.number, keyword,
                                                 "light indexes must be -1 or greater");
                    }
                    current->interiorLightIndexes[index] = value;
                }
            }
            continue;
        }
        if (keyword == "interiorlight") {
            if (requirePart() != nullptr) {
                std::vector<std::string> values;
                if (readValues(reader, line.number, keyword, 8, values, result.diagnostics)) {
                    declareIfVariable(values[0], variables);
                }
            }
            continue;
        }
        if (keyword == "light_enh" || keyword == "light_enh_2") {
            if (requirePart() != nullptr) {
                const std::size_t count = keyword == "light_enh" ? 13 : 24;
                std::vector<std::string> values;
                if (readValues(reader, line.number, keyword, count, values, result.diagnostics)) {
                    declareIfVariable(values[keyword == "light_enh" ? 7 : 17], variables);
                }
            }
            continue;
        }
        if (keyword == "spotlight") {
            if (requirePart() != nullptr) {
                std::vector<std::string> values;
                readValues(reader, line.number, keyword, 12, values, result.diagnostics);
            }
            continue;
        }
        if (keyword == "newanim") {
            ModelPart* current = requirePart();
            if (current != nullptr) {
                parseNewAnimation(reader, line, *current, variables, result.diagnostics);
            }
            continue;
        }
        if (keyword == "mouseevent") {
            if (requirePart() != nullptr) {
                Line value;
                reader.readPayload(value, result.diagnostics, keyword);
            }
            continue;
        }
        if (keyword == "matl") {
            ModelPart* current = requirePart();
            std::vector<std::string> values;
            if (current != nullptr && readValues(reader, line.number, keyword, 2, values,
                                                 result.diagnostics)) {
                const std::string textureName = trim(values[0]);
                currentMaterialKey = lower(std::filesystem::path(textureName).filename().string());
                ModelMaterialState state;
                state.textureName = textureName;
                state.texturePath = textureName;
                current->materialStates[currentMaterialKey] = std::move(state);
                if (current->textureName.empty()) {
                    current->textureName = textureName;
                }
            }
            continue;
        }
        if (keyword == "matl_alpha") {
            ModelMaterialState* current = requireMaterial();
            int alphaMode = 0;
            if (current != nullptr && readIntValue(reader, keyword, alphaMode, result.diagnostics)) {
                if (alphaMode < 0 || alphaMode > 2) {
                    result.diagnostics.error(line.number, keyword, "mode must be 0, 1, or 2");
                }
                current->alphaMode = alphaMode;
            }
            continue;
        }
        if (keyword == "alphascale") {
            ModelMaterialState* current = requireMaterial();
            Line value;
            if (current != nullptr && reader.readPayload(value, result.diagnostics, keyword)) {
                current->alphaScaleVariable = trim(value.text);
                variables.declare(current->alphaScaleVariable);
            }
            continue;
        }
        if (keyword == "matl_nozwrite") {
            ModelMaterialState* current = requireMaterial();
            if (current != nullptr) {
                current->noZwrite = true;
            }
            continue;
        }
        if (keyword == "matl_envmap") {
            ModelMaterialState* current = requireMaterial();
            std::vector<std::string> values;
            if (current != nullptr && readValues(reader, line.number, keyword, 2, values,
                                                 result.diagnostics)) {
                current->environmentTextureName = trim(values[0]);
                if (!parseDouble(values[1], current->environmentStrength)) {
                    result.diagnostics.error(line.number, keyword, "expected numeric strength");
                }
            }
            continue;
        }
        if (keyword == "matl_bumpmap" || keyword == "matl_transmap" ||
            keyword == "matl_nightmap") {
            ModelMaterialState* current = requireMaterial();
            if (current != nullptr) {
                const std::size_t count = keyword == "matl_bumpmap" ? 2 : 1;
                std::vector<std::string> values;
                readValues(reader, line.number, keyword, count, values, result.diagnostics);
            }
            continue;
        }
        if (keyword == "matl_lightmap" || keyword == "matl_freetex") {
            ModelMaterialState* current = requireMaterial();
            if (current != nullptr) {
                Line texture;
                if (reader.readPayload(texture, result.diagnostics, keyword)) {
                    Line variable;
                    if (reader.next(variable)) {
                        if (variable.isKeyword()) {
                            reader.pushBack(std::move(variable));
                        } else if (variable.text != "--") {
                            declareIfVariable(variable.text, variables);
                        }
                    } else if (keyword == "matl_freetex") {
                        result.diagnostics.error(line.number, keyword,
                                                 "missing free-text variable");
                    }
                }
            }
            continue;
        }
        if (keyword == "matl_change") {
            ModelPart* current = requirePart();
            if (current != nullptr) {
                std::vector<std::string> values;
                if (readValues(reader, line.number, keyword, 3, values, result.diagnostics)) {
                    const std::string textureName = trim(values[0]);
                    currentMaterialKey =
                        lower(std::filesystem::path(textureName).filename().string());
                    if (current->materialStates.find(currentMaterialKey) ==
                        current->materialStates.end()) {
                        ModelMaterialState state;
                        state.textureName = textureName;
                        state.texturePath = textureName;
                        current->materialStates.emplace(currentMaterialKey, std::move(state));
                    }
                    declareIfVariable(values[2], variables);
                }
            }
            continue;
        }
        if (keyword == "matl_item" || keyword == "matl_nozcheck" ||
            keyword == "matl_texadress_border" || keyword == "matl_texadress_clamp" ||
            keyword == "matl_texadress_mirror" || keyword == "matl_texadress_mirroronce") {
            requireMaterial();
            continue;
        }
        if (keyword == "texcoordtransx" || keyword == "texcoordtransy") {
            if (requireMaterial() != nullptr) {
                Line value;
                if (reader.readPayload(value, result.diagnostics, keyword)) {
                    declareIfVariable(value.text, variables);
                }
            }
            continue;
        }
        if (keyword == "usetexttexture" || keyword == "usescripttexture") {
            if (requireMaterial() != nullptr) {
                int index = 0;
                readIntValue(reader, keyword, index, result.diagnostics);
            }
            continue;
        }
        if (keyword == "fixed" || keyword == "absheight" || keyword == "isshadow") {
            requirePart();
            continue;
        }
        if (keyword == "boundingbox") {
            std::vector<std::string> values;
            readValues(reader, line.number, keyword, 6, values, result.diagnostics);
            continue;
        }
        if (keyword == "collision_mesh") {
            Line value;
            reader.readPayload(value, result.diagnostics, keyword);
            continue;
        }
        if (keyword == "nocollision") {
            continue;
        }
        if (keyword == "vfdmaxmin") {
            std::vector<std::string> values;
            readValues(reader, line.number, keyword, 6, values, result.diagnostics);
            continue;
        }
        if (keyword == "tex_detail_factor") {
            std::vector<std::string> values;
            readValues(reader, line.number, keyword, 2, values, result.diagnostics);
            continue;
        }
        if (keyword == "ctc") {
            std::vector<std::string> values;
            readValues(reader, line.number, keyword, 3, values, result.diagnostics);
            continue;
        }
        if (keyword == "ctctexture") {
            std::vector<std::string> values;
            readValues(reader, line.number, keyword, 2, values, result.diagnostics);
            continue;
        }
        if (keyword == "scripttexture") {
            std::vector<std::string> values;
            readValues(reader, line.number, keyword, 2, values, result.diagnostics);
            continue;
        }
        if (keyword == "texttexture" || keyword == "texttexture_enh") {
            Line value;
            while (reader.next(value)) {
                if (value.isKeyword()) {
                    reader.pushBack(std::move(value));
                    break;
                }
            }
            continue;
        }
    }

    for (const ModelPart& part : result.parts) {
        if (!part.animationParent.empty() && meshIdentifiers.find(part.animationParent) == meshIdentifiers.end()) {
            result.diagnostics.error(0, "animparent",
                                     "unknown parent mesh identifier: " + part.animationParent);
        }
    }
    if (result.parts.empty()) {
        result.diagnostics.error(0, "mesh", "model contains no loadable meshes");
    }
    return result;
}

void writeModelConfigurationJson(std::ostream& output,
                                 const std::filesystem::path& configPath,
                                 const ModelConfig& configuration) {
    output << std::setprecision(17)
           << "{\n"
              "  \"format_version\": 1,\n"
              "  \"config_path\": ";
    writeJsonString(output, configPath.generic_string());
    output << ",\n  \"lod_thresholds\": [";
    for (std::size_t index = 0; index < configuration.lodThresholds.size(); ++index) {
        if (index != 0) {
            output << ',';
        }
        output << configuration.lodThresholds[index];
    }
    output << "],\n  \"parts\": [\n";

    for (std::size_t index = 0; index < configuration.parts.size(); ++index) {
        if (index != 0) {
            output << ",\n";
        }
        const ModelPart& part = configuration.parts[index];
        const ModelPart* parent = nullptr;
        if (!part.animationParent.empty()) {
            const auto parentIterator = std::find_if(
                configuration.parts.begin(), configuration.parts.end(), [&](const ModelPart& candidate) {
                    return candidate.meshIdentifier == part.animationParent;
                });
            if (parentIterator != configuration.parts.end()) {
                parent = &*parentIterator;
            }
        }

        output << "    {\n      \"source_mesh\": ";
        writeJsonString(output, part.sourceMeshPath.generic_string());
        output << ",\n      \"resolved_mesh\": ";
        writeJsonString(output, part.objPath.generic_string());
        output << ",\n      \"texture_name\": ";
        writeJsonString(output, part.textureName);
        output << ",\n      \"color\": [" << part.color[0] << ',' << part.color[1] << ','
               << part.color[2] << "],\n"
                  "      \"viewpoint\": "
               << part.viewpoint << ",\n"
                  "      \"render_type\": "
               << part.renderType << ",\n      \"mesh_identifier\": ";
        writeJsonString(output, part.meshIdentifier);
        output << ",\n      \"animation_parent\": ";
        writeJsonString(output, part.animationParent);
        output << ",\n      \"visibility\": {\"variable\": ";
        writeJsonString(output, part.visibleVariable);
        output << ",\"value\": " << part.visibleValue << "},\n"
                  "      \"interior_light_indexes\": ["
               << part.interiorLightIndexes[0] << ',' << part.interiorLightIndexes[1] << ','
               << part.interiorLightIndexes[2] << ',' << part.interiorLightIndexes[3] << "],\n"
                  "      \"lod_index\": "
               << part.lodIndex << ",\n      \"animations\": ";
        writeModelAnimationsJson(output, part.animations);
        output << ",\n      \"parent_animations\": ";
        if (parent != nullptr) {
            writeModelAnimationsJson(output, parent->animations);
        } else {
            output << "[]";
        }
        output << ",\n      \"wheel_animation\": {\"rotation_variable\": ";
        writeJsonString(output, part.wheelAnimation.rotationVariable);
        output << ",\"suspension_variable\": ";
        writeJsonString(output, part.wheelAnimation.suspensionVariable);
        output << ",\"steering_variable\": ";
        writeJsonString(output, part.wheelAnimation.steeringVariable);
        output << ",\"origin\": [" << part.wheelAnimation.origin[0] << ','
               << part.wheelAnimation.origin[1] << ',' << part.wheelAnimation.origin[2]
               << "],\"has_origin\": "
               << (part.wheelAnimation.hasOrigin ? "true" : "false") << "},\n"
                  "      \"materials\": [";

        std::vector<std::string> materialKeys;
        materialKeys.reserve(part.materialStates.size());
        for (const auto& material : part.materialStates) {
            materialKeys.push_back(material.first);
        }
        std::sort(materialKeys.begin(), materialKeys.end());
        for (std::size_t materialIndex = 0; materialIndex < materialKeys.size(); ++materialIndex) {
            if (materialIndex != 0) {
                output << ',';
            }
            const ModelMaterialState& material = part.materialStates.at(materialKeys[materialIndex]);
            output << "{\"key\":";
            writeJsonString(output, materialKeys[materialIndex]);
            output << ",\"texture_name\":";
            writeJsonString(output, material.textureName);
            output << ",\"texture_path\":";
            writeJsonString(output, material.texturePath.generic_string());
            output << ",\"environment_texture\":";
            writeJsonString(output, material.environmentTextureName);
            output << ",\"environment_strength\":" << material.environmentStrength
                   << ",\"alpha_mode\":" << material.alphaMode
                   << ",\"no_zwrite\":" << (material.noZwrite ? "true" : "false")
                   << ",\"alpha_scale_variable\":";
            writeJsonString(output, material.alphaScaleVariable);
            output << '}';
        }
        output << "]\n    }";
    }

    output << "\n  ],\n  \"diagnostics\": [";
    for (std::size_t index = 0; index < configuration.diagnostics.entries.size(); ++index) {
        if (index != 0) {
            output << ',';
        }
        const ConfigurationDiagnostic& diagnostic = configuration.diagnostics.entries[index];
        output << "{\"severity\":";
        writeJsonString(output, diagnostic.severity == ConfigurationDiagnostic::Severity::Error
                                  ? "error"
                                  : "warning");
        output << ",\"line\":" << diagnostic.line << ",\"keyword\":";
        writeJsonString(output, diagnostic.keyword);
        output << ",\"message\":";
        writeJsonString(output, diagnostic.message);
        output << '}';
    }
    output << "]\n}\n";
}
