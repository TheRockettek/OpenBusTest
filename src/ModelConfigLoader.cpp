#include "ModelConfigLoader.h"

#include "ConfigurationParser.h"
#include "PerfTrace.h"
#include "Variables.h"

#include <algorithm>
#include <array>
#include <filesystem>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

using openbus::config::Line;
using openbus::config::lower;
using openbus::config::parseDouble;
using openbus::config::parseInt;
using openbus::config::Reader;
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
    // Keep this list synchronized with the dispatch table below so unsupported
    // records produce diagnostics instead of being silently ignored.
    static const std::unordered_set<std::string> keywords = {"absheight",
                                                             "alphascale",
                                                             "animparent",
                                                             "boundingbox",
                                                             "collision_mesh",
                                                             "ctc",
                                                             "ctctexture",
                                                             "fixed",
                                                             "illumination_interior",
                                                             "interiorlight",
                                                             "isshadow",
                                                             "light_enh",
                                                             "light_enh_2",
                                                             "lod",
                                                             "matl",
                                                             "matl_alpha",
                                                             "matl_bumpmap",
                                                             "matl_change",
                                                             "matl_envmap",
                                                             "matl_freetex",
                                                             "matl_item",
                                                             "matl_lightmap",
                                                             "matl_nightmap",
                                                             "matl_nozcheck",
                                                             "matl_nozwrite",
                                                             "matl_texadress_border",
                                                             "matl_texadress_clamp",
                                                             "matl_texadress_mirror",
                                                             "matl_texadress_mirroronce",
                                                             "matl_transmap",
                                                             "mesh",
                                                             "mesh_ident",
                                                             "mouseevent",
                                                             "newanim",
                                                             "nocollision",
                                                             "rendertype",
                                                             "scripttexture",
                                                             "spotlight",
                                                             "tcoordtransx",
                                                             "tcoordtransy",
                                                             "tex_detail_factor",
                                                             "texcoordtransx",
                                                             "texcoordtransy",
                                                             "texttexture",
                                                             "texttexture_enh",
                                                             "usescripttexture",
                                                             "usetexttexture",
                                                             "vfdmaxmin",
                                                             "viewpoint",
                                                             "visible"};
    return keywords.find(keyword) != keywords.end();
}

bool validForKind(const std::string& keyword, ModelConfigKind kind) {
    // Some CFG records are legal only for bus, vehicle, or scenery models.
    if ((keyword == "animparent" || keyword == "mesh_ident") && kind != ModelConfigKind::Bus) {
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
    // Read fixed-width records through the shared reader so missing payloads
    // are reported with the keyword and source location.
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
    // Configuration records mix numeric constants and script-variable names.
    double numericValue = 0.0;
    if (!value.empty() && !parseDouble(value, numericValue)) {
        variables.declare(value);
    }
}

std::filesystem::path resolveMesh(const std::filesystem::path& modelRoot,
                                  const std::string& meshValue) {
    // Converted OBJ files normally preserve the source subdirectory, but some
    // converters flatten it; try both layouts before reporting a missing mesh.
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
    // [newanim] is a variable-length block terminated by the next keyword or '--'.
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

} // namespace

ModelConfig loadModelConfig(const std::filesystem::path& configPath,
                            const std::filesystem::path& modelRoot, ModelConfigKind kind,
                            Variables& variables) {
    openbus::rendering::TraceScope trace("config", "loadModelConfig");
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
                result.diagnostics.error(line.number, line.keyword(), "keyword must follow [mesh]");
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

        // The parser keeps the current mesh, material, and LOD as context for
        // subsequent records in the configuration stream.
        // LOD records change the default LOD for following meshes and reset the
        // current mesh/material context.
        // [lod]: one positive threshold; subsequent meshes use the new LOD index.
        if (keyword == "lod") {
            double threshold = 0.0;
            if (!readDoubleValue(reader, "LOD", threshold, result.diagnostics)) {
                continue;
            }
            if (threshold <= 0.0) {
                result.diagnostics.error(line.number, "LOD", "threshold must be greater than zero");
                currentLodIndex = -1;
            } else {
                if (!result.lodThresholds.empty() && threshold <= result.lodThresholds.back()) {
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
        // [mesh]: one source .o3d path; it is mapped to the converted .obj path.
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
            // Store both source text and resolved path for diagnostics and export.
            newPart.objPath = objPath;
            newPart.sourceMeshPath = meshValue;
            newPart.lodIndex = currentLodIndex;
            result.parts.push_back(std::move(newPart));
            currentPartIndex = result.parts.size() - 1;
            currentMaterialKey.clear();
            continue;
        }
        // [mesh_ident]: one unique identifier used by later [animparent] records.
        if (keyword == "mesh_ident") {
            ModelPart* current = requirePart();
            Line value;
            if (current != nullptr && reader.readPayload(value, result.diagnostics, "mesh_ident")) {
                const std::string identifier = trim(value.text);
                if (identifier.empty()) {
                    result.diagnostics.error(value.number, "mesh_ident",
                                             "identifier cannot be empty");
                } else if (!meshIdentifiers.insert(identifier).second) {
                    result.diagnostics.error(value.number, "mesh_ident",
                                             "duplicate mesh identifier: " + identifier);
                } else {
                    // Identifiers are later used to validate [animparent] links.
                    current->meshIdentifier = identifier;
                }
            }
            continue;
        }
        // [animparent]: one previously declared [mesh_ident] to inherit transforms from.
        if (keyword == "animparent") {
            ModelPart* current = requirePart();
            Line value;
            if (current != nullptr && reader.readPayload(value, result.diagnostics, "animparent")) {
                current->animationParent = trim(value.text);
                if (current->animationParent.empty()) {
                    result.diagnostics.error(value.number, "animparent",
                                             "parent identifier cannot be empty");
                }
            }
            continue;
        }
        // [viewpoint]: one mask from 0 through 7 selecting exterior/interior views.
        if (keyword == "viewpoint") {
            ModelPart* current = requirePart();
            int viewpoint = 0;
            if (current != nullptr &&
                readIntValue(reader, "viewpoint", viewpoint, result.diagnostics)) {
                if (viewpoint < 0 || viewpoint > 7) {
                    result.diagnostics.error(line.number, "viewpoint",
                                             "value must be between 0 and 7");
                }
                current->viewpoint = viewpoint;
            }
            continue;
        }
        // [rendertype]: one render mode, or the literal "surface" (mode 2).
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
        // [visible]: script variable followed by the integer value that permits drawing.
        if (keyword == "visible") {
            ModelPart* current = requirePart();
            std::vector<std::string> values;
            if (current != nullptr &&
                readValues(reader, line.number, "visible", 2, values, result.diagnostics)) {
                int value = 0;
                if (!parseInt(values[1], value)) {
                    result.diagnostics.error(line.number, "visible", "expected an integer value");
                } else {
                    // The renderer compares this variable with the target at frame time.
                    current->visibleVariable = trim(values[0]);
                    current->visibleValue = value;
                    variables.declare(current->visibleVariable);
                }
            }
            continue;
        }
        // [illumination_interior]: four integer light-group indexes; -1 disables a slot.
        if (keyword == "illumination_interior") {
            ModelPart* current = requirePart();
            std::vector<std::string> values;
            if (current != nullptr &&
                readValues(reader, line.number, keyword, 4, values, result.diagnostics)) {
                // -1 means that an illumination slot is unused; other values
                // identify the corresponding light group.
                for (std::size_t index = 0; index < values.size(); ++index) {
                    int value = -1;
                    if (!parseInt(values[index], value)) {
                        result.diagnostics.error(line.number, keyword,
                                                 "expected integer light indexes");
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
        // [interiorlight]: eight fields; field 0 is the controller, followed by
        // position, colour, intensity, and mode data retained only for validation.
        if (keyword == "interiorlight") {
            if (requirePart() != nullptr) {
                std::vector<std::string> values;
                if (readValues(reader, line.number, keyword, 8, values, result.diagnostics)) {
                    // The first field controls the light; the remaining fields
                    // are validated but are not rendered yet.
                    declareIfVariable(values[0], variables);
                }
            }
            continue;
        }
        // [light_enh]: 13 fields; [light_enh_2]: 24 fields. Each has a fixed
        // controller-variable field, while the remaining light data is preserved
        // by consuming the record but is not yet represented in the model.
        if (keyword == "light_enh" || keyword == "light_enh_2") {
            if (requirePart() != nullptr) {
                // These layouts have different widths and fixed variable positions.
                const std::size_t count = keyword == "light_enh" ? 13 : 24;
                std::vector<std::string> values;
                if (readValues(reader, line.number, keyword, count, values, result.diagnostics)) {
                    declareIfVariable(values[keyword == "light_enh" ? 7 : 17], variables);
                }
            }
            continue;
        }
        // [spotlight]: twelve position, direction, colour, range, and cone fields.
        if (keyword == "spotlight") {
            if (requirePart() != nullptr) {
                std::vector<std::string> values;
                // Validate the complete record even though spotlight fields are
                // not yet represented in the runtime model.
                readValues(reader, line.number, keyword, 12, values, result.diagnostics);
            }
            continue;
        }
        // [newanim]: a variable-length block containing origin fields and anim_rot/
        // anim_trans pairs of controller variable plus scale.
        if (keyword == "newanim") {
            ModelPart* current = requirePart();
            if (current != nullptr) {
                parseNewAnimation(reader, line, *current, variables, result.diagnostics);
            }
            continue;
        }
        // [mouseevent]: one event identifier; the event behavior is script-defined.
        if (keyword == "mouseevent") {
            if (requirePart() != nullptr) {
                Line value;
                // Consume the event identifier so the next keyword stays aligned.
                reader.readPayload(value, result.diagnostics, keyword);
            }
            continue;
        }
        // [matl]: texture name plus material mode; the texture name keys following
        // material modifiers and supplies the part fallback texture.
        if (keyword == "matl") {
            ModelPart* current = requirePart();
            std::vector<std::string> values;
            if (current != nullptr &&
                readValues(reader, line.number, keyword, 2, values, result.diagnostics)) {
                const std::string textureName = trim(values[0]);
                // Material modifiers that follow are associated with this key.
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
        // [matl_alpha]: one mode, where 0 is opaque and 1/2 select alpha modes.
        if (keyword == "matl_alpha") {
            ModelMaterialState* current = requireMaterial();
            int alphaMode = 0;
            if (current != nullptr &&
                readIntValue(reader, keyword, alphaMode, result.diagnostics)) {
                if (alphaMode < 0 || alphaMode > 2) {
                    result.diagnostics.error(line.number, keyword, "mode must be 0, 1, or 2");
                }
                // Preserve the parsed value so diagnostics do not discard source data.
                current->alphaMode = alphaMode;
            }
            continue;
        }
        // [alphascale]: one script variable controlling the material alpha factor.
        if (keyword == "alphascale") {
            ModelMaterialState* current = requireMaterial();
            Line value;
            if (current != nullptr && reader.readPayload(value, result.diagnostics, keyword)) {
                current->alphaScaleVariable = trim(value.text);
                variables.declare(current->alphaScaleVariable);
            }
            continue;
        }
        // [matl_nozwrite]: marker with no payload; disables depth-buffer writes.
        if (keyword == "matl_nozwrite") {
            ModelMaterialState* current = requireMaterial();
            if (current != nullptr) {
                current->noZwrite = true;
            }
            continue;
        }
        // [matl_envmap]: environment texture name followed by reflection strength.
        if (keyword == "matl_envmap") {
            ModelMaterialState* current = requireMaterial();
            std::vector<std::string> values;
            if (current != nullptr &&
                readValues(reader, line.number, keyword, 2, values, result.diagnostics)) {
                current->environmentTextureName = trim(values[0]);
                if (!parseDouble(values[1], current->environmentStrength)) {
                    result.diagnostics.error(line.number, keyword, "expected numeric strength");
                }
            }
            continue;
        }
        // [matl_bumpmap]: texture plus mode; [matl_transmap]/[matl_nightmap]: one texture.
        if (keyword == "matl_bumpmap" || keyword == "matl_transmap" || keyword == "matl_nightmap") {
            ModelMaterialState* current = requireMaterial();
            if (current != nullptr) {
                const std::size_t count = keyword == "matl_bumpmap" ? 2 : 1;
                std::vector<std::string> values;
                readValues(reader, line.number, keyword, count, values, result.diagnostics);
            }
            continue;
        }
        // [matl_lightmap]/[matl_freetex]: texture name followed by an optional
        // script variable controlling the generated or free texture.
        if (keyword == "matl_lightmap" || keyword == "matl_freetex") {
            ModelMaterialState* current = requireMaterial();
            if (current != nullptr) {
                Line texture;
                if (reader.readPayload(texture, result.diagnostics, keyword)) {
                    Line variable;
                    // Push keywords back because the optional variable may be absent.
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
        // [matl_change]: replacement texture, texture-array layer, activation variable.
        if (keyword == "matl_change") {
            ModelPart* current = requirePart();
            if (current != nullptr) {
                std::vector<std::string> values;
                if (readValues(reader, line.number, keyword, 3, values, result.diagnostics)) {
                    const std::string textureName = trim(values[0]);
                    int layer = 0;
                    if (!parseInt(values[1], layer) || layer < 0) {
                        result.diagnostics.error(line.number, keyword,
                                                 "expected a non-negative texture layer");
                        continue;
                    }
                    const std::string activationVariable = trim(values[2]);
                    if (textureName.empty() || activationVariable.empty()) {
                        result.diagnostics.error(
                            line.number, keyword,
                            "texture name and activation variable are required");
                        continue;
                    }
                    currentMaterialKey =
                        lower(std::filesystem::path(textureName).filename().string());
                    ModelMaterialState& state = current->materialStates[currentMaterialKey];
                    // A material change is retained as runtime data; activation
                    // variables are evaluated when the renderer draws the part.
                    if (state.textureName.empty()) {
                        state.textureName = textureName;
                        state.texturePath = textureName;
                    }
                    state.textureChanges.push_back({{}, textureName, layer, activationVariable});
                    declareIfVariable(activationVariable, variables);
                }
            }
            continue;
        }
        // These material flags have no payload; their presence changes renderer state
        // or is accepted for compatibility with the source format.
        if (keyword == "matl_item" || keyword == "matl_nozcheck" ||
            keyword == "matl_texadress_border" || keyword == "matl_texadress_clamp" ||
            keyword == "matl_texadress_mirror" || keyword == "matl_texadress_mirroronce") {
            requireMaterial();
            continue;
        }
        // [texcoordtransx]/[texcoordtransy]: one script variable for UV translation.
        if (keyword == "texcoordtransx" || keyword == "texcoordtransy") {
            if (requireMaterial() != nullptr) {
                Line value;
                if (reader.readPayload(value, result.diagnostics, keyword)) {
                    declareIfVariable(value.text, variables);
                }
            }
            continue;
        }
        // [usetexttexture]/[usescripttexture]: one numeric texture-slot index.
        if (keyword == "usetexttexture" || keyword == "usescripttexture") {
            if (requireMaterial() != nullptr) {
                int index = 0;
                readIntValue(reader, keyword, index, result.diagnostics);
            }
            continue;
        }
        // [fixed]/[absheight]/[isshadow]: marker records with no payload consumed here.
        if (keyword == "fixed" || keyword == "absheight" || keyword == "isshadow") {
            requirePart();
            continue;
        }
        // [boundingbox]: six numeric bounds, normally min/max coordinates.
        if (keyword == "boundingbox") {
            std::vector<std::string> values;
            readValues(reader, line.number, keyword, 6, values, result.diagnostics);
            continue;
        }
        // [collision_mesh]: one collision mesh path consumed for diagnostics only.
        if (keyword == "collision_mesh") {
            Line value;
            reader.readPayload(value, result.diagnostics, keyword);
            continue;
        }
        // [nocollision]: marker disabling collision for the current object.
        if (keyword == "nocollision") {
            continue;
        }
        // [vfdmaxmin]: six numeric display limits for VFD/text rendering.
        if (keyword == "vfdmaxmin") {
            std::vector<std::string> values;
            readValues(reader, line.number, keyword, 6, values, result.diagnostics);
            continue;
        }
        // [tex_detail_factor]: texture-detail identifier and numeric blend factor.
        if (keyword == "tex_detail_factor") {
            std::vector<std::string> values;
            readValues(reader, line.number, keyword, 2, values, result.diagnostics);
            continue;
        }
        // [ctc]: three texture/color-template values retained for record alignment.
        if (keyword == "ctc") {
            std::vector<std::string> values;
            readValues(reader, line.number, keyword, 3, values, result.diagnostics);
            continue;
        }
        // [ctctexture]: texture name plus template slot/index.
        if (keyword == "ctctexture") {
            std::vector<std::string> values;
            readValues(reader, line.number, keyword, 2, values, result.diagnostics);
            continue;
        }
        // [scripttexture]: texture name plus script texture index.
        if (keyword == "scripttexture") {
            std::vector<std::string> values;
            readValues(reader, line.number, keyword, 2, values, result.diagnostics);
            continue;
        }
        // [texttexture]/[texttexture_enh]: variable-length text-display blocks;
        // consume until the next keyword because their field layout varies.
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

    // Parent references can only be checked after all mesh identifiers are known.
    for (const ModelPart& part : result.parts) {
        if (!part.animationParent.empty() &&
            meshIdentifiers.find(part.animationParent) == meshIdentifiers.end()) {
            result.diagnostics.error(0, "animparent",
                                     "unknown parent mesh identifier: " + part.animationParent);
        }
    }
    if (result.parts.empty()) {
        result.diagnostics.error(0, "mesh", "model contains no loadable meshes");
    }
    return result;
}
