#include "ModelConfigLoader.h"

#include "ConfigurationParser.h"
#include "PerfTrace.h"
#include "ScriptTextureLimits.h"
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
                                                             "smoke",
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
    if (keyword == "mouseevent" && kind == ModelConfigKind::Vehicle) {
        return false;
    }
    return true;
}

bool isSeparatorLine(const std::string& text) {
    if (text.empty()) {
        return false;
    }
    const char separator = text.front();
    if (separator != '-' && separator != '=' && separator != '+' && separator != '*') {
        return false;
    }
    return text.find_first_not_of(separator) == std::string::npos;
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
    const std::string normalized = lower(trim(value));
    double numericValue = 0.0;
    if (!normalized.empty() && !parseDouble(normalized, numericValue)) {
        variables.declare(normalized);
    }
}

std::filesystem::path resolveMesh(const std::filesystem::path& modelRoot,
                                  const std::string& meshValue, bool includeO3D = true) {
    // Prefer converted OBJ files, then fall back to the source O3D mesh.
    std::string normalized = trim(meshValue);
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    const std::filesystem::path source = normalized;
    const std::array<const char*, 2> extensions = {".obj", ".o3d"};
    const std::size_t extensionCount = includeO3D ? extensions.size() : 1;
    std::vector<std::filesystem::path> roots = {modelRoot};
    if (!modelRoot.parent_path().empty()) {
        roots.push_back(modelRoot.parent_path());
        std::filesystem::path sourceDirectory = modelRoot.filename();
        const std::string directoryName = sourceDirectory.string();
        if (directoryName.size() > 4 && directoryName.substr(directoryName.size() - 4) == "_obj") {
            sourceDirectory = directoryName.substr(0, directoryName.size() - 4);
            roots.push_back(modelRoot.parent_path() / sourceDirectory);
        }
    }
    for (std::size_t extensionIndex = 0; extensionIndex < extensionCount; ++extensionIndex) {
        const char* extension = extensions[extensionIndex];
        std::filesystem::path relative = source;
        relative.replace_extension(extension);
        for (const std::filesystem::path& root : roots) {
            for (const std::filesystem::path& candidate :
                 {root / relative, root / relative.filename()}) {
                if (std::filesystem::exists(candidate)) {
                    return candidate;
                }
            }
        }
    }
    return {};
}

bool parseNewAnimation(Reader& reader, const Line& keywordLine, ModelPart& part,
                       Variables& variables, ConfigurationDiagnostics& diagnostics) {
    // [newanim] is a variable-length block terminated by the next keyword or '--'.
    static_cast<void>(keywordLine);
    ModelAnimation animation;
    Line field;
    while (reader.next(field)) {
        if (field.isKeyword()) {
            reader.pushBack(std::move(field));
            break;
        }
        if (field.text == "--" || isSeparatorLine(field.text)) {
            break;
        }
        if (!field.text.empty() && (field.text.front() == '#' || field.text.front() == '\'')) {
            break;
        }
        const std::string name = lower(field.text);
        if (name == "origin_from_mesh") {
            animation.originFromMesh = true;
            animation.originOperations.push_back({ModelAnimationOriginType::FromMesh, {}});
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
            animation.originOperations.push_back(
                {ModelAnimationOriginType::Translation, animation.origin});
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
            if (name == "origin_rot_x") {
                animation.originRotation[0] = ignored;
                animation.originOperations.push_back(
                    {ModelAnimationOriginType::RotationX, {ignored, 0.0, 0.0}});
            } else if (name == "origin_rot_y") {
                animation.originRotation[1] = ignored;
                animation.originOperations.push_back(
                    {ModelAnimationOriginType::RotationY, {ignored, 0.0, 0.0}});
            } else if (name == "origin_rot_z") {
                animation.originRotation[2] = ignored;
                animation.originOperations.push_back(
                    {ModelAnimationOriginType::RotationZ, {ignored, 0.0, 0.0}});
            } else if (name == "maxspeed") {
                animation.maxSpeed = ignored;
            } else if (name == "delay") {
                animation.delay = ignored;
            } else if (name == "offset") {
                animation.offset = ignored;
            }
            continue;
        }
        if (name == "anim_rot" || name == "anim_trans") {
            if (!animation.type.empty()) {
                diagnostics.error(field.number, name,
                                  "only one anim_rot or anim_trans is allowed per newanim");
                return false;
            }
            std::vector<std::string> values;
            if (!readValues(reader, field.number, name, 2, values, diagnostics)) {
                return false;
            }
            animation.variable = lower(trim(values[0]));
            if (animation.variable.empty()) {
                diagnostics.error(field.number, name, "animation variable cannot be empty");
                return false;
            }
            double scale = 0.0;
            if (!parseDouble(values[1], scale)) {
                diagnostics.error(field.number, name, "expected a numeric scale");
                return false;
            }
            animation.type = name;
            animation.scale = scale;
            variables.declare(animation.variable);
            continue;
        }
        break;
    }
    part.animations.push_back(animation);
    const std::string variable = lower(animation.variable);
    if (variable.rfind("wheel_rotation_", 0) == 0) {
        part.wheelAnimation.rotationVariable = animation.variable;
        part.wheelAnimation.rotationScale = animation.scale;
        if (animation.hasOrigin) {
            part.wheelAnimation.origin = animation.origin;
            part.wheelAnimation.hasOrigin = true;
        }
    } else if (variable.rfind("axle_suspension_", 0) == 0) {
        part.wheelAnimation.suspensionVariable = animation.variable;
        part.wheelAnimation.suspensionScale = animation.scale;
    } else if (variable.rfind("axle_steering_", 0) == 0) {
        part.wheelAnimation.steeringVariable = animation.variable;
        part.wheelAnimation.steeringScale = animation.scale;
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
    std::size_t currentMaterialIndex = static_cast<std::size_t>(-1);
    ModelMaterialState* currentItem = nullptr;
    int currentLodIndex = -1;
    // OMSI meshes inherit the previous illumination assignment.  The first
    // mesh receives the first four interior lights when no assignment exists.
    std::array<int, 4> inheritedInteriorLightIndexes = {0, 1, 2, 3};
    std::array<int, 4> pendingInteriorLightIndexes = {-1, -1, -1, -1};
    bool hasPendingInteriorLightIndexes = false;
    std::unordered_set<std::string> meshIdentifiers;
    Line line;
    Line previousLine;
    while (reader.next(line)) {
        const Line precedingLine = previousLine;
        previousLine = line;
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
            if (currentItem != nullptr) {
                return currentItem;
            }
            ModelPart* current = part();
            if (current == nullptr) {
                return nullptr;
            }
            if (currentMaterialIndex != static_cast<std::size_t>(-1) &&
                currentMaterialIndex < current->materialStatesInOrder.size()) {
                return &current->materialStatesInOrder[currentMaterialIndex];
            }
            if (currentMaterialKey.empty()) {
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
        // [lod]: one threshold; subsequent meshes use the new LOD index.
        if (keyword == "lod") {
            currentItem = nullptr;
            double threshold = 0.0;
            if (!readDoubleValue(reader, "LOD", threshold, result.diagnostics)) {
                continue;
            }
            currentLodIndex = static_cast<int>(result.lodThresholds.size());
            result.lodThresholds.push_back(threshold);
            currentPartIndex = static_cast<std::size_t>(-1);
            currentMaterialKey.clear();
            currentMaterialIndex = static_cast<std::size_t>(-1);
            continue;
        }
        // [mesh]: prefer a converted OBJ, with direct O3D loading as fallback.
        if (keyword == "mesh") {
            currentItem = nullptr;
            Line meshLine;
            if (!reader.readPayload(meshLine, result.diagnostics, "mesh")) {
                continue;
            }
            const std::string meshValue = trim(meshLine.text);
            std::filesystem::path objPath;
            std::string bundleEntry;
            const std::filesystem::path bundlePath = modelRoot / "openbus.obx";
            const std::filesystem::path resolvedMesh = resolveMesh(modelRoot, meshValue, false);
            if (!resolvedMesh.empty()) {
                objPath = resolvedMesh;
            } else if (std::filesystem::exists(bundlePath)) {
                std::filesystem::path relativeMesh = meshValue;
                relativeMesh.replace_extension(".obj");
                bundleEntry = lower(relativeMesh.generic_string());
                objPath = bundlePath;
            } else {
                objPath = resolveMesh(modelRoot, meshValue);
            }
            if (objPath.empty()) {
                bundleEntry.clear();
            }
            if (objPath.empty()) {
                result.diagnostics.warning(meshLine.number, "mesh",
                                           "mesh was not found: " + meshValue);
            }
            ModelPart newPart;
            // Store both source text and resolved path for diagnostics and export.
            newPart.objPath = objPath;
            newPart.bundleEntry = std::move(bundleEntry);
            newPart.sourceMeshPath = meshValue;
            newPart.lodIndex = currentLodIndex;
            if (hasPendingInteriorLightIndexes) {
                newPart.interiorLightIndexes = pendingInteriorLightIndexes;
                hasPendingInteriorLightIndexes = false;
            } else {
                newPart.interiorLightIndexes = inheritedInteriorLightIndexes;
            }
            result.parts.push_back(std::move(newPart));
            currentPartIndex = result.parts.size() - 1;
            currentMaterialKey.clear();
            currentMaterialIndex = static_cast<std::size_t>(-1);
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
                } else if (meshIdentifiers.find(current->animationParent) ==
                           meshIdentifiers.end()) {
                    result.diagnostics.error(value.number, "animparent",
                                             "parent mesh identifier must be declared earlier");
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
                    current->visibleVariable = lower(trim(values[0]));
                    current->visibleValue = value;
                    variables.declare(current->visibleVariable);
                }
            }
            continue;
        }
        // [illumination_interior]: four integer light-group indexes; -1 disables a slot.
        if (keyword == "illumination_interior") {
            std::vector<std::string> values;
            if (readValues(reader, line.number, keyword, 4, values, result.diagnostics)) {
                // -1 means that an illumination slot is unused; other values
                // identify the corresponding light group.
                std::array<int, 4> indexes = {-1, -1, -1, -1};
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
                    indexes[index] = value;
                }
                if (ModelPart* current = part(); current != nullptr) {
                    current->interiorLightIndexes = indexes;
                } else {
                    pendingInteriorLightIndexes = indexes;
                    hasPendingInteriorLightIndexes = true;
                }
                inheritedInteriorLightIndexes = indexes;
            }
            continue;
        }
        // [interiorlight]: controller, intensity, RGB colour, and local x/y/z.
        if (keyword == "interiorlight") {
            std::vector<std::string> values;
            if (readValues(reader, line.number, keyword, 8, values, result.diagnostics)) {
                ModelInteriorLight light;
                light.controller = lower(trim(values[0]));
                declareIfVariable(light.controller, variables);
                bool valid = true;
                std::array<double, 7> parameters = {};
                for (std::size_t index = 1; index < values.size(); ++index) {
                    if (!parseDouble(values[index], parameters[index - 1])) {
                        result.diagnostics.error(line.number, keyword,
                                                 "expected numeric light parameters");
                        valid = false;
                    }
                }
                if (valid) {
                    light.intensity = parameters[0];
                    light.color = {parameters[1], parameters[2], parameters[3]};
                    light.position = {parameters[4], parameters[5], parameters[6]};
                    result.interiorLights.push_back(std::move(light));
                }
            }
            continue;
        }
        // [light_enh]: 13 fields; [light_enh_2]: 23 or 24 fields. Each has a fixed
        // controller-variable field and numeric light parameters.
        if (keyword == "light_enh" || keyword == "light_enh_2") {
            std::vector<std::string> values;
            const std::size_t count = keyword == "light_enh" ? 13 : 23;
            if (readValues(reader, line.number, keyword, count, values, result.diagnostics)) {
                if (keyword == "light_enh_2") {
                    Line optionalTexture;
                    if (reader.next(optionalTexture)) {
                        if (optionalTexture.isKeyword()) {
                            reader.pushBack(std::move(optionalTexture));
                        } else {
                            values.push_back(std::move(optionalTexture.text));
                        }
                    }
                }
                const std::size_t controllerIndex = keyword == "light_enh" ? 7 : 17;
                ModelEnhancedLight light;
                light.enhanced = keyword == "light_enh_2";
                light.controller = lower(trim(values[controllerIndex]));
                declareIfVariable(light.controller, variables);
                bool valid = true;
                for (std::size_t index = 0; index < values.size(); ++index) {
                    if (index == controllerIndex ||
                        (light.enhanced && index == values.size() - 1)) {
                        continue;
                    }
                    double value = 0.0;
                    if (!parseDouble(values[index], value)) {
                        result.diagnostics.error(line.number, keyword,
                                                 "expected numeric light parameters");
                        valid = false;
                    } else {
                        light.parameters.push_back(value);
                    }
                }
                if (light.enhanced && values.size() == count + 1) {
                    light.textureName = trim(values.back());
                }
                if (valid) {
                    result.enhancedLights.push_back(std::move(light));
                }
            }
            continue;
        }
        // [spotlight]: twelve position, direction, colour, range, and cone fields.
        if (keyword == "spotlight") {
            std::vector<std::string> values;
            if (readValues(reader, line.number, keyword, 12, values, result.diagnostics)) {
                ModelSpotlight spotlight;
                bool valid = true;
                for (std::size_t index = 0; index < values.size(); ++index) {
                    if (!parseDouble(values[index], spotlight.parameters[index])) {
                        result.diagnostics.error(line.number, keyword,
                                                 "expected numeric spotlight parameters");
                        valid = false;
                    }
                }
                if (valid) {
                    result.spotlights.push_back(std::move(spotlight));
                }
            }
            continue;
        }
        // [smoke]: nineteen position, direction, controller, and appearance fields.
        if (keyword == "smoke") {
            if (requirePart() != nullptr) {
                std::vector<std::string> values;
                readValues(reader, line.number, keyword, 19, values, result.diagnostics);
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
            ModelPart* current = requirePart();
            if (current != nullptr) {
                Line value;
                if (reader.readPayload(value, result.diagnostics, keyword)) {
                    current->mouseEvent = lower(trim(value.text));
                }
            }
            continue;
        }
        // [matl]: texture name plus material mode; the texture name keys following
        // material modifiers and supplies the part fallback texture.
        if (keyword == "matl") {
            currentItem = nullptr;
            ModelPart* current = requirePart();
            std::vector<std::string> values;
            if (current != nullptr &&
                readValues(reader, line.number, keyword, 2, values, result.diagnostics)) {
                const std::string textureName = trim(values[0]);
                // Material modifiers that follow are associated with this key.
                currentMaterialIndex = current->materialStatesInOrder.size();
                currentMaterialKey = lower(std::filesystem::path(textureName).filename().string()) +
                                     "#" + std::to_string(currentMaterialIndex);
                ModelMaterialState state;
                state.textureName = textureName;
                state.texturePath = textureName;
                if (!parseInt(values[1], state.materialIndex) || state.materialIndex < 0) {
                    result.diagnostics.error(line.number, keyword,
                                             "expected a non-negative material index");
                }
                current->materialStatesInOrder.push_back(state);
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
                current->alphaScaleVariable = lower(trim(value.text));
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
        if (keyword == "matl_nozcheck") {
            ModelMaterialState* current = requireMaterial();
            if (current != nullptr) {
                current->noZcheck = true;
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
                if (readValues(reader, line.number, keyword, count, values, result.diagnostics)) {
                    if (keyword == "matl_bumpmap") {
                        current->bumpmapTextureName = trim(values[0]);
                        if (!parseDouble(values[1], current->bumpmapStrength)) {
                            result.diagnostics.error(line.number, keyword,
                                                     "expected numeric strength");
                        }
                    } else if (keyword == "matl_transmap") {
                        current->transmapTextureName = trim(values[0]);
                    } else {
                        current->nightmapTextureName = trim(values[0]);
                    }
                }
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
                    if (keyword == "matl_lightmap") {
                        current->lightmapTextureName = trim(texture.text);
                    } else {
                        current->freeTextureName = trim(texture.text);
                    }
                    Line variable;
                    // Push keywords back because the optional variable may be absent.
                    if (reader.next(variable)) {
                        if (variable.isKeyword()) {
                            reader.pushBack(std::move(variable));
                        } else if (variable.text != "--") {
                            if (keyword == "matl_lightmap") {
                                current->lightmapStrengthVariable = lower(trim(variable.text));
                            } else {
                                current->freeTextureVariable = lower(trim(variable.text));
                            }
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
        // [matl_change]: texture occurrence, material index, item-selector variable.
        if (keyword == "matl_change") {
            currentItem = nullptr;
            ModelPart* current = requirePart();
            if (current != nullptr) {
                std::vector<std::string> values;
                if (readValues(reader, line.number, keyword, 3, values, result.diagnostics)) {
                    const std::string textureName = trim(values[0]);
                    int layer = 0;
                    if (!parseInt(values[1], layer) || layer < 0) {
                        result.diagnostics.error(line.number, keyword,
                                                 "expected a non-negative material index");
                        continue;
                    }
                    const std::string activationVariable = lower(trim(values[2]));
                    if (textureName.empty() || activationVariable.empty()) {
                        result.diagnostics.error(
                            line.number, keyword,
                            "texture name and activation variable are required");
                        continue;
                    }
                    const std::string filename =
                        lower(std::filesystem::path(textureName).filename().string());
                    auto& states = current->materialStatesInOrder;
                    auto target = std::find_if(
                        states.begin(), states.end(), [&](const ModelMaterialState& state) {
                            return state.materialIndex == layer &&
                                   lower(std::filesystem::path(state.textureName)
                                             .filename()
                                             .string()) == filename;
                        });
                    // A change can identify an earlier material, or introduce an
                    // unmodified base without a preceding [matl] declaration.
                    if (target == states.end()) {
                        ModelMaterialState base;
                        base.textureName = textureName;
                        base.texturePath = textureName;
                        base.materialIndex = layer;
                        states.push_back(std::move(base));
                        currentMaterialIndex = states.size() - 1;
                    } else {
                        currentMaterialIndex = static_cast<std::size_t>(target - states.begin());
                    }
                    currentMaterialKey = filename + "#" + std::to_string(currentMaterialIndex);
                    states[currentMaterialIndex].textureChanges.push_back(
                        {{}, textureName, layer, activationVariable, {}});
                    declareIfVariable(activationVariable, variables);
                }
            }
            continue;
        }
        // These material flags have no payload; their presence changes renderer state
        // or is accepted for compatibility with the source format.
        if (keyword == "matl_item") {
            ModelPart* current = requirePart();
            if (current != nullptr &&
                currentMaterialIndex < current->materialStatesInOrder.size()) {
                ModelMaterialState& base = current->materialStatesInOrder[currentMaterialIndex];
                if (base.textureChanges.empty()) {
                    result.diagnostics.error(line.number, keyword,
                                             "item must follow [matl_change]");
                } else {
                    auto item = std::make_shared<ModelMaterialState>(base);
                    item->textureChanges.clear();
                    base.textureChanges.back().items.push_back(item);
                    currentItem = item.get();
                }
            } else if (current != nullptr) {
                result.diagnostics.error(line.number, keyword, "item must follow [matl_change]");
            }
            continue;
        }
        if (keyword == "matl_texadress_border" || keyword == "matl_texadress_clamp" ||
            keyword == "matl_texadress_mirror" || keyword == "matl_texadress_mirroronce") {
            ModelMaterialState* current = requireMaterial();
            if (current != nullptr) {
                const TextureAddressMode mode =
                    keyword == "matl_texadress_border"   ? TextureAddressMode::Border
                    : keyword == "matl_texadress_clamp"  ? TextureAddressMode::Clamp
                    : keyword == "matl_texadress_mirror" ? TextureAddressMode::Mirror
                                                         : TextureAddressMode::MirrorOnce;
                current->textureAddressS = mode;
                current->textureAddressT = mode;
            }
            continue;
        }
        // [texcoordtransx]/[texcoordtransy]: one script variable for UV translation.
        if (keyword == "texcoordtransx" || keyword == "texcoordtransy") {
            ModelMaterialState* current = requireMaterial();
            if (current != nullptr) {
                Line value;
                if (reader.readPayload(value, result.diagnostics, keyword)) {
                    if (keyword == "texcoordtransx") {
                        current->texcoordTransXVariable = lower(trim(value.text));
                    } else {
                        current->texcoordTransYVariable = lower(trim(value.text));
                    }
                    declareIfVariable(value.text, variables);
                }
            }
            continue;
        }
        // [usetexttexture]/[usescripttexture]: one numeric texture-slot index.
        if (keyword == "usetexttexture" || keyword == "usescripttexture") {
            ModelMaterialState* current = requireMaterial();
            if (current != nullptr) {
                int index = 0;
                if (readIntValue(reader, keyword, index, result.diagnostics)) {
                    if (index < 0) {
                        result.diagnostics.error(line.number, keyword,
                                                 "texture slot must be non-negative");
                    } else if (keyword == "usescripttexture") {
                        current->scriptTextureIndex = index;
                    } else {
                        current->textTextureIndex = index;
                    }
                }
            }
            continue;
        }
        if (keyword == "isshadow") {
            ModelPart* current = requirePart();
            if (current != nullptr) {
                current->isShadow = true;
            }
            continue;
        }
        // [fixed]/[absheight]: marker records with no payload consumed here.
        if (keyword == "fixed" || keyword == "absheight") {
            requirePart();
            continue;
        }
        // [boundingbox]: collision box size X/Y/Z followed by center X/Y/Z.
        if (keyword == "boundingbox") {
            std::vector<std::string> values;
            if (readValues(reader, line.number, keyword, 6, values, result.diagnostics)) {
                std::array<double, 6> bounds = {};
                bool valid = true;
                for (std::size_t index = 0; index < values.size(); ++index) {
                    if (!parseDouble(values[index], bounds[index]) ||
                        !std::isfinite(bounds[index])) {
                        valid = false;
                        break;
                    }
                }
                if (!valid || bounds[0] <= 0.0 || bounds[1] <= 0.0 || bounds[2] <= 0.0) {
                    result.diagnostics.error(
                        line.number, keyword,
                        "expected positive finite box sizes and finite center coordinates");
                } else {
                    result.boundingBox = bounds;
                    result.hasBoundingBox = true;
                }
            }
            continue;
        }
        // [collision_mesh]: one dedicated collision mesh path.
        if (keyword == "collision_mesh") {
            Line value;
            if (reader.readPayload(value, result.diagnostics, keyword)) {
                ModelCollisionMesh collisionMesh;
                collisionMesh.sourcePath = trim(value.text);
                if (ModelPart* current = part(); current != nullptr) {
                    collisionMesh.partIndex = currentPartIndex;
                    collisionMesh.hasPart = true;
                }
                collisionMesh.resolvedPath = resolveMesh(modelRoot, value.text);
                if (collisionMesh.resolvedPath.empty()) {
                    const std::filesystem::path bundlePath = modelRoot / "openbus.obx";
                    if (std::filesystem::exists(bundlePath)) {
                        std::filesystem::path relativeMesh = collisionMesh.sourcePath;
                        relativeMesh.replace_extension(".obj");
                        collisionMesh.bundleEntry = lower(relativeMesh.generic_string());
                        collisionMesh.resolvedPath = bundlePath;
                    }
                }
                if (collisionMesh.resolvedPath.empty()) {
                    result.diagnostics.warning(value.number, keyword,
                                               "collision mesh was not found: " + value.text);
                }
                result.collisionMeshes.push_back(std::move(collisionMesh));
            }
            continue;
        }
        // [nocollision]: marker disables collision contribution from the current mesh.
        if (keyword == "nocollision") {
            if (ModelPart* current = requirePart(); current != nullptr) {
                current->noCollision = true;
            }
            continue;
        }
        // [vfdmaxmin]: six numeric display limits for VFD/text rendering.
        if (keyword == "vfdmaxmin") {
            std::vector<std::string> values;
            readValues(reader, line.number, keyword, 6, values, result.diagnostics);
            continue;
        }
        // [tex_detail_factor]: one numeric texture-detail blend factor.
        if (keyword == "tex_detail_factor") {
            std::vector<std::string> values;
            readValues(reader, line.number, keyword, 1, values, result.diagnostics);
            continue;
        }
        // [ctc]: template name, source texture path, and template index.
        if (keyword == "ctc") {
            std::vector<std::string> values;
            if (readValues(reader, line.number, keyword, 3, values, result.diagnostics)) {
                ModelCtcTemplate ctc;
                ctc.name = trim(values[0]);
                ctc.texturePath = trim(values[1]);
                if (!parseInt(values[2], ctc.index) || ctc.name.empty() ||
                    ctc.texturePath.empty() || ctc.index < 0) {
                    result.diagnostics.error(line.number, keyword,
                                             "expected template name, texture path, and index");
                } else {
                    result.ctcTemplates.push_back(std::move(ctc));
                }
            }
            continue;
        }
        // [ctctexture]: texture slot plus replacement texture name.
        if (keyword == "ctctexture") {
            std::vector<std::string> values;
            if (readValues(reader, line.number, keyword, 2, values, result.diagnostics)) {
                ModelCtcTexture texture;
                texture.slot = lower(trim(values[0]));
                texture.textureName = trim(values[1]);
                if (texture.slot.empty() || texture.textureName.empty()) {
                    result.diagnostics.error(line.number, keyword,
                                             "expected texture slot and replacement name");
                } else {
                    result.ctcTextures.push_back(std::move(texture));
                }
            }
            continue;
        }
        // [scripttexture]: dimensions followed by model-version-specific options.
        if (keyword == "scripttexture") {
            ModelScriptTexture texture;
            texture.slot = static_cast<int>(result.scriptTextures.size());
            Line value;
            while (reader.next(value)) {
                if (value.isKeyword()) {
                    reader.pushBack(std::move(value));
                    break;
                }
                texture.options.push_back(trim(value.text));
            }
            if (texture.options.size() < 2 || !parseInt(texture.options[0], texture.width) ||
                !parseInt(texture.options[1], texture.height) ||
                !openbus::scripting::validScriptTextureSize(texture.width, texture.height)) {
                result.diagnostics.error(line.number, keyword,
                                         "dimensions exceed the supported script-texture limits");
            } else if (result.scriptTextures.size() >=
                       static_cast<std::size_t>(openbus::scripting::maxScriptTextureCount)) {
                result.diagnostics.error(line.number, keyword,
                                         "script-texture slot limit exceeded");
            } else {
                result.scriptTextures.push_back(std::move(texture));
            }
            continue;
        }
        // [texttexture]/[texttexture_enh]: retain variable-length display metadata;
        // font rasterization is applied by the text-display runtime later.
        if (keyword == "texttexture" || keyword == "texttexture_enh") {
            ModelTextTexture texture;
            texture.slot = static_cast<int>(result.textTextures.size());
            int explicitSlot = -1;
            if (parseInt(precedingLine.text, explicitSlot) && explicitSlot >= 0) {
                texture.slot = explicitSlot;
            }
            texture.enhanced = keyword == "texttexture_enh";
            Line value;
            while (reader.next(value)) {
                if (value.isKeyword()) {
                    reader.pushBack(std::move(value));
                    break;
                }
                texture.values.push_back(trim(value.text));
            }
            if (!texture.values.empty()) {
                texture.values[0] = lower(texture.values[0]);
            }
            result.textTextures.push_back(std::move(texture));
            continue;
        }
    }

    for (ModelPart& part : result.parts) {
        for (std::size_t index = 0; index < part.materialStatesInOrder.size(); ++index) {
            const ModelMaterialState& state = part.materialStatesInOrder[index];
            const std::filesystem::path texturePath =
                state.textureName.empty() ? state.texturePath
                                          : std::filesystem::path(state.textureName);
            const std::string key =
                lower(texturePath.filename().string()) + "#" + std::to_string(index);
            part.materialStates[key] = state;
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
