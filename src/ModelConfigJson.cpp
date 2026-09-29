#include "ModelConfigLoader.h"

#include <algorithm>
#include <iomanip>
#include <ostream>
#include <string>
#include <vector>

namespace {

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

void writeModelAnimationsJson(std::ostream& output, const std::vector<ModelAnimation>& animations) {
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
        output << ",\"scale\":" << animation.scale << ",\"maxspeed\":" << animation.maxSpeed
               << ",\"delay\":" << animation.delay << ",\"offset\":" << animation.offset
               << ",\"origin\":[" << animation.origin[0] << ',' << animation.origin[1] << ','
               << animation.origin[2] << "],\"origin_rotation\":[" << animation.originRotation[0]
               << ',' << animation.originRotation[1] << ',' << animation.originRotation[2]
               << "],\"has_origin\":" << (animation.hasOrigin ? "true" : "false")
               << ",\"origin_from_mesh\":" << (animation.originFromMesh ? "true" : "false") << '}';
    }
    output << ']';
}

const char* textureAddressModeName(TextureAddressMode mode) {
    switch (mode) {
    case TextureAddressMode::Clamp:
        return "clamp";
    case TextureAddressMode::Border:
        return "border";
    case TextureAddressMode::Mirror:
        return "mirror";
    case TextureAddressMode::MirrorOnce:
        return "mirror_once";
    case TextureAddressMode::Repeat:
    default:
        return "repeat";
    }
}

} // namespace

void writeModelConfigurationJson(std::ostream& output, const std::filesystem::path& configPath,
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
            const auto parentIterator =
                std::find_if(configuration.parts.begin(), configuration.parts.end(),
                             [&](const ModelPart& candidate) {
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
               << part.color[2]
               << "],\n"
                  "      \"viewpoint\": "
               << part.viewpoint
               << ",\n"
                  "      \"render_type\": "
               << part.renderType
               << ",\n      \"is_shadow\": " << (part.isShadow ? "true" : "false")
               << ",\n      \"mesh_identifier\": ";
        writeJsonString(output, part.meshIdentifier);
        output << ",\n      \"animation_parent\": ";
        writeJsonString(output, part.animationParent);
        output << ",\n      \"visibility\": {\"variable\": ";
        writeJsonString(output, part.visibleVariable);
        output << ",\"value\": " << part.visibleValue
               << "},\n"
                  "      \"interior_light_indexes\": ["
               << part.interiorLightIndexes[0] << ',' << part.interiorLightIndexes[1] << ','
               << part.interiorLightIndexes[2] << ',' << part.interiorLightIndexes[3]
               << "],\n"
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
        output << ",\"rotation_scale\":" << part.wheelAnimation.rotationScale
               << ",\"suspension_scale\":" << part.wheelAnimation.suspensionScale
               << ",\"steering_scale\":" << part.wheelAnimation.steeringScale << ",\"origin\":["
               << part.wheelAnimation.origin[0] << ',' << part.wheelAnimation.origin[1] << ','
               << part.wheelAnimation.origin[2]
               << "],\"has_origin\": " << (part.wheelAnimation.hasOrigin ? "true" : "false")
               << "},\n"
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
            const ModelMaterialState& material =
                part.materialStates.at(materialKeys[materialIndex]);
            output << "{\"key\":";
            writeJsonString(output, materialKeys[materialIndex]);
            output << ",\"texture_name\":";
            writeJsonString(output, material.textureName);
            output << ",\"material_index\":" << material.materialIndex;
            output << ",\"texture_path\":";
            writeJsonString(output, material.texturePath.generic_string());
            output << ",\"environment_texture\":";
            writeJsonString(output, material.environmentTextureName);
            output << ",\"environment_strength\":" << material.environmentStrength
                   << ",\"alpha_mode\":" << material.alphaMode
                   << ",\"no_zwrite\":" << (material.noZwrite ? "true" : "false")
                   << ",\"no_zcheck\":" << (material.noZcheck ? "true" : "false")
                   << ",\"alpha_scale_variable\":";
            writeJsonString(output, material.alphaScaleVariable);
            output << ",\"transmap_texture\":";
            writeJsonString(output, material.transmapTextureName);
            output << ",\"nightmap_texture\":";
            writeJsonString(output, material.nightmapTextureName);
            output << ",\"lightmap_texture\":";
            writeJsonString(output, material.lightmapTextureName);
            output << ",\"lightmap_strength_variable\":";
            writeJsonString(output, material.lightmapStrengthVariable);
            output << ",\"free_texture_variable\":";
            writeJsonString(output, material.freeTextureVariable);
            output << ",\"texcoord_trans_x_variable\":";
            writeJsonString(output, material.texcoordTransXVariable);
            output << ",\"texcoord_trans_y_variable\":";
            writeJsonString(output, material.texcoordTransYVariable);
            output << ",\"bumpmap_texture\":";
            writeJsonString(output, material.bumpmapTextureName);
            output << ",\"bumpmap_strength\":" << material.bumpmapStrength
                   << ",\"texture_address_s\":";
            writeJsonString(output, textureAddressModeName(material.textureAddressS));
            output << ",\"texture_address_t\":";
            writeJsonString(output, textureAddressModeName(material.textureAddressT));
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
