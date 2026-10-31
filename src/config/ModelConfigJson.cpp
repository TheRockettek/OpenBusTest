#include "config/ModelConfigLoader.h"

#include <algorithm>
#include <iomanip>
#include <ostream>
#include <string>
#include <vector>

namespace {

void writeJsonString(std::ostream& output, const std::string& value) {
    output << '"';
    for (const unsigned char character : value) {
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
            if (character < 0x20U) {
                constexpr char hex[] = "0123456789abcdef";
                output << "\\u00" << hex[character >> 4U] << hex[character & 0x0fU];
            } else {
                output << static_cast<char>(character);
            }
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
               << ",\"origin_from_mesh\":" << (animation.originFromMesh ? "true" : "false")
               << ",\"origin_operations\":[";
        for (std::size_t operationIndex = 0; operationIndex < animation.originOperations.size();
             ++operationIndex) {
            if (operationIndex != 0) {
                output << ',';
            }
            const ModelAnimationOrigin& operation = animation.originOperations[operationIndex];
            const char* operationType = "translation";
            switch (operation.type) {
            case ModelAnimationOriginType::Translation:
                operationType = "translation";
                break;
            case ModelAnimationOriginType::RotationX:
                operationType = "rotation_x";
                break;
            case ModelAnimationOriginType::RotationY:
                operationType = "rotation_y";
                break;
            case ModelAnimationOriginType::RotationZ:
                operationType = "rotation_z";
                break;
            case ModelAnimationOriginType::FromMesh:
                operationType = "from_mesh";
                break;
            }
            output << "{\"type\":\"" << operationType << "\",\"value\":[" << operation.value[0]
                   << ',' << operation.value[1] << ',' << operation.value[2] << "]}";
        }
        output << "]}";
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
    output << "],\n  \"bounding_box\": ";
    if (configuration.hasBoundingBox) {
        output << "{\"size\":[" << configuration.boundingBox[0] << ','
               << configuration.boundingBox[1] << ',' << configuration.boundingBox[2]
               << "],\"center\":[" << configuration.boundingBox[3] << ','
               << configuration.boundingBox[4] << ',' << configuration.boundingBox[5] << "]}";
    } else {
        output << "null";
    }
    output << ",\n  \"texture_detail_factor\": ";
    if (configuration.textureDetailFactor) {
        output << *configuration.textureDetailFactor;
    } else {
        output << "null";
    }
    output << ",\n  \"terrain_mappings\": [";
    for (std::size_t index = 0; index < configuration.terrainMappings.size(); ++index) {
        if (index != 0) {
            output << ',';
        }
        const ModelTerrainMapping& mapping = configuration.terrainMappings[index];
        output << "{\"source_line\":" << mapping.sourceLine << ",\"payload\":[";
        for (std::size_t valueIndex = 0; valueIndex < mapping.payload.size(); ++valueIndex) {
            if (valueIndex != 0) {
                output << ',';
            }
            writeJsonString(output, mapping.payload[valueIndex]);
        }
        output << "]}";
    }
    output << ']';
    output << ",\n  \"texture_change_records\": [";
    for (std::size_t index = 0; index < configuration.textureChangeRecords.size(); ++index) {
        if (index != 0) {
            output << ',';
        }
        const ModelTextureChangeRecord& record = configuration.textureChangeRecords[index];
        output << "{\"keyword\":";
        writeJsonString(output, record.keyword);
        output << ",\"source_line\":" << record.sourceLine << ",\"payload\":[";
        for (std::size_t valueIndex = 0; valueIndex < record.payload.size(); ++valueIndex) {
            if (valueIndex != 0) {
                output << ',';
            }
            writeJsonString(output, record.payload[valueIndex]);
        }
        output << "]}";
    }
    output << ']';
    output << ",\n  \"vfd_display_bounds\": [";
    for (std::size_t index = 0; index < configuration.vfdDisplayBounds.size(); ++index) {
        if (index != 0) {
            output << ',';
        }
        const ModelDisplayBounds& bounds = configuration.vfdDisplayBounds[index];
        output << "{\"bounds\":[";
        for (std::size_t valueIndex = 0; valueIndex < bounds.minimumAndMaximum.size();
             ++valueIndex) {
            if (valueIndex != 0) {
                output << ',';
            }
            output << bounds.minimumAndMaximum[valueIndex];
        }
        output << "],\"view_context_flag\":";
        if (bounds.viewContextFlag) {
            writeJsonString(output, *bounds.viewContextFlag);
        } else {
            output << "null";
        }
        output << '}';
    }
    output << ']';
    output << ",\n  \"smoke_emitters\": [";
    for (std::size_t index = 0; index < configuration.smokes.size(); ++index) {
        if (index != 0) {
            output << ',';
        }
        output << "{\"fields\":[";
        const ModelSmoke& smoke = configuration.smokes[index];
        for (std::size_t fieldIndex = 0; fieldIndex < smoke.fields.size(); ++fieldIndex) {
            if (fieldIndex != 0) {
                output << ',';
            }
            writeJsonString(output, smoke.fields[fieldIndex]);
        }
        output << "]}";
    }
    output << ']';
    output << ",\n  \"interior_lights\": [";
    for (std::size_t index = 0; index < configuration.interiorLights.size(); ++index) {
        if (index != 0) {
            output << ',';
        }
        const ModelInteriorLight& light = configuration.interiorLights[index];
        output << "{\"controller\":";
        writeJsonString(output, light.controller);
        output << ",\"intensity\":" << light.intensity << ",\"color\":[" << light.color[0] << ','
               << light.color[1] << ',' << light.color[2] << "],\"position\":[" << light.position[0]
               << ',' << light.position[1] << ',' << light.position[2] << "]}";
    }
    output << ']';
    output << ",\n  \"enhanced_lights\": [";
    for (std::size_t index = 0; index < configuration.enhancedLights.size(); ++index) {
        if (index != 0) {
            output << ',';
        }
        const ModelEnhancedLight& light = configuration.enhancedLights[index];
        output << "{\"enhanced\":" << (light.enhanced ? "true" : "false")
               << ",\"mesh_part_index\":";
        if (light.meshPartIndex) {
            output << *light.meshPartIndex;
        } else {
            output << "null";
        }
        output << ",\"controller\":";
        writeJsonString(output, light.controller);
        output << ",\"parameters\":[";
        for (std::size_t parameterIndex = 0; parameterIndex < light.parameters.size();
             ++parameterIndex) {
            if (parameterIndex != 0) {
                output << ',';
            }
            output << light.parameters[parameterIndex];
        }
        output << "],\"texture_name\":";
        writeJsonString(output, light.textureName);
        output << '}';
    }
    output << ']';
    output << ",\n  \"spotlights\": [";
    for (std::size_t index = 0; index < configuration.spotlights.size(); ++index) {
        if (index != 0) {
            output << ',';
        }
        output << "{\"parameters\":[";
        const ModelSpotlight& spotlight = configuration.spotlights[index];
        for (std::size_t parameterIndex = 0; parameterIndex < spotlight.parameters.size();
             ++parameterIndex) {
            if (parameterIndex != 0) {
                output << ',';
            }
            output << spotlight.parameters[parameterIndex];
        }
        output << "]}";
    }
    output << ']';
    output << ",\n  \"ctc_templates\": [";
    for (std::size_t index = 0; index < configuration.ctcTemplates.size(); ++index) {
        if (index != 0) {
            output << ',';
        }
        const ModelCtcTemplate& value = configuration.ctcTemplates[index];
        output << "{\"name\":";
        writeJsonString(output, value.name);
        output << ",\"texture_path\":";
        writeJsonString(output, value.texturePath);
        output << ",\"index\":" << value.index << '}';
    }
    output << ']';
    output << ",\n  \"ctc_textures\": [";
    for (std::size_t index = 0; index < configuration.ctcTextures.size(); ++index) {
        if (index != 0) {
            output << ',';
        }
        const ModelCtcTexture& value = configuration.ctcTextures[index];
        output << "{\"slot\":";
        writeJsonString(output, value.slot);
        output << ",\"texture_name\":";
        writeJsonString(output, value.textureName);
        output << '}';
    }
    output << ']';
    output << ",\n  \"attachment_points\": [";
    for (std::size_t index = 0; index < configuration.attachmentPoints.size(); ++index) {
        if (index != 0) {
            output << ',';
        }
        const ModelAttachmentPoint& attachment = configuration.attachmentPoints[index];
        output << "{\"name\":";
        writeJsonString(output, attachment.name);
        output << ",\"translation\":[" << attachment.translation[0] << ','
               << attachment.translation[1] << ',' << attachment.translation[2]
               << "],\"rotation_degrees\":[" << attachment.rotationDegrees[0] << ','
               << attachment.rotationDegrees[1] << ',' << attachment.rotationDegrees[2]
               << "],\"has_translation\":" << (attachment.hasTranslation ? "true" : "false")
               << ",\"has_rotation\":" << (attachment.hasRotation ? "true" : "false") << '}';
    }
    output << ']';
    output << ",\n  \"collision_meshes\": [";
    for (std::size_t index = 0; index < configuration.collisionMeshes.size(); ++index) {
        if (index != 0) {
            output << ',';
        }
        const ModelCollisionMesh& mesh = configuration.collisionMeshes[index];
        output << "{\"source\":";
        writeJsonString(output, mesh.sourcePath.generic_string());
        output << ",\"resolved\":";
        writeJsonString(output, mesh.resolvedPath.generic_string());
        output << ",\"part_index\":";
        if (mesh.hasPart) {
            output << mesh.partIndex;
        } else {
            output << "null";
        }
        output << '}';
    }
    output << "],\n  \"script_textures\": [";
    for (std::size_t index = 0; index < configuration.scriptTextures.size(); ++index) {
        if (index != 0) {
            output << ',';
        }
        const ModelScriptTexture& texture = configuration.scriptTextures[index];
        output << "{\"slot\":" << texture.slot << ",\"width\":" << texture.width
               << ",\"height\":" << texture.height << ",\"options\":[";
        for (std::size_t optionIndex = 0; optionIndex < texture.options.size(); ++optionIndex) {
            if (optionIndex != 0) {
                output << ',';
            }
            writeJsonString(output, texture.options[optionIndex]);
        }
        output << "]}";
    }
    output << "],\n  \"text_textures\": [";
    for (std::size_t index = 0; index < configuration.textTextures.size(); ++index) {
        if (index != 0) {
            output << ',';
        }
        const ModelTextTexture& texture = configuration.textTextures[index];
        output << "{\"slot\":" << texture.slot
               << ",\"enhanced\":" << (texture.enhanced ? "true" : "false")
               << ",\"placement_label\":" << (texture.placementLabel ? "true" : "false")
               << ",\"values\":[";
        for (std::size_t valueIndex = 0; valueIndex < texture.values.size(); ++valueIndex) {
            if (valueIndex != 0) {
                output << ',';
            }
            writeJsonString(output, texture.values[valueIndex]);
        }
        output << "]}";
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
               << ",\n      \"has_render_type\": " << (part.hasRenderType ? "true" : "false")
               << ",\n      \"fixed\": " << (part.fixed ? "true" : "false")
               << ",\n      \"is_shadow\": " << (part.isShadow ? "true" : "false")
               << ",\n      \"no_collision\": " << (part.noCollision ? "true" : "false")
               << ",\n      \"mesh_identifier\": ";
        writeJsonString(output, part.meshIdentifier);
        output << ",\n      \"animation_parent\": ";
        writeJsonString(output, part.animationParent);
        output << ",\n      \"mouse_event\": ";
        writeJsonString(output, part.mouseEvent);
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
            output << ",\"free_texture_name\":";
            writeJsonString(output, material.freeTextureName);
            output << ",\"free_texture_variable\":";
            writeJsonString(output, material.freeTextureVariable);
            output << ",\"script_texture_index\":" << material.scriptTextureIndex
                   << ",\"text_texture_index\":" << material.textTextureIndex;
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
