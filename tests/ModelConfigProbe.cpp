#include "ModelConfigLoader.h"
#include "InteriorLighting.h"
#include "SceneryObjectConfigLoader.h"
#include "Variables.h"
#include "osc/OscConverter.h"

#include <filesystem>
#include <fstream>
#include <iostream>

int main() {
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() / "openbus_model_config_probe";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);
    const std::filesystem::path configPath = root / "display.cfg";
    std::ofstream mesh(root / "display.obj");
    mesh << "# parser fixture\n";
    mesh.close();
    std::ofstream collisionMesh(root / "collision.obj");
    collisionMesh << "v -1 -2 0\nv 1 -2 0\nv 0 2 0\nf 1 2 3\n";
    collisionMesh.close();
    std::ofstream config(configPath);
    config << "[scripttexture]\n64\n16\n0\n"
              "[CTC]\nColorscheme\nTexture\\Advert\n0\n"
              "[CTCTexture]\nFarbschema_Tex1\n3803_l.tga\n"
              "[mesh]\ndisplay.obj\n"
              "[visible]\nscenery_lamp\n1\n"
              "[absheight]\n"
              "[shadow]\n"
              "[isshadow]\n"
              "[nocollision]\n"
              "[collision_mesh]\ncollision.obj\n"
              "[boundingbox]\n2.5\n13.5\n3.66\n0\n0\n2.23\n"
              "[mouseevent]\nRouteDisplay\n"
              "[matl]\ndisplay.bmp\n0\n"
              "[matl_change]\ndisplay.bmp\n0\nCockpit_Lights\n"
              "[usescripttexture]\n0\n"
              "[illumination_interior]\n2\n-1\n4\n5\n"
              "[interiorlight]\nCockpit_Lights\n1\n2\n3\n0.1\n0.2\n0.3\n4\n"
              "[light_enh]\n-0.998978\n3.97046\n2.33735\n243\n243\n243\n"
              "0.025\ndoor_light_1\n0.5\n0.1\n3\n0.01\n"
              "[light_enh]\n-0.998942\n4.05412\n2.33734\n243\n243\n243\n"
              "0.025\ndoor_light_1\n0.5\n0.1\n3\n0.01\nalpha_mask.bmp\n"
              "[texttexture]\nident\nfont\n128\n32\n"
              "[usetexttexture]\n0\n";
    config.close();

    Variables variables(ScriptObjectKind::Vehicle);
    openbus::rendering::Matrix4 rootModelView = {1.0, 0.0, 0.0, 0.0,
                                                  0.0, 1.0, 0.0, 0.0,
                                                  0.0, 0.0, 1.0, 0.0,
                                                  10.0, 20.0, 30.0, 1.0};
    const std::array<double, 3> lightViewPosition =
        openbus::rendering::interiorLightPositionInViewSpace(rootModelView, {-0.6, -5.0, 7.0});
    if (lightViewPosition != std::array<double, 3>{5.0, 20.6, 37.0}) {
        std::cerr << "interior light model/view coordinate conversion failed\n";
        return 1;
    }
    const ModelConfig result =
        loadModelConfig(configPath, root, ModelConfigKind::Bus, variables);
    if (result.enhancedLights.size() != 2 ||
        !result.enhancedLights[0].textureName.empty() ||
        result.enhancedLights[1].textureName != "alpha_mask.bmp" ||
        result.enhancedLights[0].parameters.size() != 11 ||
        result.enhancedLights[1].parameters.size() != 11) {
        std::cerr << "enhanced-light optional alpha was not retained correctly\n";
        return 1;
    }
    const std::filesystem::path duplicateIdentifiersPath = root / "duplicate_ids.cfg";
    std::ofstream duplicateIdentifiers(duplicateIdentifiersPath);
    duplicateIdentifiers << "[mesh]\ndisplay.obj\n[mesh_ident]\nEnginebaydoor\n"
                            "[mesh]\ndisplay.obj\n[mesh_ident]\nEnginebaydoor\n"
                            "[mesh]\ndisplay.obj\n[animparent]\nEnginebaydoor\n";
    duplicateIdentifiers.close();
    const ModelConfig duplicateIdentifierConfig =
        loadModelConfig(duplicateIdentifiersPath, root, ModelConfigKind::Bus, variables);
    if (duplicateIdentifierConfig.diagnostics.hasErrors() ||
        duplicateIdentifierConfig.parts.size() != 3 ||
        duplicateIdentifierConfig.parts[0].meshIdentifier != "Enginebaydoor" ||
        duplicateIdentifierConfig.parts[1].meshIdentifier != "Enginebaydoor" ||
        duplicateIdentifierConfig.parts[2].animationParent != "Enginebaydoor") {
        std::cerr << "reused mesh identifiers were not accepted for variant meshes\n";
        return 1;
    }
    Variables sceneryVariables(ScriptObjectKind::SceneryObject);
    const ModelConfig sceneryResult =
        loadModelConfig(configPath, root, ModelConfigKind::SceneryObject, sceneryVariables);
    bool shadowMarkerWarned = false;
    for (const ConfigurationDiagnostic& diagnostic : result.diagnostics.entries) {
        shadowMarkerWarned = shadowMarkerWarned ||
                             (diagnostic.severity == ConfigurationDiagnostic::Severity::Warning &&
                              diagnostic.keyword == "shadow");
    }
    const std::filesystem::path oversizedConfigPath = root / "oversized.cfg";
    std::ofstream oversizedConfig(oversizedConfigPath);
    oversizedConfig << "[scripttexture]\n4097\n1\n";
    oversizedConfig.close();
    const ModelConfig oversized =
        loadModelConfig(oversizedConfigPath, root, ModelConfigKind::Bus, variables);
    if (!oversized.diagnostics.hasErrors() || !oversized.scriptTextures.empty()) {
        std::cerr << "oversized script-texture dimensions were not rejected\n";
        return 1;
    }
    std::ofstream variants(configPath);
    variants << "[mesh]\ndisplay.obj\n"
                "[collision_mesh]\ncollision.obj\n"
                "[boundingbox]\n2.5\n13.5\n3.66\n0\n0\n2.23\n"
                "[matl]\ndisplay.bmp\n0\n"
                "[matl_alpha]\n2\n[usetexttexture]\n7\n"
                "[matl_change]\ndisplay.bmp\n0\nDisplayMode\n"
                "[matl_item]\n[matl_nightmap]\non.bmp\n"
                "[matl_item]\n[matl_lightmap]\nwarning.bmp\nBrightness\n"
                "[matl_nozwrite]\n"
                "[matl]\nother.bmp\n0\n[matl_alpha]\n1\n"
                "[matl_change]\ndisplay.bmp\n0\nAlternateMode\n"
                "[matl_item]\n[matl_nightmap]\nalternate.bmp\n"
                "[matl_change]\nstandalone.bmp\n2\nStandaloneMode\n"
                "[matl_item]\n[matl_nightmap]\nstandalone_lit.bmp\n";
    variants.close();
    const ModelConfig conditional =
        loadModelConfig(configPath, root, ModelConfigKind::Bus, variables);
    if (conditional.diagnostics.hasErrors() ||
        conditional.absoluteHeight ||
        conditional.parts[0].materialStatesInOrder.size() != 3 ||
        conditional.collisionMeshes.size() != 1 ||
        conditional.collisionMeshes[0].sourcePath != "collision.obj" ||
        conditional.collisionMeshes[0].resolvedPath != root / "collision.obj" ||
        !conditional.collisionMeshes[0].hasPart ||
        conditional.collisionMeshes[0].partIndex != 0 || !conditional.hasBoundingBox ||
        conditional.boundingBox != std::array<double, 6>{2.5, 13.5, 3.66, 0.0, 0.0, 2.23}) {
        std::cerr << "conditional material fixture failed to parse\n";
        return 1;
    }
    const std::filesystem::path treeConfigPath = root / "tree.sco";
    std::ofstream treeConfig(treeConfigPath);
    treeConfig << "[absheight]\n[tree]\ntree.bmp\n5\n10\n0.5\n1.5\n";
    treeConfig.close();
    const SceneryObjectConfig treeConfiguration = loadSceneryObjectFile(treeConfigPath);
    if (!treeConfiguration.absoluteHeight || treeConfiguration.trees.size() != 1) {
        std::cerr << "tree SCO absolute-height marker was not retained\n";
        return 1;
    }
    const std::filesystem::path attachmentConfigPath = root / "attachment.sco";
    std::ofstream attachmentConfig(attachmentConfigPath);
    attachmentConfig << "[model]\nattachment.cfg\n"
                        "[new_attachment]\nleft\nattach_rot_z\n180\n"
                        "attach_trans\n0\n0.04\n0.8\n"
                        "[new_attachment]\nright\nattach_trans\n1\n2\n3\n";
    attachmentConfig.close();
    const SceneryObjectConfig attachmentConfiguration =
        loadSceneryObjectFile(attachmentConfigPath);
    if (attachmentConfiguration.diagnostics.hasErrors() ||
        attachmentConfiguration.attachmentPoints.size() != 2 ||
        attachmentConfiguration.attachmentPoints[0].name != "left" ||
        attachmentConfiguration.attachmentPoints[0].translation !=
            std::array<double, 3>{0.0, 0.04, 0.8} ||
        attachmentConfiguration.attachmentPoints[0].rotationDegrees !=
            std::array<double, 3>{0.0, 0.0, 180.0} ||
        attachmentConfiguration.attachmentPoints[1].name != "right" ||
        attachmentConfiguration.attachmentPoints[1].translation !=
            std::array<double, 3>{1.0, 2.0, 3.0}) {
        std::cerr << "SCO new_attachment blocks were not retained in order\n";
        return 1;
    }
    const std::filesystem::path sceneryVariableListPath = root / "signal_varlist.txt";
    std::ofstream sceneryVariableList(sceneryVariableListPath);
    sceneryVariableList << "SignalState\nLampMode\n";
    sceneryVariableList.close();
    const std::filesystem::path sceneryStringListPath = root / "signal_stringlist.txt";
    std::ofstream sceneryStringList(sceneryStringListPath);
    sceneryStringList << "SignalText\n";
    sceneryStringList.close();
    const std::filesystem::path sceneryConstantPath = root / "signal_const.cfg";
    std::ofstream sceneryConstants(sceneryConstantPath);
    sceneryConstants << "[const]\nEnabled\n1\n[newcurve]\nLampCurve\n"
                        "[pnt]\n0\n0\n[pnt]\n1\n1\n";
    sceneryConstants.close();
    const std::filesystem::path sceneryScriptPath = root / "signal.osc";
    std::ofstream sceneryScript(sceneryScriptPath);
    sceneryScript << "{init}\n(C.L.Enabled) (S.L.SignalState)\n{end}\n";
    sceneryScript.close();
    const std::filesystem::path scriptedSceneryPath = root / "signal.sco";
    std::ofstream scriptedScenery(scriptedSceneryPath);
    scriptedScenery << "[model]\nsignal.cfg\n"
                       "[script]\n1\nsignal.osc\n"
                       "[varnamelist]\n1\nsignal_varlist.txt\n"
                       "[stringvarnamelist]\n1\nsignal_stringlist.txt\n"
                       "[constfile]\n1\nsignal_const.cfg\n";
    scriptedScenery.close();
    const SceneryObjectConfig scriptedSceneryConfiguration =
        loadSceneryObjectFile(scriptedSceneryPath);
    if (scriptedSceneryConfiguration.diagnostics.hasErrors() ||
        scriptedSceneryConfiguration.scriptConfiguration.sourcePath != scriptedSceneryPath ||
        scriptedSceneryConfiguration.scriptConfiguration.scripts !=
            std::vector<std::string>{"signal.osc"} ||
        scriptedSceneryConfiguration.scriptConfiguration.floatVariables !=
            std::vector<std::string>{"signalstate", "lampmode"} ||
        scriptedSceneryConfiguration.scriptConfiguration.stringVariables !=
            std::vector<std::string>{"signaltext"} ||
        scriptedSceneryConfiguration.scriptConfiguration.constants.at("enabled") != 1.0F ||
        scriptedSceneryConfiguration.scriptConfiguration.curves.count("lampcurve") != 1 ||
        !std::filesystem::exists(generatedLuaPath(sceneryScriptPath))) {
        std::cerr << "SCO script, variable lists, constants, or OSC preparation were not loaded\n";
        return 1;
    }
    const std::filesystem::path treeWithHelperPath = root / "tree_with_editor_helper.sco";
    std::ofstream treeWithHelper(treeWithHelperPath);
    treeWithHelper << "[tree]\ntree.bmp\n5\n10\n0.5\n1.5\n"
                      "[mesh]\ntreehelper.x\n[onlyeditor]\n";
    treeWithHelper.close();
    const SceneryObjectConfig treeWithHelperConfiguration =
        loadSceneryObjectFile(treeWithHelperPath);
    if (!treeWithHelperConfiguration.onlyEditor ||
        treeWithHelperConfiguration.trees.size() != 1 ||
        treeWithHelperConfiguration.diagnostics.hasErrors()) {
        std::cerr << "tree definition was not retained with an editor-only helper mesh\n";
        return 1;
    }
    const std::filesystem::path editorOnlySCOPath = root / "editor_only.sco";
    std::ofstream editorOnlySCO(editorOnlySCOPath);
    editorOnlySCO << "[onlyeditor]\n[model]\nmodel.cfg\n";
    editorOnlySCO.close();
    const SceneryObjectConfig editorOnlyConfiguration =
        loadSceneryObjectFile(editorOnlySCOPath);
    if (!editorOnlyConfiguration.onlyEditor || editorOnlyConfiguration.diagnostics.hasErrors()) {
        std::cerr << "SCO [onlyeditor] marker was not retained\n";
        return 1;
    }
    const std::filesystem::path externalModelSCOPath = root / "external_model.sco";
    std::ofstream externalModelSCO(externalModelSCOPath);
    externalModelSCO << "[model]\nmodel\\model_parked.cfg\n";
    externalModelSCO.close();
    const SceneryObjectConfig externalModelSCOConfig =
        loadSceneryObjectFile(externalModelSCOPath);
    if (resolveSceneryObjectModelConfigPath(externalModelSCOConfig) !=
        root / "model" / "model_parked.cfg") {
        std::cerr << "relative SCO [model] reference was not resolved from its config directory\n";
        return 1;
    }
    const std::filesystem::path xModelRoot = root / "scenery_model";
    std::filesystem::create_directories(xModelRoot);
    std::ofstream xMesh(xModelRoot / "tree.x");
    xMesh << "xof 0303txt 0032\n";
    xMesh.close();
    std::ofstream convertedXMesh(xModelRoot / "tree.obj");
    convertedXMesh << "# converted alternate\n";
    convertedXMesh.close();
    const std::filesystem::path xModelConfigPath = root / "scenery_model.cfg";
    std::ofstream xModelConfig(xModelConfigPath);
    xModelConfig << "[new_attachment]\nmodel_anchor\nattach_rot_y\n15\n"
                    "attach_trans\n2\n3\n4\n[mesh]\ntree.x\n";
    xModelConfig.close();
    Variables xSceneryVariables(ScriptObjectKind::SceneryObject);
    const ModelConfig xModelConfiguration =
        loadModelConfig(xModelConfigPath, xModelRoot, ModelConfigKind::SceneryObject,
                        xSceneryVariables);
    if (xModelConfiguration.parts.size() != 1 ||
        xModelConfiguration.parts[0].objPath != xModelRoot / "tree.x" ||
        xModelConfiguration.attachmentPoints.size() != 1 ||
        xModelConfiguration.attachmentPoints[0].name != "model_anchor" ||
        xModelConfiguration.attachmentPoints[0].translation !=
            std::array<double, 3>{2.0, 3.0, 4.0} ||
        xModelConfiguration.attachmentPoints[0].rotationDegrees !=
            std::array<double, 3>{0.0, 15.0, 0.0}) {
        std::cerr << "scenery model resolver did not retain the authored X mesh path\n";
        return 1;
    }
    const auto& base = conditional.parts[0].materialStatesInOrder[0];
    const auto selected = [&](double mode) -> const ModelMaterialState& {
        variables.set("displaymode", mode);
        return selectModelMaterial(base, [&](const std::string& name) { return variables.get(name); });
    };
    if (!base.nightmapTextureName.empty() || !base.lightmapTextureName.empty() ||
        &selected(0) != &base || selected(1).nightmapTextureName != "on.bmp" ||
        selected(1).textTextureIndex != 7 || selected(1).alphaMode != 2 ||
        selected(0.9).nightmapTextureName != "on.bmp" ||
        !selected(2).nightmapTextureName.empty() ||
        selected(2).lightmapTextureName != "warning.bmp" ||
        selected(2).lightmapStrengthVariable != "brightness" || !selected(2).noZwrite ||
        &selected(-1) != &base || &selected(3) != &base || &selected(0) != &base ||
        conditional.parts[0].materialStatesInOrder[1].alphaMode != 1) {
        std::cerr << "material items leaked into the base or were not selected generically\n";
        return 1;
    }
    variables.set("alternatemode", 1.0);
    if (selected(0).nightmapTextureName != "alternate.bmp" ||
        conditional.parts[0].materialStatesInOrder[1].textureChanges.size() != 0 ||
        conditional.parts[0].materialStatesInOrder[2].textureChanges.size() != 1) {
        std::cerr << "material changes were attached to the preceding rather than named material\n";
        return 1;
    }

    std::ofstream laxConfig(configPath);
    laxConfig << "[mesh]\ndisplay.obj\n"
                 "[matl_envmap]\nenvmap.bmp\n0.03\n"
                 "[newanim]\norigin_from_mesh\nanim_rot\norigin_rot_x\n0\n"
                 "origin_rot_y\n90\nanim_rot\nflap1_pos\n-170\n"
                 "[newanim]\nanim_rot\nsecond_rotation\n45\n"
                 "anim_trans\nsecond_translation\n1\n";
    laxConfig.close();
    const ModelConfig lax = loadModelConfig(configPath, root, ModelConfigKind::Bus, variables);
    if (lax.diagnostics.hasErrors() || lax.parts.size() != 1 ||
        lax.parts[0].materialStatesInOrder.size() != 1 ||
        lax.parts[0].materialStatesInOrder[0].materialIndex != 0 ||
        lax.parts[0].materialStatesInOrder[0].environmentTextureName != "envmap.bmp" ||
        lax.parts[0].animations.size() != 3 ||
        lax.parts[0].animations[0].variable != "flap1_pos" ||
        lax.parts[0].animations[0].originRotation[1] != 90.0 ||
        lax.parts[0].animations[1].variable != "second_rotation" ||
        lax.parts[0].animations[2].variable != "second_translation") {
        std::cerr << "lax material or animation records were not recovered\n";
        return 1;
    }
    std::filesystem::remove_all(root);

    if (result.diagnostics.hasErrors() || shadowMarkerWarned || result.parts.size() != 1 ||
        !result.parts[0].isShadow || !result.parts[0].noCollision ||
        result.collisionMeshes.size() != 1 || !result.collisionMeshes[0].hasPart ||
        !result.hasBoundingBox ||
        result.scriptTextures.size() != 1 || result.scriptTextures[0].slot != 0 ||
        result.scriptTextures[0].width != 64 || result.scriptTextures[0].height != 16 ||
        result.textTextures.size() != 1 || result.textTextures[0].slot != 0 ||
        result.textTextures[0].values.empty() || result.textTextures[0].values[0] != "ident" ||
        result.ctcTemplates.size() != 1 || result.ctcTemplates[0].name != "Colorscheme" ||
        result.ctcTemplates[0].texturePath != "Texture\\Advert" ||
        result.ctcTextures.size() != 1 || result.ctcTextures[0].slot != "farbschema_tex1" ||
        result.ctcTextures[0].textureName != "3803_l.tga" ||
        result.parts[0].mouseEvent != "routedisplay" ||
        sceneryResult.parts.size() != 1 ||
        sceneryResult.parts[0].visibleVariable != "scenery_lamp" ||
        sceneryResult.parts[0].visibleValue != 1 || !sceneryVariables.has("scenery_lamp") ||
        result.parts[0].interiorLightIndexes != std::array<int, 4>{2, -1, 4, 5} ||
        result.interiorLights.size() != 1 ||
        result.interiorLights[0].controller != "cockpit_lights" ||
        result.interiorLights[0].intensity != 1.0 ||
        result.interiorLights[0].color != std::array<double, 3>{2.0, 3.0, 0.1} ||
        result.interiorLights[0].position != std::array<double, 3>{0.2, 0.3, 4.0} ||
        result.parts[0].materialStatesInOrder.size() != 1 ||
        result.parts[0].materialStatesInOrder[0].scriptTextureIndex != 0 ||
        result.parts[0].materialStatesInOrder[0].textTextureIndex != 0 ||
        result.parts[0].materialStatesInOrder[0].textureChanges.size() != 1 ||
        result.parts[0].materialStatesInOrder[0].textureChanges[0].activationVariable !=
            "cockpit_lights" || !sceneryResult.absoluteHeight) {
        std::cerr << "model texture records were not retained and bound\n";
        return 1;
    }
    return 0;
}
