#include "map/MapAssetValidation.h"

#include "assets/O3DLoader.h"
#include "assets/ObjLoader.h"
#include "assets/TextureLoader.h"
#include "assets/VehicleTextureResolver.h"
#include "assets/WavLoader.h"
#include "assets/XLoader.h"
#include "assets/CollisionMesh.h"
#include "config/ConfigurationParser.h"
#include "config/HofData.h"
#include "config/HumanConfigLoader.h"
#include "config/ModelConfigLoader.h"
#include "config/SceneryObjectConfigLoader.h"
#include "config/ScriptTextureLimits.h"
#include "config/VehicleConfigLoader.h"
#include "config/VehicleCatalog.h"
#include "core/AssetPath.h"
#include "core/Utf8Path.h"
#include "core/Variables.h"
#include "map/MapCarparkList.h"
#include "map/MapConfigLoader.h"
#include "map/MapSplineProfile.h"
#include "map/SceneryMaterialBinding.h"
#include "render/map/TreeSeason.h"

#include <algorithm>
#include <cctype>
#include <exception>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace openbus::map {
namespace {

std::filesystem::path assetPath(const std::string& reference) {
    std::string normalized = reference;
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    return openbus::pathFromUtf8(normalized);
}

std::string cacheKey(const std::filesystem::path& path) {
    std::string key = pathToUtf8(std::filesystem::absolute(path).lexically_normal());
#ifdef _WIN32
    std::transform(key.begin(), key.end(), key.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
#endif
    return key;
}

void recordSourceFile(MapAssetValidationReport& report, const std::filesystem::path& path) {
    if (!path.empty()) {
        report.sourceFiles.push_back(std::filesystem::absolute(path).lexically_normal());
    }
}

void recordExistingSourceFile(MapAssetValidationReport& report, const std::filesystem::path& path) {
    if (path.empty()) {
        return;
    }

    std::error_code statusError;
    const bool exists = std::filesystem::exists(path, statusError);
    if (statusError) {
        report.findings.push_back({"map input", path, pathToUtf8(path),
                                   "Could not inspect source path: " + statusError.message()});
        return;
    }
    if (!exists) {
        return;
    }

    const bool regularFile = std::filesystem::is_regular_file(path, statusError);
    if (statusError) {
        report.findings.push_back({"map input", path, pathToUtf8(path),
                                   "Could not inspect source path: " + statusError.message()});
    } else if (!regularFile) {
        report.findings.push_back(
            {"map input", path, pathToUtf8(path), "Source path exists but is not a regular file"});
    } else {
        recordSourceFile(report, path);
    }
}

std::filesystem::path findAsset(const std::vector<std::filesystem::path>& roots,
                                const std::string& reference, bool texture = false,
                                bool strictAccess = false,
                                MapAssetValidationReport* report = nullptr) {
    const auto relative = assetPath(reference);
    for (const auto& root : roots) {
        const auto candidate = root / relative;
        const auto resolved = texture ? assets::resolveTexturePath(candidate)
                                      : assets::resolveCaseInsensitivePath(candidate, strictAccess);
        if (std::filesystem::is_regular_file(resolved)) {
            if (report) {
                recordSourceFile(*report, resolved);
            }
            return resolved;
        }
    }
    return {};
}

void checkReference(MapAssetValidationReport& report, const std::string& category,
                    const std::filesystem::path& source, const std::string& reference,
                    const std::vector<std::filesystem::path>& roots, bool texture = false) {
    if (reference.empty()) {
        return;
    }

    std::filesystem::path resolved;
    try {
        resolved = findAsset(roots, reference, texture, false, &report);
    } catch (const std::filesystem::filesystem_error& error) {
        report.findings.push_back(
            {category, source, reference,
             "Could not inspect referenced file: " + std::string(error.what())});
        return;
    }
    if (resolved.empty()) {
        const auto relative = assetPath(reference);
        for (const auto& root : roots) {
            const auto candidate = root / relative;
            std::error_code statusError;
            const bool exists = std::filesystem::exists(candidate, statusError);
            if (statusError) {
                report.findings.push_back(
                    {category, source, reference,
                     "Could not inspect referenced path: " + statusError.message()});
                return;
            }
            if (!exists) {
                continue;
            }
            const bool regularFile = std::filesystem::is_regular_file(candidate, statusError);
            if (statusError) {
                report.findings.push_back(
                    {category, source, reference,
                     "Could not inspect referenced path: " + statusError.message()});
                return;
            }
            if (!regularFile) {
                report.findings.push_back(
                    {category, source, reference,
                     "Referenced path exists but is not a regular file: " + pathToUtf8(candidate)});
                return;
            }
        }
        report.findings.push_back({category, source, reference, "Referenced file was not found"});
    } else if (texture) {
        try {
            rendering::Image image;
            if (!rendering::TextureLoader::readImage(resolved, image)) {
                report.findings.push_back(
                    {category, source, reference,
                     "Resolved texture could not be decoded as an image: " + pathToUtf8(resolved)});
            }
        } catch (const std::exception& error) {
            report.findings.push_back({category, source, reference,
                                       "Could not decode resolved texture " + pathToUtf8(resolved) +
                                           ": " + error.what()});
        }
    }
}

void appendDiagnostic(MapAssetValidationReport& report, const std::string& category,
                      const std::filesystem::path& source,
                      const ConfigurationDiagnostic& diagnostic) {
    const auto location =
        diagnostic.line == 0 ? std::string{} : " (line " + std::to_string(diagnostic.line) + ")";
    const auto prefix =
        diagnostic.severity == ConfigurationDiagnostic::Severity::Warning ? "Parser warning: " : "";
    report.findings.push_back(
        {category, source, diagnostic.keyword, prefix + diagnostic.message + location});
}

bool checkResolvedRegularFile(MapAssetValidationReport& report, const std::string& category,
                              const std::filesystem::path& source, const std::string& reference,
                              const std::filesystem::path& resolvedPath,
                              const std::string& missingDetail) {
    std::error_code statusError;
    const bool exists = std::filesystem::exists(resolvedPath, statusError);
    if (statusError) {
        report.findings.push_back({category, source, reference,
                                   "Could not inspect referenced path: " + statusError.message()});
        return false;
    }
    if (!exists) {
        report.findings.push_back({category, source, reference, missingDetail});
        return false;
    }
    const bool regularFile = std::filesystem::is_regular_file(resolvedPath, statusError);
    if (statusError) {
        report.findings.push_back({category, source, reference,
                                   "Could not inspect referenced path: " + statusError.message()});
        return false;
    }
    if (!regularFile) {
        report.findings.push_back(
            {category, source, reference,
             "Referenced path exists but is not a regular file: " + pathToUtf8(resolvedPath)});
        return false;
    }
    return true;
}

void appendUnsupportedSections(MapAssetValidationReport& report,
                               const std::filesystem::path& source,
                               const std::vector<MapOpaqueSection>& sections,
                               const std::string& sectionType) {
    for (const auto& section : sections) {
        const auto findingSource = section.sourcePath.empty() ? source : section.sourcePath;
        report.findings.push_back({"unsupported section",
                                   findingSource,
                                   section.keyword,
                                   "OpenBus does not implement this " + sectionType +
                                       " section (line " + std::to_string(section.sectionLine) +
                                       ")",
                                   {},
                                   true});
    }
}

void appendUnsupportedHumanListEntries(MapAssetValidationReport& report,
                                       const std::filesystem::path& source,
                                       const MapHumanModelList& list) {
    for (const auto& entry : list.unsupportedEntries) {
        report.findings.push_back(
            {"unsupported list entry",
             source,
             entry.value,
             "List line is not a .hum asset path (line " + std::to_string(entry.line) + ")",
             {},
             true});
    }
}

void appendUnsupportedKeywordWarnings(MapAssetValidationReport& report,
                                      const std::filesystem::path& source,
                                      const ConfigurationDiagnostics& diagnostics,
                                      const std::string& warningMessage,
                                      const std::string& configType) {
    for (const auto& diagnostic : diagnostics.entries) {
        if (diagnostic.severity != ConfigurationDiagnostic::Severity::Warning ||
            diagnostic.message != warningMessage) {
            continue;
        }
        report.findings.push_back({"unsupported section",
                                   source,
                                   diagnostic.keyword,
                                   "OpenBus does not implement this " + configType +
                                       " keyword (line " + std::to_string(diagnostic.line) + ")",
                                   {},
                                   true});
    }
}

void appendErrors(MapAssetValidationReport& report, const std::string& category,
                  const std::filesystem::path& source, const ConfigurationDiagnostics& diagnostics,
                  bool includeWarnings = false, std::string_view skippedWarning = {}) {
    for (const auto& diagnostic : diagnostics.entries) {
        if (diagnostic.severity == ConfigurationDiagnostic::Severity::Warning &&
            !skippedWarning.empty() && diagnostic.message == skippedWarning) {
            continue;
        }
        if (includeWarnings || diagnostic.severity == ConfigurationDiagnostic::Severity::Error) {
            appendDiagnostic(report, category, source, diagnostic);
        }
    }
}

ConfigurationDiagnostics excludingDiagnostics(const ConfigurationDiagnostics& diagnostics,
                                              const ConfigurationDiagnostics& excluded) {
    ConfigurationDiagnostics result = diagnostics;
    for (const auto& excludedDiagnostic : excluded.entries) {
        const auto match =
            std::find_if(result.entries.begin(), result.entries.end(),
                         [&](const ConfigurationDiagnostic& diagnostic) {
                             return diagnostic.severity == excludedDiagnostic.severity &&
                                    diagnostic.line == excludedDiagnostic.line &&
                                    diagnostic.keyword == excludedDiagnostic.keyword &&
                                    diagnostic.message == excludedDiagnostic.message &&
                                    diagnostic.referencedPath == excludedDiagnostic.referencedPath;
                         });
        if (match != result.entries.end()) {
            result.entries.erase(match);
        }
    }
    return result;
}

void checkReferencedConfig(MapAssetValidationReport& report, const std::string& category,
                           const std::filesystem::path& source,
                           const std::filesystem::path& configPath,
                           const ConfigurationDiagnostics& diagnostics,
                           const std::string& openDiagnosticKeyword) {
    if (configPath.empty()) {
        return;
    }

    const auto findingsBeforeReferenceCheck = report.findings.size();
    checkReference(report, category, source, pathToUtf8(configPath), {configPath.root_path()});
    const bool referenceFindingReported = report.findings.size() != findingsBeforeReferenceCheck;
    for (const auto& diagnostic : diagnostics.entries) {
        if (diagnostic.severity != ConfigurationDiagnostic::Severity::Error) {
            continue;
        }
        if (referenceFindingReported && diagnostic.keyword == openDiagnosticKeyword &&
            diagnostic.message.rfind("unable to open ", 0) == 0) {
            continue;
        }
        appendDiagnostic(report, category, configPath, diagnostic);
    }
}

template <typename Loader>
void checkMapSupportConfig(MapAssetValidationReport& report, const std::filesystem::path& source,
                           const std::filesystem::path& omsiRoot, const std::string& reference,
                           const std::string& category, Loader loader) {
    if (reference.empty()) {
        return;
    }
    std::filesystem::path path;
    try {
        path = findAsset({omsiRoot}, reference, false, true, &report);
    } catch (const std::filesystem::filesystem_error& error) {
        report.findings.push_back(
            {category, source, reference,
             "Could not resolve referenced file: " + std::string(error.what())});
        return;
    }
    if (path.empty()) {
        checkReference(report, category, source, reference, {omsiRoot});
        return;
    }
    try {
        const auto diagnostics = loader(path).diagnostics;
        const std::string_view unsupportedMessage =
            category.find("ticket pack") != std::string::npos ? "unsupported ticket-pack section"
                                                              : std::string_view{};
        if (!unsupportedMessage.empty()) {
            appendUnsupportedKeywordWarnings(report, path, diagnostics,
                                             std::string(unsupportedMessage), "ticket-pack");
        }
        appendErrors(report, category, path, diagnostics, true, unsupportedMessage);
    } catch (const std::exception& error) {
        report.findings.push_back({category, path, {}, error.what()});
    }
}

void checkMesh(MapAssetValidationReport& report, const ModelPart& part,
               const std::filesystem::path& modelPath, const std::filesystem::path& objectRoot,
               const std::filesystem::path& modelRoot, const std::filesystem::path& omsiRoot,
               bool checkTextures, assets::VehicleTextureResolver* vehicleTextures,
               bool hasCtcTextures);

void checkCurrencyConfig(MapAssetValidationReport& report, const std::filesystem::path& source,
                         const std::filesystem::path& omsiRoot, const std::string& reference) {
    if (reference.empty()) {
        return;
    }
    std::filesystem::path path;
    try {
        path = findAsset({omsiRoot}, reference, false, true, &report);
    } catch (const std::filesystem::filesystem_error& error) {
        report.findings.push_back(
            {"currency", source, reference,
             "Could not resolve currency configuration: " + std::string(error.what())});
        return;
    }
    if (path.empty()) {
        checkReference(report, "currency", source, reference, {omsiRoot});
        return;
    }
    try {
        const auto currency = loadMapCurrencyDefinition(path);
        appendUnsupportedKeywordWarnings(report, path, currency.diagnostics,
                                         "unsupported currency section", "currency");
        appendErrors(report, "currency", path, currency.diagnostics, true,
                     "unsupported currency section");
        std::unordered_set<std::string> checkedModelMeshes;
        const auto checkModels = [&](const auto& items, const std::string& kind) {
            for (const auto& item : items) {
                try {
                    const auto modelPath =
                        assets::resolveReferencedPath(path, item.modelPath, true);
                    if (!checkResolvedRegularFile(report, "currency model", path, item.modelPath,
                                                  modelPath,
                                                  "Referenced " + kind + " model was not found")) {
                        continue;
                    }
                    if (checkedModelMeshes.insert(cacheKey(modelPath)).second) {
                        recordSourceFile(report, modelPath);
                        ModelPart part;
                        part.objPath = modelPath;
                        const auto modelRoot = modelPath.parent_path();
                        checkMesh(report, part, modelPath, modelRoot, modelRoot, omsiRoot, true,
                                  nullptr, false);
                    }
                } catch (const std::filesystem::filesystem_error& error) {
                    report.findings.push_back(
                        {"currency model", path, item.modelPath,
                         "Could not resolve " + kind + " model: " + std::string(error.what())});
                }
            }
        };
        checkModels(currency.coins, "coin");
        checkModels(currency.bills, "bill");
    } catch (const std::exception& error) {
        report.findings.push_back({"currency", path, {}, error.what()});
    }
}

void checkScripts(MapAssetValidationReport& report, const VehicleConfig& configuration) {
    for (const auto& script : configuration.scripts) {
        if (script.empty()) {
            continue;
        }
        try {
            const auto path = assets::resolveReferencedPath(configuration.sourcePath, script, true);
            if (checkResolvedRegularFile(report, "script", configuration.sourcePath, script, path,
                                         "Referenced script was not found")) {
                recordSourceFile(report, path);
            }
        } catch (const std::filesystem::filesystem_error& error) {
            report.findings.push_back({"script", configuration.sourcePath, script,
                                       "Could not resolve script: " + std::string(error.what())});
        }
    }
}

void appendScriptOwnerErrors(MapAssetValidationReport& report, const std::string& category,
                             const std::filesystem::path& source,
                             const ConfigurationDiagnostics& diagnostics,
                             const VehicleConfig& configuration) {
    struct ScriptList {
        const char* keyword;
        const char* category;
        const std::vector<std::string>& references;
    };
    const ScriptList lists[] = {
        {"varnamelist", "variable list", configuration.variableLists},
        {"stringvarnamelist", "string variable list", configuration.stringVariableLists},
        {"constfile", "constant file", configuration.constantFiles},
    };
    std::unordered_set<std::string> unavailable;
    for (const auto& list : lists) {
        for (const auto& reference : list.references) {
            const std::string key = std::string(list.keyword) + "\n" + reference;
            if (unavailable.contains(key)) {
                continue;
            }
            try {
                const auto strictPath =
                    assets::resolveReferencedPath(configuration.sourcePath, reference, true);
                if (!checkResolvedRegularFile(report, list.category, source, reference, strictPath,
                                              "Referenced file was not found")) {
                    unavailable.insert(key);
                } else {
                    recordSourceFile(report, strictPath);
                }
            } catch (const std::filesystem::filesystem_error& error) {
                if (unavailable.insert(key).second) {
                    report.findings.push_back(
                        {list.category, source, reference,
                         "Could not inspect referenced file: " + std::string(error.what())});
                }
            }
        }
    }
    for (const auto& diagnostic : diagnostics.entries) {
        if (diagnostic.severity != ConfigurationDiagnostic::Severity::Error) {
            continue;
        }
        bool handled = false;
        if (!diagnostic.referencedPath.empty()) {
            for (const auto& list : lists) {
                if (diagnostic.keyword != list.keyword) {
                    continue;
                }
                if (!unavailable.contains(diagnostic.keyword + "\n" + diagnostic.referencedPath)) {
                    report.findings.push_back(
                        {list.category, source, diagnostic.referencedPath, diagnostic.message});
                }
                handled = true;
                break;
            }
        }
        if (!handled) {
            appendDiagnostic(report, category, source, diagnostic);
        }
    }
}

void checkSoundConfig(MapAssetValidationReport& report, const std::filesystem::path& path) {
    try {
        recordSourceFile(report, path);
        std::unordered_map<std::string, bool> decodedSounds;
        config::Reader reader(path);
        if (!reader.isOpen()) {
            report.findings.push_back({"sound configuration", path, {}, "Could not open file"});
            return;
        }
        config::Line line;
        ConfigurationDiagnostics diagnostics;
        static const std::unordered_set<std::string> knownSoundKeywords = {
            "sound",  "loopsound", "trigger", "noloop",   "loop",
            "looped", "viewpoint", "3d",      "volcurve", "conditionsingle"};
        while (reader.next(line)) {
            if (!line.isKeyword()) {
                continue;
            }
            const auto keyword = config::lower(line.keyword());
            if (!knownSoundKeywords.contains(keyword)) {
                report.findings.push_back(
                    {"unsupported section",
                     path,
                     line.keyword(),
                     "OpenBus does not implement this sound configuration keyword; any asset "
                     "references in this directive are not checked (line " +
                         std::to_string(line.number) + ")",
                     {},
                     true});
                continue;
            }
            if (keyword != "sound" && keyword != "loopsound") {
                continue;
            }
            config::Line value;
            bool hadBlank = false;
            bool hasValue = false;
            while (reader.nextIncludingBlank(value)) {
                if (!value.text.empty()) {
                    hasValue = true;
                    break;
                }
                hadBlank = true;
            }
            if (!hasValue) {
                diagnostics.error(line.number, keyword, "missing sound filename at end of file");
            } else if (value.isKeyword()) {
                diagnostics.error(line.number, keyword,
                                  "missing sound filename before " + value.text);
                reader.pushBack(std::move(value));
            } else {
                double numericValue = 0.0;
                if (hadBlank && config::parseDouble(value.text, numericValue)) {
                    report.findings.push_back(
                        {"sound configuration", path, keyword,
                         "Blank sound filename may be dynamic or missing (line " +
                             std::to_string(line.number) + ")"});
                } else {
                    const std::string reference = config::trim(value.text);
                    const auto soundPath =
                        findAsset({path.parent_path()}, reference, false, false, &report);
                    if (soundPath.empty()) {
                        checkReference(report, "sound", path, reference, {path.parent_path()});
                    } else {
                        const auto [decoded, inserted] =
                            decodedSounds.try_emplace(cacheKey(soundPath), false);
                        if (inserted) {
                            decoded->second = assets::decodeWavFile(soundPath).has_value();
                        }
                        if (!decoded->second) {
                            report.findings.push_back(
                                {"sound", path, reference,
                                 "File could not be decoded by the runtime WAV decoder"});
                        }
                    }
                }
            }
        }
        appendErrors(report, "sound configuration", path, diagnostics);
    } catch (const std::exception& error) {
        report.findings.push_back({"sound configuration", path, {}, error.what()});
    }
}

void checkPassengerStepSounds(MapAssetValidationReport& report, const ::VehicleConfig& vehicle,
                              const std::filesystem::path& omsiRoot) {
    if (vehicle.pathsConfigPath.empty()) {
        return;
    }

    const std::filesystem::path passengerSoundRoot = omsiRoot / "Sounds" / "Passengers";
    std::unordered_set<std::string> checkedSounds;
    for (const auto& pack : vehicle.passengerStepSoundPacks) {
        for (const std::string& value : pack) {
            const std::string reference = config::trim(value);
            if (reference.empty()) {
                continue;
            }
            const auto soundPath =
                findAsset({passengerSoundRoot}, reference, false, false, &report);
            if (soundPath.empty()) {
                checkReference(report, "step sound", vehicle.pathsConfigPath, reference,
                               {passengerSoundRoot});
                continue;
            }
            if (!checkedSounds.insert(cacheKey(soundPath)).second) {
                continue;
            }
            if (!assets::decodeWavFile(soundPath)) {
                report.findings.push_back({"step sound", vehicle.pathsConfigPath, reference,
                                           "File could not be decoded by the runtime WAV decoder"});
            }
        }
    }
}

void checkSeasonalSceneryTextures(MapAssetValidationReport& report,
                                  const std::filesystem::path& source,
                                  const std::filesystem::path& texturePath) {
    try {
        for (const char* season : rendering::map_detail::treeSeasonDirectories) {
            const auto directory =
                assets::resolveCaseInsensitivePath(texturePath.parent_path() / season, true);
            if (!std::filesystem::is_directory(directory)) {
                continue;
            }
            const auto seasonalPath =
                assets::resolveTexturePath(directory / texturePath.filename());
            std::error_code statusError;
            const bool exists = std::filesystem::exists(seasonalPath, statusError);
            if (statusError) {
                report.findings.push_back(
                    {"seasonal scenery texture", source, pathToUtf8(seasonalPath),
                     "Could not inspect seasonal texture: " + statusError.message()});
                continue;
            }
            if (!exists) {
                continue;
            }
            const bool regularFile = std::filesystem::is_regular_file(seasonalPath, statusError);
            if (statusError) {
                report.findings.push_back(
                    {"seasonal scenery texture", source, pathToUtf8(seasonalPath),
                     "Could not inspect seasonal texture: " + statusError.message()});
                continue;
            }
            if (!regularFile) {
                report.findings.push_back(
                    {"seasonal scenery texture", source, pathToUtf8(seasonalPath),
                     "Seasonal texture path exists but is not a regular file"});
                continue;
            }

            recordSourceFile(report, seasonalPath);
            try {
                rendering::Image image;
                if (!rendering::TextureLoader::readImage(seasonalPath, image)) {
                    report.findings.push_back(
                        {"seasonal scenery texture", source, pathToUtf8(seasonalPath),
                         "Resolved seasonal texture could not be decoded as an image"});
                }
            } catch (const std::exception& error) {
                report.findings.push_back(
                    {"seasonal scenery texture", source, pathToUtf8(seasonalPath),
                     "Could not decode resolved seasonal texture: " + std::string(error.what())});
            }
        }
    } catch (const std::filesystem::filesystem_error& error) {
        report.findings.push_back(
            {"seasonal scenery texture", source, pathToUtf8(texturePath),
             "Could not inspect seasonal texture variants: " + std::string(error.what())});
    }
}

void checkResolvedSceneryTexture(MapAssetValidationReport& report,
                                 const std::filesystem::path& source, const std::string& reference,
                                 const std::filesystem::path& path,
                                 bool checkSeasonalVariants = false) {
    recordSourceFile(report, path);
    try {
        rendering::Image image;
        if (!rendering::TextureLoader::readImage(path, image)) {
            report.findings.push_back(
                {"model texture", source, reference,
                 "Resolved texture could not be decoded as an image: " + pathToUtf8(path)});
        }
    } catch (const std::exception& error) {
        report.findings.push_back(
            {"model texture", source, reference,
             "Could not decode resolved texture " + pathToUtf8(path) + ": " + error.what()});
    }
    if (checkSeasonalVariants) {
        checkSeasonalSceneryTextures(report, source, path);
    }
}

void checkSceneryTexture(MapAssetValidationReport& report, const std::filesystem::path& source,
                         const std::filesystem::path& objectRoot,
                         const std::filesystem::path& modelRoot,
                         const std::filesystem::path& omsiRoot, const std::string& name,
                         bool checkSeasonalVariants = false) {
    if (name.empty()) {
        return;
    }
    const auto path = assets::findSceneryTexture(objectRoot, modelRoot, omsiRoot, name);
    if (path.empty()) {
        try {
            for (const auto& candidate :
                 assets::sceneryTextureCandidates(objectRoot, modelRoot, omsiRoot, name)) {
                if (candidate.empty()) {
                    continue;
                }
                const auto resolved = assets::resolveTexturePath(candidate);
                std::error_code statusError;
                const bool exists = std::filesystem::exists(resolved, statusError);
                if (statusError) {
                    report.findings.push_back(
                        {"model texture", source, name,
                         "Could not inspect referenced texture: " + statusError.message()});
                    return;
                }
                if (!exists) {
                    continue;
                }
                const bool regularFile = std::filesystem::is_regular_file(resolved, statusError);
                if (statusError) {
                    report.findings.push_back(
                        {"model texture", source, name,
                         "Could not inspect referenced texture: " + statusError.message()});
                    return;
                }
                if (!regularFile) {
                    report.findings.push_back(
                        {"model texture", source, name,
                         "Referenced texture path exists but is not a regular file: " +
                             pathToUtf8(resolved)});
                    return;
                }
            }
        } catch (const std::filesystem::filesystem_error& error) {
            report.findings.push_back(
                {"model texture", source, name,
                 "Could not inspect referenced texture: " + std::string(error.what())});
            return;
        }
        report.findings.push_back({"model texture", source, name, "Referenced file was not found"});
    } else {
        checkResolvedSceneryTexture(report, source, name, path, checkSeasonalVariants);
    }
}

void checkMaterial(MapAssetValidationReport& report, const ModelMaterialState& material,
                   const std::filesystem::path& source, const std::filesystem::path& objectRoot,
                   const std::filesystem::path& modelRoot, const std::filesystem::path& omsiRoot) {
    if (material.textTextureIndex < 0 && material.scriptTextureIndex < 0) {
        checkSceneryTexture(report, source, objectRoot, modelRoot, omsiRoot, material.textureName,
                            true);
    }
    for (const auto* name :
         {&material.transmapTextureName, &material.nightmapTextureName,
          &material.lightmapTextureName, &material.freeTextureName, &material.bumpmapTextureName}) {
        checkSceneryTexture(report, source, objectRoot, modelRoot, omsiRoot, *name);
    }
    if (material.environmentStrength > 0.0) {
        checkSceneryTexture(report, source, objectRoot, modelRoot, omsiRoot,
                            material.environmentTextureName);
    }
    for (const auto& change : material.textureChanges) {
        for (const auto& item : change.items) {
            if (item) {
                checkMaterial(report, *item, source, objectRoot, modelRoot, omsiRoot);
            }
        }
    }
}

void checkVehicleTexture(MapAssetValidationReport& report, const std::string& name,
                         const std::filesystem::path& source, const std::filesystem::path& root,
                         assets::VehicleTextureResolver& resolver, bool scriptReference,
                         const std::string& bundleEntry = {}) {
    if (name.empty() || (scriptReference && scripting::scriptTextureReferenceIndex(name) >= 0)) {
        return;
    }
    const std::string context = bundleEntry.empty() ? "" : " (bundle entry: " + bundleEntry + ")";
    try {
        const auto resolved = resolver.find(root, name);
        if (resolved.empty()) {
            report.findings.push_back({"vehicle texture", source, name,
                                       "Referenced file was not found (mesh directory: " +
                                           pathToUtf8(root) + ")" + context});
        } else if (!std::filesystem::is_regular_file(resolved)) {
            report.findings.push_back(
                {"vehicle texture", source, name,
                 "Resolved texture is not a regular file: " + pathToUtf8(resolved) + context});
        } else {
            recordSourceFile(report, resolved);
            try {
                rendering::Image image;
                if (!rendering::TextureLoader::readImage(resolved, image)) {
                    report.findings.push_back(
                        {"vehicle texture", source, name,
                         "Resolved texture could not be decoded as an image: " +
                             pathToUtf8(resolved) + context});
                }
            } catch (const std::exception& error) {
                report.findings.push_back({"vehicle texture", source, name,
                                           "Could not decode resolved texture " +
                                               pathToUtf8(resolved) + ": " + error.what() +
                                               context});
            }
        }
    } catch (const std::filesystem::filesystem_error& error) {
        report.findings.push_back(
            {"vehicle texture", source, name,
             "Could not resolve texture in " + pathToUtf8(root) + ": " + error.what() + context});
    }
}

void checkVehicleMaterial(MapAssetValidationReport& report, const ModelMaterialState& material,
                          const std::filesystem::path& source, const std::filesystem::path& root,
                          assets::VehicleTextureResolver& resolver) {
    if (material.textTextureIndex < 0 && material.scriptTextureIndex < 0) {
        checkVehicleTexture(report, material.textureName, source, root, resolver, true);
    }
    for (const auto* name : {&material.transmapTextureName, &material.nightmapTextureName,
                             &material.lightmapTextureName, &material.bumpmapTextureName}) {
        checkVehicleTexture(report, *name, source, root, resolver, true);
    }
    if (material.environmentStrength > 0.0) {
        checkVehicleTexture(report, material.environmentTextureName, source, root, resolver, false);
    }
    if (material.textTextureIndex < 0 && material.scriptTextureIndex < 0 &&
        !material.freeTextureVariable.empty()) {
        checkVehicleTexture(report, material.freeTextureName, source, root, resolver, true);
    }
    for (const auto& change : material.textureChanges) {
        for (const auto& item : change.items) {
            if (item) {
                checkVehicleMaterial(report, *item, source, root, resolver);
            }
        }
    }
}

void checkObjLibraries(MapAssetValidationReport& report, const std::filesystem::path& meshPath) {
    recordSourceFile(report, meshPath);
    std::ifstream input(meshPath);
    if (!input) {
        throw std::runtime_error("Could not open OBJ mesh");
    }
    std::string line;
    while (std::getline(input, line)) {
        std::istringstream fields(line);
        std::string keyword;
        std::string reference;
        fields >> keyword;
        if (keyword == "mtllib") {
            if (!(fields >> reference)) {
                report.findings.push_back(
                    {"material library", meshPath, {}, "Missing material library reference"});
            } else {
                checkReference(report, "material library", meshPath, reference,
                               {meshPath.parent_path()});
            }
        }
    }
    if (input.bad()) {
        throw std::runtime_error("Could not read OBJ mesh completely");
    }
}

void checkMesh(MapAssetValidationReport& report, const ModelPart& part,
               const std::filesystem::path& modelPath, const std::filesystem::path& objectRoot,
               const std::filesystem::path& modelRoot, const std::filesystem::path& omsiRoot,
               bool checkTextures, assets::VehicleTextureResolver* vehicleTextures,
               bool hasCtcTextures) {
    if (part.objPath.empty()) {
        return;
    }
    try {
        recordSourceFile(report, part.objPath);
        std::string extension = pathToUtf8(part.objPath.extension());
        std::transform(extension.begin(), extension.end(), extension.begin(),
                       [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
        std::shared_ptr<rendering::ParsedObj> parsed;
        if (!part.bundleEntry.empty()) {
            parsed = rendering::ObjLoader::parse(part.objPath, part.bundleEntry);
        } else if (extension == ".obj") {
            checkObjLibraries(report, part.objPath);
            parsed = rendering::ObjLoader::parse(part.objPath);
        } else if (extension == ".o3d") {
            parsed = rendering::O3DLoader::parse(part.objPath);
        } else if (extension == ".x") {
            parsed = rendering::XLoader::parse(part.objPath);
        }
        if (!parsed) {
            report.findings.push_back(
                {"mesh", modelPath,
                 part.bundleEntry.empty() ? pathToUtf8(part.objPath) : part.bundleEntry,
                 "Could not parse mesh; embedded dependencies were not checked"});
            return;
        }
        if (!checkTextures) {
            if (vehicleTextures && !hasCtcTextures) {
                std::unordered_set<std::string> checkedMaterials;
                for (const auto& triangle : parsed->triangles) {
                    if (!checkedMaterials.insert(triangle.material).second) {
                        continue;
                    }
                    const auto material = parsed->materials.find(triangle.material);
                    if (material == parsed->materials.end()) {
                        continue;
                    }
                    const auto state = std::find_if(
                        part.materialStatesInOrder.begin(), part.materialStatesInOrder.end(),
                        [&](const ModelMaterialState& item) {
                            return item.materialIndex == material->second.materialIndex;
                        });
                    if (state != part.materialStatesInOrder.end() &&
                        (state->textTextureIndex >= 0 || state->scriptTextureIndex >= 0)) {
                        continue;
                    }
                    const std::string name =
                        state != part.materialStatesInOrder.end() && !state->textureName.empty()
                            ? state->textureName
                            : (material->second.textureName.empty()
                                   ? material->second.texturePath.string()
                                   : material->second.textureName);
                    checkVehicleTexture(report, name, part.objPath, part.objPath.parent_path(),
                                        *vehicleTextures, true, part.bundleEntry);
                }
            }
            return;
        }
        const auto states = resolveSceneryMaterialStates(*parsed, part);
        for (const auto& [name, material] : parsed->materials) {
            (void)name;
            const auto state = states.find(material.materialIndex);
            if (state != states.end() &&
                (state->second->textTextureIndex >= 0 || state->second->scriptTextureIndex >= 0 ||
                 !state->second->textureName.empty())) {
                continue;
            }
            if (!material.textureName.empty()) {
                checkSceneryTexture(report, part.objPath, objectRoot, modelRoot, omsiRoot,
                                    material.textureName, true);
            }
        }
    } catch (const std::exception& error) {
        report.findings.push_back({"mesh", modelPath, pathToUtf8(part.objPath), error.what()});
    }
}

void checkVehicleCollisionMesh(MapAssetValidationReport& report,
                               const ModelCollisionMesh& collision,
                               const std::filesystem::path& modelPath,
                               const std::filesystem::path& modelRoot) {
    const std::string reference = pathToUtf8(collision.sourcePath);
    if (collision.resolvedPath.empty()) {
        if (!reference.empty()) {
            checkReference(report, "collision mesh", modelPath, reference,
                           {modelRoot, modelRoot.parent_path()});
        }
        return;
    }
    if (!checkResolvedRegularFile(report, "collision mesh", modelPath, reference,
                                  collision.resolvedPath, "Referenced mesh was not found")) {
        return;
    }
    try {
        recordSourceFile(report, collision.resolvedPath);
        std::string extension = pathToUtf8(collision.resolvedPath.extension());
        std::transform(extension.begin(), extension.end(), extension.begin(),
                       [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
        const auto parsed =
            extension == ".o3d"
                ? rendering::O3DLoader::parse(collision.resolvedPath)
                : rendering::ObjLoader::parse(collision.resolvedPath, collision.bundleEntry);
        if (!parsed) {
            report.findings.push_back(
                {"collision mesh", modelPath, reference,
                 "Could not parse collision mesh: " + pathToUtf8(collision.resolvedPath)});
        } else if (std::none_of(parsed->triangles.begin(), parsed->triangles.end(),
                                [&](const auto& triangle) {
                                    return assets::isUsableCollisionTriangle(*parsed, triangle);
                                })) {
            report.findings.push_back(
                {"collision mesh", modelPath, reference,
                 "Collision mesh has no usable triangles: " + pathToUtf8(collision.resolvedPath)});
        }
    } catch (const std::exception& error) {
        report.findings.push_back({"collision mesh", modelPath, reference, error.what()});
    }
}

void checkSceneryCollisionMesh(MapAssetValidationReport& report,
                               const std::filesystem::path& source, const std::string& reference,
                               const std::filesystem::path& resolvedPath) {
    if (resolvedPath.empty()) {
        if (!reference.empty()) {
            report.findings.push_back(
                {"collision mesh", source, reference, "Referenced mesh was not found"});
        }
        return;
    }
    if (!checkResolvedRegularFile(report, "collision mesh", source, reference, resolvedPath,
                                  "Referenced mesh was not found")) {
        return;
    }
    try {
        recordSourceFile(report, resolvedPath);
        const auto parsed = rendering::O3DLoader::parse(resolvedPath);
        if (!parsed) {
            report.findings.push_back(
                {"collision mesh", source, reference,
                 "Could not parse scenery collision mesh: " + pathToUtf8(resolvedPath)});
        } else if (std::none_of(parsed->triangles.begin(), parsed->triangles.end(),
                                [&](const auto& triangle) {
                                    return assets::hasValidCollisionIndices(*parsed, triangle);
                                })) {
            report.findings.push_back(
                {"collision mesh", source, reference,
                 "Scenery collision mesh has no usable triangles: " + pathToUtf8(resolvedPath)});
        }
    } catch (const std::exception& error) {
        report.findings.push_back({"collision mesh", source, reference, error.what()});
    }
}

void checkModel(MapAssetValidationReport& report, const std::filesystem::path& modelPath,
                const std::filesystem::path& modelRoot, ModelConfigKind kind,
                const std::filesystem::path& objectRoot, const std::filesystem::path& omsiRoot,
                bool checkTextures, const std::filesystem::path& sceneryMapRoot = {},
                assets::VehicleTextureResolver* vehicleTextures = nullptr,
                std::vector<ModelCtcTexture>* vehicleCtcTextures = nullptr,
                std::vector<ModelCtcTemplate>* vehicleCtcTemplates = nullptr) {
    recordSourceFile(report, modelPath);
    Variables variables(kind == ModelConfigKind::SceneryObject ? ScriptObjectKind::SceneryObject
                                                               : ScriptObjectKind::Vehicle);
    const ModelConfig model = loadModelConfig(modelPath, modelRoot, kind, variables);
    if (vehicleCtcTextures) {
        *vehicleCtcTextures = model.ctcTextures;
    }
    if (vehicleCtcTemplates) {
        *vehicleCtcTemplates = model.ctcTemplates;
    }
    appendErrors(report, "model", modelPath, model.diagnostics);
    appendUnsupportedKeywordWarnings(report, modelPath, model.diagnostics, "unknown keyword",
                                     "model.cfg");
    for (const auto& part : model.parts) {
        if (part.objPath.empty() && !part.sourceMeshPath.empty()) {
            std::vector<std::filesystem::path> roots = {modelRoot, modelRoot.parent_path()};
            const std::string modelDirectory = pathToUtf8(modelRoot.filename());
            if (modelDirectory.size() > 4 &&
                modelDirectory.compare(modelDirectory.size() - 4, 4, "_obj") == 0) {
                roots.push_back(modelRoot.parent_path() /
                                pathFromUtf8(modelDirectory.substr(0, modelDirectory.size() - 4)));
            }
            checkReference(report, "mesh", modelPath, pathToUtf8(part.sourceMeshPath), roots);
        }
        checkMesh(report, part, modelPath, objectRoot, modelRoot, omsiRoot, checkTextures,
                  vehicleTextures, !model.ctcTextures.empty());
        if (checkTextures) {
            for (const auto& material : part.materialStatesInOrder) {
                checkMaterial(report, material, modelPath, objectRoot, modelRoot, omsiRoot);
            }
        } else if (vehicleTextures && !part.objPath.empty()) {
            for (const auto& material : part.materialStatesInOrder) {
                checkVehicleMaterial(report, material, modelPath, part.objPath.parent_path(),
                                     *vehicleTextures);
            }
        }
    }
    for (const auto& collision : model.collisionMeshes) {
        if (kind != ModelConfigKind::SceneryObject) {
            checkVehicleCollisionMesh(report, collision, modelPath, modelRoot);
        } else if (!sceneryMapRoot.empty()) {
            auto resolvedPath = collision.resolvedPath;
            const std::vector<std::filesystem::path> roots = {modelRoot, omsiRoot, sceneryMapRoot};
            if (resolvedPath.empty()) {
                resolvedPath =
                    findAsset(roots, pathToUtf8(collision.sourcePath), false, false, &report);
            }
            if (resolvedPath.empty()) {
                checkReference(report, "collision mesh", modelPath,
                               pathToUtf8(collision.sourcePath), roots);
            } else {
                checkSceneryCollisionMesh(report, modelPath, pathToUtf8(collision.sourcePath),
                                          resolvedPath);
            }
            if (!resolvedPath.empty()) {
                recordSourceFile(report, resolvedPath);
            }
        } else if (collision.resolvedPath.empty() && !collision.sourcePath.empty()) {
            checkReference(report, "collision mesh", modelPath, pathToUtf8(collision.sourcePath),
                           {modelRoot, modelRoot.parent_path(), omsiRoot});
        }
    }
}

bool checkScenery(MapAssetValidationReport& report, const std::filesystem::path& source,
                  const std::string& reference, const std::filesystem::path& mapRoot,
                  const std::filesystem::path& omsiRoot,
                  std::unordered_map<std::string, bool>& checkedObjects) {
    const auto path = findAsset({omsiRoot, mapRoot}, reference, false, false, &report);
    if (path.empty()) {
        checkReference(report, "object", source, reference, {omsiRoot, mapRoot});
        return false;
    }
    const auto [cached, inserted] = checkedObjects.try_emplace(cacheKey(path), false);
    if (!inserted) {
        return cached->second;
    }
    try {
        const auto object = loadSceneryObjectFile(path, false);
        cached->second = object.carPark;
        const auto objectDiagnostics =
            excludingDiagnostics(object.diagnostics, object.passengerCabinDiagnostics);
        appendUnsupportedKeywordWarnings(report, path, objectDiagnostics,
                                         "unknown SceneryObject keyword", "scenery-object");
        appendScriptOwnerErrors(report, "object", path, objectDiagnostics,
                                object.scriptConfiguration);
        checkScripts(report, object.scriptConfiguration);
        checkReferencedConfig(report, "passenger cabin", path, object.passengerCabinConfigPath,
                              object.passengerCabinDiagnostics, "passengercabin");
        const auto objectRoot = path.parent_path();
        if (!object.collisionMesh.empty()) {
            const auto collisionReference = pathToUtf8(object.collisionMesh);
            const std::vector<std::filesystem::path> roots = {objectRoot / "model", omsiRoot,
                                                              mapRoot, objectRoot};
            const auto collisionPath = findAsset(roots, collisionReference, false, false, &report);
            if (collisionPath.empty()) {
                checkReference(report, "collision mesh", path, collisionReference, roots);
            } else {
                checkSceneryCollisionMesh(report, path, collisionReference, collisionPath);
            }
        }
        auto modelPath = resolveSceneryObjectModelConfigPath(object);
        bool external = !object.modelPath.empty();
        if (external &&
            !checkResolvedRegularFile(report, "model", path, pathToUtf8(object.modelPath),
                                      modelPath, "Referenced model configuration was not found")) {
            modelPath = path;
            external = false;
        }
        const auto modelRoot = external ? modelPath.parent_path() : objectRoot / "model";
        for (const auto& tree : object.trees) {
            checkSceneryTexture(report, path, objectRoot, modelRoot, omsiRoot, tree.texturePath,
                                true);
        }
        checkModel(report, modelPath, modelRoot, ModelConfigKind::SceneryObject, objectRoot,
                   omsiRoot, true, mapRoot);
    } catch (const std::exception& error) {
        report.findings.push_back({"object", path, {}, error.what()});
    }
    return cached->second;
}

bool checkCarparkList(MapAssetValidationReport& report, const std::filesystem::path& source,
                      const std::vector<std::string>& labels, const std::filesystem::path& mapRoot,
                      const std::filesystem::path& omsiRoot,
                      std::unordered_map<std::string, bool>& checkedObjects,
                      std::unordered_set<std::string>& checkedLists,
                      const std::filesystem::path& listRoot, bool fallbackToMapRoot = true) {
    const auto index = carparkListIndex(labels);
    if (!index) {
        report.findings.push_back(
            {"park list", source, labels.front(), "Invalid car-park list index"});
        return false;
    }
    auto requested = listRoot / carparkListFilename(*index);
    try {
        auto path = assets::resolveCaseInsensitivePath(requested, true);
        std::error_code statusError;
        bool exists = std::filesystem::exists(path, statusError);
        if (statusError) {
            throw std::filesystem::filesystem_error("Could not inspect car-park list", path,
                                                    statusError);
        }
        if (!exists && listRoot != mapRoot && fallbackToMapRoot) {
            requested = mapRoot / carparkListFilename(*index);
            path = assets::resolveCaseInsensitivePath(requested, true);
            exists = std::filesystem::exists(path, statusError);
            if (statusError) {
                throw std::filesystem::filesystem_error("Could not inspect car-park list", path,
                                                        statusError);
            }
        }
        if (!exists && listRoot != mapRoot && !fallbackToMapRoot) {
            return false;
        }
        if (!checkedLists.insert(cacheKey(path)).second) {
            return exists;
        }
        if (!checkResolvedRegularFile(report, "park list", source, pathToUtf8(requested.filename()),
                                      path, "Referenced car-park list was not found")) {
            return exists;
        }
        recordSourceFile(report, path);
        bool unresolvedAccess = false;
        const auto list = readCarparkList(path, [&](const std::string& reference) {
            try {
                if (findAsset({omsiRoot, mapRoot}, reference, false, true, &report).empty()) {
                    checkReference(report, "parked object", path, reference, {omsiRoot, mapRoot});
                    return false;
                }
                (void)checkScenery(report, path, reference, mapRoot, omsiRoot, checkedObjects);
                return true;
            } catch (const std::filesystem::filesystem_error& error) {
                unresolvedAccess = true;
                report.findings.push_back(
                    {"parked object", path, reference,
                     "Could not resolve scenery reference: " + std::string(error.what())});
                return false;
            }
        });
        if (!list.opened) {
            report.findings.push_back(
                {"park list", source, pathToUtf8(path), "Could not open car-park list"});
            return true;
        }
        if (!list.complete) {
            report.findings.push_back(
                {"park list", source, pathToUtf8(path), "Could not read car-park list completely"});
        }
        if (list.complete && !unresolvedAccess && list.entries.empty()) {
            report.findings.push_back(
                {"park list", source, pathToUtf8(path), "Car-park list has no available objects"});
        }
        return true;
    } catch (const std::filesystem::filesystem_error& error) {
        report.findings.push_back(
            {"park list", source, pathToUtf8(requested),
             "Could not resolve car-park list: " + std::string(error.what())});
        return false;
    }
}

void checkHuman(MapAssetValidationReport& report, const std::filesystem::path& source,
                const std::string& reference, const std::filesystem::path& omsiRoot,
                std::unordered_set<std::string>& checkedHumans, const std::string& category) {
    const auto path = findAsset({omsiRoot}, reference, false, false, &report);
    if (path.empty()) {
        checkReference(report, category, source, reference, {omsiRoot});
        return;
    }
    if (!checkedHumans.insert(cacheKey(path)).second) {
        return;
    }
    try {
        const auto human = config::loadHumanConfig(path);
        appendErrors(report, category, path, human.diagnostics);
        appendUnsupportedKeywordWarnings(report, path, human.diagnostics, "unknown human keyword",
                                         "human config");
        if (human.modelConfigPath.empty()) {
            return;
        }
        const auto modelPath = assets::resolveReferencedPath(path, human.modelConfigPath);
        if (!checkResolvedRegularFile(report, "model", path, human.modelConfigPath, modelPath,
                                      "Referenced model configuration was not found")) {
            return;
        }
        recordSourceFile(report, modelPath);
        checkModel(report, modelPath, modelPath.parent_path(), ModelConfigKind::SceneryObject,
                   modelPath.parent_path(), omsiRoot, true);
    } catch (const std::exception& error) {
        report.findings.push_back({category, path, {}, error.what()});
    }
}

struct VehicleReference {
    std::filesystem::path source;
    std::string reference;
    std::string category;
};

struct TrainReference {
    std::filesystem::path source;
    std::string reference;
    std::string category;
};

struct AiTrafficReference {
    std::filesystem::path source;
    MapAiTrafficData traffic;
};

struct HumanReference {
    std::filesystem::path source;
    std::string reference;
    std::string category;
};

void checkNonRegularVehicleSideFiles(MapAssetValidationReport& report,
                                     const std::filesystem::path& vehiclePath,
                                     const std::filesystem::path& omsiRoot,
                                     const std::string& extension, const std::string& category,
                                     const std::string& fileType) {
    const auto reportDirectory = [&](const std::filesystem::path& path) {
        report.findings.push_back({category, vehiclePath, pathToUtf8(path),
                                   fileType + " path exists but is not a regular file"});
    };
    const auto inspectEntry = [&](const std::filesystem::directory_entry& entry) {
        const auto entryExtension = config::lower(pathToUtf8(entry.path().extension()));
        if (entryExtension != extension) {
            return;
        }
        std::error_code statusError;
        const bool regularFile = entry.is_regular_file(statusError);
        if (statusError) {
            report.findings.push_back(
                {category, vehiclePath, pathToUtf8(entry.path()),
                 "Could not inspect " + fileType + " path: " + statusError.message()});
        } else if (!regularFile) {
            reportDirectory(entry.path());
        }
    };

    const auto vehicleDirectory = vehiclePath.parent_path();
    std::error_code iterationError;
    std::filesystem::recursive_directory_iterator nested(
        vehicleDirectory, std::filesystem::directory_options::skip_permission_denied,
        iterationError);
    const std::filesystem::recursive_directory_iterator end;
    while (!iterationError && nested != end) {
        std::error_code typeError;
        if (nested->is_directory(typeError) &&
            config::lower(pathToUtf8(nested->path().extension())) == extension) {
            reportDirectory(nested->path());
            nested.disable_recursion_pending();
        } else if (typeError) {
            report.findings.push_back({category, vehiclePath, pathToUtf8(nested->path()),
                                       "Could not inspect vehicle asset: " + typeError.message()});
        } else {
            inspectEntry(*nested);
        }
        nested.increment(iterationError);
    }
    if (iterationError) {
        report.findings.push_back(
            {category,
             vehiclePath,
             {},
             "Could not enumerate vehicle assets: " + iterationError.message()});
        return;
    }

    const auto vehiclesRoot = std::filesystem::absolute(omsiRoot / "Vehicles").lexically_normal();
    for (auto parent = vehicleDirectory.parent_path(); !parent.empty();
         parent = parent.parent_path()) {
        std::filesystem::directory_iterator entry(parent, iterationError), parentEnd;
        for (; !iterationError && entry != parentEnd; entry.increment(iterationError)) {
            inspectEntry(*entry);
        }
        if (iterationError) {
            report.findings.push_back(
                {category, vehiclePath, pathToUtf8(parent),
                 "Could not enumerate vehicle assets: " + iterationError.message()});
            return;
        }
        if (std::filesystem::absolute(parent).lexically_normal() == vehiclesRoot ||
            parent == parent.parent_path()) {
            break;
        }
    }
}

void checkAiDepotHofs(MapAssetValidationReport& report, const MapDefinition& map,
                      const std::filesystem::path& omsiRoot,
                      const std::vector<AiTrafficReference>& chronoTraffic) {
    std::unordered_map<std::string, std::vector<config::VehicleDepot>> vehicleDepots;
    std::unordered_set<std::string> checkedAssociations;
    std::unordered_set<std::string> checkedHofs;
    const auto checkAssociation = [&](const std::string& hofName,
                                      const std::string& vehicleReference,
                                      const std::filesystem::path& source,
                                      const std::string& category) {
        const auto extension = config::lower(pathToUtf8(assetPath(vehicleReference).extension()));
        if (extension != ".bus") {
            return;
        }
        const auto vehiclePath = findAsset({omsiRoot}, vehicleReference, false, false, &report);
        if (vehiclePath.empty()) {
            return;
        }
        const auto vehicleKey = cacheKey(vehiclePath);
        const auto associationKey =
            vehicleKey + "\n" + config::lower(hofName) + "\n" + cacheKey(source);
        if (!checkedAssociations.insert(associationKey).second) {
            return;
        }
        try {
            auto [depots, inserted] = vehicleDepots.try_emplace(vehicleKey);
            if (inserted) {
                depots->second = config::discoverVehicleDepots(vehiclePath, omsiRoot / "Vehicles");
                for (const auto& vehicleDepot : depots->second) {
                    recordSourceFile(report, vehicleDepot.path);
                }
                checkNonRegularVehicleSideFiles(report, vehiclePath, omsiRoot, ".hof",
                                                "vehicle depot", "HOF");
            }
            const auto matchingDepot = std::find_if(
                depots->second.begin(), depots->second.end(), [&](const auto& vehicleDepot) {
                    return config::lower(vehicleDepot.name) == config::lower(hofName);
                });
            if (matchingDepot == depots->second.end()) {
                report.findings.push_back(
                    {category, source, hofName,
                     "No matching HOF was found for vehicle " + vehicleReference});
            } else if (checkedHofs.insert(cacheKey(matchingDepot->path)).second) {
                const auto hof = config::loadHofData(matchingDepot->path);
                for (const auto& section : hof.unsupportedSections) {
                    report.findings.push_back(
                        {"unsupported section",
                         matchingDepot->path,
                         section.keyword,
                         "OpenBus does not implement this HOF section (line " +
                             std::to_string(section.line) + ")",
                         {},
                         true});
                }
            }
        } catch (const std::exception& error) {
            report.findings.push_back({category, source, hofName,
                                       "Could not inspect HOF files for vehicle " +
                                           vehicleReference + ": " + error.what()});
        }
    };
    const auto checkAssociations = [&](const MapAiTrafficData& traffic,
                                       const std::filesystem::path& source) {
        for (const auto& depot : traffic.depots) {
            for (const auto typeIndex : depot.busTypeDefinitionIndices) {
                if (typeIndex < traffic.busTypes.size()) {
                    checkAssociation(depot.hofName, traffic.busTypes[typeIndex].assetPath, source,
                                     "AI depot HOF");
                }
            }
        }
        for (const auto& group : traffic.vehicleGroups) {
            if (group.hofName.empty()) {
                continue;
            }
            for (const auto& vehicle : group.vehicles) {
                checkAssociation(group.hofName, vehicle.assetPath, source, "AI group HOF");
            }
        }
    };
    checkAssociations(map.aiTraffic, map.rootPath / "ailists.cfg");
    for (const auto& chronoReference : chronoTraffic) {
        checkAssociations(chronoReference.traffic, chronoReference.source);
    }
}

std::string normalizeCtcSlot(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return ch == '\\' ? '/' : static_cast<char>(std::tolower(ch));
    });
    return value;
}

void checkVehicleRepaintTextures(MapAssetValidationReport& report,
                                 const std::filesystem::path& vehicleConfigPath,
                                 const std::filesystem::path& omsiRoot,
                                 const std::vector<ModelCtcTexture>& modelTextures,
                                 const std::vector<ModelCtcTemplate>& ctcTemplates) {
    if (modelTextures.empty()) {
        return;
    }
    std::unordered_set<std::string> modelSlots;
    for (const auto& texture : modelTextures) {
        modelSlots.insert(normalizeCtcSlot(texture.slot));
    }
    std::vector<std::filesystem::path> appearanceFiles;
    try {
        checkNonRegularVehicleSideFiles(report, vehicleConfigPath, omsiRoot / "Vehicles", ".cti",
                                        "vehicle appearance", "CTI appearance");
        const auto appearances = config::discoverVehicleAppearances(
            vehicleConfigPath, omsiRoot / "Vehicles", ctcTemplates, &appearanceFiles);
        for (const auto& appearance : appearances) {
            for (const auto& [slot, texture] : appearance.textureReplacements) {
                if (!modelSlots.contains(normalizeCtcSlot(slot))) {
                    continue;
                }
                const auto path = assetPath(texture);
                if (!checkResolvedRegularFile(report, "vehicle repaint texture",
                                              appearance.sourcePath, texture, path,
                                              "CTI replacement texture was not found")) {
                    continue;
                }
                recordSourceFile(report, path);
                try {
                    rendering::Image image;
                    if (!rendering::TextureLoader::readImage(path, image)) {
                        report.findings.push_back(
                            {"vehicle repaint texture", appearance.sourcePath, texture,
                             "CTI replacement texture could not be decoded as an image"});
                    }
                } catch (const std::exception& error) {
                    report.findings.push_back(
                        {"vehicle repaint texture", appearance.sourcePath, texture,
                         "Could not decode CTI replacement texture: " + std::string(error.what())});
                }
            }
        }
    } catch (const std::exception& error) {
        report.findings.push_back({"vehicle repaint", vehicleConfigPath, {}, error.what()});
    }
    for (const auto& source : appearanceFiles) {
        recordSourceFile(report, source);
    }
}

void checkVehicle(MapAssetValidationReport& report, const VehicleReference& item,
                  const std::filesystem::path& omsiRoot,
                  std::unordered_set<std::string>& checkedVehicles,
                  std::unordered_set<std::string>& checkedSounds,
                  std::vector<VehicleReference>& pendingVehicles,
                  assets::VehicleTextureResolver& vehicleTextures) {
    const auto path = findAsset({omsiRoot}, item.reference, false, false, &report);
    if (path.empty()) {
        checkReference(report, item.category, item.source, item.reference, {omsiRoot});
        return;
    }
    if (!checkedVehicles.insert(cacheKey(path)).second) {
        return;
    }
    try {
        std::string extension = pathToUtf8(path.extension());
        std::transform(extension.begin(), extension.end(), extension.begin(),
                       [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
        const auto kind = extension == ".bus" ? VehicleFileKind::Bus : VehicleFileKind::Vehicle;
        const auto vehicle = loadVehicleConfig(path, kind, false);
        auto vehicleDiagnostics =
            excludingDiagnostics(vehicle.diagnostics, vehicle.passengerCabinDiagnostics);
        vehicleDiagnostics = excludingDiagnostics(vehicleDiagnostics, vehicle.pathsDiagnostics);
        vehicleDiagnostics = excludingDiagnostics(vehicleDiagnostics, vehicle.numberDiagnostics);
        vehicleDiagnostics =
            excludingDiagnostics(vehicleDiagnostics, vehicle.registrationListDiagnostics);
        appendScriptOwnerErrors(report, item.category, path, vehicleDiagnostics, vehicle);
        appendUnsupportedKeywordWarnings(report, path, vehicleDiagnostics,
                                         "unknown vehicle keyword", "vehicle config");
        if (vehicle.rearCouplingReference) {
            pendingVehicles.push_back(
                {path,
                 pathToUtf8(
                     std::filesystem::absolute(vehicle.rearCouplingReference->vehicleConfigPath)
                         .lexically_normal()),
                 "coupled vehicle"});
        }
        for (const auto& depotPath :
             config::discoverVehicleDepotSourceFiles(path, omsiRoot / "Vehicles")) {
            recordSourceFile(report, depotPath);
        }
        checkScripts(report, vehicle);
        for (const auto& resource : {vehicle.soundConfigPath, vehicle.soundAiConfigPath}) {
            if (!resource.empty()) {
                checkReference(report, "vehicle resource", path, pathToUtf8(resource),
                               {resource.root_path()});
            }
        }
        checkReferencedConfig(report, "passenger cabin", path, vehicle.passengerCabinConfigPath,
                              vehicle.passengerCabinDiagnostics, "passengercabin");
        checkReferencedConfig(report, "passenger paths", path, vehicle.pathsConfigPath,
                              vehicle.pathsDiagnostics, "paths");
        checkReferencedConfig(report, "vehicle resource", path, vehicle.numberConfigPath,
                              vehicle.numberDiagnostics, "registration");
        checkReferencedConfig(report, "vehicle resource", path, vehicle.registrationListConfigPath,
                              vehicle.registrationListDiagnostics, "registration");
        for (const auto& sound : {vehicle.soundConfigPath, vehicle.soundAiConfigPath}) {
            if (!sound.empty() && std::filesystem::is_regular_file(sound) &&
                checkedSounds.insert(cacheKey(sound)).second) {
                checkSoundConfig(report, sound);
            }
        }
        checkPassengerStepSounds(report, vehicle, omsiRoot);
        if (vehicle.modelPath.empty()) {
            report.findings.push_back({"model", path, {}, "Vehicle has no model reference"});
            return;
        }
        const auto modelPath = assets::resolveReferencedPath(path, pathToUtf8(vehicle.modelPath));
        if (!checkResolvedRegularFile(report, "model", path, pathToUtf8(vehicle.modelPath),
                                      modelPath, "Referenced model configuration was not found")) {
            return;
        }
        recordSourceFile(report, modelPath);
        auto modelRoot = modelPath.parent_path();
        std::string directory = pathToUtf8(modelRoot.filename());
        std::transform(directory.begin(), directory.end(), directory.begin(),
                       [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
        if (directory == "configuration files") {
            modelRoot = modelRoot.parent_path();
        }
        std::vector<ModelCtcTexture> ctcTextures;
        std::vector<ModelCtcTemplate> ctcTemplates;
        checkModel(report, modelPath, modelRoot,
                   kind == VehicleFileKind::Bus ? ModelConfigKind::Bus : ModelConfigKind::Vehicle,
                   modelRoot, omsiRoot, false, {}, &vehicleTextures, &ctcTextures, &ctcTemplates);
        checkVehicleRepaintTextures(report, path, omsiRoot, ctcTextures, ctcTemplates);
    } catch (const std::exception& error) {
        report.findings.push_back({item.category, path, {}, error.what()});
    }
}

void checkTrainConsist(MapAssetValidationReport& report, const TrainReference& item,
                       const std::filesystem::path& omsiRoot,
                       std::unordered_set<std::string>& checkedTrains,
                       std::vector<VehicleReference>& pendingVehicles) {
    const std::vector<std::filesystem::path> roots = {omsiRoot / "Trains", omsiRoot};
    const auto path = findAsset(roots, item.reference, false, false, &report);
    if (path.empty()) {
        checkReference(report, item.category, item.source, item.reference, roots);
        return;
    }
    if (!checkedTrains.insert(cacheKey(path)).second) {
        return;
    }
    try {
        const auto consist = loadMapTrainConsistDefinition(path);
        appendUnsupportedSections(report, path, consist.unsupportedSections, "AI train consist");
        for (const auto& vehicle : consist.vehicleAssetPaths) {
            pendingVehicles.push_back({path, vehicle, item.category + " vehicle"});
        }
    } catch (const std::exception& error) {
        report.findings.push_back({item.category, path, {}, error.what()});
    }
}

void checkSpline(MapAssetValidationReport& report, const std::filesystem::path& source,
                 const std::string& reference, const std::filesystem::path& mapRoot,
                 const std::filesystem::path& omsiRoot,
                 std::unordered_set<std::string>& checkedProfiles) {
    const std::vector<std::filesystem::path> roots = {omsiRoot, mapRoot};
    const auto profilePath = findAsset(roots, reference, false, false, &report);
    if (profilePath.empty()) {
        checkReference(report, "spline", source, reference, roots);
        return;
    }
    if (!checkedProfiles.insert(cacheKey(profilePath)).second) {
        return;
    }
    try {
        const auto profile = loadMapSplineProfile(profilePath);
        for (const auto& directive : profile.unsupportedDirectives) {
            report.findings.push_back(
                {"unsupported section",
                 profilePath,
                 directive.keyword,
                 "OpenBus does not implement this spline-profile directive (line " +
                     std::to_string(directive.line) + ")",
                 {},
                 true});
        }
        const auto profileRoot = profilePath.parent_path();
        for (const auto& texture : profile.textures) {
            const auto relative = assetPath(texture);
            const std::vector<std::filesystem::path> textureRoots = {
                profileRoot / "texture", profileRoot, omsiRoot / "Texture", omsiRoot};
            const bool hasSubdirectoryTexture =
                relative.has_parent_path() &&
                !findAsset({profileRoot / relative.parent_path() / "texture"},
                           pathToUtf8(relative.filename()), false, false, &report)
                     .empty();
            if (!hasSubdirectoryTexture) {
                checkReference(report, "spline texture", profilePath, texture, textureRoots, true);
            } else {
                checkReference(report, "spline texture", profilePath,
                               pathToUtf8(relative.filename()),
                               {profileRoot / relative.parent_path() / "texture"}, true);
            }
        }
    } catch (const std::exception& error) {
        report.findings.push_back({"spline", profilePath, {}, error.what()});
    }
}

struct CarparkPlacementCandidate {
    std::filesystem::path source;
    std::string assetPath;
    std::vector<std::string> labels;
};

template <typename Placement>
bool checkPlacement(MapAssetValidationReport& report, const std::filesystem::path& source,
                    const Placement& object, const std::filesystem::path& mapRoot,
                    const std::filesystem::path& omsiRoot,
                    std::unordered_map<std::string, bool>& checkedObjects,
                    std::unordered_set<std::string>& checkedCarparkLists,
                    const std::filesystem::path& listRoot = {}, bool fallbackToMapRoot = true,
                    std::vector<CarparkPlacementCandidate>* carparkCandidates = nullptr) {
    if (checkScenery(report, source, object.assetPath, mapRoot, omsiRoot, checkedObjects)) {
        if (carparkCandidates) {
            carparkCandidates->push_back({source, object.assetPath, object.labels});
        }
        const auto& effectiveListRoot = listRoot.empty() ? source.parent_path() : listRoot;
        return checkCarparkList(report, source, object.labels, mapRoot, omsiRoot, checkedObjects,
                                checkedCarparkLists, effectiveListRoot, fallbackToMapRoot);
    }
    return false;
}

void collectMapPlacementIds(const MapTileData& tile, std::unordered_set<int>& placementIds) {
    const auto collectObjects = [&placementIds](const auto& objects) {
        for (const auto& object : objects) {
            if (object.id != 0) {
                placementIds.insert(object.id);
            }
        }
    };
    collectObjects(tile.sceneryObjects);
    collectObjects(tile.attachedObjects);
    collectObjects(tile.splineAttachments);
    for (const auto& spline : tile.splines) {
        if (spline.splineId != 0) {
            placementIds.insert(spline.splineId);
        }
    }
}

void checkUnsupportedTileSidecar(MapAssetValidationReport& report,
                                 const std::filesystem::path& tilePath, const std::string& suffix,
                                 const std::string& formatName) {
    auto sidecarPath = tilePath;
    sidecarPath += suffix;
    try {
        if (!std::filesystem::exists(sidecarPath)) {
            return;
        }
        if (!std::filesystem::is_regular_file(sidecarPath)) {
            report.findings.push_back({"tile sidecar", tilePath, pathToUtf8(sidecarPath),
                                       "Tile sidecar is not a regular file"});
            return;
        }
        recordSourceFile(report, sidecarPath);
        report.findings.push_back(
            {"unsupported tile sidecar",
             sidecarPath,
             pathToUtf8(sidecarPath),
             formatName + " is inventoried but its payload is not interpreted",
             {},
             true});
    } catch (const std::exception& error) {
        report.findings.push_back(
            {"tile sidecar", tilePath, pathToUtf8(sidecarPath), error.what()});
    }
}

void checkTileLightmap(MapAssetValidationReport& report, const std::filesystem::path& tilePath,
                       const std::filesystem::path& lightmapPath) {
    std::error_code statusError;
    const bool exists = std::filesystem::exists(lightmapPath, statusError);
    if (statusError) {
        report.findings.push_back({"lightmap", tilePath, pathToUtf8(lightmapPath),
                                   "Could not inspect tile lightmap: " + statusError.message()});
        return;
    }
    if (!exists) {
        return;
    }
    if (!std::filesystem::is_regular_file(lightmapPath, statusError)) {
        report.findings.push_back({"lightmap", tilePath, pathToUtf8(lightmapPath),
                                   statusError
                                       ? "Could not inspect tile lightmap: " + statusError.message()
                                       : "Tile lightmap is not a regular file"});
        return;
    }

    recordSourceFile(report, lightmapPath);
    try {
        rendering::Image lightmap;
        if (!rendering::TextureLoader::readImage(lightmapPath, lightmap)) {
            report.findings.push_back({"lightmap", tilePath, pathToUtf8(lightmapPath),
                                       "Tile lightmap could not be decoded as an image"});
        }
    } catch (const std::exception& error) {
        report.findings.push_back({"lightmap", tilePath, pathToUtf8(lightmapPath), error.what()});
    }
}

void checkMissingTileSidecar(MapAssetValidationReport& report,
                             const std::filesystem::path& tilePath,
                             const std::filesystem::path& sidecarPath,
                             const std::string& category) {
    std::error_code statusError;
    const bool exists = std::filesystem::exists(sidecarPath, statusError);
    if (statusError) {
        report.findings.push_back({category, tilePath, pathToUtf8(sidecarPath),
                                   "Could not inspect tile sidecar: " + statusError.message()});
        return;
    }
    if (!exists) {
        report.findings.push_back(
            {category, tilePath, pathToUtf8(sidecarPath),
             category == "terrain" ? "Terrain file was not found" : "Water file was not found"});
        return;
    }

    const bool regularFile = std::filesystem::is_regular_file(sidecarPath, statusError);
    if (statusError) {
        report.findings.push_back({category, tilePath, pathToUtf8(sidecarPath),
                                   "Could not inspect tile sidecar: " + statusError.message()});
    } else if (!regularFile) {
        report.findings.push_back({category, tilePath, pathToUtf8(sidecarPath),
                                   "Sidecar path exists but is not a regular file"});
    } else {
        report.findings.push_back({category, tilePath, pathToUtf8(sidecarPath),
                                   "Sidecar file was not available during map discovery"});
    }
}

void checkOptionalMapWaterTexture(MapAssetValidationReport& report,
                                  const std::filesystem::path& mapRoot,
                                  const std::string& filename) {
    const auto requestedPath = mapRoot / "texture" / filename;
    std::filesystem::path resolvedPath;
    try {
        resolvedPath = assets::resolveCaseInsensitivePath(requestedPath, true);
    } catch (const std::filesystem::filesystem_error& error) {
        report.findings.push_back(
            {"water texture", mapRoot / "global.cfg", filename,
             "Could not inspect map water texture: " + std::string(error.what())});
        return;
    }

    std::error_code statusError;
    const bool exists = std::filesystem::exists(resolvedPath, statusError);
    if (statusError) {
        report.findings.push_back(
            {"water texture", mapRoot / "global.cfg", filename,
             "Could not inspect map water texture: " + statusError.message()});
        return;
    }
    if (!exists) {
        return;
    }
    if (!std::filesystem::is_regular_file(resolvedPath, statusError)) {
        report.findings.push_back(
            {"water texture", mapRoot / "global.cfg", filename,
             statusError ? "Could not inspect map water texture: " + statusError.message()
                         : "Map water texture is not a regular file"});
        return;
    }

    recordSourceFile(report, resolvedPath);
    try {
        rendering::Image image;
        if (!rendering::TextureLoader::readImage(resolvedPath, image)) {
            report.findings.push_back({"water texture", mapRoot / "global.cfg", filename,
                                       "Map water texture could not be decoded as an image"});
        }
    } catch (const std::exception& error) {
        report.findings.push_back(
            {"water texture", mapRoot / "global.cfg", filename, error.what()});
    }
}

void checkMapWaterTextures(MapAssetValidationReport& report, const std::filesystem::path& mapRoot) {
    for (const auto* filename : {"water.tga", "water_bump.bmp", "water_envmap.bmp"}) {
        checkOptionalMapWaterTexture(report, mapRoot, filename);
    }
}

void checkGroundTextureSidecars(MapAssetValidationReport& report,
                                const std::filesystem::path& tilePath,
                                const std::vector<MapGroundTextureSidecar>& sidecars,
                                std::size_t groundTextureCount) {
    for (const auto& sidecar : sidecars) {
        recordSourceFile(report, sidecar.path);
        if (!resolveMapGroundTextureSidecarIndex(sidecar.ordinalSuffix, groundTextureCount)) {
            report.findings.push_back(
                {"ground texture sidecar", tilePath, pathToUtf8(sidecar.path),
                 "Sidecar suffix does not resolve to a configured non-base ground-texture layer"});
            continue;
        }
        try {
            rendering::Image image;
            if (!rendering::TextureLoader::readImage(sidecar.path, image)) {
                report.findings.push_back(
                    {"ground texture sidecar", tilePath, pathToUtf8(sidecar.path),
                     "Ground-texture sidecar could not be decoded as an image"});
            }
        } catch (const std::exception& error) {
            report.findings.push_back(
                {"ground texture sidecar", tilePath, pathToUtf8(sidecar.path), error.what()});
        }
    }
}

void checkTile(MapAssetValidationReport& report, const MapTileReference& reference,
               const std::filesystem::path& mapRoot, const std::filesystem::path& omsiRoot,
               std::size_t groundTextureCount, std::unordered_set<std::string>& checkedProfiles,
               std::unordered_map<std::string, bool>& checkedObjects,
               std::unordered_set<std::string>& checkedCarparkLists,
               std::vector<CarparkPlacementCandidate>& basePlacements,
               std::unordered_set<int>& mapPlacementIds) {
    recordSourceFile(report, reference.textPath);
    if (reference.hasTerrainFile) {
        recordSourceFile(report, reference.terrainPath);
        try {
            (void)loadTerrainGrid(reference.terrainPath);
        } catch (const std::exception& error) {
            report.findings.push_back(
                {"terrain", reference.textPath, pathToUtf8(reference.terrainPath), error.what()});
        }
    }
    if (reference.hasWaterFile) {
        recordSourceFile(report, reference.waterPath);
        try {
            (void)loadWaterData(reference.waterPath);
        } catch (const std::exception& error) {
            report.findings.push_back(
                {"water", reference.textPath, pathToUtf8(reference.waterPath), error.what()});
        }
    }
    checkTileLightmap(report, reference.textPath, reference.lightmapPath);
    if (reference.hasTerrainReadinessFile) {
        recordSourceFile(report, reference.terrainReadinessPath);
    }
    checkGroundTextureSidecars(report, reference.textPath, reference.groundTextureSidecars,
                               groundTextureCount);
    checkUnsupportedTileSidecar(report, reference.textPath, ".prt",
                                "The .prt sidecar format and runtime role");
    checkUnsupportedTileSidecar(report, reference.textPath, ".terrain_0.rdy",
                                "The terrain-readiness sidecar");
    try {
        const MapTileData tile = loadMapTile(reference);
        collectMapPlacementIds(tile, mapPlacementIds);
        ++report.tilesScanned;
        const auto& source = reference.textPath;
        appendUnsupportedSections(report, source, tile.unsupportedSections, "map tile");
        if (tile.hasTerrainMarker && !reference.hasTerrainFile) {
            checkMissingTileSidecar(report, source, reference.terrainPath, "terrain");
        }
        if (tile.hasWaterMarker && !reference.hasWaterFile) {
            checkMissingTileSidecar(report, source, reference.waterPath, "water");
        }
        for (const auto& diagnostic : tile.diagnostics.entries) {
            if (diagnostic.severity == ConfigurationDiagnostic::Severity::Error) {
                report.findings.push_back({"tile", source, diagnostic.keyword, diagnostic.message});
            }
        }
        for (const auto& object : tile.sceneryObjects) {
            checkPlacement(report, source, object, mapRoot, omsiRoot, checkedObjects,
                           checkedCarparkLists, {}, true, &basePlacements);
        }
        for (const auto& object : tile.attachedObjects) {
            checkPlacement(report, source, object, mapRoot, omsiRoot, checkedObjects,
                           checkedCarparkLists, {}, true, &basePlacements);
        }
        for (const auto& object : tile.splineAttachments) {
            checkPlacement(report, source, object, mapRoot, omsiRoot, checkedObjects,
                           checkedCarparkLists, {}, true, &basePlacements);
        }
        for (const auto& spline : tile.splines) {
            checkSpline(report, source, spline.assetPath, mapRoot, omsiRoot, checkedProfiles);
        }
    } catch (const std::exception& error) {
        report.findings.push_back({"tile", reference.textPath, {}, error.what()});
    }
}

std::string chronoFilenameKey(const std::filesystem::path& path) {
    std::string name = pathToUtf8(path.filename());
    std::transform(name.begin(), name.end(), name.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return name;
}

void checkGroundTextureSidecarIssues(MapAssetValidationReport& report,
                                     const std::filesystem::path& tilePath,
                                     const std::vector<MapGroundTextureSidecarIssue>& issues) {
    const std::string tileFilename = chronoFilenameKey(tilePath);
    for (const auto& issue : issues) {
        if (issue.tileFilename == tileFilename) {
            report.findings.push_back(
                {"ground texture sidecar", tilePath, pathToUtf8(issue.path), issue.detail});
        }
    }
}

void checkOpaqueMapCompanion(MapAssetValidationReport& report, const std::filesystem::path& path) {
    try {
        if (!std::filesystem::exists(path)) {
            return;
        }
        if (!std::filesystem::is_regular_file(path)) {
            report.findings.push_back(
                {"map companion", path, pathToUtf8(path), "Map companion is not a regular file"});
            return;
        }
        recordSourceFile(report, path);
        report.findings.push_back(
            {"unsupported map companion",
             path,
             pathToUtf8(path.filename()),
             "This companion file is inventoried, but its format or runtime role is not "
             "interpreted",
             {},
             true});
    } catch (const std::exception& error) {
        report.findings.push_back({"map companion", path, pathToUtf8(path), error.what()});
    }
}

void checkOpaqueMapCompanions(MapAssetValidationReport& report,
                              const std::filesystem::path& mapRoot) {
    for (const auto* filename : {"timeline.prt.cfg", "laststn.osn", "laststn.osn.owt",
                                 "signalroutes.cfg", "registrations.txt"}) {
        checkOpaqueMapCompanion(report, mapRoot / filename);
    }

    try {
        for (const auto& entry : std::filesystem::directory_iterator(mapRoot)) {
            const std::string filename = chronoFilenameKey(entry.path());
            const std::string extension = chronoFilenameKey(entry.path().extension());
            if (!((filename.starts_with("laststn.osn_") && extension == ".dds") ||
                  (filename.starts_with("holidays") && extension == ".txt"))) {
                continue;
            }
            std::error_code statusError;
            const bool regularFile = entry.is_regular_file(statusError);
            if (statusError) {
                report.findings.push_back(
                    {"map companion", entry.path(), pathToUtf8(entry.path().filename()),
                     "Could not inspect map companion: " + statusError.message()});
            } else if (regularFile) {
                checkOpaqueMapCompanion(report, entry.path());
            } else {
                report.findings.push_back({"map companion", entry.path(),
                                           pathToUtf8(entry.path().filename()),
                                           "Map companion is not a regular file"});
            }
        }
    } catch (const std::exception& error) {
        report.findings.push_back(
            {"map companion",
             mapRoot,
             {},
             "Could not inspect map-root companion files: " + std::string(error.what())});
    }
}

void checkUnclassifiedDirectoryFiles(
    MapAssetValidationReport& report, const std::filesystem::path& directory,
    const std::string& category, const std::string& location,
    const std::function<void(const MapAssetValidationProgress&)>& onProgress = {}) {
    const std::string phase = "Auditing " + location + " files";
    try {
        std::size_t entriesScanned = 0;
        for (const auto& entry : std::filesystem::recursive_directory_iterator(directory)) {
            ++entriesScanned;
            if (onProgress && entriesScanned % 32 == 0) {
                onProgress({phase, entriesScanned, 0});
            }
            std::error_code statusError;
            const bool regularFile = entry.is_regular_file(statusError);
            if (statusError) {
                report.findings.push_back(
                    {"map inventory", entry.path(), pathToUtf8(entry.path().filename()),
                     "Could not inspect map-root entry: " + statusError.message()});
                continue;
            }
            if (!regularFile) {
                std::error_code linkError;
                const auto linkStatus = entry.symlink_status(linkError);
                if (linkError) {
                    report.findings.push_back(
                        {"map inventory", entry.path(), pathToUtf8(entry.path().filename()),
                         "Could not inspect map-root entry: " + linkError.message()});
                } else if (std::filesystem::is_symlink(linkStatus)) {
                    std::error_code targetError;
                    const bool directoryTarget =
                        std::filesystem::is_directory(entry.path(), targetError);
                    report.findings.push_back(
                        {"map inventory", entry.path(), pathToUtf8(entry.path().filename()),
                         targetError
                             ? "Could not inspect map-root symlink target: " + targetError.message()
                         : directoryTarget
                             ? location + " directory symlink is not traversed; its contents were "
                                          "not validated"
                             : "Map-root symlink does not resolve to a regular file"});
                }
                continue;
            }

            const auto key = cacheKey(entry.path());
            const bool alreadyInventoried =
                std::any_of(report.sourceFiles.begin(), report.sourceFiles.end(),
                            [&](const auto& source) { return cacheKey(source) == key; });
            const bool alreadyReported = std::any_of(
                report.findings.begin(), report.findings.end(), [&](const auto& finding) {
                    if (cacheKey(finding.source) == key) {
                        return true;
                    }
                    if (finding.reference.empty()) {
                        return false;
                    }
                    return cacheKey(pathFromUtf8(finding.reference)) == key;
                });
            if (!alreadyInventoried && !alreadyReported) {
                recordSourceFile(report, entry.path());
                report.findings.push_back(
                    {category,
                     entry.path(),
                     pathToUtf8(entry.path().filename()),
                     location + " file is inventoried, but its format or runtime role is not "
                                "interpreted",
                     {},
                     true});
            }
        }
    } catch (const std::filesystem::filesystem_error& error) {
        report.findings.push_back({"map inventory",
                                   directory,
                                   {},
                                   "Could not enumerate " + location + " files: " + error.what()});
    }
}

void checkUnclassifiedMapRootFiles(
    MapAssetValidationReport& report, const std::filesystem::path& mapRoot,
    const std::function<void(const MapAssetValidationProgress&)>& onProgress) {
    checkUnclassifiedDirectoryFiles(report, mapRoot, "unclassified map file", "Map-root",
                                    onProgress);
}

void recordTimetableSources(MapAssetValidationReport& report,
                            const std::filesystem::path& directory) {
    std::error_code statusError;
    const bool exists = std::filesystem::exists(directory, statusError);
    if (statusError) {
        report.findings.push_back(
            {"timetable source",
             directory,
             {},
             "Could not inspect timetable source path: " + statusError.message()});
        return;
    }
    if (!exists) {
        return;
    }

    const bool isDirectory = std::filesystem::is_directory(directory, statusError);
    if (statusError) {
        report.findings.push_back(
            {"timetable source",
             directory,
             {},
             "Could not inspect timetable source path: " + statusError.message()});
        return;
    }
    if (!isDirectory) {
        report.findings.push_back({"timetable source",
                                   directory,
                                   {},
                                   "Timetable source path exists but is not a directory"});
        return;
    }

    std::error_code iterationError;
    std::filesystem::directory_iterator entry(directory, iterationError), end;
    if (iterationError) {
        report.findings.push_back(
            {"timetable source",
             directory,
             {},
             "Could not enumerate timetable source directory: " + iterationError.message()});
        return;
    }
    while (entry != end) {
        const auto path = entry->path();
        std::error_code entryError;
        const bool regularFile = entry->is_regular_file(entryError);
        if (entryError) {
            report.findings.push_back(
                {"timetable source",
                 path,
                 {},
                 "Could not inspect timetable source entry: " + entryError.message()});
        } else {
            std::string extension = pathToUtf8(path.extension());
            std::transform(extension.begin(), extension.end(), extension.begin(),
                           [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
            const std::string filename = chronoFilenameKey(path);
            const bool recognized = extension == ".ttr" || extension == ".ttp" ||
                                    extension == ".ttl" || filename == "busstops.cfg" ||
                                    filename == "stnlinks.cfg";
            if (recognized && !regularFile) {
                report.findings.push_back(
                    {"timetable source", path, pathToUtf8(path.filename()),
                     "Recognized timetable source path is not a regular file"});
            } else if (recognized) {
                recordSourceFile(report, path);
            }
        }

        entry.increment(iterationError);
        if (iterationError) {
            report.findings.push_back(
                {"timetable source",
                 directory,
                 {},
                 "Could not enumerate timetable source directory: " + iterationError.message()});
            return;
        }
    }
}

void checkTimetableTrackReferences(
    MapAssetValidationReport& report, const std::vector<MapAiTimetableRoute>& routes,
    const std::unordered_set<int>& mapPlacementIds,
    std::set<std::pair<std::string, int>>& checkedTimetableTrackReferences) {
    for (const auto& route : routes) {
        if (route.trackFilePath.empty()) {
            continue;
        }
        recordSourceFile(report, route.trackFilePath);
        for (const auto& entry : route.trackEntries) {
            if (mapPlacementIds.contains(entry.splineId) ||
                !checkedTimetableTrackReferences
                     .emplace(cacheKey(route.trackFilePath), entry.splineId)
                     .second) {
                continue;
            }
            report.findings.push_back(
                {"timetable track", route.trackFilePath, std::to_string(entry.splineId),
                 "TTR track entry ID was not found among scanned base or Chrono tile placement "
                 "IDs (line " +
                     std::to_string(entry.sourceLine) + ")"});
        }
    }
}

void checkChronoTile(MapAssetValidationReport& report, const MapTileReference& baseReference,
                     const std::filesystem::path& overlay, const std::filesystem::path& mapRoot,
                     const std::filesystem::path& omsiRoot,
                     const std::vector<MapGroundTextureSidecar>& groundTextureSidecars,
                     std::size_t groundTextureCount,
                     std::unordered_set<std::string>& checkedProfiles,
                     std::unordered_map<std::string, bool>& checkedObjects,
                     std::unordered_set<std::string>& checkedCarparkLists,
                     std::unordered_set<int>& mapPlacementIds) {
    checkUnsupportedTileSidecar(report, overlay, ".prt",
                                "The .prt sidecar format and runtime role");
    checkUnsupportedTileSidecar(report, overlay, ".terrain_0.rdy", "The terrain-readiness sidecar");
    auto lightmapPath = overlay;
    lightmapPath += ".LM.bmp";
    checkTileLightmap(report, overlay, lightmapPath);
    checkGroundTextureSidecars(report, overlay, groundTextureSidecars, groundTextureCount);
    for (const auto& sidecar : {std::pair{".terrain", "terrain"}, std::pair{".water", "water"}}) {
        auto path = overlay;
        path += sidecar.first;
        try {
            if (std::filesystem::exists(path)) {
                if (!std::filesystem::is_regular_file(path)) {
                    report.findings.push_back({sidecar.second, overlay, pathToUtf8(path),
                                               "Chrono sidecar is not a regular file"});
                } else {
                    recordSourceFile(report, path);
                    if (std::string_view(sidecar.first) == ".terrain") {
                        (void)loadTerrainGrid(path);
                    } else {
                        (void)loadWaterData(path);
                    }
                }
            }
        } catch (const std::exception& error) {
            report.findings.push_back({sidecar.second, overlay, pathToUtf8(path), error.what()});
        }
    }

    try {
        recordSourceFile(report, overlay);
        const auto chrono = loadChronoTile(overlay);
        appendErrors(report, "chrono tile", overlay, chrono.diagnostics, true);
        const auto base = loadMapTile(baseReference);
        auto variantReference = baseReference;
        variantReference.chronoBaseAssets =
            MapChronoBaseAssets{baseReference.textPath,        baseReference.terrainPath,
                                baseReference.hasTerrainFile,  baseReference.waterPath,
                                baseReference.hasWaterFile,    baseReference.lightmapPath,
                                baseReference.hasLightmapFile, baseReference.groundTextureSidecars};
        variantReference.textPath = overlay;
        variantReference.chronoOverlayPaths = {overlay};
        const auto variant = loadMapTile(variantReference);
        ++report.chronoTilesScanned;
        collectMapPlacementIds(variant, mapPlacementIds);
        std::vector<MapOpaqueSection> overlayUnsupportedSections;
        for (const auto& section : variant.unsupportedSections) {
            if (section.sourcePath == overlay) {
                overlayUnsupportedSections.push_back(section);
            }
        }
        appendUnsupportedSections(report, overlay, overlayUnsupportedSections, "map tile");
        for (std::size_t index = base.diagnostics.entries.size();
             index < variant.diagnostics.entries.size(); ++index) {
            const auto& diagnostic = variant.diagnostics.entries[index];
            const bool alreadyReported =
                std::any_of(chrono.diagnostics.entries.begin(), chrono.diagnostics.entries.end(),
                            [&](const auto& entry) {
                                return entry.line == diagnostic.line &&
                                       entry.keyword == diagnostic.keyword &&
                                       entry.message == diagnostic.message &&
                                       entry.severity == diagnostic.severity;
                            });
            if (!alreadyReported) {
                auto contextual = diagnostic;
                contextual.message = "Independent scenario check: " + contextual.message;
                appendDiagnostic(report, "chrono tile", overlay, contextual);
            }
        }

        const auto checkScenarioPlacements = [&](const auto& placements) {
            for (const auto& placement : placements) {
                checkPlacement(report, overlay, placement, mapRoot, omsiRoot, checkedObjects,
                               checkedCarparkLists, overlay.parent_path());
            }
        };
        checkScenarioPlacements(variant.sceneryObjects);
        checkScenarioPlacements(variant.attachedObjects);
        checkScenarioPlacements(variant.splineAttachments);
        for (const auto& spline : variant.splines) {
            const auto existing =
                std::find_if(base.splines.begin(), base.splines.end(), [&](const auto& candidate) {
                    return candidate.splineId == spline.splineId;
                });
            if (existing == base.splines.end() || existing->assetPath != spline.assetPath) {
                checkSpline(report, overlay, spline.assetPath, mapRoot, omsiRoot, checkedProfiles);
            }
        }
    } catch (const std::exception& error) {
        report.findings.push_back({"chrono tile", overlay, {}, error.what()});
    }
}

using ChronoDiagnosticKey = std::tuple<std::filesystem::path, std::size_t, std::string, std::string,
                                       ConfigurationDiagnostic::Severity>;

void checkChronoCombination(MapAssetValidationReport& report, const MapTileReference& baseReference,
                            const MapTileReference& activeReference, int date,
                            const std::filesystem::path& mapRoot,
                            const std::filesystem::path& omsiRoot,
                            std::unordered_set<std::string>& checkedProfiles,
                            std::unordered_map<std::string, bool>& checkedObjects,
                            std::unordered_set<std::string>& checkedCarparkLists,
                            std::set<ChronoDiagnosticKey>& checkedDiagnostics) {
    std::filesystem::path source = baseReference.textPath;
    try {
        const auto base = loadMapTile(baseReference);
        auto previous = base;
        struct PlacementOrigins {
            std::unordered_map<int, std::filesystem::path> assets;
            std::unordered_map<int, std::filesystem::path> labels;
        };
        PlacementOrigins objects;
        PlacementOrigins attached;
        PlacementOrigins attachments;
        std::unordered_map<int, std::filesystem::path> splines;
        auto reference = activeReference;
        reference.chronoOverlayPaths.clear();
        const auto rememberPlacements = [&](const auto& current, const auto& original,
                                            auto& origins) {
            for (const auto& placement : current) {
                const auto existing =
                    std::find_if(original.begin(), original.end(), [&](const auto& candidate) {
                        return candidate.id == placement.id;
                    });
                if (existing == original.end() || existing->assetPath != placement.assetPath) {
                    origins.assets[placement.id] = source;
                }
                if (existing == original.end() || existing->labels != placement.labels) {
                    origins.labels[placement.id] = source;
                }
            }
        };
        for (const auto& overlay : activeReference.chronoOverlayPaths) {
            source = overlay;
            reference.textPath = overlay;
            reference.chronoOverlayPaths.push_back(overlay);
            auto current = loadMapTile(reference);
            if (reference.chronoOverlayPaths.size() > 1) {
                for (std::size_t index = previous.diagnostics.entries.size();
                     index < current.diagnostics.entries.size(); ++index) {
                    auto diagnostic = current.diagnostics.entries[index];
                    if (!checkedDiagnostics
                             .emplace(overlay, diagnostic.line, diagnostic.keyword,
                                      diagnostic.message, diagnostic.severity)
                             .second) {
                        continue;
                    }
                    diagnostic.message = "Active Chrono combination on " + std::to_string(date) +
                                         ": " + diagnostic.message;
                    appendDiagnostic(report, "chrono tile", overlay, diagnostic);
                }
            }
            rememberPlacements(current.sceneryObjects, previous.sceneryObjects, objects);
            rememberPlacements(current.attachedObjects, previous.attachedObjects, attached);
            rememberPlacements(current.splineAttachments, previous.splineAttachments, attachments);
            for (const auto& spline : current.splines) {
                const auto existing = std::find_if(
                    previous.splines.begin(), previous.splines.end(),
                    [&](const auto& candidate) { return candidate.splineId == spline.splineId; });
                if (existing == previous.splines.end() || existing->assetPath != spline.assetPath) {
                    splines[spline.splineId] = source;
                }
            }
            previous = std::move(current);
        }
        ++report.chronoCombinedTilesScanned;

        const auto checkFinalPlacements = [&](const auto& current, const auto& original,
                                              const auto& origins) {
            for (const auto& placement : current) {
                const auto existing =
                    std::find_if(original.begin(), original.end(), [&](const auto& candidate) {
                        return candidate.id == placement.id;
                    });
                const bool changedAsset =
                    existing == original.end() || existing->assetPath != placement.assetPath;
                const bool changedLabels =
                    existing == original.end() || existing->labels != placement.labels;
                if (changedAsset || changedLabels) {
                    const auto& assetSource =
                        changedAsset ? origins.assets.at(placement.id) : baseReference.textPath;
                    if (checkScenery(report, assetSource, placement.assetPath, mapRoot, omsiRoot,
                                     checkedObjects)) {
                        const auto& labelSource =
                            changedLabels ? origins.labels.at(placement.id) : assetSource;
                        checkCarparkList(report, labelSource, placement.labels, mapRoot, omsiRoot,
                                         checkedObjects, checkedCarparkLists,
                                         labelSource.parent_path());
                    }
                }
            }
        };
        checkFinalPlacements(previous.sceneryObjects, base.sceneryObjects, objects);
        checkFinalPlacements(previous.attachedObjects, base.attachedObjects, attached);
        checkFinalPlacements(previous.splineAttachments, base.splineAttachments, attachments);
        for (const auto& spline : previous.splines) {
            const auto existing =
                std::find_if(base.splines.begin(), base.splines.end(), [&](const auto& candidate) {
                    return candidate.splineId == spline.splineId;
                });
            if (existing == base.splines.end() || existing->assetPath != spline.assetPath) {
                checkSpline(report, splines.at(spline.splineId), spline.assetPath, mapRoot,
                            omsiRoot, checkedProfiles);
            }
        }

        std::vector<std::filesystem::path> scenarioRoots;
        for (const auto& overlay : activeReference.chronoOverlayPaths) {
            const auto root = overlay.parent_path();
            if (std::find(scenarioRoots.begin(), scenarioRoots.end(), root) ==
                scenarioRoots.end()) {
                scenarioRoots.push_back(root);
            }
        }
        const auto checkCombinedCarparkLists = [&](const auto& placements) {
            for (const auto& placement : placements) {
                if (!carparkListIndex(placement.labels)) {
                    checkPlacement(report, source, placement, mapRoot, omsiRoot, checkedObjects,
                                   checkedCarparkLists, mapRoot);
                    continue;
                }
                bool scenarioListFound = false;
                for (const auto& root : scenarioRoots) {
                    scenarioListFound =
                        checkPlacement(report, source, placement, mapRoot, omsiRoot, checkedObjects,
                                       checkedCarparkLists, root, false) ||
                        scenarioListFound;
                }
                if (!scenarioListFound) {
                    checkPlacement(report, source, placement, mapRoot, omsiRoot, checkedObjects,
                                   checkedCarparkLists, mapRoot);
                }
            }
        };
        checkCombinedCarparkLists(previous.sceneryObjects);
        checkCombinedCarparkLists(previous.attachedObjects);
        checkCombinedCarparkLists(previous.splineAttachments);
    } catch (const std::exception& error) {
        report.findings.push_back(
            {"chrono tile",
             source,
             {},
             "Combination on " + std::to_string(date) + " could not be checked: " + error.what()});
    }
}

void checkChronoTiles(MapAssetValidationReport& report, const MapDefinition& map,
                      const std::filesystem::path& omsiRoot,
                      std::unordered_set<std::string>& checkedProfiles,
                      std::unordered_map<std::string, bool>& checkedObjects,
                      std::unordered_set<std::string>& checkedCarparkLists,
                      std::vector<VehicleReference>& chronoVehicles,
                      std::vector<AiTrafficReference>& chronoAiTraffic,
                      std::vector<TrainReference>& chronoTrains,
                      std::vector<HumanReference>& chronoHumanModels,
                      const std::vector<CarparkPlacementCandidate>& basePlacements,
                      std::unordered_set<int>& mapPlacementIds,
                      std::set<std::pair<std::string, int>>& checkedTimetableTrackReferences,
                      const std::function<void(const MapAssetValidationProgress&)>& onProgress) {
    const auto chronoRoot = map.rootPath / "Chrono";
    std::map<int, std::ptrdiff_t> activityChanges;
    bool discoveryComplete = true;
    try {
        if (!std::filesystem::exists(chronoRoot)) {
            return;
        }
        if (!std::filesystem::is_directory(chronoRoot)) {
            report.findings.push_back({"chrono", chronoRoot, {}, "Chrono path is not a directory"});
            report.findings.push_back({"chrono",
                                       chronoRoot,
                                       {},
                                       "Date-selected combinations were not checked because "
                                       "scenario discovery or configuration loading failed"});
            return;
        }
        std::unordered_map<std::string, const MapTileReference*> tiles;
        for (const auto& tile : map.tiles) {
            tiles.emplace(chronoFilenameKey(tile.textPath), &tile);
        }
        for (const auto& entry : std::filesystem::recursive_directory_iterator(chronoRoot)) {
            if (chronoFilenameKey(entry.path()) != "chrono.cfg") {
                continue;
            }
            std::error_code statusError;
            if (!entry.is_regular_file(statusError)) {
                report.findings.push_back(
                    {"chrono",
                     entry.path(),
                     {},
                     statusError ? "Could not inspect Chrono scenario configuration: " +
                                       statusError.message()
                                 : "Chrono scenario configuration is not a regular file"});
                discoveryComplete = false;
                continue;
            }
            recordSourceFile(report, entry.path());
            std::ifstream config(entry.path(), std::ios::binary);
            if (!config) {
                report.findings.push_back(
                    {"chrono", entry.path(), {}, "Could not open Chrono scenario configuration"});
                discoveryComplete = false;
            } else {
                config.ignore((std::numeric_limits<std::streamsize>::max)());
                if (config.bad()) {
                    report.findings.push_back({"chrono",
                                               entry.path(),
                                               {},
                                               "Could not read Chrono scenario configuration"});
                    discoveryComplete = false;
                }
            }
            try {
                ConfigurationDiagnostics diagnostics;
                const auto chronoConfig = loadMapChronoConfigData(entry.path(), diagnostics);
                const auto dates = loadMapChronoTransitionDates(entry.path(), diagnostics);
                appendErrors(report, "chrono", entry.path(), diagnostics, true);
                appendUnsupportedSections(report, entry.path(), chronoConfig.unsupportedSections,
                                          "Chrono.cfg");
                for (const auto& reference : chronoConfig.assetReferences) {
                    if (reference.keyword == "ticketpack") {
                        checkMapSupportConfig(report, entry.path(), omsiRoot, reference.path,
                                              "Chrono ticket pack", loadMapTicketPackDefinition);
                    } else if (reference.keyword == "moneysystem") {
                        checkCurrencyConfig(report, entry.path(), omsiRoot, reference.path);
                    }
                }
                if (!dates.empty()) {
                    ++activityChanges[dates.front()];
                    if (dates.size() > 1) {
                        --activityChanges[dates.back()];
                    }
                }
            } catch (const std::exception& error) {
                report.findings.push_back({"chrono", entry.path(), {}, error.what()});
                discoveryComplete = false;
            }
            const auto scenario = entry.path().parent_path();
            std::error_code sidecarDiscoveryError;
            std::vector<MapGroundTextureSidecarIssue> groundTextureSidecarIssues;
            const auto scenarioGroundTextureSidecars = discoverMapGroundTextureSidecars(
                scenario, sidecarDiscoveryError, &groundTextureSidecarIssues);
            if (sidecarDiscoveryError) {
                report.findings.push_back({"ground texture sidecar",
                                           scenario,
                                           {},
                                           "Could not enumerate Chrono ground-texture sidecars: " +
                                               sidecarDiscoveryError.message()});
                discoveryComplete = false;
            }
            const std::vector<MapGroundTextureSidecar> noGroundTextureSidecars;
            recordTimetableSources(report, scenario / "TTData");
            for (const auto& placement : basePlacements) {
                checkPlacement(report, placement.source, placement, map.rootPath, omsiRoot,
                               checkedObjects, checkedCarparkLists, scenario, false);
            }
            try {
                for (const auto& candidate : std::filesystem::directory_iterator(scenario)) {
                    const std::string candidateName = chronoFilenameKey(candidate.path());
                    std::error_code statusError;
                    const bool regularFile = candidate.is_regular_file(statusError);
                    if (statusError) {
                        report.findings.push_back(
                            {"chrono", candidate.path(), candidateName,
                             "Could not inspect Chrono scenario entry: " + statusError.message()});
                        discoveryComplete = false;
                        continue;
                    }
                    if (!regularFile) {
                        if (chronoFilenameKey(candidate.path().extension()) == ".map" &&
                            tiles.contains(candidateName)) {
                            report.findings.push_back(
                                {"chrono tile", candidate.path(),
                                 pathToUtf8(candidate.path().filename()),
                                 "Chrono tile path exists but is not a regular file"});
                            discoveryComplete = false;
                        } else if (candidateName == "humans.txt" ||
                                   candidateName == "drivers.txt" ||
                                   candidateName == "ailists_#upd.cfg" ||
                                   candidateName == "chrono_#upd.cfg" ||
                                   candidateName.ends_with("_#upd.cfg")) {
                            const std::string category =
                                candidateName == "humans.txt"         ? "human list"
                                : candidateName == "drivers.txt"      ? "driver list"
                                : candidateName == "ailists_#upd.cfg" ? "chrono traffic"
                                : candidateName == "chrono_#upd.cfg"  ? "Chrono traffic update"
                                                                      : "Chrono update";
                            report.findings.push_back(
                                {category, candidate.path(),
                                 pathToUtf8(candidate.path().filename()),
                                 "Recognized Chrono dependency path is not a regular file"});
                            discoveryComplete = false;
                        }
                        continue;
                    }
                    if (candidateName == "humans.txt" || candidateName == "drivers.txt") {
                        recordSourceFile(report, candidate.path());
                        const bool isDriverList = candidateName == "drivers.txt";
                        try {
                            const auto list = loadMapHumanModelList(candidate.path());
                            appendUnsupportedHumanListEntries(report, candidate.path(), list);
                            for (const auto& path : list.assetPaths) {
                                chronoHumanModels.push_back(
                                    {candidate.path(), path, isDriverList ? "driver" : "human"});
                            }
                        } catch (const std::exception& error) {
                            report.findings.push_back({isDriverList ? "driver list" : "human list",
                                                       candidate.path(),
                                                       {},
                                                       error.what()});
                        }
                        continue;
                    }
                    if (chronoFilenameKey(candidate.path()) == "ailists_#upd.cfg") {
                        recordSourceFile(report, candidate.path());
                        try {
                            MapAiTrafficData traffic = loadMapChronoAiTrafficData(candidate.path());
                            appendUnsupportedSections(report, candidate.path(),
                                                      traffic.unsupportedSections,
                                                      "Chrono AI list");
                            chronoAiTraffic.push_back({candidate.path(), std::move(traffic)});
                        } catch (const std::exception& error) {
                            report.findings.push_back(
                                {"chrono traffic", candidate.path(), {}, error.what()});
                        }
                        try {
                            for (const auto& path :
                                 loadMapChronoAiVehicleAssetPaths(candidate.path())) {
                                chronoVehicles.push_back(
                                    {candidate.path(), path, "Chrono AI vehicle"});
                            }
                        } catch (const std::exception& error) {
                            report.findings.push_back(
                                {"chrono traffic", candidate.path(), {}, error.what()});
                        }
                        try {
                            for (const auto& path :
                                 loadMapChronoAiTrainAssetPaths(candidate.path())) {
                                chronoTrains.push_back({candidate.path(), path, "Chrono AI train"});
                            }
                        } catch (const std::exception& error) {
                            report.findings.push_back(
                                {"chrono traffic", candidate.path(), {}, error.what()});
                        }
                        continue;
                    }
                    if (chronoFilenameKey(candidate.path()) == "chrono_#upd.cfg") {
                        recordSourceFile(report, candidate.path());
                        report.findings.push_back(
                            {"Chrono traffic update",
                             candidate.path(),
                             {},
                             "OpenBus does not interpret this update format; references in the "
                             "file are not checked",
                             {},
                             true});
                        continue;
                    }
                    if (chronoFilenameKey(candidate.path()).ends_with("_#upd.cfg")) {
                        recordSourceFile(report, candidate.path());
                        const std::string updateName = chronoFilenameKey(candidate.path());
                        const bool isDriverUpdate = updateName == "drivers_#upd.cfg";
                        if (isDriverUpdate || updateName == "humans_#upd.cfg") {
                            try {
                                const auto paths =
                                    isDriverUpdate ? loadMapDriverModelAssetPaths(candidate.path())
                                                   : loadMapHumanModelAssetPaths(candidate.path());
                                for (const auto& path : paths) {
                                    chronoHumanModels.push_back(
                                        {candidate.path(), path,
                                         isDriverUpdate ? "driver" : "human"});
                                }
                            } catch (const std::exception& error) {
                                report.findings.push_back(
                                    {isDriverUpdate ? "driver update" : "human update",
                                     candidate.path(),
                                     {},
                                     error.what()});
                            }
                            report.findings.push_back(
                                {"Chrono update",
                                 candidate.path(),
                                 {},
                                 "Human and driver references are checked, but full update "
                                 "semantics are not validated",
                                 {},
                                 true});
                            continue;
                        }
                        report.findings.push_back(
                            {"Chrono update",
                             candidate.path(),
                             {},
                             "OpenBus does not interpret this update format; references in the "
                             "file are not checked",
                             {},
                             true});
                        continue;
                    }
                    const std::string extension = chronoFilenameKey(candidate.path().extension());
                    if (extension == ".ttr" || extension == ".ttp" || extension == ".ttl" ||
                        chronoFilenameKey(candidate.path()) == "busstops.cfg" ||
                        chronoFilenameKey(candidate.path()) == "stnlinks.cfg") {
                        recordSourceFile(report, candidate.path());
                    }
                    if (extension != ".map") {
                        continue;
                    }
                    const auto tile = tiles.find(chronoFilenameKey(candidate.path()));
                    if (tile != tiles.end()) {
                        checkGroundTextureSidecarIssues(report, candidate.path(),
                                                        groundTextureSidecarIssues);
                        const auto sidecars =
                            scenarioGroundTextureSidecars.find(chronoFilenameKey(candidate.path()));
                        checkChronoTile(report, *tile->second, candidate.path(), map.rootPath,
                                        omsiRoot,
                                        sidecars == scenarioGroundTextureSidecars.end()
                                            ? noGroundTextureSidecars
                                            : sidecars->second,
                                        map.groundTextures.size(), checkedProfiles, checkedObjects,
                                        checkedCarparkLists, mapPlacementIds);
                    }
                }
                if (onProgress) {
                    onProgress({"Auditing Chrono scenario files", 0, 0});
                }
                checkUnclassifiedDirectoryFiles(report, scenario, "unclassified Chrono file",
                                                "Chrono scenario", onProgress);
            } catch (const std::filesystem::filesystem_error& error) {
                report.findings.push_back({"chrono", scenario, {}, error.what()});
                discoveryComplete = false;
            }
        }
    } catch (const std::filesystem::filesystem_error& error) {
        report.findings.push_back({"chrono", chronoRoot, {}, error.what()});
        discoveryComplete = false;
    }
    if (!discoveryComplete) {
        report.findings.push_back({"chrono",
                                   chronoRoot,
                                   {},
                                   "Date-selected combinations were not checked because "
                                   "scenario discovery or configuration loading failed"});
        return;
    }

    std::set<std::vector<std::filesystem::path>> checkedCombinations;
    std::set<ChronoDiagnosticKey> checkedDiagnostics;
    std::set<std::string> checkedTimetableDiagnostics;
    std::ptrdiff_t activeCount = 0;
    for (const auto& [date, change] : activityChanges) {
        activeCount += change;
        if (activeCount != 0) {
            auto timetableMap = map;
            try {
                applyMapChronoTimetables(timetableMap, date);
                ++report.chronoTimetableDatesScanned;
                checkTimetableTrackReferences(report, timetableMap.aiTraffic.timetableRoutes,
                                              mapPlacementIds, checkedTimetableTrackReferences);
                for (std::size_t index = map.diagnostics.entries.size();
                     index < timetableMap.diagnostics.entries.size(); ++index) {
                    const auto& diagnostic = timetableMap.diagnostics.entries[index];
                    const std::string detail =
                        (diagnostic.severity == ConfigurationDiagnostic::Severity::Warning
                             ? "Parser warning: "
                             : "") +
                        diagnostic.message +
                        (diagnostic.line == 0 ? std::string{}
                                              : " (line " + std::to_string(diagnostic.line) + ")");
                    if (checkedTimetableDiagnostics.insert(detail).second) {
                        if (diagnostic.message.starts_with("unsupported timetable section in ")) {
                            report.findings.push_back(
                                {"unsupported section",
                                 chronoRoot,
                                 diagnostic.keyword,
                                 "Date " + std::to_string(date) + ": " + detail,
                                 {},
                                 true});
                        } else {
                            report.findings.push_back(
                                {"chrono timetable", chronoRoot, diagnostic.keyword,
                                 "Date " + std::to_string(date) + ": " + detail});
                        }
                    }
                }
            } catch (const std::exception& error) {
                report.findings.push_back({"chrono timetable",
                                           chronoRoot,
                                           {},
                                           "Date-selected timetable state on " +
                                               std::to_string(date) +
                                               " could not be checked: " + error.what()});
            }
        }
        if (activeCount < 2) {
            continue;
        }
        auto activeMap = map;
        try {
            applyMapChronoTiles(activeMap, date);
        } catch (const std::exception& error) {
            report.findings.push_back({"chrono",
                                       chronoRoot,
                                       {},
                                       "Date-selected combinations could not be checked on " +
                                           std::to_string(date) + ": " + error.what()});
            continue;
        }
        for (std::size_t index = 0; index < activeMap.tiles.size(); ++index) {
            const auto& active = activeMap.tiles[index];
            if (active.chronoOverlayPaths.size() < 2) {
                continue;
            }
            std::vector<std::filesystem::path> key{map.tiles[index].textPath};
            key.insert(key.end(), active.chronoOverlayPaths.begin(),
                       active.chronoOverlayPaths.end());
            if (checkedCombinations.insert(std::move(key)).second) {
                checkChronoCombination(report, map.tiles[index], active, date, map.rootPath,
                                       omsiRoot, checkedProfiles, checkedObjects,
                                       checkedCarparkLists, checkedDiagnostics);
            }
        }
    }
}

} // namespace

MapAssetValidationReport validateMapAssets(const std::filesystem::path& mapDirectory,
                                           const std::filesystem::path& omsiRoot) {
    return validateMapAssets(mapDirectory, omsiRoot, {});
}

MapAssetValidationReport validateMapAssets(const std::filesystem::path& mapDirectory,
                                           const std::filesystem::path& omsiRoot,
                                           const MapAssetValidationOptions& options) {
    const auto notifyProgress = [&](std::string phase, std::size_t completed, std::size_t total) {
        if (options.onProgress) {
            options.onProgress({std::move(phase), completed, total});
        }
    };
    MapAssetValidationReport report;
    const auto finalizeReport = [&] {
        std::set<std::tuple<std::string, std::string, std::string, std::string, std::string>>
            seenFindings;
        std::erase_if(report.findings, [&](const MapAssetFinding& finding) {
            return !seenFindings
                        .emplace(finding.category, cacheKey(finding.source), finding.reference,
                                 finding.detail,
                                 finding.context.empty() ? std::string{}
                                                         : cacheKey(finding.context))
                        .second;
        });
        std::sort(
            report.sourceFiles.begin(), report.sourceFiles.end(),
            [](const auto& left, const auto& right) { return cacheKey(left) < cacheKey(right); });
        report.sourceFiles.erase(std::unique(report.sourceFiles.begin(), report.sourceFiles.end(),
                                             [](const auto& left, const auto& right) {
                                                 return cacheKey(left) == cacheKey(right);
                                             }),
                                 report.sourceFiles.end());
    };
    notifyProgress("Checking map inputs", 0, 1);
    std::error_code statusError;
    const bool omsiRootExists = std::filesystem::exists(omsiRoot, statusError);
    if (statusError) {
        report.findings.push_back({"OMSI root", omsiRoot, pathToUtf8(omsiRoot),
                                   "Could not inspect OMSI root: " + statusError.message()});
        return report;
    }
    if (!omsiRootExists) {
        report.findings.push_back(
            {"OMSI root", omsiRoot, pathToUtf8(omsiRoot), "OMSI root was not found"});
        return report;
    }
    const bool omsiRootIsDirectory = std::filesystem::is_directory(omsiRoot, statusError);
    if (statusError) {
        report.findings.push_back({"OMSI root", omsiRoot, pathToUtf8(omsiRoot),
                                   "Could not inspect OMSI root: " + statusError.message()});
        return report;
    }
    if (!omsiRootIsDirectory) {
        report.findings.push_back(
            {"OMSI root", omsiRoot, pathToUtf8(omsiRoot), "OMSI root is not a directory"});
        return report;
    }

    const bool mapDirectoryExists = std::filesystem::is_directory(mapDirectory, statusError);
    if (statusError) {
        report.findings.push_back({"map input", mapDirectory, pathToUtf8(mapDirectory),
                                   "Could not inspect map directory: " + statusError.message()});
        return report;
    }
    if (!mapDirectoryExists) {
        report.findings.push_back(
            {"map input", mapDirectory, pathToUtf8(mapDirectory), "Map path is not a directory"});
        return report;
    }

    const auto globalConfigPath = mapDirectory / "global.cfg";
    const bool globalConfigExists = std::filesystem::exists(globalConfigPath, statusError);
    if (statusError) {
        report.findings.push_back({"map configuration",
                                   globalConfigPath,
                                   {},
                                   "Could not inspect global.cfg: " + statusError.message()});
        return report;
    }
    if (!globalConfigExists) {
        report.findings.push_back(
            {"map configuration", globalConfigPath, {}, "Map global.cfg was not found"});
        return report;
    }
    const bool globalConfigIsRegularFile =
        std::filesystem::is_regular_file(globalConfigPath, statusError);
    if (statusError) {
        report.findings.push_back({"map configuration",
                                   globalConfigPath,
                                   {},
                                   "Could not inspect global.cfg: " + statusError.message()});
        return report;
    }
    if (!globalConfigIsRegularFile) {
        report.findings.push_back({"map configuration",
                                   globalConfigPath,
                                   {},
                                   "global.cfg exists but is not a regular file"});
        return report;
    }
    recordSourceFile(report, globalConfigPath);

    MapDefinition map;
    try {
        map = loadMapDefinition(mapDirectory, true);
    } catch (const std::filesystem::filesystem_error& error) {
        report.findings.push_back(
            {"map configuration",
             globalConfigPath,
             {},
             "Could not load map configuration: " + std::string(error.what())});
        return report;
    } catch (const std::runtime_error& error) {
        report.findings.push_back(
            {"map configuration",
             globalConfigPath,
             {},
             "Could not parse map configuration: " + std::string(error.what())});
        return report;
    }
    notifyProgress("Checking map inputs", 1, 1);
    std::error_code groundTextureSidecarDiscoveryError;
    std::vector<MapGroundTextureSidecarIssue> groundTextureSidecarIssues;
    (void)discoverMapGroundTextureSidecars(map.rootPath, groundTextureSidecarDiscoveryError,
                                           &groundTextureSidecarIssues);
    if (groundTextureSidecarDiscoveryError) {
        report.findings.push_back({"ground texture sidecar",
                                   map.rootPath / "texture" / "map",
                                   {},
                                   "Could not enumerate base ground-texture sidecars: " +
                                       groundTextureSidecarDiscoveryError.message()});
    }
    for (const auto& tile : map.tiles) {
        checkGroundTextureSidecarIssues(report, tile.textPath, groundTextureSidecarIssues);
    }
    recordSourceFile(report, map.rootPath / "global.cfg");
    appendUnsupportedSections(report, map.rootPath / "global.cfg", map.unsupportedSections,
                              "global.cfg");
    if (map.metadata.hasBackgroundImage) {
        report.findings.push_back(
            {"unsupported section",
             map.rootPath / "global.cfg",
             "backgroundimage",
             "Background-image fields are retained but their asset references are not interpreted",
             {},
             true});
    }
    appendUnsupportedSections(report, map.rootPath / "ailists.cfg",
                              map.aiTraffic.unsupportedSections, "AI list");
    appendUnsupportedSections(report, map.rootPath / "environment",
                              map.environment.unsupportedSections, "environment config");
    for (const auto* filename : {"ailists.cfg", "unsched_vehgroups.txt", "unsched_trafficdens.txt",
                                 "drivers.txt", "humans.txt"}) {
        recordExistingSourceFile(report, map.rootPath / filename);
    }
    checkOpaqueMapCompanions(report, map.rootPath);
    for (const auto* filename : {"sky.cfg", "clouds.cfg"}) {
        recordExistingSourceFile(report, map.rootPath / "environment" / filename);
    }
    recordTimetableSources(report, map.rootPath / "TTData");
    constexpr std::string_view missingTile = "listed map tile does not exist: ";
    for (const auto& diagnostic : map.diagnostics.entries) {
        if (diagnostic.keyword == "map" && diagnostic.message.starts_with(missingTile)) {
            const auto tilePath = pathFromUtf8(diagnostic.message.substr(missingTile.size()));
            const auto tileReference = pathToUtf8(tilePath);
            (void)checkResolvedRegularFile(report, "tile", mapDirectory / "global.cfg",
                                           tileReference, tilePath, "Listed tile was not found");
        } else if (diagnostic.message.starts_with("unsupported timetable section in ")) {
            report.findings.push_back(
                {"unsupported section",
                 map.rootPath / "TTData",
                 diagnostic.keyword,
                 diagnostic.message + (diagnostic.line == 0
                                           ? std::string{}
                                           : " (line " + std::to_string(diagnostic.line) + ")"),
                 {},
                 true});
        } else if (diagnostic.severity == ConfigurationDiagnostic::Severity::Error) {
            report.findings.push_back(
                {"map", mapDirectory / "global.cfg", diagnostic.keyword, diagnostic.message});
        }
    }
    std::unordered_set<std::string> checkedProfiles;
    std::unordered_map<std::string, bool> checkedObjects;
    std::unordered_set<std::string> checkedCarparkLists;
    std::vector<VehicleReference> chronoVehicles;
    std::vector<AiTrafficReference> chronoAiTraffic;
    std::vector<TrainReference> chronoTrains;
    std::vector<HumanReference> chronoHumanModels;
    std::vector<TrainReference> mapTrains;
    std::vector<CarparkPlacementCandidate> basePlacements;
    std::unordered_set<int> mapPlacementIds;
    std::set<std::pair<std::string, int>> checkedTimetableTrackReferences;
    for (const auto& train : map.aiTraffic.trainAssetPaths) {
        mapTrains.push_back({mapDirectory / "ailists.cfg", train, "AI train"});
    }
    notifyProgress("Checking base tiles", 0, map.tiles.size());
    for (std::size_t index = 0; index < map.tiles.size(); ++index) {
        const auto& tile = map.tiles[index];
        checkTile(report, tile, mapDirectory, omsiRoot, map.groundTextures.size(), checkedProfiles,
                  checkedObjects, checkedCarparkLists, basePlacements, mapPlacementIds);
        notifyProgress("Checking base tiles", index + 1, map.tiles.size());
    }
    notifyProgress("Checking Chrono scenarios", 0, 1);
    checkChronoTiles(report, map, omsiRoot, checkedProfiles, checkedObjects, checkedCarparkLists,
                     chronoVehicles, chronoAiTraffic, chronoTrains, chronoHumanModels,
                     basePlacements, mapPlacementIds, checkedTimetableTrackReferences,
                     options.onProgress);
    notifyProgress("Checking Chrono scenarios", 1, 1);
    const bool hasWaterSidecar =
        std::any_of(report.sourceFiles.begin(), report.sourceFiles.end(), [](const auto& source) {
            return pathToUtf8(source.filename()).ends_with(".map.water");
        });
    if (hasWaterSidecar) {
        checkMapWaterTextures(report, map.rootPath);
    }
    checkAiDepotHofs(report, map, omsiRoot, chronoAiTraffic);
    checkTimetableTrackReferences(report, map.aiTraffic.timetableRoutes, mapPlacementIds,
                                  checkedTimetableTrackReferences);
    for (const auto& ground : map.groundTextures) {
        checkReference(report, "ground texture", mapDirectory / "global.cfg", ground.texturePath,
                       {mapDirectory, omsiRoot}, true);
        checkReference(report, "ground texture", mapDirectory / "global.cfg",
                       ground.detailTexturePath, {mapDirectory, omsiRoot}, true);
    }
    const std::vector<std::filesystem::path> environmentTextureRoots = {
        mapDirectory / "environment", omsiRoot / "Texture", omsiRoot, mapDirectory};
    for (const auto& texture : map.environment.skyTexturePaths) {
        checkReference(report, "sky texture", mapDirectory / "environment" / "sky.cfg", texture,
                       environmentTextureRoots, true);
    }
    for (const auto& cloud : map.environment.cloudTypes) {
        checkReference(report, "cloud texture", mapDirectory / "environment" / "clouds.cfg",
                       cloud.texturePath, environmentTextureRoots, true);
    }
    checkMapSupportConfig(report, mapDirectory / "global.cfg", omsiRoot, map.metadata.ticketPack,
                          "ticket pack", loadMapTicketPackDefinition);
    checkCurrencyConfig(report, mapDirectory / "global.cfg", omsiRoot, map.metadata.moneySystem);
    std::unordered_set<std::string> checkedVehicles;
    std::unordered_set<std::string> checkedSounds;
    assets::VehicleTextureResolver vehicleTextures(false,
                                                   assets::VehicleTextureAccessPolicy::Strict);
    std::vector<VehicleReference> pendingVehicles;
    for (const auto& group : map.aiTraffic.vehicleGroups) {
        for (const auto& vehicle : group.vehicles) {
            pendingVehicles.push_back(
                {mapDirectory / "ailists.cfg", vehicle.assetPath, "AI vehicle"});
        }
    }
    for (const auto& bus : map.aiTraffic.busTypes) {
        pendingVehicles.push_back({mapDirectory / "ailists.cfg", bus.assetPath, "AI vehicle"});
    }
    pendingVehicles.insert(pendingVehicles.end(), chronoVehicles.begin(), chronoVehicles.end());
    std::unordered_set<std::string> checkedTrains;
    for (const auto& train : mapTrains) {
        checkTrainConsist(report, train, omsiRoot, checkedTrains, pendingVehicles);
    }
    for (const auto& train : chronoTrains) {
        checkTrainConsist(report, train, omsiRoot, checkedTrains, pendingVehicles);
    }
    for (std::size_t index = 0; index < pendingVehicles.size(); ++index) {
        const auto item = pendingVehicles[index];
        checkVehicle(report, item, omsiRoot, checkedVehicles, checkedSounds, pendingVehicles,
                     vehicleTextures);
        notifyProgress("Checking vehicles", index + 1, 0);
    }
    std::unordered_set<std::string> checkedHumans;
    for (const auto* filename : {"humans.txt", "drivers.txt"}) {
        const auto listPath = map.rootPath / filename;
        if (std::filesystem::is_regular_file(listPath)) {
            appendUnsupportedHumanListEntries(report, listPath, loadMapHumanModelList(listPath));
        }
    }
    for (const auto& human : map.aiTraffic.humanModelPaths) {
        checkHuman(report, mapDirectory / "humans.txt", human, omsiRoot, checkedHumans, "human");
    }
    for (const auto& driver : map.aiTraffic.driverModelPaths) {
        checkHuman(report, mapDirectory / "drivers.txt", driver, omsiRoot, checkedHumans, "driver");
    }
    for (const auto& human : chronoHumanModels) {
        checkHuman(report, human.source, human.reference, omsiRoot, checkedHumans, human.category);
    }
    notifyProgress("Auditing Map-root files", 0, 0);
    checkUnclassifiedMapRootFiles(report, map.rootPath, options.onProgress);
    finalizeReport();
    notifyProgress("Complete (dependency coverage remains incomplete)", 1, 1);
    return report;
}

} // namespace openbus::map
