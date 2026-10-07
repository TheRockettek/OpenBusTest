#include "ModelConfigLoader.h"
#include "VehicleConfigLoader.h"
#include "Variables.h"
#include "osc/OscConverter.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <system_error>
#include <unordered_set>
#include <vector>

namespace {

enum class Status { Valid, Invalid, Unsupported };

struct Finding {
    Status status = Status::Valid;
    std::string kind;
    std::vector<ConfigurationDiagnostic> diagnostics;
};

std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

std::string trim(std::string value) {
    const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    });
    value.erase(value.begin(), first);
    const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    }).base();
    value.erase(last, value.end());
    return value;
}

bool hasModelSignature(const std::filesystem::path& path) {
    static const std::unordered_set<std::string> modelKeywords = {
        "absheight", "alphascale", "animparent", "boundingbox", "collision_mesh", "ctc",
        "ctctexture", "fixed", "illumination_interior", "interiorlight", "isshadow",
        "light_enh", "light_enh_2", "lod", "matl", "matl_alpha", "matl_bumpmap",
        "matl_change", "matl_envmap", "matl_freetex", "matl_item", "matl_lightmap",
        "matl_nightmap", "matl_nozcheck", "matl_nozwrite", "matl_texadress_border",
        "matl_texadress_clamp", "matl_texadress_mirror", "matl_texadress_mirroronce",
        "matl_transmap", "mesh", "mesh_ident", "mouseevent", "newanim", "new_attachment",
        "nocollision", "rendertype", "scripttexture", "shadow", "smoke", "spotlight",
        "tcoordtransx", "tcoordtransy", "tex_detail_factor", "texcoordtransx",
        "texcoordtransy", "texttexture", "texttexture_enh", "usescripttexture",
        "usetexttexture", "vfdmaxmin", "viewpoint", "visible"};
    std::ifstream input(path);
    std::string line;
    while (std::getline(input, line)) {
        line = trim(line);
        if (!line.empty() && line.front() == '[') {
            const std::size_t close = line.find(']');
            if (close != std::string::npos &&
                modelKeywords.find(lower(trim(line.substr(1, close - 1)))) != modelKeywords.end()) {
                return true;
            }
        }
    }
    return false;
}

ModelConfigKind modelKindForPath(const std::filesystem::path& path) {
    for (const auto& component : path) {
        const std::string name = lower(component.string());
        if (name == "sceneryobjects" || name == "scenery_objects") {
            return ModelConfigKind::SceneryObject;
        }
    }
    return ModelConfigKind::Bus;
}

std::filesystem::path modelRootFor(const std::filesystem::path& configPath) {
    for (std::filesystem::path directory = configPath.parent_path(); !directory.empty();) {
        if (lower(directory.filename().string()) == "model") {
            return directory;
        }
        const std::filesystem::path parent = directory.parent_path();
        if (parent == directory) {
            break;
        }
        directory = parent;
    }
    return configPath.parent_path();
}

void appendDiagnostics(Finding& finding, const ConfigurationDiagnostics& diagnostics) {
    finding.diagnostics.insert(finding.diagnostics.end(), diagnostics.entries.begin(),
                               diagnostics.entries.end());
    if (diagnostics.hasErrors()) {
        finding.status = Status::Invalid;
    }
}

Finding validateVehicle(const std::filesystem::path& path, const std::filesystem::path& root) {
    Finding finding;
    finding.kind = "bus";
    const VehicleConfig vehicle = loadVehicleConfig(path, VehicleFileKind::Bus, false);
    appendDiagnostics(finding, vehicle.diagnostics);
    if (vehicle.massTonnes <= 0.0) {
        finding.status = Status::Invalid;
        finding.diagnostics.push_back({ConfigurationDiagnostic::Severity::Error, 0, "mass",
                                       "bus configuration has no positive [mass]"});
    }
    if (!vehicle.hasBoundingBox || vehicle.boundingBox[0] <= 0.0 ||
        vehicle.boundingBox[1] <= 0.0 || vehicle.boundingBox[2] <= 0.0) {
        finding.status = Status::Invalid;
        finding.diagnostics.push_back({ConfigurationDiagnostic::Severity::Error, 0, "boundingbox",
                                       "bus configuration has no valid [boundingbox]"});
    }
    if (!vehicle.hasCenterOfGravityHeight || vehicle.centerOfGravityHeight <= 0.0) {
        finding.status = Status::Invalid;
        finding.diagnostics.push_back({ConfigurationDiagnostic::Severity::Error, 0, "schwerpunkt",
                                       "bus configuration has no positive [schwerpunkt]"});
    }
    if (vehicle.axles.empty()) {
        finding.status = Status::Invalid;
        finding.diagnostics.push_back({ConfigurationDiagnostic::Severity::Error, 0, "newachse",
                                       "bus configuration contains no valid [newachse] entries"});
    }

    if (!vehicle.modelPath.empty()) {
        std::filesystem::path modelPath = vehicle.modelPath;
        if (modelPath.is_relative()) {
            const std::filesystem::path busRelative = path.parent_path() / modelPath;
            const std::filesystem::path rootRelative = root / modelPath;
            if (std::filesystem::exists(busRelative)) {
                modelPath = busRelative;
            } else if (std::filesystem::exists(rootRelative)) {
                modelPath = rootRelative;
            } else {
                modelPath = busRelative;
            }
        }
        if (!std::filesystem::exists(modelPath)) {
            finding.status = Status::Invalid;
            finding.diagnostics.push_back({ConfigurationDiagnostic::Severity::Error, 0, "model",
                                           "configured model file was not found: " +
                                               modelPath.string()});
        } else {
            openbus::scripting::Vehicle variables;
            const ModelConfig model = loadModelConfig(modelPath, modelRootFor(modelPath),
                                                       ModelConfigKind::Bus, variables);
            appendDiagnostics(finding, model.diagnostics);
        }
    }
    return finding;
}

Finding validateModel(const std::filesystem::path& path) {
    Finding finding;
    finding.kind = "model-cfg";
    openbus::scripting::Vehicle variables;
    const ModelConfig model =
        loadModelConfig(path, modelRootFor(path), modelKindForPath(path), variables);
    appendDiagnostics(finding, model.diagnostics);
    return finding;
}

Finding validateOsc(const std::filesystem::path& path) {
    Finding finding;
    finding.kind = "osc";
    OscProgram program;
    std::string error;
    if (!compileOscToBytecode(path, program, error)) {
        const std::string message = error.empty() ? "OSC compilation failed" : error;
        const bool unsupported = lower(message).find("unsupported") != std::string::npos;
        finding.status = unsupported ? Status::Unsupported : Status::Invalid;
        std::size_t line = 0;
        const std::string marker = " at line ";
        const std::size_t markerAt = message.rfind(marker);
        if (markerAt != std::string::npos) {
            try {
                line = static_cast<std::size_t>(std::stoul(message.substr(markerAt + marker.size())));
            } catch (...) {
                line = 0;
            }
        }
        finding.diagnostics.push_back({unsupported ? ConfigurationDiagnostic::Severity::Warning
                                                    : ConfigurationDiagnostic::Severity::Error,
                                       line, "osc", message});
    }
    return finding;
}

Finding validate(const std::filesystem::path& path, const std::filesystem::path& root) {
    const std::string extension = lower(path.extension().string());
    if (extension == ".bus") {
        return validateVehicle(path, root);
    }
    if (extension == ".osc") {
        return validateOsc(path);
    }
    if (extension == ".cfg") {
        std::ifstream input(path);
        if (!input) {
            Finding finding;
            finding.status = Status::Invalid;
            finding.kind = "cfg";
            finding.diagnostics.push_back({ConfigurationDiagnostic::Severity::Error, 0, "file",
                                           "unable to open CFG file"});
            return finding;
        }
        if (!hasModelSignature(path)) {
            Finding finding;
            finding.status = Status::Unsupported;
            finding.kind = "cfg-unrecognized";
            finding.diagnostics.push_back({ConfigurationDiagnostic::Severity::Warning, 0, "cfg",
                                           "CFG family not recognized; file was not parsed"});
            return finding;
        }
        return validateModel(path);
    }
    return {};
}

const char* statusName(Status status) {
    switch (status) {
    case Status::Valid:
        return "VALID";
    case Status::Invalid:
        return "INVALID";
    case Status::Unsupported:
        return "UNSUPPORTED";
    }
    return "UNKNOWN";
}

bool excludedDirectory(const std::filesystem::path& path) {
    static const std::unordered_set<std::string> excluded = {
        ".git", ".venv", "venv", "env", "build", "build-benchmark", "build-ode",
        "cmakefiles", "node_modules", "third_party", "vendor", "screenshots",
        "render-benchmark-results"};
        const std::string name = lower(path.filename().string());
        return excluded.find(name) != excluded.end() || name.rfind("build-", 0) == 0 ||
            name.rfind("build_", 0) == 0;
}

void printFinding(const std::filesystem::path& path, const Finding& finding) {
    std::cout << statusName(finding.status) << "\t" << finding.kind << "\t" << path.string()
              << '\n';
    for (const ConfigurationDiagnostic& diagnostic : finding.diagnostics) {
        std::cout << "  " << (diagnostic.severity == ConfigurationDiagnostic::Severity::Error
                                   ? "error"
                                   : "warning");
        if (diagnostic.line != 0) {
            std::cout << " line " << diagnostic.line;
        }
        std::cout << " [" << diagnostic.keyword << "]: " << diagnostic.message << '\n';
    }
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: OpenBusConfigAudit <OMSI-root-or-directory>\n";
        return 2;
    }

    const std::filesystem::path root = std::filesystem::absolute(argv[1]);
    std::error_code error;
    if (!std::filesystem::is_directory(root, error) || error) {
        std::cerr << "Not a readable directory: " << root.string() << '\n';
        return 2;
    }

    std::size_t files = 0;
    std::size_t invalid = 0;
    std::size_t unsupported = 0;
    std::filesystem::recursive_directory_iterator iterator(
        root, std::filesystem::directory_options::skip_permission_denied, error);
    const std::filesystem::recursive_directory_iterator end;
    for (; iterator != end; iterator.increment(error)) {
        if (error) {
            std::cerr << "Warning: " << error.message() << '\n';
            error.clear();
            continue;
        }
        if (iterator->is_directory(error)) {
            if (excludedDirectory(iterator->path())) {
                iterator.disable_recursion_pending();
            }
            continue;
        }
        if (error) {
            error.clear();
            continue;
        }
        const std::string extension = lower(iterator->path().extension().string());
        if (extension != ".bus" && extension != ".cfg" && extension != ".osc") {
            continue;
        }
        ++files;
        try {
            const Finding finding = validate(iterator->path(), root);
            printFinding(iterator->path(), finding);
            invalid += finding.status == Status::Invalid ? 1U : 0U;
            unsupported += finding.status == Status::Unsupported ? 1U : 0U;
        } catch (const std::exception& exception) {
            Finding finding;
            finding.status = Status::Invalid;
            finding.kind = extension.substr(1);
            finding.diagnostics.push_back({ConfigurationDiagnostic::Severity::Error, 0, "file",
                                           exception.what()});
            printFinding(iterator->path(), finding);
            ++invalid;
        }
    }

    std::cout << "\nScanned " << files << " file(s): " << invalid << " invalid, " << unsupported
              << " unsupported/unrecognized.\n";
    return invalid == 0 ? 0 : 1;
}
