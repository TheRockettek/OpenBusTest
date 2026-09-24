#pragma once

#include "ModelConfigTypes.h"

#include <filesystem>
#include <ostream>

class Variables;

ModelConfig loadModelConfig(const std::filesystem::path& configPath,
                            const std::filesystem::path& modelRoot, ModelConfigKind kind,
                            Variables& variables);
void writeModelConfigurationJson(std::ostream& output, const std::filesystem::path& configPath,
                                 const ModelConfig& configuration);
