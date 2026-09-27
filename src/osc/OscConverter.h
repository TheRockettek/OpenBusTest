#pragma once

#include <filesystem>
#include <string>

inline constexpr char kOscConverterVersion[] = "3";

std::filesystem::path generatedLuaPath(const std::filesystem::path& inputPath);

bool convertOscToLua(const std::filesystem::path& inputPath,
                     const std::filesystem::path& outputPath, std::string& error);