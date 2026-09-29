#pragma once

#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

inline constexpr char kOscConverterVersion[] = "10";

enum class OscOpcode {
    PushNumber,
    PushString,
    LoadLocal,
    LoadLocalString,
    StoreLocal,
    StoreLocalString,
    LoadSystem,
    StoreSystem,
    LoadConstant,
    CallCurve,
    LoadRegister,
    StoreRegister,
    Duplicate,
    Add,
    Subtract,
    Multiply,
    Divide,
    Modulo,
    Negate,
    LogicalNot,
    Equal,
    NotEqual,
    Less,
    LessEqual,
    Greater,
    GreaterEqual,
    LogicalAnd,
    LogicalOr,
    Absolute,
    Minimum,
    Maximum,
    Floor,
    Ceiling,
    Sine,
    Cosine,
    Tangent,
    ArcTangent,
    SquareRoot,
    ArcSine,
    Exponential,
    Square,
    Sign,
    Random,
    StringDuplicate,
    StringConcat,
    StringRepeat,
    StringLength,
    StringCutBegin,
    StringCutEnd,
    StringSetLengthLeft,
    StringSetLengthRight,
    StringSetLengthCenter,
    IntegerToString,
    IntegerToStringEnhanced,
    StringToFloat,
    RemoveSpaces,
    StringEqual,
    StringLess,
    StringGreater,
    StringLessEqual,
    StringGreaterEqual,
    StringNoOp,
    DebugString,
    StackDump,
    JumpIfFalse,
    Jump,
    CallFunction,
    CallSystemMacro,
    SoundTrigger,
    SoundTriggerFile,
};

struct OscInstruction {
    OscOpcode opcode = OscOpcode::PushNumber;
    double number = 0.0;
    int index = 0;
    std::string name;
};

struct OscProgram {
    std::unordered_map<std::string, std::vector<OscInstruction>> functions;
};

std::filesystem::path generatedLuaPath(const std::filesystem::path& inputPath);

bool convertOscToLua(const std::filesystem::path& inputPath,
                     const std::filesystem::path& outputPath, std::string& error);

bool compileOscToBytecode(const std::filesystem::path& inputPath, OscProgram& output,
                          std::string& error);