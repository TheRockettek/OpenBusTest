// Public ScriptRuntime API; forwards to ScriptRuntime::Impl.

#include "scripting/ScriptRuntimeImpl.h"

ScriptRuntime::ScriptRuntime(
    const VehicleConfig& configuration, Variables& localState, SimulationState& sharedState,
    std::function<void(const std::string&, const std::string&, float)> soundTrigger)
    : impl_(std::make_unique<Impl>(configuration, localState, sharedState)) {
    impl_->onSoundTrigger = std::move(soundTrigger);
}

ScriptRuntime::~ScriptRuntime() = default;

void ScriptRuntime::configureScriptTextures(const std::vector<ModelScriptTexture>& definitions) {
    if (!impl_) {
        return;
    }
    for (const ModelScriptTexture& definition : definitions) {
        if (!openbus::scripting::validScriptTextureIndex(definition.slot) ||
            !openbus::scripting::validScriptTextureSize(definition.width, definition.height)) {
            continue;
        }
        const std::size_t byteSize =
            openbus::scripting::scriptTextureByteSize(definition.width, definition.height);
        const auto existing = impl_->scriptTextures.find(definition.slot);
        const std::size_t oldBytes =
            existing == impl_->scriptTextures.end() ? 0U : existing->second.pixels.size();
        if (!impl_->canReplaceTextureBytes(oldBytes, byteSize)) {
            continue;
        }
        impl_->scriptTextureDimensions[definition.slot] = {definition.width, definition.height};
        impl_->scriptTextures[definition.slot] = impl_->configuredScriptTexture(definition.slot);
    }
}

void ScriptRuntime::configureTextTextures(const std::vector<ModelTextTexture>& definitions,
                                          const std::vector<std::string>& placementLabels) {
    if (!impl_) {
        return;
    }
    const std::size_t initializedStringCount =
        std::min(impl_->configuration.stringVariables.size(), placementLabels.size());
    for (std::size_t index = 0; index < initializedStringCount; ++index) {
        impl_->localState.setString(impl_->configuration.stringVariables[index],
                                    placementLabels[index]);
    }
    for (const ModelTextTexture& definition : definitions) {
        const std::size_t minimumValues = definition.placementLabel ? 7U : 4U;
        if (!openbus::scripting::validScriptTextureIndex(definition.slot) ||
            definition.values.size() < minimumValues) {
            continue;
        }
        ScriptRuntime::Impl::TextTextureDefinition configured;
        const std::size_t fontIndex = definition.placementLabel ? 0U : 1U;
        const std::size_t widthIndex = definition.placementLabel ? 1U : 2U;
        const std::size_t heightIndex = definition.placementLabel ? 2U : 3U;
        const std::size_t colorModeIndex = definition.placementLabel ? 3U : 4U;
        const std::size_t colorStartIndex = definition.placementLabel ? 4U : 5U;
        configured.variable = definition.placementLabel
                                  ? "__openbus_placement_label_" + std::to_string(definition.slot)
                                  : definition.values[0];
        configured.font = lower(definition.values[fontIndex]);
        try {
            configured.width = std::stoi(definition.values[widthIndex]);
            configured.height = std::stoi(definition.values[heightIndex]);
        } catch (const std::exception&) {
            continue;
        }
        if (configured.variable.empty() ||
            !openbus::scripting::validScriptTextureSize(configured.width, configured.height)) {
            continue;
        }
        if (definition.placementLabel) {
            impl_->localState.declareString(configured.variable);
            const std::size_t labelIndex = static_cast<std::size_t>(definition.slot);
            impl_->localState.setString(configured.variable, labelIndex < placementLabels.size()
                                                                 ? placementLabels[labelIndex]
                                                                 : std::string{});
        }
        const std::size_t pixelBytes =
            openbus::scripting::scriptTextureByteSize(configured.width, configured.height);
        const auto existing = impl_->textTextures.find(definition.slot);
        const std::size_t oldBytes =
            existing == impl_->textTextures.end() ? 0U : existing->second.pixels.size();
        if (!impl_->canReplaceTextureBytes(oldBytes, pixelBytes)) {
            continue;
        }
        if (definition.values.size() > colorModeIndex) {
            try {
                configured.useFontColor = std::stoi(definition.values[colorModeIndex]) != 0;
            } catch (const std::exception&) {
                configured.useFontColor = false;
            }
        }
        // Enhanced alignment values are two sets of the same horizontal modes:
        // 0/3=center, 1/4=left, and 2/5=right. The following field is the
        // additional per-character grid spacing used by enhanced displays.
        if (definition.enhanced && definition.values.size() > 8) {
            try {
                const int alignment = std::stoi(definition.values[8]);
                switch (alignment) {
                case 1:
                case 4:
                    configured.alignment = ScriptRuntime::Impl::TextAlignment::Left;
                    break;
                case 2:
                case 5:
                    configured.alignment = ScriptRuntime::Impl::TextAlignment::Right;
                    break;
                case 0:
                case 3:
                default:
                    configured.alignment = ScriptRuntime::Impl::TextAlignment::Center;
                    break;
                }
            } catch (const std::exception&) {
                configured.alignment = ScriptRuntime::Impl::TextAlignment::Center;
            }
        }
        if (definition.enhanced && definition.values.size() > 9) {
            try {
                std::size_t consumed = 0;
                const int spacing = std::stoi(definition.values[9], &consumed);
                if (consumed == definition.values[9].size()) {
                    configured.gridSpacing = std::clamp(spacing, 0, 64);
                }
            } catch (const std::exception&) {
                configured.gridSpacing = 0;
            }
        }
        for (std::size_t channel = 0;
             channel < 3 && channel + colorStartIndex < definition.values.size(); ++channel) {
            try {
                const float value = std::stof(definition.values[channel + colorStartIndex]);
                configured.color[channel] = ScriptRuntime::Impl::colorChannel(value);
            } catch (const std::exception&) {
                continue;
            }
        }
        impl_->textTextureDefinitions[definition.slot] = configured;
        ScriptRuntime::Impl::ScriptTexture texture;
        texture.width = configured.width;
        texture.height = configured.height;
        texture.pixels.assign(pixelBytes, 0);
        texture.revision = ++impl_->revisionCounter;
        impl_->textTextures[definition.slot] = std::move(texture);
    }
}

bool ScriptRuntime::valid() const {
    return impl_ && impl_->errors.empty();
}

const std::vector<std::string>& ScriptRuntime::errors() const {
    return impl_->errors;
}

void ScriptRuntime::initialize() {
    if (impl_ && (impl_->nativeBackend || impl_->state)) {
        impl_->floatStack.clear();
        impl_->stringStack.clear();
        if (impl_->nativeBackend) {
            impl_->executeNativeFunction("init");
        } else {
            impl_->invoke("init");
        }
        impl_->updateTextTextures(true);
    }
}

void ScriptRuntime::invokeEntryPoint(const std::string& functionName) {
    if (impl_ && (impl_->nativeBackend || impl_->state) && !functionName.empty()) {
        impl_->floatStack.clear();
        impl_->stringStack.clear();
        const bool invoked = impl_->nativeBackend ? impl_->executeNativeFunction(functionName)
                                                  : impl_->invoke(functionName.c_str());
        if (!invoked) {
            impl_->log("warning: entry point " + functionName + " not found");
        }
    }
}

void ScriptRuntime::invokeSystemTrigger(const std::string& triggerName) {
    if (impl_ && (impl_->nativeBackend || impl_->state) &&
        impl_->localState.supportsSystemTrigger(triggerName)) {
        impl_->floatStack.clear();
        impl_->stringStack.clear();
        const std::string functionName = "trigger_" + scriptName(triggerName);
        const bool invoked = impl_->nativeBackend ? impl_->executeNativeFunction(functionName)
                                                  : impl_->invoke(functionName.c_str());
        if (!invoked) {
            impl_->log("warning: system trigger " + triggerName + " -> " + functionName +
                       " not found");
        }
    }
}

void ScriptRuntime::invokeInputEvent(const std::string& keyName, bool pressed) {
    if (!impl_ || keyName.empty()) {
        return;
    }
    const std::string normalized = lower(keyName);
    impl_->localState.set("key_pressed", pressed ? 1.0 : 0.0);
    impl_->localState.setString("key_name", normalized);
    const std::string functionName =
        "key_" + scriptName(normalized) + (pressed ? "_pressed" : "_released");
    if (impl_->nativeBackend || impl_->state) {
        impl_->floatStack.clear();
        impl_->stringStack.clear();
        const bool invoked = impl_->nativeBackend ? impl_->executeNativeFunction(functionName)
                                                  : impl_->invoke(functionName.c_str());
        if (!invoked) {
            impl_->log("warning: key event " + normalized + (pressed ? " pressed" : " released") +
                       " -> " + functionName + " not found");
        }
    }
}

void ScriptRuntime::invokeKeyBinding(const std::string& bindingName, bool pressed, bool logEvent) {
    if (!impl_ || bindingName.empty()) {
        return;
    }
    const std::string normalized = lower(bindingName);
    impl_->localState.set("key_pressed", pressed ? 1.0 : 0.0);
    impl_->localState.setString("key_name", normalized);
    bool invoked = false;
    const std::string functionName = "trigger_" + scriptName(normalized) + (pressed ? "" : "_off");
    if (impl_->nativeBackend || impl_->state) {
        impl_->floatStack.clear();
        impl_->stringStack.clear();
        invoked = impl_->nativeBackend ? impl_->executeNativeFunction(functionName)
                                       : impl_->invoke(functionName.c_str());
        if (!invoked) {
            if (logEvent && hasScriptEntryPoint(functionName)) {
                impl_->log("warning: key binding " + normalized +
                           (pressed ? " pressed" : " released") + " -> " + functionName +
                           " not found");
            }
        } else if (logEvent) {
            // impl_->log("trigger ran: " + functionName + " (key binding " + normalized +
            //            (pressed ? " pressed)" : " released)"));
        }
    }
}

void ScriptRuntime::invokeMouseEvent(const std::string& eventName) {
    if (!impl_ || eventName.empty()) {
        return;
    }
    const std::string normalized = lower(eventName);
    impl_->localState.setString("mouse_event", normalized);
    if (impl_->nativeBackend || impl_->state) {
        impl_->floatStack.clear();
        impl_->stringStack.clear();
        const std::string functionName = "trigger_" + scriptName(normalized);
        const bool invoked = impl_->nativeBackend ? impl_->executeNativeFunction(functionName)
                                                  : impl_->invoke(functionName.c_str());
        if (!invoked) {
            impl_->log("warning: mouse event " + normalized + " -> " + functionName + " not found");
        } else {
            // impl_->log("trigger ran: " + functionName + " (mouse event " + normalized + ")");
        }
    }
}

bool ScriptRuntime::hasScriptEntryPoint(const std::string& functionName) const {
    if (!impl_ || functionName.empty()) {
        return false;
    }
    const std::string normalized = scriptName(functionName);
    if (impl_->nativeBackend) {
        for (const std::shared_ptr<const OscProgram>& program : impl_->nativePrograms) {
            if (program->functions.find(normalized) != program->functions.end()) {
                return true;
            }
        }
        return false;
    }
    if (!impl_->state) {
        return false;
    }
    lua_rawgeti(impl_->state, LUA_REGISTRYINDEX, impl_->scriptEnvironment.reference);
    lua_getfield(impl_->state, -1, normalized.c_str());
    const bool found = lua_isfunction(impl_->state, -1) != 0;
    lua_pop(impl_->state, 2);
    return found;
}

void ScriptRuntime::invokeMouseRelease(const std::string& eventName) {
    if (!impl_ || eventName.empty()) {
        return;
    }
    const std::string normalized = lower(eventName);
    impl_->localState.setString("mouse_event", normalized);
    if (impl_->nativeBackend || impl_->state) {
        impl_->floatStack.clear();
        impl_->stringStack.clear();
        const std::string functionName = "trigger_" + scriptName(normalized) + "_off";
        // Toggle controls need not define a release handler.
        if (!hasScriptEntryPoint(functionName)) {
            return;
        }
        const bool invoked = impl_->nativeBackend ? impl_->executeNativeFunction(functionName)
                                                  : impl_->invoke(functionName.c_str());
        if (!invoked) {
            impl_->log("warning: mouse release " + normalized + " -> " + functionName +
                       " not found");
        } else {
            // impl_->log("trigger ran: " + functionName + " (mouse release " + normalized + ")");
        }
    }
}

void ScriptRuntime::invokeMouseDrag(const std::string& eventName, float deltaX, float deltaY,
                                    float cursorX, float cursorY) {
    if (!impl_ || eventName.empty()) {
        return;
    }
    const std::string normalized = lower(eventName);
    impl_->localState.set("mouse_drag_x", deltaX);
    impl_->localState.set("mouse_drag_y", deltaY);
    impl_->localState.set("mouse_cursor_x", cursorX);
    impl_->localState.set("mouse_cursor_y", cursorY);
    // GLFW and the authored OMSI drag handlers use screen-space Y deltas: positive
    // means moving down. Keep that convention unchanged for every mouse event;
    // object scripts provide any control-specific scale or sign themselves.
    Variables& system = impl_->sharedState.sharedVariables();
    struct ScopedMouseMotion {
        Variables& variables;
        float previousX;
        float previousY;
        ~ScopedMouseMotion() {
            variables.set("mouse_x", previousX);
            variables.set("mouse_y", previousY);
        }
    } motion{system, system.get("mouse_x"), system.get("mouse_y")};
    system.set("mouse_x", deltaX);
    system.set("mouse_y", deltaY);
    impl_->localState.setString("mouse_event", normalized);
    if (impl_->nativeBackend || impl_->state) {
        impl_->floatStack.clear();
        impl_->stringStack.clear();
        const std::string functionName = "trigger_" + scriptName(normalized) + "_drag";
        // _drag is optional, even for a mesh whose button remains held.
        if (!hasScriptEntryPoint(functionName)) {
            return;
        }
        const bool invoked = impl_->nativeBackend ? impl_->executeNativeFunction(functionName)
                                                  : impl_->invoke(functionName.c_str());
        if (!invoked) {
            impl_->log("warning: mouse drag " + normalized + " -> " + functionName + " not found");
        }
    }
}

bool ScriptRuntime::copyScriptTexture(int index, ScriptTextureSnapshot& snapshot) const {
    return impl_ && impl_->copyScriptTexture(index, snapshot);
}

bool ScriptRuntime::copyTextTexture(int index, ScriptTextureSnapshot& snapshot) const {
    return impl_ && impl_->copyTextTexture(index, snapshot);
}

bool ScriptRuntime::copyScriptTextureIfChanged(int index, std::uint64_t knownRevision,
                                               ScriptTextureSnapshot& snapshot) const {
    return impl_ && impl_->copyScriptTextureIfChanged(index, knownRevision, snapshot);
}

bool ScriptRuntime::copyTextTextureIfChanged(int index, std::uint64_t knownRevision,
                                             ScriptTextureSnapshot& snapshot) const {
    return impl_ && impl_->copyTextTextureIfChanged(index, knownRevision, snapshot);
}

void ScriptRuntime::update(bool isAiVehicle) {
    if (impl_ && (impl_->nativeBackend || impl_->state)) {
        impl_->floatStack.clear();
        impl_->stringStack.clear();
        if (impl_->nativeBackend) {
            if (isAiVehicle && !impl_->executeNativeFunction("frame_ai")) {
                impl_->executeNativeFunction("frame");
            } else if (!isAiVehicle) {
                impl_->executeNativeFunction("frame");
            }
        } else {
            impl_->invokeFrame(isAiVehicle);
        }
        impl_->updateTextTextures();
    }
}
