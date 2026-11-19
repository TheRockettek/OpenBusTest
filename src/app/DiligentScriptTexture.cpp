#include "app/DiligentScriptTexture.h"

#include "scripting/ScriptRuntime.h"

#include <stdexcept>

namespace openbus::app {

DiligentScriptTexture::DiligentScriptTexture(Diligent::IRenderDevice& device,
                                             DiligentScriptTextureKind kind, int index)
    : kind_(kind), index_(index), texture_(device) {
    if (index_ < 0) {
        throw std::invalid_argument("Diligent script texture index must be non-negative");
    }
    if (kind_ != DiligentScriptTextureKind::Script && kind_ != DiligentScriptTextureKind::Text) {
        throw std::invalid_argument("Diligent script texture kind is invalid");
    }
}

DiligentScriptTexture::~DiligentScriptTexture() = default;

bool DiligentScriptTexture::synchronize(ScriptRuntime& scripts,
                                        Diligent::IDeviceContext& immediateContext) {
    ScriptRuntime::ScriptTextureSnapshot snapshot;
    const bool alreadyInitialized = texture_.initialized();
    const bool copied =
        kind_ == DiligentScriptTextureKind::Script
            ? (alreadyInitialized
                   ? scripts.copyScriptTextureIfChanged(index_, texture_.revision(), snapshot)
                   : scripts.copyScriptTexture(index_, snapshot))
            : (alreadyInitialized
                   ? scripts.copyTextTextureIfChanged(index_, texture_.revision(), snapshot)
                   : scripts.copyTextTexture(index_, snapshot));
    if (!copied) {
        return false;
    }
    if (snapshot.width <= 0 || snapshot.height <= 0) {
        return false;
    }
    if (snapshot.pixels.empty()) {
        if (!alreadyInitialized || snapshot.width != texture_.width() ||
            snapshot.height != texture_.height() || snapshot.revision != texture_.revision()) {
            throw std::runtime_error(
                "Script texture revision changed without providing pixel data");
        }
        texture_.synchronize(immediateContext, snapshot.width, snapshot.height, {},
                             snapshot.filtered, snapshot.revision);
        return true;
    }
    texture_.synchronize(immediateContext, snapshot.width, snapshot.height, snapshot.pixels,
                         snapshot.filtered, snapshot.revision);
    return true;
}

const openbus::rendering::DiligentRuntimeTexture2D&
DiligentScriptTexture::texture() const noexcept {
    return texture_;
}

} // namespace openbus::app
