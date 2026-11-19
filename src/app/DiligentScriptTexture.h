#pragma once

#include "render/diligent/DiligentRuntimeTexture2D.h"

class ScriptRuntime;

namespace Diligent {
struct IDeviceContext;
struct IRenderDevice;
} // namespace Diligent

namespace openbus::app {

enum class DiligentScriptTextureKind {
    Script,
    Text,
};

// Bridges one fixed ScriptRuntime texture slot to revision-aware Diligent storage.
// The caller synchronizes on the immediate context before submitting draws.
class DiligentScriptTexture final {
  public:
    DiligentScriptTexture(Diligent::IRenderDevice& device, DiligentScriptTextureKind kind,
                          int index);
    ~DiligentScriptTexture();
    DiligentScriptTexture(const DiligentScriptTexture&) = delete;
    DiligentScriptTexture& operator=(const DiligentScriptTexture&) = delete;

    // Returns false if the slot does not exist or has no drawable image yet.
    bool synchronize(ScriptRuntime& scripts, Diligent::IDeviceContext& immediateContext);

    [[nodiscard]] const openbus::rendering::DiligentRuntimeTexture2D& texture() const noexcept;

  private:
    DiligentScriptTextureKind kind_;
    int index_;
    openbus::rendering::DiligentRuntimeTexture2D texture_;
};

} // namespace openbus::app
