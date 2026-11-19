#pragma once

#include "assets/ObjLoader.h"
#include "render/diligent/DiligentTexturedMesh.h"

#include <array>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace Diligent {
struct IRenderDevice;
}

namespace openbus::rendering {

struct DiligentObjSubmesh {
    std::string materialName;
    ObjMaterial material;
    std::unique_ptr<DiligentTexturedMesh> mesh;
    std::shared_ptr<DiligentTexture2D> baseTexture;

    // Maps diffuse color, alpha, and specular fields; emission stays in ObjMaterial
    // because its MTL meaning differs from DiligentMaterialState::emissionColor.
    [[nodiscard]] DiligentMaterialState materialState() const;
    void draw(DiligentTexturedMeshRenderer& renderer, Diligent::IDeviceContext& context,
              Diligent::ITextureView& fallbackTexture, const Matrix4& modelViewProjection,
              const DiligentLightingParameters& lighting = {}) const;
};

enum class DiligentObjCoordinateFrame {
    Parsed,
    Vehicle,
};

struct DiligentObjImportOptions {
    DiligentObjCoordinateFrame coordinateFrame = DiligentObjCoordinateFrame::Parsed;
    bool flipTextureV = false;
};

// Converts an already parsed OBJ/O3D asset into stable, material-grouped Diligent meshes.
// Triangle order is retained within a material group, but interleaved groups are batched;
// callers must sort transparent groups for their view.
class DiligentObjModel final {
  public:
    // When textureRoot is non-empty, material texture references are resolved beneath it and
    // loaded through DiligentTexture2D. An empty root leaves texture binding to the caller.
    DiligentObjModel(Diligent::IRenderDevice& device, const ParsedObj& model,
                     const std::filesystem::path& textureRoot = {},
                     DiligentObjImportOptions options = {});
    ~DiligentObjModel();
    DiligentObjModel(const DiligentObjModel&) = delete;
    DiligentObjModel& operator=(const DiligentObjModel&) = delete;

    [[nodiscard]] std::span<const DiligentObjSubmesh> submeshes() const noexcept;
    [[nodiscard]] bool hasTransform() const noexcept;
    [[nodiscard]] const std::array<double, 16>& transform() const noexcept;

  private:
    std::vector<DiligentObjSubmesh> submeshes_;
    bool hasTransform_ = false;
    std::array<double, 16> transform_ = {
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    };
};

} // namespace openbus::rendering
