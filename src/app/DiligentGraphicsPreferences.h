#pragma once

#include "core/GraphicsPreferences.h"
#include "render/diligent/DiligentColorPresent.h"
#include "render/diligent/DiligentMultisampling.h"

#include <stdexcept>
#include <string_view>

namespace openbus::app {

[[nodiscard]] inline openbus::rendering::DiligentDebugView
diligentDebugViewForPreference(std::string_view preference) {
    if (preference == "off")
        return openbus::rendering::DiligentDebugView::None;
    if (preference == "motion_vectors")
        return openbus::rendering::DiligentDebugView::MotionVectors;
    if (preference == "motion_magnitude")
        return openbus::rendering::DiligentDebugView::MotionMagnitude;
    if (preference == "motion_validity")
        return openbus::rendering::DiligentDebugView::MotionValidity;
    if (preference == "linear_depth")
        return openbus::rendering::DiligentDebugView::LinearDepth;
    throw std::invalid_argument("Unsupported Diligent temporal debug-view preference");
}

[[nodiscard]] inline std::string_view
preferenceForDiligentDebugView(openbus::rendering::DiligentDebugView view) {
    switch (view) {
    case openbus::rendering::DiligentDebugView::None:
        return "off";
    case openbus::rendering::DiligentDebugView::MotionVectors:
        return "motion_vectors";
    case openbus::rendering::DiligentDebugView::MotionMagnitude:
        return "motion_magnitude";
    case openbus::rendering::DiligentDebugView::MotionValidity:
        return "motion_validity";
    case openbus::rendering::DiligentDebugView::LinearDepth:
        return "linear_depth";
    }
    throw std::invalid_argument("Unsupported Diligent temporal debug view");
}

struct DiligentGraphicsOptions {
    openbus::rendering::DiligentMultisampling multisampling;
    openbus::rendering::DiligentColorFilter colorFilter =
        openbus::rendering::DiligentColorFilter::None;
    Diligent::Uint32 anisotropy = 1;
    bool generateTextureMipmaps = false;
    bool verticalSync = true;
};

[[nodiscard]] inline DiligentGraphicsOptions
makeDiligentGraphicsOptions(const openbus::core::GraphicsPreferences& preferences) {
    DiligentGraphicsOptions options;
    options.generateTextureMipmaps = preferences.generateTextureMipmaps;
    options.verticalSync = preferences.vsync;

    if (preferences.antiAliasingMode == "off") {
        options.colorFilter = openbus::rendering::DiligentColorFilter::None;
    } else if (preferences.antiAliasingMode == "fxaa") {
        options.colorFilter = openbus::rendering::DiligentColorFilter::Fxaa;
    } else if (preferences.antiAliasingMode == "smaa") {
        options.colorFilter = openbus::rendering::DiligentColorFilter::Smaa;
    } else if (preferences.antiAliasingMode == "msaa") {
        options.multisampling.samples = 4;
    } else {
        throw std::invalid_argument("Unsupported Diligent anti-aliasing preference");
    }

    switch (preferences.anisotropicFiltering) {
    case 1:
    case 2:
    case 4:
    case 8:
    case 16:
        options.anisotropy = static_cast<Diligent::Uint32>(preferences.anisotropicFiltering);
        break;
    default:
        throw std::invalid_argument("Unsupported Diligent anisotropic filtering preference");
    }

    return options;
}

} // namespace openbus::app
