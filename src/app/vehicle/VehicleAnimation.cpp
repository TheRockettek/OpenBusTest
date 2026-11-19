// Model animation evaluation and per-part transforms.

#include "app/vehicle/Vehicle.h"

#include <cmath>

void Vehicle::updateAnimationStates() {
    TraceScope trace("render", "Vehicle::updateAnimationStates");
    ++animationGeneration;
    const double timeStep = std::clamp(animationTimeStep, 0.0, 0.25);
    for (DisplayPart& part : displayLists) {
        if (part.animationStates.size() != part.animations.size()) {
            part.animationStates.resize(part.animations.size());
        }
        for (std::size_t index = 0; index < part.animations.size(); ++index) {
            const ModelAnimation& animation = part.animations[index];
            DisplayPart::AnimationState& state = part.animationStates[index];
            if (animation.type.empty()) {
                state.currentAmount = 0.0;
                state.targetAmount = 0.0;
                state.initialized = true;
                continue;
            }
            const double variableValue = variables.get(animation.variable);
            const double targetAmount = variableValue * animation.scale + animation.offset;
            if (!state.initialized) {
                state.currentAmount = targetAmount;
                state.targetAmount = targetAmount;
                state.initialized = true;
                continue;
            }
            if (targetAmount != state.targetAmount) {
                state.targetAmount = targetAmount;
            }
            double nextAmount = state.targetAmount;
            if (animation.delay > 0.0) {
                const double interpolation = std::min(1.0, animation.delay * timeStep);
                nextAmount = state.currentAmount +
                             (state.targetAmount - state.currentAmount) * interpolation;
            }
            if (animation.maxSpeed > 0.0) {
                const double maximumStep = animation.maxSpeed * timeStep;
                state.currentAmount +=
                    std::clamp(nextAmount - state.currentAmount, -maximumStep, maximumStep);
            } else {
                state.currentAmount = nextAmount;
            }
        }
        for (DisplayPart::EnhancedLightBatch& lightBatch : part.enhancedLightBatches) {
            if (lightBatch.lightIndex >= enhancedLights.size() ||
                lightBatch.lightIndex >= enhancedLightControllers.size()) {
                continue;
            }
            const ModelEnhancedLight& light = enhancedLights[lightBatch.lightIndex];
            const InteriorLightController& controller =
                enhancedLightControllers[lightBatch.lightIndex];
            const std::size_t multiplierIndex = light.enhanced ? 17 : 7;
            const std::size_t timeConstantIndex = light.enhanced ? 21 : 10;
            if (light.parameters.size() <= timeConstantIndex) {
                continue;
            }
            const double controllerValue =
                controller.isNumeric ? controller.numericValue : variables.get(light.controller);
            const double sunlightFactor =
                std::clamp(std::max(static_cast<double>(variables.get("nightlighta")),
                                    1.0 - static_cast<double>(variables.get("envir_brightness"))),
                           0.0, 1.0);
            const double targetBrightness =
                std::max(0.0, controllerValue * light.parameters[multiplierIndex] * sunlightFactor);
            const double timeConstant = std::max(0.0, light.parameters[timeConstantIndex]);
            if (!lightBatch.brightnessInitialized || timeConstant == 0.0 || timeStep == 0.0) {
                lightBatch.currentBrightness = targetBrightness;
                lightBatch.brightnessInitialized = true;
            } else {
                const double change = 1.0 - std::exp(-timeStep / timeConstant);
                lightBatch.currentBrightness +=
                    (targetBrightness - lightBatch.currentBrightness) * change;
            }
        }
    }
}

Matrix4 Vehicle::animationOriginMatrix(const ModelAnimation& animation) const {
    return openbus::rendering::modelAnimationOriginMatrix(animation);
}

Matrix4 Vehicle::animationTransform(const ModelAnimation& animation, double amount) const {
    return openbus::rendering::modelAnimationTransform(animation, amount);
}

Matrix4 Vehicle::animationTransformForPart(const DisplayPart& part,
                                           std::vector<const DisplayPart*>& active) const {
    if (part.animationCacheGeneration == animationGeneration) {
        return part.cachedAnimationTransform;
    }
    if (wheelsFromOde && odeSimulation != nullptr && part.odeWheelIndex >= 0 &&
        static_cast<std::size_t>(part.odeWheelIndex) >= odeWheelIndexOffset) {
        const std::size_t wheelIndex =
            static_cast<std::size_t>(part.odeWheelIndex) - odeWheelIndexOffset;
        const BodyPose chassis = odeSimulation->chassisPose();
        const BodyPose wheel = odeSimulation->wheelPose(wheelIndex);
        part.cachedAnimationTransform = openbus::rendering::makeOdeWheelAnimationTransform(
            chassis, wheel, part.center, modelOffsetZ);
        part.animationCacheGeneration = animationGeneration;
        return part.cachedAnimationTransform;
    }
    openbus::rendering::ModelAnimationTransformBuilder localTransform;
    for (std::size_t index = 0; index < part.animations.size(); ++index) {
        const ModelAnimation& animation = part.animations[index];
        const double value = variables.get(animation.variable);
        const double amount = index < part.animationStates.size()
                                  ? part.animationStates[index].currentAmount
                                  : value * animation.scale + animation.offset;
        localTransform.append(animation, amount);
    }
    const Matrix4& local = localTransform.matrix();
    if (!modelPartInheritsAnimationParent(part.fixed, part.animationParent) ||
        std::find(active.begin(), active.end(), &part) != active.end()) {
        if (std::find(active.begin(), active.end(), &part) != active.end()) {
            return local;
        }
        part.cachedAnimationTransform = local;
        part.animationCacheGeneration = animationGeneration;
        return local;
    }
    const DisplayPart* parent = nullptr;
    const auto findParent = [&](const std::vector<DisplayPart>& parts) {
        const auto found =
            std::find_if(parts.begin(), parts.end(), [&](const DisplayPart& candidate) {
                return candidate.meshIdentifier == part.animationParent;
            });
        return found == parts.end() ? nullptr : &*found;
    };
    parent = findParent(displayLists);
    if (parent == nullptr) {
        part.cachedAnimationTransform = local;
        part.animationCacheGeneration = animationGeneration;
        return local;
    }
    active.push_back(&part);
    const Matrix4 parentTransform = animationTransformForPart(*parent, active);
    active.pop_back();
    part.cachedAnimationTransform = multiplyMatrix4(parentTransform, local);
    part.animationCacheGeneration = animationGeneration;
    return part.cachedAnimationTransform;
}

Matrix4 Vehicle::animationTransformForPart(const DisplayPart& part) const {
    if (part.animationCacheGeneration == animationGeneration) {
        return part.cachedAnimationTransform;
    }
    std::vector<const DisplayPart*> active;
    return animationTransformForPart(part, active);
}

void Vehicle::applyAnimations(const DisplayPart& part) const {
    multiplyMatrix(animationTransformForPart(part));
}

void Vehicle::applyDrawTransform(const DisplayPart& part,
                                 const Matrix4* groundShadowTransform) const {
    if (part.isShadow && groundShadowTransform != nullptr) {
        multiplyMatrix(*groundShadowTransform);
    }
    applyAnimations(part);
}

std::array<double, 3> Vehicle::animatedPartCenter(const DisplayPart& part) const {
    const std::array<double, 4> local = {part.center[0], part.center[1], part.center[2], 1.0};
    const Matrix4 animation = animationTransformForPart(part);
    const std::array<double, 4> transformed = transformPoint(animation, local);
    return {transformed[0], transformed[1], transformed[2] + modelOffsetZ};
}
