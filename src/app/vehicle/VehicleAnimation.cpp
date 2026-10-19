// Model animation evaluation and per-part transforms.

#include "app/vehicle/Vehicle.h"

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
    }
}

Matrix4 Vehicle::animationOriginMatrix(const ModelAnimation& animation) const {
    Matrix4 origin = identityMatrix();
    for (const ModelAnimationOrigin& operation : animation.originOperations) {
        Matrix4 operationMatrix = identityMatrix();
        switch (operation.type) {
        case ModelAnimationOriginType::Translation:
            operationMatrix = translationMatrix(operation.value);
            break;
        case ModelAnimationOriginType::RotationX:
            // OpenOMSI applies -angle around model X. Model X converts to
            // render -Y, which is equivalent to +angle around render +Y.
            operationMatrix = rotationMatrix(operation.value[0], 0.0, 1.0, 0.0);
            break;
        case ModelAnimationOriginType::RotationY:
            // Model Y converts to render +X.
            operationMatrix = rotationMatrix(-operation.value[0], 1.0, 0.0, 0.0);
            break;
        case ModelAnimationOriginType::RotationZ:
            // Model Z converts directly to render +Z.
            operationMatrix = rotationMatrix(-operation.value[0], 0.0, 0.0, 1.0);
            break;
        case ModelAnimationOriginType::FromMesh:
            if (animation.hasMeshTransform) {
                operationMatrix = animation.meshTransform;
            }
            break;
        }
        origin = multiplyMatrix4(origin, operationMatrix);
    }
    if (animation.originOperations.empty() && animation.hasOrigin) {
        origin = translationMatrix(animation.origin);
    }
    return origin;
}

Matrix4 Vehicle::animationTransform(const ModelAnimation& animation, double amount) const {
    const Matrix4 origin = animationOriginMatrix(animation);
    Matrix4 inverseOrigin = identityMatrix();
    // A singular origin leaves inverseOrigin as identity, which is the desired fallback.
    static_cast<void>(invertAffineMatrix(origin, inverseOrigin));
    // anim_rot and anim_trans both use the animation frame's local model
    // X axis. Model X converts to render -Y. OpenOMSI's -amount rotation
    // therefore becomes +amount around render +Y; translation converts to
    // negative render Y.
    const Matrix4 local = animation.type == "anim_rot"     ? rotationMatrix(amount, 0.0, 1.0, 0.0)
                          : animation.type == "anim_trans" ? translationMatrix({0.0, -amount, 0.0})
                                                           : identityMatrix();
    return multiplyMatrix4(multiplyMatrix4(origin, local), inverseOrigin);
}

Matrix4 Vehicle::animationTransformForPart(const DisplayPart& part,
                                           std::vector<const DisplayPart*>& active) const {
    if (part.animationCacheGeneration == animationGeneration) {
        return part.cachedAnimationTransform;
    }
    if (wheelsFromOde && odeSimulation != nullptr && part.odeWheelIndex >= 0) {
        const BodyPose chassis = odeSimulation->chassisPose();
        const BodyPose wheel =
            odeSimulation->wheelPose(static_cast<std::size_t>(part.odeWheelIndex));
        Matrix4 relative = identityMatrix();
        const std::array<double, 3> delta = {wheel.position[0] - chassis.position[0],
                                             wheel.position[1] - chassis.position[1],
                                             wheel.position[2] - chassis.position[2]};
        for (int row = 0; row < 3; ++row) {
            const double odeLocalPosition = chassis.rotation[row] * delta[0] +
                                            chassis.rotation[3 + row] * delta[1] +
                                            chassis.rotation[6 + row] * delta[2];
            const double modelLocalPosition = part.center[row] + (row == 2 ? modelOffsetZ : 0.0);
            relative[12 + row] = odeLocalPosition - modelLocalPosition;
            for (int column = 0; column < 3; ++column) {
                relative[column * 4 + row] =
                    chassis.rotation[row] * wheel.rotation[column] +
                    chassis.rotation[3 + row] * wheel.rotation[3 + column] +
                    chassis.rotation[6 + row] * wheel.rotation[6 + column];
            }
        }
        Matrix4 base = identityMatrix();
        base[5] = 0.0;
        base[6] = -1.0;
        base[9] = 1.0;
        base[10] = 0.0;
        Matrix4 inverseBase = identityMatrix();
        static_cast<void>(invertAffineMatrix(base, inverseBase));
        relative = multiplyMatrix4(relative, inverseBase);
        const Matrix4 toOrigin =
            translationMatrix({-part.center[0], -part.center[1], -part.center[2]});
        const Matrix4 fromOrigin =
            translationMatrix({part.center[0], part.center[1], part.center[2]});
        part.cachedAnimationTransform =
            multiplyMatrix4(fromOrigin, multiplyMatrix4(relative, toOrigin));
        part.animationCacheGeneration = animationGeneration;
        return part.cachedAnimationTransform;
    }
    Matrix4 local = identityMatrix();
    for (std::size_t index = 0; index < part.animations.size(); ++index) {
        const ModelAnimation& animation = part.animations[index];
        const double value = variables.get(animation.variable);
        const double amount = index < part.animationStates.size()
                                  ? part.animationStates[index].currentAmount
                                  : value * animation.scale + animation.offset;
        local = multiplyMatrix4(animationTransform(animation, amount), local);
    }
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
