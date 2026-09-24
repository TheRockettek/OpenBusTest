#pragma once

#include "BusTypes.h"

#include <array>

namespace openbus::rendering {

void setPerspective(double width, double height, double fieldOfView);
void lookAt(double eyeX, double eyeY, double eyeZ, double targetX, double targetY, double targetZ);
void applyPose(const BodyPose& pose);
std::array<double, 3> transformLocalPoint(const BodyPose& pose,
                                          const std::array<double, 3>& local);

}  // namespace openbus::rendering
