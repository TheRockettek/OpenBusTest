#include "physics/BusSimulation.h"

#include "physics/BusConfiguration.h"
#include "physics/RoadFeatures.h"
#include "core/Logger.h"
#include "core/PerfTrace.h"
#include "core/Variables.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ode/ode.h>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace {
Logger simulationLog("Simulation");

constexpr dReal GRAVITY = -9.81;

constexpr dReal SUSP_REST = 0.35;
constexpr dReal SUSP_MAX_TRAVEL = 0.25;
constexpr dReal MINIMUM_CHASSIS_MASS = 1.0;
constexpr dReal TIRE_GRIP_LONG = 24000.0;
constexpr dReal LEGACY_ROLLING_RESIST_COEFFICIENT = 0.012;
constexpr dReal WHEEL_MASS = 180.0;
constexpr dReal KNUCKLE_MASS = 40.0;
constexpr dReal ROAD_FRICTION_COEFFICIENT = 0.85;
constexpr dReal OMSI_INERTIA_SCALE = 1000.0;
constexpr dReal WHEELSPIN_SLIP_RATIO = 0.15;
constexpr dReal ARB_STIFFNESS_FRONT = 35000.0;
constexpr dReal ARB_STIFFNESS_REAR = 28000.0;
constexpr dReal BODY_ROLL_STIFFNESS = 600000.0;
constexpr dReal BODY_PITCH_STIFFNESS = 250000.0;
constexpr dReal BODY_ATTITUDE_DAMPING = 100000.0;
constexpr dReal MAX_CHASSIS_ANGULAR_SPEED = 2.0;
constexpr dReal STOP_LATERAL_DAMPING = 90000.0;
constexpr dReal STOP_YAW_DAMPING = 300000.0;
constexpr dReal MAX_STEER_ANGLE = 0.45;
constexpr dReal STEER_SPEED = 100;
constexpr dReal STEERING_MAX_TORQUE = 10000.0;
constexpr dReal STEERING_POSITION_GAIN = 20.0;
constexpr dReal STEERING_RATE_DAMPING = 0.8;
constexpr dReal STEERING_MAX_RATE = 20.0;
constexpr dReal MAX_BRAKE_FORCE = 32000.0;
constexpr dReal YAW_DAMPING = 180000.0;
constexpr dReal AERO_CD = 0.65;
constexpr dReal AERO_FRONT_AREA = 7.90;
constexpr dReal AIR_DENSITY = 1.225;
constexpr dReal ENGINE_IDLE_RPM = 650.0;
constexpr dReal ENGINE_RATED_RPM = 1900.0;
constexpr dReal ENGINE_REDLINE = 2300.0;
constexpr dReal PEAK_TORQUE = 1650.0;
constexpr dReal FINAL_DRIVE = 5.13;
constexpr dReal DRIVETRAIN_EFF = 0.85;
constexpr int NUM_GEARS = 6;
constexpr std::array<dReal, NUM_GEARS> GEAR_RATIOS = {4.171, 2.340, 1.521, 1.143, 0.867, 0.686};
constexpr dReal UPSHIFT_RPM = 1900.0;
constexpr dReal DOWNSHIFT_RPM = 950.0;
constexpr dReal SHIFT_LOCKOUT = 0.0;
constexpr dReal PI = 3.14159265358979323846;
constexpr dReal MAX_FRAME_SECONDS = 0.25;
bool hasValidMomentOfInertia(const BusConfiguration& configuration) {
    return std::all_of(configuration.momentOfInertia.begin(), configuration.momentOfInertia.end(),
                       [](double value) { return std::isfinite(value) && value > 0.0; });
}

std::array<dReal, 3> configuredOdeInertia(const BusConfiguration& configuration) {
    return {static_cast<dReal>(configuration.momentOfInertia[1] * OMSI_INERTIA_SCALE),
            static_cast<dReal>(configuration.momentOfInertia[0] * OMSI_INERTIA_SCALE),
            static_cast<dReal>(configuration.momentOfInertia[2] * OMSI_INERTIA_SCALE)};
}

int checkedOdeCount(std::size_t count, const char* label) {
    if (!std::in_range<int>(count)) {
        throw std::runtime_error(std::string(label) + " exceeds ODE's integer limit");
    }
    return static_cast<int>(count);
}

} // namespace

struct BusSimulation::Impl {
    struct CollisionProfileCounters {
        std::uint64_t callbackPairs = 0;
        std::uint64_t filteredPairs = 0;
        std::uint64_t narrowphaseCalls = 0;
        std::uint64_t contacts = 0;
        std::uint64_t contactJoints = 0;
    };

    struct Corner {
        dReal x;
        dReal y;
        dReal wheelRadius;
        dReal springCompression = 0.0;
        dReal verticalVelocity = 0.0;
        dReal animationSuspension = 0.0;
        dReal suspensionSliderZero = 0.0;
        dBodyID suspensionBody = nullptr;
        dBodyID steeringBody = nullptr;
        dBodyID wheelBody = nullptr;
        dGeomID wheelGeom = nullptr;
        dJointID suspensionJoint = nullptr;
        dJointID steeringJoint = nullptr;
        dJointID wheelJoint = nullptr;
        dReal wheelOmega = 0.0;
        dReal wheelRotation = 0.0;
        dReal previousWheelHingeAngle = 0.0;
        dReal wheelSurfaceSpeed = 0.0;
        dReal pointLongitudinalSpeed = 0.0;
        dReal longitudinalSlipRatio = 0.0;
        dReal normalLoad = 0.0;
        dReal longitudinalForce = 0.0;
        dReal lateralForce = 0.0;
        dReal frictionUtilization = 0.0;
        dReal driveTorque = 0.0;
        dReal brakeTorque = 0.0;
        bool wheelspin = false;
        bool skid = false;
    };

    World& world;
    dBodyID chassis = nullptr;
    std::unordered_set<dGeomID> wheelGeometries;
    dGeomID chassisGeom = nullptr;
    WheelStaticContactSettings wheelStaticContact;
    BusConfiguration configuration;
    VehiclePlacement placement;
    std::vector<Corner> corners;
    dTriMeshDataID chassisCollisionMeshData = nullptr;
    double fixedStep = 0.0;
    int maxCatchUpSteps;
    double accumulator = 0.0;
    double steeringAngle = 0.0;
    dReal maxSteerAngle = MAX_STEER_ANGLE;
    int currentGear = 0;
    double shiftTimer = 0.0;
    int lastSteps = 0;
    bool dropped = false;
    bool dropReported = false;
    bool frozen = false;
    double simulationTime = 0.0;
    CollisionProfileCounters collisionProfileCounters;
    std::chrono::steady_clock::time_point collisionProfileWindowStart;
    bool collisionProfileWindowStarted = false;

    dReal suspensionAnchorZ(std::size_t index) const {
        return corners[index].wheelRadius + SUSP_REST - configuration.centerOfGravityHeight;
    }

    ~Impl() {
        if (chassisGeom) {
            dGeomDestroy(chassisGeom);
            chassisGeom = nullptr;
        }
        if (chassisCollisionMeshData) {
            dGeomTriMeshDataDestroy(chassisCollisionMeshData);
            chassisCollisionMeshData = nullptr;
        }
        for (Corner& corner : corners) {
            if (corner.wheelGeom) {
                dGeomDestroy(corner.wheelGeom);
            }
            if (corner.suspensionBody) {
                dBodyDestroy(corner.suspensionBody);
            }
            if (corner.steeringBody) {
                dBodyDestroy(corner.steeringBody);
            }
            if (corner.wheelBody) {
                dBodyDestroy(corner.wheelBody);
            }
        }
        if (chassis) {
            dBodyDestroy(chassis);
        }
    }

    static void nearCallback(void* context, dGeomID first, dGeomID second) {
        auto* simulation = static_cast<Impl*>(context);
        const bool profileRecording = openbus::rendering::IsRecording();
        if (profileRecording) {
            ++simulation->collisionProfileCounters.callbackPairs;
        }
        World& world = simulation->world;
        const bool firstHasBody = dGeomGetBody(first) != nullptr;
        const bool secondHasBody = dGeomGetBody(second) != nullptr;
        const bool firstIsPole = world.isMapPoleGeometry(first);
        const bool secondIsPole = world.isMapPoleGeometry(second);
        const bool isPoleContact = firstIsPole || secondIsPole;
        if ((!firstHasBody && !secondHasBody) ||
            (firstHasBody && secondHasBody && !isPoleContact)) {
            if (profileRecording) {
                ++simulation->collisionProfileCounters.filteredPairs;
            }
            return;
        }
        const bool firstIsWheel = simulation->wheelGeometries.contains(first);
        const bool secondIsWheel = simulation->wheelGeometries.contains(second);
        const bool isWheelStaticContact =
            (firstIsWheel && !secondHasBody) || (secondIsWheel && !firstHasBody);
        dContact contacts[4] = {};
        int count = 0;
        {
            openbus::rendering::TraceScope narrowphase(
                "physics", "BusSimulation::nearCallback.dCollide");
            count = dCollide(first, second, 4, &contacts[0].geom, sizeof(dContact));
        }
        if (profileRecording) {
            ++simulation->collisionProfileCounters.narrowphaseCalls;
            simulation->collisionProfileCounters.contacts += static_cast<std::uint64_t>(count);
        }
        {
            openbus::rendering::TraceScope contactCreation(
                "physics", "BusSimulation::nearCallback.createContactJoints");
            for (int index = 0; index < count; ++index) {
                if (firstIsPole && secondHasBody && !secondIsPole) {
                    world.recordMapPoleContact(first);
                } else if (secondIsPole && firstHasBody && !firstIsPole) {
                    world.recordMapPoleContact(second);
                }
                if (firstIsPole && dGeomGetBody(second) == simulation->chassis) {
                    world.recordMapPoleImpact(first, simulation->chassis, contacts[index].geom);
                } else if (secondIsPole && dGeomGetBody(first) == simulation->chassis) {
                    world.recordMapPoleImpact(second, simulation->chassis, contacts[index].geom);
                }
                contacts[index].surface.mode =
                    dContactSoftERP | dContactSoftCFM | dContactApprox1;
                contacts[index].surface.mu = ROAD_FRICTION_COEFFICIENT;
                contacts[index].surface.soft_erp =
                    isWheelStaticContact ? simulation->wheelStaticContact.erp : 0.8;
                contacts[index].surface.soft_cfm =
                    isWheelStaticContact ? simulation->wheelStaticContact.cfm : 1e-6;
                const dJointID contact = dJointCreateContact(
                    world.physicsWorld(), world.contactGroup(), &contacts[index]);
                dJointAttach(contact, dGeomGetBody(first), dGeomGetBody(second));
                if (profileRecording) {
                    ++simulation->collisionProfileCounters.contactJoints;
                }
            }
        }
    }

    void logCollisionProfileIfDue() {
        const bool profileRecording = openbus::rendering::IsRecording();
        if (!profileRecording) {
            collisionProfileCounters = {};
            collisionProfileWindowStarted = false;
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (!collisionProfileWindowStarted) {
            collisionProfileWindowStart = now;
            collisionProfileWindowStarted = true;
            return;
        }
        const double elapsedSeconds =
            std::chrono::duration<double>(now - collisionProfileWindowStart).count();
        if (elapsedSeconds < 2.0) {
            return;
        }
        simulationLog.Log(
            LogLevel::Info,
            "Physics collision profile (" + std::to_string(elapsedSeconds) +
                " s): callback pairs=" + std::to_string(collisionProfileCounters.callbackPairs) +
                ", filtered pairs=" + std::to_string(collisionProfileCounters.filteredPairs) +
                ", dCollide calls=" +
                std::to_string(collisionProfileCounters.narrowphaseCalls) +
                ", contacts=" + std::to_string(collisionProfileCounters.contacts) +
                ", contact joints=" +
                std::to_string(collisionProfileCounters.contactJoints));
        collisionProfileCounters = {};
        collisionProfileWindowStart = now;
    }

    Impl(World& simulationWorld, BusConfiguration vehicle, VehiclePlacement vehiclePlacement,
         double physicsHz, int catchUpSteps, WheelStaticContactSettings wheelContact)
        : world(simulationWorld), wheelStaticContact(wheelContact),
          configuration(std::move(vehicle)), placement(vehiclePlacement),
          maxCatchUpSteps(catchUpSteps) {
        openbus::rendering::TraceScope initializeTrace("physics", "BusSimulation::Impl.initialize");
        if (!std::isfinite(physicsHz) || physicsHz <= 0.0 || catchUpSteps <= 0) {
            simulationLog.Error("Invalid physics timing configuration");
            throw std::invalid_argument(
                "Physics rate must be finite and positive, and catch-up steps must be positive");
        }
        const double candidateFixedStep = 1.0 / physicsHz;
        if (!std::isfinite(candidateFixedStep) || candidateFixedStep <= 0.0) {
            simulationLog.Error("Physics rate produced an invalid fixed step");
            throw std::invalid_argument(
                "Physics rate is outside the representable fixed-step range");
        }
        fixedStep = candidateFixedStep;
        if (!std::isfinite(wheelStaticContact.erp) || wheelStaticContact.erp < 0.0 ||
            wheelStaticContact.erp > 1.0 || !std::isfinite(wheelStaticContact.cfm) ||
            wheelStaticContact.cfm < 0.0) {
            throw std::invalid_argument("Wheel-to-static contact settings are invalid");
        }
        if (configuration.axles.empty()) {
            simulationLog.Error("Bus configuration has no axles");
            throw std::invalid_argument("A bus must define at least one axle");
        }
        if (configuration.rollingResistanceForceNewtons.has_value() &&
            (!std::isfinite(*configuration.rollingResistanceForceNewtons) ||
             *configuration.rollingResistanceForceNewtons < 0.0)) {
            simulationLog.Error("Invalid rolling-resistance force");
            throw std::invalid_argument("Rolling-resistance force must be finite and non-negative");
        }
        if (configuration.inverseMinimumTurnRadius > 0.0) {
            const auto steeringAxle =
                std::find_if(configuration.axles.begin(), configuration.axles.end(),
                             [](const BusAxle& axle) { return axle.steerable; });
            if (steeringAxle != configuration.axles.end()) {
                double wheelbase = 0.0;
                for (const BusAxle& axle : configuration.axles) {
                    if (!axle.steerable) {
                        wheelbase =
                            std::max(wheelbase, std::abs(steeringAxle->position - axle.position));
                    }
                }
                if (wheelbase > 0.0) {
                    maxSteerAngle = std::atan(configuration.inverseMinimumTurnRadius * wheelbase);
                }
            }
        }
        for (const BusAxle& axle : configuration.axles) {
            if (axle.trackWidth <= 0.0 || std::abs(axle.position) > configuration.bodyHalfLength) {
                simulationLog.Error("Bus axle geometry is outside the chassis");
                throw std::invalid_argument("Bus axle geometry is outside the chassis");
            }
            const dReal wheelCenterOffset = axle.trackWidth * 0.5 - configuration.wheelHalfWidth;
            if (wheelCenterOffset <= 0.0) {
                simulationLog.Error("Bus axle width is too small for configured wheel width");
                throw std::invalid_argument("Bus axle width must exceed wheel width");
            }
            corners.push_back({axle.position, wheelCenterOffset, axle.wheelDiameter * 0.5});
            corners.push_back({axle.position, -wheelCenterOffset, axle.wheelDiameter * 0.5});
        }
        const dReal componentMass = corners.size() * (WHEEL_MASS + 2.0 * KNUCKLE_MASS);
        const dReal minimumMass = componentMass + MINIMUM_CHASSIS_MASS;
        if (configuration.mass < minimumMass) {
            simulationLog.Warning("Configured bus mass is below the physics minimum; using " +
                                  std::to_string(minimumMass) + " kg");
            configuration.mass = minimumMass;
        }
        openbus::rendering::TraceScope vehiclePhysicsTrace(
            "physics", "BusSimulation::Impl.initialize.vehiclePhysics");
        chassis = dBodyCreate(world.physicsWorld());
        dMass mass;
        const dReal chassisMass =
            configuration.mass - corners.size() * (WHEEL_MASS + 2.0 * KNUCKLE_MASS);
        if (chassisMass <= 0.0) {
            simulationLog.Error("Bus mass is too low for configured wheel and suspension masses");
            throw std::invalid_argument("Bus mass must exceed wheel and suspension masses");
        }
        dMassSetBoxTotal(&mass, chassisMass, configuration.collisionLength,
                         configuration.collisionWidth, configuration.collisionHeight);
        if (hasValidMomentOfInertia(configuration)) {
            const std::array<dReal, 3> configuredInertia = configuredOdeInertia(configuration);
            mass.I[0] = std::max(mass.I[0], configuredInertia[0]);
            mass.I[5] = std::max(mass.I[5], configuredInertia[1]);
            mass.I[10] = std::max(mass.I[10], configuredInertia[2]);
        }
        dBodySetMass(chassis, &mass);
        const dReal placementYaw = static_cast<dReal>(placement.yawDegrees * PI / 180.0);
        dMatrix3 placementRotation;
        dRFromAxisAndAngle(placementRotation, 0.0, 0.0, 1.0, placementYaw);
        dBodySetPosition(chassis, placement.position[0], placement.position[1],
                         placement.position[2] + configuration.centerOfGravityHeight);
        dBodySetRotation(chassis, placementRotation);
        if (configuration.collisionEnabled && configuration.hasCollisionMesh) {
            if (configuration.collisionMeshVertices.empty() ||
                configuration.collisionMeshVertices.size() % 3 != 0 ||
                configuration.collisionMeshIndices.empty() ||
                configuration.collisionMeshIndices.size() % 3 != 0) {
                throw std::invalid_argument("Bus collision mesh has invalid vertex/index buffers");
            }
            chassisCollisionMeshData = dGeomTriMeshDataCreate();
            if (!chassisCollisionMeshData) {
                throw std::runtime_error("Failed to create bus collision mesh data");
            }
            dGeomTriMeshDataBuildDouble(
                chassisCollisionMeshData, configuration.collisionMeshVertices.data(),
                3 * sizeof(double),
                checkedOdeCount(configuration.collisionMeshVertices.size() / 3, "vertex count"),
                configuration.collisionMeshIndices.data(),
                checkedOdeCount(configuration.collisionMeshIndices.size(), "index count"),
                3 * sizeof(int));
            chassisGeom = dCreateTriMesh(world.collisionSpace(), chassisCollisionMeshData, nullptr,
                                         nullptr, nullptr);
            if (!chassisGeom) {
                dGeomTriMeshDataDestroy(chassisCollisionMeshData);
                chassisCollisionMeshData = nullptr;
                throw std::runtime_error("Failed to create bus collision mesh geometry");
            }
            dGeomSetBody(chassisGeom, chassis);
            // Mesh vertices use the visible model's origin; the ODE body origin
            // is at the configured centre of gravity.
            dGeomSetOffsetPosition(chassisGeom, 0.0, 0.0, -configuration.centerOfGravityHeight);
        } else if (configuration.collisionEnabled) {
            chassisGeom = dCreateBox(world.collisionSpace(), configuration.collisionLength,
                                     configuration.collisionWidth, configuration.collisionHeight);
            dGeomSetBody(chassisGeom, chassis);
            dGeomSetOffsetPosition(chassisGeom, configuration.collisionOffsetX,
                                   configuration.collisionOffsetY, configuration.collisionOffsetZ);
        }
        dBodySetAutoDisableFlag(chassis, 0);
        dBodySetMaxAngularSpeed(chassis, MAX_CHASSIS_ANGULAR_SPEED);

        dMass knuckleMass;
        dMassSetSphereTotal(&knuckleMass, KNUCKLE_MASS, 0.1);
        for (std::size_t index = 0; index < corners.size(); ++index) {
            Corner& corner = corners[index];
            const std::size_t axleIndex = index / 2;
            dMass wheelMass;
            dMassSetCylinderTotal(&wheelMass, WHEEL_MASS, 3, corner.wheelRadius,
                                  configuration.wheelHalfWidth * 2.0);
            const dReal wheelLocalZ = suspensionAnchorZ(index) - SUSP_REST;
            corner.suspensionBody = dBodyCreate(world.physicsWorld());
            corner.steeringBody = dBodyCreate(world.physicsWorld());
            corner.wheelBody = dBodyCreate(world.physicsWorld());
            dBodySetMass(corner.suspensionBody, &knuckleMass);
            dBodySetMass(corner.steeringBody, &knuckleMass);
            dBodySetMass(corner.wheelBody, &wheelMass);
            dBodySetAutoDisableFlag(corner.suspensionBody, 0);
            dBodySetAutoDisableFlag(corner.steeringBody, 0);
            dBodySetAutoDisableFlag(corner.wheelBody, 0);
            dBodySetRotation(corner.suspensionBody, placementRotation);
            dBodySetRotation(corner.steeringBody, placementRotation);

            const dReal localX = configuration.axles[axleIndex].position;
            const dReal localY = corner.y;
            const dReal worldX = placement.position[0] + placementRotation[0] * localX +
                                 placementRotation[1] * localY;
            const dReal worldY = placement.position[1] + placementRotation[4] * localX +
                                 placementRotation[5] * localY;
            const dReal worldZ = dBodyGetPosition(chassis)[2] + wheelLocalZ;
            dBodySetPosition(corner.suspensionBody, worldX, worldY, worldZ);
            dBodySetPosition(corner.steeringBody, worldX, worldY, worldZ);
            dBodySetPosition(corner.wheelBody, worldX, worldY, worldZ);

            dMatrix3 localWheelRotation;
            dRFromAxisAndAngle(localWheelRotation, 1.0, 0.0, 0.0, -PI * 0.5);
            dMatrix3 wheelRotation = {};
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    for (int inner = 0; inner < 3; ++inner) {
                        wheelRotation[row * 4 + column] += placementRotation[row * 4 + inner] *
                                                           localWheelRotation[inner * 4 + column];
                    }
                }
            }
            dBodySetRotation(corner.wheelBody, wheelRotation);

            corner.wheelGeom = dCreateCylinder(world.collisionSpace(), corner.wheelRadius,
                                               configuration.wheelHalfWidth * 2.0);
            dGeomSetBody(corner.wheelGeom, corner.wheelBody);
            wheelGeometries.insert(corner.wheelGeom);

            corner.suspensionJoint = dJointCreateSlider(world.physicsWorld(), nullptr);
            dJointAttach(corner.suspensionJoint, chassis, corner.suspensionBody);
            dJointSetSliderAxis(corner.suspensionJoint, 0.0, 0.0, 1.0);
            const dReal sliderCenter = dJointGetSliderPosition(corner.suspensionJoint);
            corner.suspensionSliderZero = sliderCenter;
            dJointSetSliderParam(corner.suspensionJoint, dParamLoStop,
                                 sliderCenter - SUSP_MAX_TRAVEL);
            dJointSetSliderParam(corner.suspensionJoint, dParamHiStop,
                                 sliderCenter + SUSP_MAX_TRAVEL);

            corner.steeringJoint = dJointCreateHinge(world.physicsWorld(), nullptr);
            dJointAttach(corner.steeringJoint, corner.suspensionBody, corner.steeringBody);
            dJointSetHingeAnchor(corner.steeringJoint, worldX, worldY, worldZ);
            dJointSetHingeAxis(corner.steeringJoint, 0.0, 0.0, 1.0);
            const bool steerable = configuration.axles[axleIndex].steerable;
            dJointSetHingeParam(corner.steeringJoint, dParamLoStop,
                                steerable ? -maxSteerAngle : 0.0);
            dJointSetHingeParam(corner.steeringJoint, dParamHiStop,
                                steerable ? maxSteerAngle : 0.0);

            corner.wheelJoint = dJointCreateHinge(world.physicsWorld(), nullptr);
            dJointAttach(corner.wheelJoint, corner.steeringBody, corner.wheelBody);
            dJointSetHingeAnchor(corner.wheelJoint, worldX, worldY, worldZ);
            dJointSetHingeAxis(corner.wheelJoint, placementRotation[1], placementRotation[5], 0.0);
        }
        simulationLog.Log("Initialized ODE bus with " + std::to_string(configuration.axles.size()) +
                          " axles and " + std::to_string(corners.size()) + " wheels");
    }

    void refreshWheelTelemetry() {
        openbus::rendering::TraceScope trace("physics", "BusSimulation::refreshWheelTelemetry");
        const dReal* chassisPosition = dBodyGetPosition(chassis);
        const dReal* chassisRotation = dBodyGetRotation(chassis);
        const dReal* chassisVelocity = dBodyGetLinearVel(chassis);
        const dReal* chassisAngularVelocity = dBodyGetAngularVel(chassis);
        for (std::size_t index = 0; index < corners.size(); ++index) {
            Corner& corner = corners[index];
            const dReal localZ = suspensionAnchorZ(index);
            const dReal* wheelPosition = dBodyGetPosition(corner.wheelBody);
            const dReal wheelOffsetX = wheelPosition[0] - chassisPosition[0];
            const dReal wheelOffsetY = wheelPosition[1] - chassisPosition[1];
            const dReal wheelOffsetZ = wheelPosition[2] - chassisPosition[2];
            const dReal wheelLocalZ = chassisRotation[2] * wheelOffsetX +
                                      chassisRotation[6] * wheelOffsetY +
                                      chassisRotation[10] * wheelOffsetZ;
            const dReal anchorLocalZ = localZ;
            const dReal* wheelVelocity = dBodyGetLinearVel(corner.wheelBody);
            const dReal wheelLocalVelocityZ = chassisRotation[2] * wheelVelocity[0] +
                                              chassisRotation[6] * wheelVelocity[1] +
                                              chassisRotation[10] * wheelVelocity[2];
            const dReal localAngularX = chassisRotation[0] * chassisAngularVelocity[0] +
                                        chassisRotation[4] * chassisAngularVelocity[1] +
                                        chassisRotation[8] * chassisAngularVelocity[2];
            const dReal localAngularY = chassisRotation[1] * chassisAngularVelocity[0] +
                                        chassisRotation[5] * chassisAngularVelocity[1] +
                                        chassisRotation[9] * chassisAngularVelocity[2];
            const dReal anchorLocalVelocityZ = chassisRotation[2] * chassisVelocity[0] +
                                               chassisRotation[6] * chassisVelocity[1] +
                                               chassisRotation[10] * chassisVelocity[2] +
                                               localAngularX * corner.y - localAngularY * corner.x;
            const dReal projectedCompression = SUSP_REST - (anchorLocalZ - wheelLocalZ);
            corner.springCompression = std::clamp(projectedCompression, 0.0, SUSP_MAX_TRAVEL);
            corner.verticalVelocity = wheelLocalVelocityZ - anchorLocalVelocityZ;
            // The authored wheel CFG's origin_rot_y -90 frame turns positive
            // anim_trans into downward render-Z, opposite ODE's positive
            // compression/upward wheel displacement.
            corner.animationSuspension =
                std::isfinite(projectedCompression)
                    ? std::clamp(-projectedCompression, -SUSP_MAX_TRAVEL, SUSP_MAX_TRAVEL)
                    : 0.0;
            const dReal wheelHingeAngle = dJointGetHingeAngle(corner.wheelJoint);
            corner.wheelRotation -=
                std::remainder(wheelHingeAngle - corner.previousWheelHingeAngle, 2.0 * PI);
            corner.previousWheelHingeAngle = wheelHingeAngle;
            // Wheel telemetry follows the same forward-positive sign as wheelRotation.
            corner.wheelOmega = -dJointGetHingeAngleRate(corner.wheelJoint);
            corner.wheelSurfaceSpeed = corner.wheelOmega * corner.wheelRadius;

            const dReal* steeringRotation = dBodyGetRotation(corner.steeringBody);
            const dReal forwardX = steeringRotation[0];
            const dReal forwardY = steeringRotation[4];
            const dReal forwardZ = steeringRotation[8];
            const dReal lateralX = steeringRotation[1];
            const dReal lateralY = steeringRotation[5];
            const dReal lateralZ = steeringRotation[9];
            corner.pointLongitudinalSpeed = wheelVelocity[0] * forwardX +
                                            wheelVelocity[1] * forwardY +
                                            wheelVelocity[2] * forwardZ;
            const dReal lateralSpeed = wheelVelocity[0] * lateralX + wheelVelocity[1] * lateralY +
                                       wheelVelocity[2] * lateralZ;
            const dReal slipDenominator = std::max(
                {1.0, std::abs(corner.pointLongitudinalSpeed), std::abs(corner.wheelSurfaceSpeed)});
            corner.longitudinalSlipRatio =
                (corner.wheelSurfaceSpeed - corner.pointLongitudinalSpeed) / slipDenominator;
            const dReal forceLimit = std::min(TIRE_GRIP_LONG, corner.normalLoad);
            corner.frictionUtilization =
                forceLimit > 0.0 ? std::min(std::abs(corner.longitudinalForce) / forceLimit, 1.0)
                                 : 0.0;
            corner.wheelspin = std::abs(corner.longitudinalSlipRatio) > WHEELSPIN_SLIP_RATIO &&
                               std::abs(corner.driveTorque) > 1.0;
            corner.skid = std::abs(corner.longitudinalSlipRatio) > WHEELSPIN_SLIP_RATIO ||
                          std::abs(lateralSpeed) > 0.75;
        }
    }

    void applyCornerForces(double driveCommand, double steering,
                           const std::vector<double>& wheelBrakeForces,
                           const std::vector<double>& wheelSpringFactors, bool scriptDriven) {
        openbus::rendering::TraceScope trace("physics", "BusSimulation::applyCornerForces");
        refreshWheelTelemetry();
        const dReal* velocity = dBodyGetLinearVel(chassis);
        const dReal* rotation = dBodyGetRotation(chassis);
        const dReal longitudinalSpeed = rotation[0] * velocity[0] + rotation[4] * velocity[1];
        const dReal lateralSpeed = rotation[1] * velocity[0] + rotation[5] * velocity[1];
        const dReal* angularVelocity = dBodyGetAngularVel(chassis);
        const dReal yawRate = rotation[2] * angularVelocity[0] + rotation[6] * angularVelocity[1] +
                              rotation[10] * angularVelocity[2];
        const dReal steerTarget =
            std::clamp(static_cast<dReal>(steering), -1.0, 1.0) * maxSteerAngle;
        const dReal maxSteerDelta = STEER_SPEED * static_cast<dReal>(fixedStep);
        steeringAngle += std::clamp(steerTarget - steeringAngle, -maxSteerDelta, maxSteerDelta);

        dReal driveTorque = static_cast<dReal>(driveCommand);
        if (!scriptDriven) {
            const dReal engineWheelRadius = [&] {
                for (std::size_t axleIndex = 0; axleIndex < configuration.axles.size();
                     ++axleIndex) {
                    if (configuration.axles[axleIndex].driven) {
                        return corners[axleIndex * 2].wheelRadius;
                    }
                }
                return corners.front().wheelRadius;
            }();
            const dReal wheelRpm =
                std::abs(longitudinalSpeed) / (2.0 * PI * engineWheelRadius) * 60.0;
            const dReal ratio = GEAR_RATIOS[currentGear] * FINAL_DRIVE;
            const dReal engineRpm = std::clamp(wheelRpm * ratio, ENGINE_IDLE_RPM, ENGINE_REDLINE);
            if (shiftTimer <= 0.0 && engineRpm > UPSHIFT_RPM && currentGear + 1 < NUM_GEARS) {
                ++currentGear;
                shiftTimer = SHIFT_LOCKOUT;
                simulationLog.Debug("Upshifted to gear " + std::to_string(currentGear + 1));
            } else if (shiftTimer <= 0.0 && engineRpm < DOWNSHIFT_RPM && currentGear > 0) {
                --currentGear;
                shiftTimer = SHIFT_LOCKOUT;
                simulationLog.Debug("Downshifted to gear " + std::to_string(currentGear + 1));
            }
            shiftTimer = std::max(0.0, shiftTimer - fixedStep);

            const dReal torqueFactor = std::clamp(1.0 - std::abs(engineRpm - ENGINE_RATED_RPM) /
                                                            (ENGINE_REDLINE - ENGINE_IDLE_RPM),
                                                  0.35, 1.0);
            const dReal engineTorque = PEAK_TORQUE * torqueFactor;
            const dReal throttleValue = std::clamp(static_cast<dReal>(driveCommand), -1.0, 1.0);
            driveTorque = throttleValue * engineTorque * ratio * DRIVETRAIN_EFF;
        }
        bool brakesApplied = false;
        for (const double force : wheelBrakeForces) {
            brakesApplied = brakesApplied || (std::isfinite(force) && force > 1.0);
        }
        if (brakesApplied) {
            driveTorque = 0.0;
        }
        std::size_t drivenAxles = 0;
        for (const BusAxle& axle : configuration.axles) {
            drivenAxles += axle.driven ? 1U : 0U;
        }
        std::vector<dReal> compression(corners.size());
        for (std::size_t index = 0; index < corners.size(); ++index) {
            compression[index] = corners[index].springCompression;
        }

        for (std::size_t index = 0; index < corners.size(); ++index) {
            const std::size_t axleIndex = index / 2;
            const bool front = configuration.axles[axleIndex].steerable;
            const dReal antiRoll = axleIndex == 0 ? ARB_STIFFNESS_FRONT : ARB_STIFFNESS_REAR;
            const BusAxle& axle = configuration.axles[axleIndex];
            const dReal configuredSpringFactor = index < wheelSpringFactors.size()
                                                     ? static_cast<dReal>(wheelSpringFactors[index])
                                                     : 1.0;
            const dReal springFactor = std::isfinite(configuredSpringFactor)
                                           ? std::clamp(configuredSpringFactor, 0.0, 10.0)
                                           : 1.0;
            const dReal springForce = compression[index] * axle.springRate * springFactor +
                                      corners[index].verticalVelocity * axle.damperRate;
            const dReal antiRollForce = antiRoll * (compression[index ^ 1] - compression[index]);
            const dReal supportForce = std::max(0.0, springForce + antiRollForce);
            dBodyAddForce(corners[index].suspensionBody, 0.0, 0.0, -supportForce);
            dBodyAddForceAtRelPos(chassis, 0.0, 0.0, supportForce, corners[index].x,
                                  corners[index].y, suspensionAnchorZ(index));
            corners[index].normalLoad = supportForce;

            const dReal targetSteering = front ? steeringAngle : 0.0;
            const dReal currentSteering = dJointGetHingeAngle(corners[index].steeringJoint);
            const dReal steeringRate = dJointGetHingeAngleRate(corners[index].steeringJoint);
            dJointSetHingeParam(
                corners[index].steeringJoint, dParamVel,
                std::clamp((targetSteering - currentSteering) * STEERING_POSITION_GAIN -
                               steeringRate * STEERING_RATE_DAMPING,
                           -STEERING_MAX_RATE, STEERING_MAX_RATE));
            dJointSetHingeParam(corners[index].steeringJoint, dParamFMax, STEERING_MAX_TORQUE);

            const dReal requestedDriveTorquePerWheel =
                configuration.axles[axleIndex].driven && drivenAxles > 0
                    ? driveTorque / (2.0 * drivenAxles)
                    : 0.0;
            const dReal staticWheelLoad =
                configuration.mass * std::abs(GRAVITY) / static_cast<dReal>(corners.size());
            const dReal effectiveNormalLoad = std::max(supportForce, staticWheelLoad);
            const dReal tireForceLimit =
                std::min(TIRE_GRIP_LONG, effectiveNormalLoad) * ROAD_FRICTION_COEFFICIENT;
            const dReal driveTorqueLimit = tireForceLimit * corners[index].wheelRadius;
            const dReal driveTorquePerWheel =
                std::copysign(std::min(std::abs(requestedDriveTorquePerWheel), driveTorqueLimit),
                              requestedDriveTorquePerWheel);
            const dReal wheelBrakeForce =
                index < wheelBrakeForces.size() && std::isfinite(wheelBrakeForces[index])
                    ? std::max(0.0, static_cast<dReal>(wheelBrakeForces[index]))
                    : 0.0;
            const dReal brakeCapacity = wheelBrakeForce * corners[index].wheelRadius;
            const dReal stoppingTorque = 0.5 * WHEEL_MASS * corners[index].wheelRadius *
                                         corners[index].wheelRadius *
                                         std::abs(corners[index].wheelOmega) / fixedStep;
            const dReal brakeTorque = std::abs(corners[index].wheelOmega) > 1e-6
                                          ? -std::copysign(std::min(brakeCapacity, stoppingTorque),
                                                           corners[index].wheelOmega)
                                          : 0.0;
            // A bounded zero-speed hinge motor can hold the wheel at rest; a
            // speed-scaled torque would vanish precisely when the bus stops.
            dJointSetHingeParam(corners[index].wheelJoint, dParamVel, 0.0);
            dJointSetHingeParam(corners[index].wheelJoint, dParamFMax, brakeCapacity);
            corners[index].driveTorque = driveTorquePerWheel;
            corners[index].brakeTorque = brakeTorque;
            corners[index].longitudinalForce =
                (driveTorquePerWheel + brakeTorque) / corners[index].wheelRadius;
            corners[index].lateralForce = 0.0;
            dBodyAddRelTorque(corners[index].wheelBody, 0.0, 0.0, driveTorquePerWheel);

            if (configuration.axles[axleIndex].driven) {
                const dReal tractionForce = driveTorquePerWheel / corners[index].wheelRadius;
                dBodyAddRelForce(chassis, tractionForce, 0.0, 0.0);
            }
        }

        const dReal drag = -0.5 * AIR_DENSITY * AERO_CD * AERO_FRONT_AREA * longitudinalSpeed *
                           std::abs(longitudinalSpeed);
        const dReal rollingMagnitude =
            configuration.rollingResistanceForceNewtons.has_value()
                ? static_cast<dReal>(*configuration.rollingResistanceForceNewtons)
                : LEGACY_ROLLING_RESIST_COEFFICIENT * configuration.mass * std::abs(GRAVITY);
        const dReal rolling =
            std::abs(longitudinalSpeed) > 0.01
                ? -std::min(rollingMagnitude,
                            configuration.mass * std::abs(longitudinalSpeed) / fixedStep) *
                      (longitudinalSpeed > 0.0 ? 1.0 : -1.0)
                : 0.0;
        dBodyAddRelForce(chassis, drag + rolling, 0.0, 0.0);
        dBodyAddRelTorque(chassis, 0.0, 0.0, -yawRate * YAW_DAMPING);
        if (brakesApplied && std::abs(longitudinalSpeed) < 1.0) {
            dBodyAddRelForce(chassis, 0.0, -lateralSpeed * STOP_LATERAL_DAMPING, 0.0);
            dBodyAddRelTorque(chassis, 0.0, 0.0, -yawRate * STOP_YAW_DAMPING);
        }

        const dReal* chassisRotation = dBodyGetRotation(chassis);
        const dReal* chassisAngularVelocity = dBodyGetAngularVel(chassis);
        const dReal rollAngle = std::atan2(chassisRotation[9], chassisRotation[10]);
        const dReal pitchAngle =
            std::atan2(chassisRotation[2], std::sqrt(chassisRotation[0] * chassisRotation[0] +
                                                     chassisRotation[1] * chassisRotation[1]));
        const dReal localAngularX = chassisRotation[0] * chassisAngularVelocity[0] +
                                    chassisRotation[4] * chassisAngularVelocity[1] +
                                    chassisRotation[8] * chassisAngularVelocity[2];
        const dReal localAngularY = chassisRotation[1] * chassisAngularVelocity[0] +
                                    chassisRotation[5] * chassisAngularVelocity[1] +
                                    chassisRotation[9] * chassisAngularVelocity[2];
        dBodyAddRelTorque(chassis,
                          -std::clamp(rollAngle, -0.6, 0.6) * BODY_ROLL_STIFFNESS -
                              localAngularX * BODY_ATTITUDE_DAMPING,
                          -std::clamp(pitchAngle, -0.6, 0.6) * BODY_PITCH_STIFFNESS -
                              localAngularY * BODY_ATTITUDE_DAMPING,
                          0.0);
    }
    void fixedUpdate(double driveCommand, double steering,
                     const std::vector<double>& wheelBrakeForces,
                     const std::vector<double>& wheelSpringFactors, bool scriptDriven) {
        openbus::rendering::TraceScope trace("physics", "BusSimulation::fixedUpdate");
        if (!frozen) {
            applyCornerForces(driveCommand, steering, wheelBrakeForces, wheelSpringFactors,
                              scriptDriven);
            world.collideAndStep(fixedStep, this, &nearCallback);
            refreshWheelTelemetry();
        }
        logCollisionProfileIfDue();
        simulationTime += fixedStep;
    }

    void update(double elapsedSeconds, double driveCommand, double steering,
                const std::vector<double>& wheelBrakeForces,
                const std::vector<double>& wheelSpringFactors, bool scriptDriven) {
        openbus::rendering::TraceScope trace("physics", "BusSimulation::update");
        accumulator += std::clamp(elapsedSeconds, 0.0, MAX_FRAME_SECONDS);
        lastSteps = 0;
        dropped = false;
        while (accumulator >= fixedStep && lastSteps < maxCatchUpSteps) {
            fixedUpdate(driveCommand, steering, wheelBrakeForces, wheelSpringFactors, scriptDriven);
            accumulator -= fixedStep;
            ++lastSteps;
        }
        if (accumulator >= fixedStep) {
            dropped = true;
            if (!dropReported) {
                simulationLog.Warning(
                    "Dropped accumulated simulation time after exceeding catch-up "
                    "limit. Dropped seconds: " +
                    std::to_string(elapsedSeconds));
                dropReported = true;
            }
            accumulator = std::fmod(accumulator, fixedStep);
        } else {
            dropReported = false;
        }
    }
};

BusSimulation::BusSimulation(World& world, BusConfiguration configuration,
                             VehiclePlacement placement, double physicsHz, int maxCatchUpSteps,
                             WheelStaticContactSettings wheelStaticContact)
    : impl_(std::make_unique<Impl>(world, std::move(configuration), placement, physicsHz,
                                   maxCatchUpSteps, wheelStaticContact)) {
    simulationLog.Log("Bus simulation started at " + std::to_string(physicsHz) + " Hz");
}

BusSimulation::~BusSimulation() {
    simulationLog.Log("Bus simulation stopped at " + std::to_string(impl_->simulationTime) +
                      " seconds");
}

void BusSimulation::update(double elapsedSeconds, double throttle, double steering, double brake) {
    const double forcePerWheel = std::clamp(brake, 0.0, 1.0) * MAX_BRAKE_FORCE * 0.5;
    impl_->update(elapsedSeconds, throttle, steering,
                  std::vector<double>(impl_->corners.size(), forcePerWheel), {}, false);
}

void BusSimulation::updateWithWheelTorque(double elapsedSeconds, double wheelTorque,
                                          double steering, double brake) {
    const double forcePerWheel = std::clamp(brake, 0.0, 1.0) * MAX_BRAKE_FORCE * 0.5;
    impl_->update(elapsedSeconds, wheelTorque, steering,
                  std::vector<double>(impl_->corners.size(), forcePerWheel), {}, true);
}

void BusSimulation::updateWithWheelTorqueAndBrakeForces(
    double elapsedSeconds, double wheelTorque, double steering,
    const std::vector<double>& wheelBrakeForces, const std::vector<double>& wheelSpringFactors) {
    impl_->update(elapsedSeconds, wheelTorque, steering, wheelBrakeForces, wheelSpringFactors,
                  true);
}

void BusSimulation::step(double throttle, double steering, double brake) {
    const double forcePerWheel = std::clamp(brake, 0.0, 1.0) * MAX_BRAKE_FORCE * 0.5;
    impl_->fixedUpdate(throttle, steering,
                       std::vector<double>(impl_->corners.size(), forcePerWheel), {}, false);
}

void BusSimulation::stepWithWheelTorque(double wheelTorque, double steering, double brake) {
    const double forcePerWheel = std::clamp(brake, 0.0, 1.0) * MAX_BRAKE_FORCE * 0.5;
    impl_->fixedUpdate(wheelTorque, steering,
                       std::vector<double>(impl_->corners.size(), forcePerWheel), {}, true);
}

void BusSimulation::stepWithWheelTorqueAndBrakeForces(
    double wheelTorque, double steering, const std::vector<double>& wheelBrakeForces,
    const std::vector<double>& wheelSpringFactors) {
    impl_->fixedUpdate(wheelTorque, steering, wheelBrakeForces, wheelSpringFactors, true);
}

void BusSimulation::setFrozen(bool frozen) {
    if (impl_->frozen == frozen) {
        return;
    }
    impl_->frozen = frozen;
    impl_->accumulator = 0.0;
}

void BusSimulation::updateVariables(openbus::scripting::Vehicle& variables, double throttle,
                                    double steering, double brake) const {
    variables.set("throttle", std::clamp(throttle, -1.0, 1.0));
    variables.set("brake", std::clamp(brake, 0.0, 1.0));
    // OMSI vehicle scripts expect velocity in km/h, while ODE reports m/s.
    const double speedKmh = speed() * 3.6;
    variables.set("velocity", speedKmh);
    variables.set("velocity_ground", speedKmh);
    variables.set("steering", std::clamp(steering, -1.0, 1.0));
    variables.set("steeringangle", steeringAngle());
    // OMSI scripts expect wheel rotation speeds in rpm; ODE reports rad/s.
    constexpr double radiansPerSecondToRpm = 60.0 / (2.0 * PI);
    double drivenWheelSpeed = 0.0;
    std::size_t drivenWheelCount = 0;
    for (std::size_t index = 0; index < impl_->corners.size(); ++index) {
        const std::size_t axleIndex = index / 2;
        const char* side = index % 2 == 0 ? "l" : "r";
        const std::string prefix = "_" + std::to_string(axleIndex) + "_" + side;
        variables.set("wheel_rotation" + prefix, impl_->corners[index].wheelRotation);
        const double wheelSpeedRpm = impl_->corners[index].wheelOmega * radiansPerSecondToRpm;
        variables.set("wheel_rotationspeed" + prefix, wheelSpeedRpm);
        variables.set("axle_suspension" + prefix, impl_->corners[index].animationSuspension);
        variables.set("axle_steering_" + std::to_string(axleIndex) + "_" + side,
                      dJointGetHingeAngle(impl_->corners[index].steeringJoint));
        if (impl_->configuration.axles[axleIndex].driven) {
            drivenWheelSpeed += wheelSpeedRpm;
            ++drivenWheelCount;
        }
    }
    variables.set("n_wheel", drivenWheelCount == 0
                                 ? 0.0
                                 : drivenWheelSpeed / static_cast<double>(drivenWheelCount));
}

double BusSimulation::positionX() const {
    return dBodyGetPosition(impl_->chassis)[0];
}

void BusSimulation::translateVertically(double meters) {
    if (!std::isfinite(meters)) {
        throw std::invalid_argument("Vertical translation must be finite");
    }
    const auto translateBody = [meters](dBodyID body) {
        const dReal* position = dBodyGetPosition(body);
        const dReal x = position[0];
        const dReal y = position[1];
        const dReal z = position[2] + static_cast<dReal>(meters);
        dBodySetPosition(body, x, y, z);
    };
    translateBody(impl_->chassis);
    for (const Impl::Corner& corner : impl_->corners) {
        translateBody(corner.suspensionBody);
        translateBody(corner.steeringBody);
        translateBody(corner.wheelBody);
    }
}

double BusSimulation::positionY() const {
    return dBodyGetPosition(impl_->chassis)[1];
}

double BusSimulation::positionZ() const {
    return dBodyGetPosition(impl_->chassis)[2];
}

double BusSimulation::yaw() const {
    const dReal* bodyRotation = dBodyGetRotation(impl_->chassis);
    return std::atan2(bodyRotation[4], bodyRotation[0]);
}

double BusSimulation::steeringAngle() const {
    return impl_->steeringAngle;
}

std::array<double, 3> BusSimulation::centerOfGravity() const {
    const dReal chassisMass =
        impl_->configuration.mass - impl_->corners.size() * (WHEEL_MASS + 2.0 * KNUCKLE_MASS);
    std::array<dReal, 3> center = {0.0, 0.0, 0.0};
    auto addBody = [&](dBodyID body, dReal bodyMass) {
        const dReal* position = dBodyGetPosition(body);
        center[0] += position[0] * bodyMass;
        center[1] += position[1] * bodyMass;
        center[2] += position[2] * bodyMass;
    };
    addBody(impl_->chassis, chassisMass);
    for (const auto& corner : impl_->corners) {
        addBody(corner.suspensionBody, KNUCKLE_MASS);
        addBody(corner.steeringBody, KNUCKLE_MASS);
        addBody(corner.wheelBody, WHEEL_MASS);
    }
    const dReal inverseMass = 1.0 / impl_->configuration.mass;
    return {center[0] * inverseMass, center[1] * inverseMass, center[2] * inverseMass};
}

std::array<double, 3> BusSimulation::outsideCameraCenter() const {
    if (impl_->configuration.hasOutsideCameraCenter) {
        return impl_->configuration.outsideCameraCenter;
    }
    return {impl_->configuration.collisionOffsetX, impl_->configuration.collisionOffsetY,
            impl_->configuration.collisionOffsetZ};
}

BodyPose BusSimulation::chassisPose() const {
    const dReal* position = dBodyGetPosition(impl_->chassis);
    const dReal* rotation = dBodyGetRotation(impl_->chassis);
    return {{position[0], position[1], position[2]},
            {rotation[0], rotation[1], rotation[2], rotation[4], rotation[5], rotation[6],
             rotation[8], rotation[9], rotation[10]}};
}

ChassisCollisionBox BusSimulation::chassisCollisionBox() const {
    return {impl_->configuration.collisionLength,
            impl_->configuration.collisionWidth,
            impl_->configuration.collisionHeight,
            impl_->configuration.collisionOffsetX,
            impl_->configuration.collisionOffsetY,
            impl_->configuration.collisionOffsetZ,
            impl_->configuration.collisionEnabled,
            impl_->configuration.hasCollisionMesh && impl_->configuration.collisionEnabled};
}

BodyPose BusSimulation::wheelPose(std::size_t index) const {
    if (index >= impl_->corners.size()) {
        throw std::out_of_range("Wheel index is outside the bus configuration");
    }
    const dReal* position = dBodyGetPosition(impl_->corners[index].wheelBody);
    const dReal* rotation = dBodyGetRotation(impl_->corners[index].wheelBody);
    return {{position[0], position[1], position[2]},
            {rotation[0], rotation[1], rotation[2], rotation[4], rotation[5], rotation[6],
             rotation[8], rotation[9], rotation[10]}};
}

BodyPose BusSimulation::wheelMountPose(std::size_t index) const {
    if (index >= impl_->corners.size()) {
        throw std::out_of_range("Wheel index is outside the bus configuration");
    }
    const dReal* position = dBodyGetPosition(impl_->corners[index].wheelBody);
    const dReal* chassisRotation = dBodyGetRotation(impl_->chassis);
    // Wheel animation owns steering and rolling. ODE supplies only the physical
    // corner position, while the mesh keeps the chassis basis and fixed wheel basis.
    constexpr dReal fixedWheelRotation[3][3] = {
        {1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0},
        {0.0, -1.0, 0.0},
    };
    dReal rotation[3][3] = {};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            for (int inner = 0; inner < 3; ++inner) {
                rotation[row][column] +=
                    chassisRotation[row * 4 + inner] * fixedWheelRotation[inner][column];
            }
        }
    }
    return {{position[0], position[1], position[2]},
            {rotation[0][0], rotation[0][1], rotation[0][2], rotation[1][0], rotation[1][1],
             rotation[1][2], rotation[2][0], rotation[2][1], rotation[2][2]}};
}

double BusSimulation::wheelSteeringAngle(std::size_t index) const {
    if (index >= impl_->corners.size()) {
        throw std::out_of_range("Wheel index is outside the bus configuration");
    }
    return dJointGetHingeAngle(impl_->corners[index].steeringJoint);
}

double BusSimulation::wheelRotation(std::size_t index) const {
    if (index >= impl_->corners.size()) {
        throw std::out_of_range("Wheel index is outside the bus configuration");
    }
    return impl_->corners[index].wheelRotation;
}

double BusSimulation::wheelSuspensionCompression(std::size_t index) const {
    if (index >= impl_->corners.size()) {
        throw std::out_of_range("Wheel index is outside the bus configuration");
    }
    return impl_->corners[index].springCompression;
}

double BusSimulation::wheelSuspensionSliderTravel(std::size_t index) const {
    if (index >= impl_->corners.size()) {
        throw std::out_of_range("Wheel index is outside the bus configuration");
    }
    const auto& corner = impl_->corners[index];
    return corner.suspensionSliderZero - dJointGetSliderPosition(corner.suspensionJoint);
}

double BusSimulation::wheelRadius() const {
    return impl_->configuration.wheelRadius;
}

double BusSimulation::wheelRadius(std::size_t index) const {
    if (index >= impl_->corners.size()) {
        throw std::out_of_range("Wheel index is outside the bus configuration");
    }
    return impl_->corners[index].wheelRadius;
}

double BusSimulation::wheelHalfWidth() const {
    return impl_->configuration.wheelHalfWidth;
}

double BusSimulation::speed() const {
    const dReal* velocity = dBodyGetLinearVel(impl_->chassis);
    return std::sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1]);
}

double BusSimulation::physicsHz() const {
    return 1.0 / impl_->fixedStep;
}

int BusSimulation::lastStepCount() const {
    return impl_->lastSteps;
}

bool BusSimulation::droppedTime() const {
    return impl_->dropped;
}

std::size_t BusSimulation::axleCount() const {
    return impl_->configuration.axles.size();
}

std::size_t BusSimulation::wheelCount() const {
    return impl_->corners.size();
}

const std::vector<RoadBump>& BusSimulation::roadBumps() const {
    return defaultRoadBumps();
}

BusAxle BusSimulation::axle(std::size_t index) const {
    if (index >= impl_->configuration.axles.size()) {
        throw std::out_of_range("Axle index is outside the bus configuration");
    }
    return impl_->configuration.axles[index];
}

double BusSimulation::wheelLocalZ(std::size_t index) const {
    if (index >= impl_->corners.size()) {
        throw std::out_of_range("Wheel index is outside the bus configuration");
    }
    return impl_->corners[index].wheelRadius - impl_->configuration.centerOfGravityHeight -
           SUSP_REST + impl_->corners[index].springCompression;
}

double BusSimulation::simulationTime() const {
    return impl_->simulationTime;
}