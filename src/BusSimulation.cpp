#include "BusSimulation.h"

#include "BusConfiguration.h"
#include "Logger.h"
#include "PerfTrace.h"
#include "RoadFeatures.h"
#include "Variables.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <ode/ode.h>
#include <stdexcept>
#include <string>
#include <utility>

namespace {
Logger simulationLog("Simulation");

constexpr dReal GRAVITY = -9.81;

constexpr dReal SUSP_REST = 0.35;
constexpr dReal SUSP_MAX_TRAVEL = 0.25;
constexpr dReal SUSP_SPRING_K = 250000.0;
constexpr dReal SUSP_DAMPER_C = 16000.0;
constexpr dReal TIRE_GRIP_LONG = 24000.0;
constexpr dReal ROLLING_RESIST = 0.012;
constexpr dReal WHEEL_MASS = 180.0;
constexpr dReal KNUCKLE_MASS = 40.0;
constexpr dReal ROAD_FRICTION_COEFFICIENT = 0.85;
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
constexpr dReal STEER_SPEED = 0.65;
constexpr dReal STEERING_MAX_TORQUE = 3000.0;
constexpr dReal STEERING_POSITION_GAIN = 4.0;
constexpr dReal STEERING_RATE_DAMPING = 0.8;
constexpr dReal STEERING_MAX_RATE = 2.0;
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
constexpr int ROAD_BUMP_SEGMENTS = 12;
constexpr int ROAD_RAMP_SEGMENTS = 6;
constexpr int ROAD_INCLINE_SEGMENTS = 128;

class OdeRuntime {
  public:
    OdeRuntime() {
        dInitODE2(0);
        world = dWorldCreate();
        space = dHashSpaceCreate(nullptr);
        contacts = dJointGroupCreate(0);
        if (!world || !space || !contacts) {
            throw std::runtime_error("Failed to create ODE runtime");
        }
        dWorldSetGravity(world, 0.0, 0.0, GRAVITY);
        dWorldSetERP(world, 0.8);
        dWorldSetCFM(world, 1e-5);
    }

    ~OdeRuntime() {
        dJointGroupDestroy(contacts);
        dSpaceDestroy(space);
        dWorldDestroy(world);
        dCloseODE();
    }

    dWorldID world = nullptr;
    dSpaceID space = nullptr;
    dJointGroupID contacts = nullptr;
};
} // namespace

struct BusSimulation::Impl {
    struct Corner {
        dReal x;
        dReal y;
        dReal wheelRadius;
        dReal springCompression = 0.0;
        dReal verticalVelocity = 0.0;
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

    OdeRuntime ode;
    dBodyID chassis = nullptr;
    dGeomID ground = nullptr;
    std::vector<dGeomID> roadBumpGeoms;
    std::vector<dGeomID> roadMeshGeoms;
    std::vector<dTriMeshDataID> roadMeshData;
    std::vector<std::vector<double>> roadMeshVertices;
    std::vector<std::vector<int>> roadMeshIndices;
    dGeomID chassisGeom = nullptr;
    BusConfiguration configuration;
    std::vector<Corner> corners;
    double fixedStep;
    int maxCatchUpSteps;
    double accumulator = 0.0;
    double steeringAngle = 0.0;
    dReal maxSteerAngle = MAX_STEER_ANGLE;
    int currentGear = 0;
    double shiftTimer = 0.0;
    int lastSteps = 0;
    bool dropped = false;
    bool dropReported = false;
    double simulationTime = 0.0;

    dReal suspensionAnchorZ(std::size_t index) const {
        return corners[index].wheelRadius + SUSP_REST - configuration.centerOfGravityHeight;
    }

    ~Impl() {
        for (dGeomID geometry : roadMeshGeoms) {
            if (geometry) {
                dGeomDestroy(geometry);
            }
        }
        for (dTriMeshDataID data : roadMeshData) {
            if (data) {
                dGeomTriMeshDataDestroy(data);
            }
        }
    }

    static void nearCallback(void* context, dGeomID first, dGeomID second) {
        auto* simulation = static_cast<Impl*>(context);
        const bool firstHasBody = dGeomGetBody(first) != nullptr;
        const bool secondHasBody = dGeomGetBody(second) != nullptr;
        if (firstHasBody == secondHasBody) {
            return;
        }
        dContact contacts[4] = {};
        const int count = dCollide(first, second, 4, &contacts[0].geom, sizeof(dContact));
        for (int index = 0; index < count; ++index) {
            contacts[index].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1;
            contacts[index].surface.mu = ROAD_FRICTION_COEFFICIENT;
            contacts[index].surface.soft_erp = 0.35;
            contacts[index].surface.soft_cfm = 2e-5;
            dJointID contact = dJointCreateContact(simulation->ode.world, simulation->ode.contacts,
                                                   &contacts[index]);
            dJointAttach(contact, dGeomGetBody(first), dGeomGetBody(second));
        }
    }

    void addRoadBox(dReal centerX, dReal centerY, dReal length, dReal width, dReal height,
                    dReal bottomZ = 0.0) {
        if (length <= 0.0 || width <= 0.0 || height <= 0.0) {
            return;
        }
        const dGeomID geometry = dCreateBox(ode.space, length, width, height);
        dGeomSetPosition(geometry, centerX, centerY, bottomZ + height * 0.5);
        roadBumpGeoms.push_back(geometry);
    }

    void addRoadIncline(const RoadBump& feature) {
        constexpr int verticesPerStation = 4;
        const int stationCount = ROAD_INCLINE_SEGMENTS + 1;
        const dReal halfWidth = static_cast<dReal>(feature.width) * 0.5;
        const dReal startX =
            static_cast<dReal>(feature.centerX) - static_cast<dReal>(feature.length) * 0.5;
        const dReal segmentLength = static_cast<dReal>(feature.length) / ROAD_INCLINE_SEGMENTS;

        std::vector<double> vertices;
        std::vector<int> indices;
        vertices.reserve(stationCount * verticesPerStation * 3);
        indices.reserve(ROAD_INCLINE_SEGMENTS * 8 * 3 + 12);
        auto addVertex = [&](dReal x, dReal y, dReal z) {
            vertices.push_back(static_cast<double>(x));
            vertices.push_back(static_cast<double>(y));
            vertices.push_back(static_cast<double>(z));
        };
        for (int index = 0; index < stationCount; ++index) {
            const dReal localX = segmentLength * static_cast<dReal>(index) -
                                 static_cast<dReal>(feature.length) * 0.5;
            const dReal x = startX + segmentLength * index;
            const dReal height =
                static_cast<dReal>(roadFeatureHeightAt(feature, static_cast<double>(localX)));
            addVertex(x, static_cast<dReal>(feature.centerY) - halfWidth, height);
            addVertex(x, static_cast<dReal>(feature.centerY) + halfWidth, height);
            addVertex(x, static_cast<dReal>(feature.centerY) - halfWidth, 0.0);
            addVertex(x, static_cast<dReal>(feature.centerY) + halfWidth, 0.0);
        }

        auto addTriangle = [&](int first, int second, int third) {
            indices.push_back(first);
            indices.push_back(second);
            indices.push_back(third);
        };
        for (int segment = 0; segment < ROAD_INCLINE_SEGMENTS; ++segment) {
            const int current = segment * verticesPerStation;
            const int next = (segment + 1) * verticesPerStation;
            const int topNegative = current;
            const int nextTopNegative = next;
            const int topPositive = current + 1;
            const int nextTopPositive = next + 1;
            const int bottomNegative = current + 2;
            const int nextBottomNegative = next + 2;
            const int bottomPositive = current + 3;
            const int nextBottomPositive = next + 3;

            addTriangle(topNegative, nextTopNegative, nextTopPositive);
            addTriangle(topNegative, nextTopPositive, topPositive);
            addTriangle(topNegative, bottomNegative, nextBottomNegative);
            addTriangle(topNegative, nextBottomNegative, nextTopNegative);
            addTriangle(topPositive, nextTopPositive, nextBottomPositive);
            addTriangle(topPositive, nextBottomPositive, bottomPositive);
            addTriangle(bottomNegative, bottomPositive, nextBottomPositive);
            addTriangle(bottomNegative, nextBottomPositive, nextBottomNegative);
        }
        addTriangle(0, 1, 3);
        addTriangle(0, 3, 2);
        const int last = ROAD_INCLINE_SEGMENTS * verticesPerStation;
        addTriangle(last, last + 2, last + 3);
        addTriangle(last, last + 3, last + 1);

        roadMeshVertices.push_back(std::move(vertices));
        roadMeshIndices.push_back(std::move(indices));
        const std::vector<double>& storedVertices = roadMeshVertices.back();
        const std::vector<int>& storedIndices = roadMeshIndices.back();
        const dTriMeshDataID meshData = dGeomTriMeshDataCreate();
        if (!meshData) {
            throw std::runtime_error("Failed to create incline mesh data");
        }
        dGeomTriMeshDataBuildDouble(meshData, storedVertices.data(), 3 * sizeof(double),
                                    static_cast<int>(storedVertices.size() / 3),
                                    storedIndices.data(), static_cast<int>(storedIndices.size()),
                                    3 * sizeof(int));
        const dGeomID geometry = dCreateTriMesh(ode.space, meshData, nullptr, nullptr, nullptr);
        if (!geometry) {
            dGeomTriMeshDataDestroy(meshData);
            throw std::runtime_error("Failed to create incline collision mesh");
        }
        roadMeshData.push_back(meshData);
        roadMeshGeoms.push_back(geometry);
    }

    void addRoadFeature(const RoadBump& feature) {
        if (feature.type == RoadFeatureType::Barrier) {
            addRoadBox(feature.centerX, feature.centerY, feature.length, feature.width,
                       feature.height);
            return;
        }

        if (feature.type == RoadFeatureType::Bridge) {
            const dReal deckLength = std::clamp(static_cast<dReal>(feature.flatLength), 0.0,
                                                static_cast<dReal>(feature.length));
            const dReal rampLength = (static_cast<dReal>(feature.length) - deckLength) * 0.5;
            const int rampSegments = 6;
            const dReal segmentLength = rampLength / rampSegments;
            const dReal startX = feature.centerX - feature.length * 0.5;
            for (int segment = 0; segment < rampSegments; ++segment) {
                const dReal phase = (static_cast<dReal>(segment) + 0.5) / rampSegments;
                addRoadBox(startX + segmentLength * (segment + 0.5), feature.centerY, segmentLength,
                           feature.width, feature.height * phase);
            }
            addRoadBox(feature.centerX, feature.centerY, deckLength, feature.width, feature.height);
            const dReal downStartX = feature.centerX + feature.length * 0.5 - rampLength;
            for (int segment = 0; segment < rampSegments; ++segment) {
                const dReal phase = (static_cast<dReal>(segment) + 0.5) / rampSegments;
                addRoadBox(downStartX + segmentLength * (segment + 0.5), feature.centerY,
                           segmentLength, feature.width, feature.height * (1.0 - phase));
            }
            if (feature.railHeight > 0.0 && feature.railWidth > 0.0) {
                const dReal railY = feature.width * 0.5 - feature.railWidth * 0.5;
                addRoadBox(feature.centerX, feature.centerY + railY, feature.length,
                           feature.railWidth, feature.railHeight, feature.height);
                addRoadBox(feature.centerX, feature.centerY - railY, feature.length,
                           feature.railWidth, feature.railHeight, feature.height);
            }
            return;
        }

        if (feature.type == RoadFeatureType::Incline) {
            addRoadIncline(feature);
            return;
        }

        const dReal segmentLength = static_cast<dReal>(feature.length) / ROAD_BUMP_SEGMENTS;
        for (int segment = 0; segment < ROAD_BUMP_SEGMENTS; ++segment) {
            const dReal phase = (static_cast<dReal>(segment) + 0.5) / ROAD_BUMP_SEGMENTS;
            const dReal profile =
                feature.height * (phase <= 0.5 ? phase * 2.0 : (1.0 - phase) * 2.0);
            const dReal segmentCenterX = feature.centerX - feature.length * 0.5 +
                                         segmentLength * (static_cast<dReal>(segment) + 0.5);
            addRoadBox(segmentCenterX, feature.centerY, segmentLength, feature.width, profile);
        }
    }

    Impl(BusConfiguration vehicle, double physicsHz, int catchUpSteps)
        : configuration(std::move(vehicle)), fixedStep(1.0 / physicsHz),
          maxCatchUpSteps(catchUpSteps) {
        if (physicsHz <= 0.0 || catchUpSteps <= 0) {
            simulationLog.Log("Invalid physics timing configuration");
            throw std::invalid_argument("Physics rate and catch-up steps must be positive");
        }
        if (configuration.axles.empty()) {
            simulationLog.Log("Bus configuration has no axles");
            throw std::invalid_argument("A bus must define at least one axle");
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
                simulationLog.Log("Bus axle geometry is outside the chassis");
                throw std::invalid_argument("Bus axle geometry is outside the chassis");
            }
            const dReal wheelCenterOffset = axle.trackWidth * 0.5 - configuration.wheelHalfWidth;
            if (wheelCenterOffset <= 0.0) {
                simulationLog.Log("Bus axle width is too small for configured wheel width");
                throw std::invalid_argument("Bus axle width must exceed wheel width");
            }
            corners.push_back({axle.position, wheelCenterOffset, axle.wheelDiameter * 0.5});
            corners.push_back({axle.position, -wheelCenterOffset, axle.wheelDiameter * 0.5});
        }
        ground = dCreatePlane(ode.space, 0.0, 0.0, 1.0, 0.0);
        for (const RoadBump& bump : defaultRoadBumps()) {
            addRoadFeature(bump);
        }
        chassis = dBodyCreate(ode.world);
        dMass mass;
        const dReal chassisMass =
            configuration.mass - corners.size() * (WHEEL_MASS + 2.0 * KNUCKLE_MASS);
        if (chassisMass <= 0.0) {
            simulationLog.Log("Bus mass is too low for configured wheel and suspension masses");
            throw std::invalid_argument("Bus mass must exceed wheel and suspension masses");
        }
        dMassSetBoxTotal(&mass, chassisMass, configuration.collisionLength,
                         configuration.collisionWidth, configuration.collisionHeight);
        dBodySetMass(chassis, &mass);
        const dReal averageSpringRate = [&] {
            dReal total = 0.0;
            for (const BusAxle& axle : configuration.axles) {
                total += axle.springRate;
            }
            return total / configuration.axles.size();
        }();
        const dReal staticCompression = std::clamp(configuration.mass * std::abs(GRAVITY) /
                                                       (corners.size() * averageSpringRate),
                                                   0.0, SUSP_MAX_TRAVEL * 0.9);
        dBodySetPosition(chassis, 0.0, 0.0,
                         configuration.centerOfGravityHeight +
                             (configuration.mass - chassisMass) / configuration.mass * 0.5);
        chassisGeom = dCreateBox(ode.space, configuration.collisionLength,
                                 configuration.collisionWidth, configuration.collisionHeight);
        dGeomSetBody(chassisGeom, chassis);
        dGeomSetOffsetPosition(chassisGeom, configuration.collisionOffsetX,
                               configuration.collisionOffsetY, configuration.collisionOffsetZ);
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
            const dReal wheelLocalZ = suspensionAnchorZ(index) - SUSP_REST + staticCompression;
            corner.suspensionBody = dBodyCreate(ode.world);
            corner.steeringBody = dBodyCreate(ode.world);
            corner.wheelBody = dBodyCreate(ode.world);
            dBodySetMass(corner.suspensionBody, &knuckleMass);
            dBodySetMass(corner.steeringBody, &knuckleMass);
            dBodySetMass(corner.wheelBody, &wheelMass);
            dBodySetAutoDisableFlag(corner.suspensionBody, 0);
            dBodySetAutoDisableFlag(corner.steeringBody, 0);
            dBodySetAutoDisableFlag(corner.wheelBody, 0);

            const dReal worldX = configuration.axles[axleIndex].position;
            const dReal worldY = corner.y;
            const dReal worldZ = dBodyGetPosition(chassis)[2] + wheelLocalZ;
            dBodySetPosition(corner.suspensionBody, worldX, worldY, worldZ);
            dBodySetPosition(corner.steeringBody, worldX, worldY, worldZ);
            dBodySetPosition(corner.wheelBody, worldX, worldY, worldZ);

            dMatrix3 wheelRotation;
            dRFromAxisAndAngle(wheelRotation, 1.0, 0.0, 0.0, -PI * 0.5);
            dBodySetRotation(corner.wheelBody, wheelRotation);

            corner.wheelGeom =
                dCreateCylinder(ode.space, corner.wheelRadius, configuration.wheelHalfWidth * 2.0);
            dGeomSetBody(corner.wheelGeom, corner.wheelBody);

            corner.suspensionJoint = dJointCreateSlider(ode.world, nullptr);
            dJointAttach(corner.suspensionJoint, chassis, corner.suspensionBody);
            dJointSetSliderAxis(corner.suspensionJoint, 0.0, 0.0, 1.0);
            const dReal sliderCenter = dJointGetSliderPosition(corner.suspensionJoint);
            dJointSetSliderParam(corner.suspensionJoint, dParamLoStop,
                                 sliderCenter - SUSP_MAX_TRAVEL);
            dJointSetSliderParam(corner.suspensionJoint, dParamHiStop,
                                 sliderCenter + SUSP_MAX_TRAVEL);

            corner.steeringJoint = dJointCreateHinge(ode.world, nullptr);
            dJointAttach(corner.steeringJoint, corner.suspensionBody, corner.steeringBody);
            dJointSetHingeAnchor(corner.steeringJoint, worldX, worldY, worldZ);
            dJointSetHingeAxis(corner.steeringJoint, 0.0, 0.0, 1.0);
            const bool steerable = configuration.axles[axleIndex].steerable;
            dJointSetHingeParam(corner.steeringJoint, dParamLoStop,
                                steerable ? -maxSteerAngle : 0.0);
            dJointSetHingeParam(corner.steeringJoint, dParamHiStop,
                                steerable ? maxSteerAngle : 0.0);

            corner.wheelJoint = dJointCreateHinge(ode.world, nullptr);
            dJointAttach(corner.wheelJoint, corner.steeringBody, corner.wheelBody);
            dJointSetHingeAnchor(corner.wheelJoint, worldX, worldY, worldZ);
            dJointSetHingeAxis(corner.wheelJoint, 0.0, 1.0, 0.0);
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
            corner.springCompression =
                std::clamp(SUSP_REST - (anchorLocalZ - wheelLocalZ), 0.0, SUSP_MAX_TRAVEL);
            corner.verticalVelocity = wheelLocalVelocityZ - anchorLocalVelocityZ;
            const dReal* wheelRotation = dBodyGetRotation(corner.wheelBody);
            const dReal* wheelAngularVelocity = dBodyGetAngularVel(corner.wheelBody);
            const dReal wheelAxisX = wheelRotation[2];
            const dReal wheelAxisY = wheelRotation[6];
            const dReal wheelAxisZ = wheelRotation[10];
            corner.wheelOmega = wheelAngularVelocity[0] * wheelAxisX +
                                wheelAngularVelocity[1] * wheelAxisY +
                                wheelAngularVelocity[2] * wheelAxisZ;
            const dReal wheelHingeAngle = dJointGetHingeAngle(corner.wheelJoint);
            corner.wheelRotation -=
                std::remainder(wheelHingeAngle - corner.previousWheelHingeAngle, 2.0 * PI);
            corner.previousWheelHingeAngle = wheelHingeAngle;
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

    void applyCornerForces(double throttle, double steering, double brake) {
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

        const dReal engineWheelRadius = [&] {
            for (std::size_t axleIndex = 0; axleIndex < configuration.axles.size(); ++axleIndex) {
                if (configuration.axles[axleIndex].driven) {
                    return corners[axleIndex * 2].wheelRadius;
                }
            }
            return corners.front().wheelRadius;
        }();
        const dReal wheelRpm = std::abs(longitudinalSpeed) / (2.0 * PI * engineWheelRadius) * 60.0;
        const dReal ratio = GEAR_RATIOS[currentGear] * FINAL_DRIVE;
        const dReal engineRpm = std::clamp(wheelRpm * ratio, ENGINE_IDLE_RPM, ENGINE_REDLINE);
        if (shiftTimer <= 0.0 && engineRpm > UPSHIFT_RPM && currentGear + 1 < NUM_GEARS) {
            ++currentGear;
            shiftTimer = SHIFT_LOCKOUT;
            simulationLog.Log("Upshifted to gear " + std::to_string(currentGear + 1));
        } else if (shiftTimer <= 0.0 && engineRpm < DOWNSHIFT_RPM && currentGear > 0) {
            --currentGear;
            shiftTimer = SHIFT_LOCKOUT;
            simulationLog.Log("Downshifted to gear " + std::to_string(currentGear + 1));
        }
        shiftTimer = std::max(0.0, shiftTimer - fixedStep);

        const dReal torqueFactor = std::clamp(1.0 - std::abs(engineRpm - ENGINE_RATED_RPM) /
                                                        (ENGINE_REDLINE - ENGINE_IDLE_RPM),
                                              0.35, 1.0);
        const dReal engineTorque = PEAK_TORQUE * torqueFactor;
        const dReal throttleValue = std::clamp(static_cast<dReal>(throttle), -1.0, 1.0);
        const dReal driveTorque = throttleValue * engineTorque * ratio * DRIVETRAIN_EFF;
        std::size_t drivenAxles = 0;
        for (const BusAxle& axle : configuration.axles) {
            drivenAxles += axle.driven ? 1U : 0U;
        }
        const dReal brakeValue = std::clamp(static_cast<dReal>(brake), 0.0, 1.0);
        std::vector<dReal> compression(corners.size());
        for (std::size_t index = 0; index < corners.size(); ++index) {
            compression[index] = corners[index].springCompression;
        }

        for (std::size_t index = 0; index < corners.size(); ++index) {
            const std::size_t axleIndex = index / 2;
            const bool front = configuration.axles[axleIndex].steerable;
            const dReal antiRoll = axleIndex == 0 ? ARB_STIFFNESS_FRONT : ARB_STIFFNESS_REAR;
            const BusAxle& axle = configuration.axles[axleIndex];
            const dReal springForce = compression[index] * axle.springRate +
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
            const dReal brakeCapacity =
                brakeValue * MAX_BRAKE_FORCE * 0.5 * corners[index].wheelRadius;
            const dReal stoppingTorque = 0.5 * WHEEL_MASS * corners[index].wheelRadius *
                                         corners[index].wheelRadius *
                                         std::abs(corners[index].wheelOmega) / fixedStep;
            const dReal brakeTorque = std::abs(corners[index].wheelOmega) > 1e-6
                                          ? -std::copysign(std::min(brakeCapacity, stoppingTorque),
                                                           corners[index].wheelOmega)
                                          : 0.0;
            corners[index].driveTorque = driveTorquePerWheel;
            corners[index].brakeTorque = brakeTorque;
            corners[index].longitudinalForce =
                (driveTorquePerWheel + brakeTorque) / corners[index].wheelRadius;
            corners[index].lateralForce = 0.0;
            dBodyAddRelTorque(corners[index].wheelBody, 0.0, 0.0,
                              driveTorquePerWheel + brakeTorque);

            if (configuration.axles[axleIndex].driven) {
                const dReal tractionForce = driveTorquePerWheel / corners[index].wheelRadius;
                dBodyAddRelForce(chassis, tractionForce, 0.0, 0.0);
            }
        }

        const dReal drag = -0.5 * AIR_DENSITY * AERO_CD * AERO_FRONT_AREA * longitudinalSpeed *
                           std::abs(longitudinalSpeed);
        const dReal rollingMagnitude = ROLLING_RESIST * configuration.mass * std::abs(GRAVITY);
        const dReal rolling =
            std::abs(longitudinalSpeed) > 0.01
                ? -std::min(rollingMagnitude,
                            configuration.mass * std::abs(longitudinalSpeed) / fixedStep) *
                      (longitudinalSpeed > 0.0 ? 1.0 : -1.0)
                : 0.0;
        dBodyAddRelForce(chassis, drag + rolling, 0.0, 0.0);
        dBodyAddRelTorque(chassis, 0.0, 0.0, -yawRate * YAW_DAMPING);
        if (brakeValue > 0.01 && std::abs(longitudinalSpeed) < 1.0) {
            dBodyAddRelForce(chassis, 0.0, -lateralSpeed * STOP_LATERAL_DAMPING, 0.0);
            dBodyAddRelTorque(chassis, 0.0, 0.0, -yawRate * STOP_YAW_DAMPING);
        }

        const dReal* chassisRotation = dBodyGetRotation(chassis);
        const dReal* chassisAngularVelocity = dBodyGetAngularVel(chassis);
        const dReal rollAngle = std::atan2(-chassisRotation[6], chassisRotation[10]);
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

        if (std::abs(throttle) < 0.01 && (brake < 0.01 || std::abs(longitudinalSpeed) < 0.15) &&
            std::abs(longitudinalSpeed) < 0.08 && std::abs(lateralSpeed) < 0.08 &&
            std::abs(yawRate) < 0.08) {
            dBodySetLinearVel(chassis, 0.0, 0.0, velocity[2]);
            dBodySetAngularVel(chassis, 0.0, 0.0, 0.0);
        }
    }
    void fixedUpdate(double throttle, double steering, double brake) {
        openbus::rendering::TraceScope trace("physics", "BusSimulation::fixedUpdate");
        applyCornerForces(throttle, steering, brake);
        dSpaceCollide(ode.space, this, &nearCallback);
        dWorldStep(ode.world, fixedStep);
        dJointGroupEmpty(ode.contacts);
        refreshWheelTelemetry();
        simulationTime += fixedStep;
    }
};

BusSimulation::BusSimulation(BusConfiguration configuration, double physicsHz, int maxCatchUpSteps)
    : impl_(std::make_unique<Impl>(std::move(configuration), physicsHz, maxCatchUpSteps)) {
    openbus::rendering::TraceScope trace("startup", "BusSimulation::BusSimulation");
    simulationLog.Log("Bus simulation started at " + std::to_string(physicsHz) + " Hz");
}

BusSimulation::~BusSimulation() {
    simulationLog.Log("Bus simulation stopped at " + std::to_string(impl_->simulationTime) +
                      " seconds");
}

void BusSimulation::update(double elapsedSeconds, double throttle, double steering, double brake) {
    openbus::rendering::TraceScope trace("physics", "BusSimulation::update");
    {
        openbus::rendering::TraceScope phase("physics", "BusSimulation::update.accumulate");
        impl_->accumulator += std::clamp(elapsedSeconds, 0.0, MAX_FRAME_SECONDS);
        impl_->lastSteps = 0;
        impl_->dropped = false;
    }
    {
        openbus::rendering::TraceScope phase("physics", "BusSimulation::update.fixedSteps");
        while (impl_->accumulator >= impl_->fixedStep &&
               impl_->lastSteps < impl_->maxCatchUpSteps) {
            impl_->fixedUpdate(throttle, steering, brake);
            impl_->accumulator -= impl_->fixedStep;
            ++impl_->lastSteps;
        }
    }
    {
        openbus::rendering::TraceScope phase("physics", "BusSimulation::update.catchUpPolicy");
        if (impl_->accumulator >= impl_->fixedStep) {
            impl_->dropped = true;
            if (!impl_->dropReported) {
                simulationLog.Log("Dropped accumulated simulation time after exceeding catch-up "
                                  "limit. Dropped frames: " +
                                  std::to_string(impl_->accumulator));
                impl_->dropReported = true;
            }

            impl_->accumulator = std::fmod(impl_->accumulator, impl_->fixedStep);
        } else {
            impl_->dropReported = false;
        }
    }
}

void BusSimulation::step(double throttle, double steering, double brake) {
    impl_->fixedUpdate(throttle, steering, brake);
}

void BusSimulation::updateVariables(openbus::scripting::Vehicle& variables, double throttle,
                                    double steering, double brake) const {
    variables.set("Throttle", std::clamp(throttle, -1.0, 1.0));
    variables.set("Brake", std::clamp(brake, 0.0, 1.0));
    variables.set("Velocity", speed());
    variables.set("Velocity_Ground", speed());
    variables.set("Steering", std::clamp(steering, -1.0, 1.0));
    variables.set("SteeringAngle", steeringAngle());
    for (std::size_t index = 0; index < impl_->corners.size(); ++index) {
        const std::size_t axleIndex = index / 2;
        const char* side = index % 2 == 0 ? "L" : "R";
        const std::string prefix = "_" + std::to_string(axleIndex) + "_" + side;
        variables.set("Wheel_Rotation" + prefix, impl_->corners[index].wheelRotation);
        variables.set("Wheel_RotationSpeed" + prefix, impl_->corners[index].wheelOmega);
        variables.set("Axle_Suspension" + prefix, impl_->corners[index].springCompression);
        variables.set("Axle_Steering_" + std::to_string(axleIndex) + "_" + side,
                      dJointGetHingeAngle(impl_->corners[index].steeringJoint));
    }
}

double BusSimulation::positionX() const {
    return dBodyGetPosition(impl_->chassis)[0];
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
    return {impl_->configuration.collisionLength,  impl_->configuration.collisionWidth,
            impl_->configuration.collisionHeight,  impl_->configuration.collisionOffsetX,
            impl_->configuration.collisionOffsetY, impl_->configuration.collisionOffsetZ};
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