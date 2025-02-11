// Copyright 2022 Haply Robotics Inc. All rights reserved.
#pragma once
#include <cstdint>
#include <cstring>
#include <istream>

#include "Device.h"
#include "UUID.h"

namespace Haply
{
    namespace HardwareAPI
    {
        namespace Devices
        {
            class Inverse3 : public Haply::HardwareAPI::Devices::Device
            {
                static const char HC_DEVICE_WAKEUP = 0x0A;
                static const char HC_DEVICE_ID = 0x0B;
                static const char HC_DEVICE_ORIENTATION = 0x0C;
                static const char HC0_DEVICE_INFO_EXT = 0x0D;
                static const char HC1_DEVICE_INFO_EXT_DEVICE_ID = 0xD1;
                static const char HC1_DEVICE_INFO_EXT_FIRMWARE_VERSION = 0xD2;
                static const char HC0_CONFIG = 0x0E;
                static const char HC1_CONFIG_SAVE = 0xE0;
                static const char HC1_CONFIG_GRAVITY_COMPENSATION = 0xE1;
                static const char HC1_CONFIG_TORQUE_SCALING = 0xE2;
                static const char HC1_CONFIG_HANDEDNESS = 0xE3;
                static const char HC2_CONFIG_GET = 0x00;
                static const char HC2_CONFIG_SET = 0x01;
                static const char HC_POWER_STATUS = 0xAC;
                static const char HC_TEMPERATURE_STATUS = 0xAA;
                static const char HC_CURRENT_STATUS = 0xAB;
                static const char HC_JOINT_TORQUES = 0x1A;
                static const char HC_JOINT_ANGLES = 0x1C;
                static const char HC_JOINT_STATES = 0x1B;
                static const char HC_END_EFFECTOR_FORCE = 0x2A;
                static const char HC_END_EFFECTOR_POSITION = 0x2C;
                static const char HC_END_EFFECTOR_STATE = 0x2B;

             public:
                using Haply::HardwareAPI::Devices::Device::Device;

                static const uint8_t NUM_JOINTS = 3;

                static const uint8_t LEFT_HANDED = 0x00;
                static const uint8_t RIGHT_HANDED = 0x01;

                struct DeviceInfoResponse
                {
                    [[deprecated(
                        "This field may be removed in a subsequent release. "
                        "Please use device_id_ext instead.")]] uint16_t
                        device_id;
                    unsigned char device_model_number;
                    unsigned char hardware_version;
                    [[deprecated(
                        "This field may be removed in a subsequent release. "
                        "Please use FirmwareVersionExtQuery() "
                        "instead.")]] unsigned char firmware_version;
                    UUID device_id_ext;
                };

                struct JointStatesResponse
                {
                    float angles[NUM_JOINTS];
                    float angularVelocities[NUM_JOINTS];

                    JointStatesResponse() {}

                    JointStatesResponse(float* angles, float* angularVelocities)
                    {
                        memcpy(this->angles, angles,
                               sizeof(float) * NUM_JOINTS);
                        memcpy(this->angularVelocities, angularVelocities,
                               sizeof(float) * NUM_JOINTS);
                    }

                    JointStatesResponse& operator=(
                        const JointStatesResponse& other)
                    {
                        memcpy(this->angles, other.angles,
                               sizeof(float) * NUM_JOINTS);
                        memcpy(this->angularVelocities, other.angularVelocities,
                               sizeof(float) * NUM_JOINTS);
                        return *this;
                    }
                };

                struct JointTorquesRequest
                {
                    float torques[NUM_JOINTS];

                    JointTorquesRequest() {}

                    JointTorquesRequest(float t1, float t2, float t3)
                        : torques{t1, t2, t3}
                    {
                    }
                    explicit JointTorquesRequest(float* torques)
                    {
                        memcpy(this->torques, torques,
                               sizeof(float) * NUM_JOINTS);
                    }

                    JointTorquesRequest& operator=(
                        const JointTorquesRequest& other)
                    {
                        memcpy(this->torques, other.torques,
                               sizeof(float) * NUM_JOINTS);
                        return *this;
                    }
                };

                struct JointAnglesRequest
                {
                    float angles[NUM_JOINTS];

                    JointAnglesRequest() {}

                    JointAnglesRequest(float q1, float q2, float q3)
                        : angles{q1, q2, q3}
                    {
                    }

                    explicit JointAnglesRequest(float* angles)
                    {
                        memcpy(this->angles, angles,
                               sizeof(float) * NUM_JOINTS);
                    }

                    JointAnglesRequest& operator=(
                        const JointAnglesRequest& other)
                    {
                        memcpy(this->angles, other.angles,
                               sizeof(float) * NUM_JOINTS);
                        return *this;
                    }
                };

                struct EndEffectorForceRequest
                {
                    float force[VECTOR_SIZE];

                    EndEffectorForceRequest() {}

                    EndEffectorForceRequest(float f1, float f2, float f3)
                        : force{f1, f2, f3}
                    {
                    }

                    explicit EndEffectorForceRequest(float* force)
                    {
                        memcpy(this->force, force, sizeof(float) * VECTOR_SIZE);
                    }

                    EndEffectorForceRequest& operator=(
                        const EndEffectorForceRequest& other)
                    {
                        memcpy(this->force, other.force,
                               sizeof(float) * VECTOR_SIZE);
                        return *this;
                    }
                };

                struct EndEffectorPositionRequest
                {
                    float position[VECTOR_SIZE];

                    EndEffectorPositionRequest() {}

                    EndEffectorPositionRequest(float p1, float p2, float p3)
                        : position{p1, p2, p3}
                    {
                    }

                    explicit EndEffectorPositionRequest(float* position)
                    {
                        memcpy(this->position, position,
                               sizeof(float) * VECTOR_SIZE);
                    }

                    EndEffectorPositionRequest& operator=(
                        const EndEffectorPositionRequest& other)
                    {
                        memcpy(this->position, other.position,
                               sizeof(float) * VECTOR_SIZE);
                        return *this;
                    }
                };

                struct EndEffectorStateResponse
                {
                    float position[VECTOR_SIZE];
                    float velocity[VECTOR_SIZE];

                    EndEffectorStateResponse() {}

                    EndEffectorStateResponse(float* position, float* velocity)
                    {
                        memcpy(this->position, position,
                               sizeof(float) * VECTOR_SIZE);
                        memcpy(this->velocity, velocity,
                               sizeof(float) * VECTOR_SIZE);
                    }

                    EndEffectorStateResponse& operator=(
                        const EndEffectorStateResponse& other)
                    {
                        memcpy(this->position, other.position,
                               sizeof(float) * VECTOR_SIZE);
                        memcpy(this->velocity, other.velocity,
                               sizeof(float) * VECTOR_SIZE);
                        return *this;
                    }
                };

                struct DeviceOrientationResponse
                {
                    float quaternion[QUATERNION_SIZE];

                    DeviceOrientationResponse() {}

                    explicit DeviceOrientationResponse(float* quaternion)
                    {
                        memcpy(this->quaternion, quaternion,
                               sizeof(float) * QUATERNION_SIZE);
                    }

                    DeviceOrientationResponse& operator=(
                        const DeviceOrientationResponse& other)
                    {
                        memcpy(this->quaternion, other.quaternion,
                               sizeof(float) * QUATERNION_SIZE);
                        return *this;
                    }
                };

                struct DeviceTemperatureResponse
                {
                    float temperature;
                };

                struct DevicePowerResponse
                {
                    bool powered;
                };

                struct MotorCurrentsResponse
                {
                    float currents[NUM_JOINTS];

                    MotorCurrentsResponse() {}

                    explicit MotorCurrentsResponse(float* currents)
                    {
                        memcpy(this->currents, currents,
                               sizeof(float) * NUM_JOINTS);
                    }

                    MotorCurrentsResponse& operator=(
                        const MotorCurrentsResponse& other)
                    {
                        memcpy(this->currents, other.currents,
                               sizeof(float) * NUM_JOINTS);
                        return *this;
                    }
                };

                struct DeviceIdExtResponse
                {
                    UUID device_id;
                };

                struct FirmwareVersionExtResponse
                {
                    UUID firmware_version_id;
                };

                struct DeviceHandednessPayload
                {
                    uint8_t handedness;
                };

                struct TorqueScalingPayload
                {
                    bool enabled;
                };

                struct GravityCompensationPayload
                {
                    bool enabled;
                    float gravity_scale_factor;
                };

                struct SaveConfigResponse
                {
                    uint8_t parameters_updated;
                };

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic DeviceWakeup function "
                    "instead.")]] void
                SendDeviceWakeup();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic DeviceWakeup function "
                    "instead.")]] int
                ReceiveDeviceInfo();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic DeviceWakeup function "
                    "instead.")]] int
                ReceiveDeviceInfo(uint16_t* device_id,
                                  unsigned char* device_model_number,
                                  unsigned char* hardware_version,
                                  unsigned char* firmware_version,
                                  UUID* quaternion);

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic DeviceIdExtQuery function "
                    "instead.")]] void
                SendDeviceIdExtQuery();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic DeviceIdExtQuery function "
                    "instead.")]] int
                ReceiveDeviceIdExt();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic DeviceIdExtQuery function "
                    "instead.")]] int
                ReceiveDeviceIdExt(UUID* device_id);

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic FirmwareVersionExtQuery function "
                    "instead.")]] void
                SendFirmwareVersionExtQuery();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic FirmwareVersionExtQuery function "
                    "instead.")]] int
                ReceiveFirmwareVersionExt();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic FirmwareVersionExtQuery function "
                    "instead.")]] int
                ReceiveFirmwareVersionExt(UUID* firmware_version_id);

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic DeviceOrientationQuery function "
                    "instead.")]] void
                SendDeviceOrientationQuery();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic DeviceOrientationQuery function "
                    "instead.")]] int
                ReceiveDeviceOrientation();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic DeviceOrientationQuery function "
                    "instead.")]] int
                ReceiveDeviceOrientation(float quaternion[4]);

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic DevicePowerQuery function "
                    "instead.")]] void
                SendDevicePowerQuery();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic DevicePowerQuery function "
                    "instead.")]] int
                ReceiveDevicePower();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic DevicePowerQuery function "
                    "instead.")]] int
                ReceiveDevicePower(bool* powered);

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic DeviceTemperatureQuery function "
                    "instead.")]] void
                SendDeviceTemperatureQuery();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic DeviceTemperatureQuery function "
                    "instead.")]] int
                ReceiveDeviceTemperature();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic DeviceTemperatureQuery function "
                    "instead.")]] int
                ReceiveDeviceTemperature(float* temperature);

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic MotorCurrentsQuery function "
                    "instead.")]] void
                SendMotorCurrentsQuery();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic MotorCurrentsQuery function "
                    "instead.")]] int
                ReceiveMotorCurrents();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic MotorCurrentsQuery function "
                    "instead.")]] int
                ReceiveMotorCurrents(float* currents);

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic JointTorques function "
                    "instead.")]] void
                SendJointTorques();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic JointTorques function "
                    "instead.")]] void
                SendJointTorques(const float torques[3]);

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic JointAngles function "
                    "instead.")]] void
                SendJointAngles();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic JointAngles function "
                    "instead.")]] void
                SendJointAngles(const float angles[3]);

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic JointTorques/JointAngles functions "
                    "instead.")]] int
                ReceiveJointStates();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic JointTorques/JointAngles functions "
                    "instead.")]] int
                ReceiveJointStates(float* angles, float* angular_velocities);

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic EndEffectorForce function "
                    "instead.")]] void
                SendEndEffectorForce();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic EndEffectorForce function "
                    "instead.")]] void
                SendEndEffectorForce(const float force[3]);

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic EndEffectorPosition function "
                    "instead.")]] void
                SendEndEffectorPosition();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic EndEffectorPosition function "
                    "instead.")]] void
                SendEndEffectorPosition(const float position[3]);

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic "
                    "EndEffectorForce/EndEffectorPosition functions "
                    "instead.")]] int
                ReceiveEndEffectorState();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic "
                    "EndEffectorForce/EndEffectorPosition functions "
                    "instead.")]] int
                ReceiveEndEffectorState(float* position, float* velocity);

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic SetDeviceHandedness function "
                    "instead.")]] void
                SendSetDeviceHandedness(const uint8_t& handedness);

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic GetDeviceHandedness function "
                    "instead.")]] void
                SendGetDeviceHandedness();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic "
                    "GetDeviceHandedness/SetDeviceHandedness functions "
                    "instead.")]] int
                ReceiveDeviceHandedness();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic "
                    "GetDeviceHandedness/SetDeviceHandedness functions "
                    "instead.")]] int
                ReceiveDeviceHandedness(uint8_t* handedness);

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic SetGravityCompensation function "
                    "instead.")]] void
                SendSetGravityCompensation(const bool& enabled,
                                           const float& gravity_scale_factor);

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic GetGravityCompensation function "
                    "instead.")]] void
                SendGetGravityCompensation();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic "
                    "GetGravityCompensation/SetGravityCompensation functions "
                    "instead.")]] int
                ReceiveGravityCompensation();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic "
                    "GetGravityCompensation/SetGravityCompensation functions "
                    "instead.")]] int
                ReceiveGravityCompensation(bool* enabled,
                                           float* gravity_scale_factor);

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic SetTorqueScaling function "
                    "instead.")]] void
                SendSetTorqueScaling(const bool& enabled);

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic GetTorqueScaling function "
                    "instead.")]] void
                SendGetTorqueScaling();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic GetTorqueScaling/SetTorqueScaling "
                    "functions instead.")]] int
                ReceiveTorqueScaling();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic GetTorqueScaling/SetTorqueScaling "
                    "functions instead.")]] int
                ReceiveTorqueScaling(bool* enabled);

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic SaveConfig function instead.")]] void
                SendSaveConfig();

                [[deprecated(
                    "This function will be removed in a subsequent release. "
                    "Please use the atomic SaveConfig function instead.")]] int
                ReceiveSaveConfig(uint8_t* parameters_updated);

                DeviceInfoResponse DeviceWakeup();

                DeviceIdExtResponse DeviceIdExtQuery();
                FirmwareVersionExtResponse FirmwareVersionExtQuery();

                JointStatesResponse JointTorques(
                    const JointTorquesRequest& request);
                JointStatesResponse JointAngles(
                    const JointAnglesRequest& request);

                EndEffectorStateResponse EndEffectorForce(
                    const EndEffectorForceRequest& request);
                EndEffectorStateResponse EndEffectorPosition(
                    const EndEffectorPositionRequest& request);

                DeviceOrientationResponse DeviceOrientationQuery();
                DevicePowerResponse DevicePowerQuery();
                DeviceTemperatureResponse DeviceTemperatureQuery();
                MotorCurrentsResponse MotorCurrentsQuery();

                DeviceHandednessPayload GetDeviceHandedness();
                DeviceHandednessPayload SetDeviceHandedness(
                    const DeviceHandednessPayload& payload);

                TorqueScalingPayload GetTorqueScaling();
                TorqueScalingPayload SetTorqueScaling(
                    const TorqueScalingPayload& payload);

                GravityCompensationPayload GetGravityCompensation();
                GravityCompensationPayload SetGravityCompensation(
                    const GravityCompensationPayload& payload);

                SaveConfigResponse SaveConfig();
            };
        }  // namespace Devices
    }      // namespace HardwareAPI
}  // namespace Haply
