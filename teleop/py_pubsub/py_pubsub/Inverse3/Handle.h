// Copyright 2022 Haply Robotics Inc. All rights reserved.

#pragma once
#include <cstdint>
#include <cstring>
#include <istream>

#include "Device.h"

#define HAPLY_TOOL_ID 0x00D0
#define HAPLY_TOOL_STATUS 0x00DA
#define HAPLY_TOOL_ERROR 0x00DF

namespace Haply
{
    namespace HardwareAPI
    {
        namespace Devices
        {
            class Handle : public Haply::HardwareAPI::Devices::Device
            {
                static const char HC_DEVICE_WAKEUP = 0x0A;
                static const char HC_TOOL_ID = 0xD0;
                static const char HC_TOOL_STATE = 0xDB;
                static const char HC_TOOL_STATUS = 0xDA;
                static const char HC_TOOL_ERROR = 0xDF;

                static const uint8_t USER_DATA_MAX = 255;

                int ReceiveHandleInfo();
                int ReceiveHandleInfo(unsigned char* handle_data_remaining,
                                      uint16_t* device_id,
                                      unsigned char* device_model_number,
                                      unsigned char* hardware_version,
                                      unsigned char* firmware_version);
                int ReceiveHandleStatusMessage();
                int ReceiveHandleStatusMessage(
                    uint16_t* device_id, float* quaternion, uint8_t* error_flag,
                    uint8_t* hall_effect_sensor_level,
                    unsigned char* user_data_length, unsigned char* user_data);
                int ReceiveHandleErrorResponse();
                int ReceiveHandleErrorResponse(uint16_t* device_id,
                                               unsigned char* error_code);

             public:
                struct HandleInfoResponse
                {
                    unsigned char handle_data_remaining;
                    uint16_t device_id;
                    unsigned char device_model_number;
                    unsigned char hardware_version;
                    unsigned char firmware_version;
                };

                struct HandleStatusResponse
                {
                    uint16_t device_id;
                    float quaternion[QUATERNION_SIZE];
                    uint8_t error_flag;
                    uint8_t handle_connection_sensor;
                    unsigned char user_data_length;
                    unsigned char user_data[USER_DATA_MAX];

                    HandleStatusResponse() {}

                    HandleStatusResponse(uint16_t device_id, float* quaternion,
                                         uint8_t error_flag,
                                         uint8_t handle_connection_sensor,
                                         unsigned char user_data_length,
                                         unsigned char* user_data)
                        : device_id(device_id),
                          error_flag(error_flag),
                          handle_connection_sensor(handle_connection_sensor),
                          user_data_length(user_data_length)
                    {
                        memcpy(this->quaternion, quaternion,
                               sizeof(float) * QUATERNION_SIZE);
                        memcpy(this->user_data, user_data,
                               sizeof(unsigned char) * user_data_length);
                    }

                    HandleStatusResponse& operator=(
                        const HandleStatusResponse& other)
                    {
                        this->device_id = other.device_id;
                        this->error_flag = other.error_flag;
                        this->handle_connection_sensor =
                            other.handle_connection_sensor;
                        this->user_data_length = other.user_data_length;
                        memcpy(this->quaternion, other.quaternion,
                               sizeof(float) * QUATERNION_SIZE);
                        memcpy(this->user_data, other.user_data,
                               sizeof(unsigned char) * user_data_length);
                        return *this;
                    }
                };

                struct HandleErrorResponse
                {
                    uint16_t device_id;
                    unsigned char error_code;
                };

             protected:
                virtual void OnReceiveHandleInfo(
                    unsigned char handle_data_remaining, uint16_t device_id,
                    unsigned char device_model_number,
                    unsigned char hardware_version,
                    unsigned char firmware_version);
                virtual void OnReceiveHandleStatusMessage(
                    uint16_t device_id, float* quaternion, uint8_t error_flag,
                    uint8_t hall_effect_sensor_level,
                    unsigned char user_data_length, unsigned char* user_data);
                virtual void OnReceiveHandleErrorResponse(
                    uint16_t device_id, unsigned char error_code);

                virtual void OnReceiveHandleInfo(HandleInfoResponse& response);
                virtual void OnReceiveHandleStatusMessage(
                    HandleStatusResponse& response);
                virtual void OnReceiveHandleErrorResponse(
                    HandleErrorResponse& response);

             public:
                using Haply::HardwareAPI::Devices::Device::Device;

                void SendDeviceWakeup();
                void SendHandleState(const uint16_t device_id,
                                     const unsigned char user_data_length,
                                     const unsigned char* user_data);
                void SendHandleErrorRequest(const uint16_t device_id);

                int Receive();
                int Receive(unsigned char* n_bytes);
            };
        }  // namespace Devices
    }      // namespace HardwareAPI
}  // namespace Haply
