// Copyright 2022 Haply Robotics Inc. All rights reserved.

#pragma once
#include <cstdint>
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
            class Tool : public Haply::HardwareAPI::Devices::Device
            {
                int ReceiveToolInfo();
                int ReceiveToolInfo(unsigned char* tool_data_remaining,
                                    uint16_t* device_id,
                                    unsigned char* device_model_number,
                                    unsigned char* hardware_version,
                                    unsigned char* firmware_version);
                int ReceiveToolStatusMessage();
                int ReceiveToolStatusMessage(uint16_t* device_id,
                                             float* quaternion,
                                             uint8_t* error_flag,
                                             uint8_t* hall_effect_sensor_level,
                                             unsigned char* user_data_length,
                                             unsigned char* user_data);
                int ReceiveToolErrorResponse();
                int ReceiveToolErrorResponse(uint16_t* device_id,
                                             unsigned char* error_code);

             protected:
                virtual void OnReceiveToolInfo(
                    unsigned char tool_data_remaining, uint16_t device_id,
                    unsigned char device_model_number,
                    unsigned char hardware_version,
                    unsigned char firmware_version);
                virtual void OnReceiveToolStatusMessage(
                    uint16_t device_id, float* quaternion, uint8_t error_flag,
                    uint8_t hall_effect_sensor_level,
                    unsigned char user_data_length, unsigned char* user_data);
                virtual void OnReceiveToolErrorResponse(
                    uint16_t device_id, unsigned char error_code);

             public:
                using Haply::HardwareAPI::Devices::Device::Device;

                void SendDeviceWakeup();
                void SendToolState(const uint16_t device_id,
                                   const unsigned char user_data_length,
                                   const unsigned char* user_data);
                void SendToolErrorRequest(const uint16_t device_id);

                int Receive();
            };
        }  // namespace Devices
    }      // namespace HardwareAPI
}  // namespace Haply
