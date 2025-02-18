// copyright 2022 Haply Robotics Inc. All rights reserved.
#pragma once

#include <functional>
#include <string>

#include "SerialDeviceProvider.h"

namespace Haply
{
    namespace HardwareAPI
    {
        namespace Devices
        {
            class DeviceDetection
            {
             protected:
                static int AutoDetect(std::string portNames[],
                                      std::function<int(const char*)> func,
                                      const wchar_t* portType);

             public:
                static int AutoDetectInverse3(std::string portNames[]);
                static int AutoDetectHandle(std::string portNames[]);
                static int IsInverse3(const char* address);
                static int InverseThread(const char* address);
                static int HandleThread(const char* address);
                static bool IsHandle(const char* address);
            };
        }  // namespace Devices
    }      // namespace HardwareAPI
}  // namespace Haply
