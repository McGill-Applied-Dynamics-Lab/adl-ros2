// copyright 2022 Haply Robotics Inc. All rights reserved.
#pragma once
#include <string>
namespace Haply
{
    namespace HardwareAPI
    {
        namespace Devices
        {
            class SerialDeviceProvider
            {
             public:
                static int ListSerialDevices(std::string portNames[],
                                             const wchar_t* portType);
                static int SelectComPort(std::string portNames[]);
            };
        }  // namespace Devices
    }      // namespace HardwareAPI
}  // namespace Haply
