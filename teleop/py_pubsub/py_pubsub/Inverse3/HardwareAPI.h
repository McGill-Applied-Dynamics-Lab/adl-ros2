// copyright 2022 Haply Robotics Inc. All rights reserved.
#pragma once

#include "DeviceDetection.h"
#include "Handle.h"
#include "Inverse3.h"
#include "SerialStream.h"
#include "UUID.h"

namespace Haply
{

    //! @brief The HardwareAPI class is a simple interface to the hardware.
    namespace HardwareAPI
    {

        void PrintLibraryVersion();
        char* GetLibraryVersion();

    }  // namespace HardwareAPI
}  // namespace Haply
