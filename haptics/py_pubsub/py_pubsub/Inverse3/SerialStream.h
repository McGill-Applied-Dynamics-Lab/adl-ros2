// Copyright 2022 Haply Robotics Inc. All rights reserved.
#pragma once

#include <cassert>
#include <iomanip>
#include <iostream>
#include <istream>

#include "./serialib.h"

namespace Haply
{
    namespace HardwareAPI
    {
        namespace IO
        {
            class SerialStream : public std::iostream
            {
                const char* address;
                serialib* serialbuf;
                bool open;

                const char * newAddress;
                std::string buffer;

             public:
                explicit SerialStream(const char* address, bool Open = true);
                SerialStream();
                ~SerialStream();
                char OpenDevice();
                void CloseDevice();
            };
        }  // namespace IO
    }      // namespace HardwareAPI
}  // namespace Haply
