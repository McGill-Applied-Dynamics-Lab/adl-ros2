// Copyright 2022 Haply Robotics Inc. All rights reserved.
#pragma once
#include <cstdint>
#include <istream>

namespace Haply
{
    namespace HardwareAPI
    {
        struct UUID
        {
            static const unsigned char SIZE = 16;
            unsigned char bytes[SIZE];

            void SetBytes(unsigned char const * const & b);
            void Print(std::ostream& os);
        };
    }   // namespace HardwareAPI
}   // namespace Haply
