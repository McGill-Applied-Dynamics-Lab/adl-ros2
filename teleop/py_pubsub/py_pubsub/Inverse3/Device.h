// Copyright 2022 Haply Robotics Inc. All rights reserved.
#pragma once
#include <cstdint>
#include <istream>

namespace Haply
{
    namespace HardwareAPI
    {
        static const uint8_t VECTOR_SIZE = 3;
        static const uint8_t QUATERNION_SIZE = 4;

        namespace Devices
        {
            class Device
            {
             public:
                explicit Device(std::iostream* stream);

             protected:
                static const uint16_t BUFFER_SIZE = 1024;
                std::iostream* stream;
                char* w_buffer;
                char* r_buffer;
                void WriteBytes(int n);
                char ReadHeaderCode();
                int ReadBytes(int n);
                void SendCommand(const unsigned char& header_code,
                                 const float* data, const int& float_count);
#if defined(_DEBUG)
                void LogInt(int i);
                void LogByte(unsigned char b);
                void LogBytes(unsigned char* buffer, int offset, int length);
#endif
            };
        }  // namespace Devices
    }      // namespace HardwareAPI
}  // namespace Haply
