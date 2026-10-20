#pragma once

#include "crc.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace host_protocol
{
constexpr uint8_t Header = 0xA5;
constexpr uint8_t Version = 0x01;
constexpr uint8_t SetVoltage = 0x01;
constexpr uint8_t Feedback = 0x81;
constexpr std::size_t FrameSize = 14;
using Frame = std::array<uint8_t, FrameSize>;
using FrameCrc = crc::CRCX<16, 0x1021, 0xFFFF, false, false, 0x0000>;

static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
              "Host protocol requires IEEE 754 binary32");

// Explicit little-endian serialization: no packed structs or unaligned float accesses.
inline void WriteFloat(uint8_t* data, float value)
{
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    for (unsigned index = 0; index < 4; ++index)
    {
        data[index] = static_cast<uint8_t>(bits >> (index * 8U));
    }
}

inline float ReadFloat(const uint8_t* data)
{
    const uint32_t bits = static_cast<uint32_t>(data[0]) |
                          (static_cast<uint32_t>(data[1]) << 8U) |
                          (static_cast<uint32_t>(data[2]) << 16U) |
                          (static_cast<uint32_t>(data[3]) << 24U);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

inline void FinishFrame(Frame& frame)
{
    const uint16_t crc = FrameCrc::calc(frame.data(), FrameSize - 2);
    frame[12] = static_cast<uint8_t>(crc);
    frame[13] = static_cast<uint8_t>(crc >> 8U);
}

inline bool DecodeTarget(const Frame& frame, float max_voltage, float& target)
{
    const uint16_t received_crc = static_cast<uint16_t>(frame[12]) |
                                  (static_cast<uint16_t>(frame[13]) << 8U);
    if (frame[0] != Header || frame[1] != Version || frame[2] != SetVoltage ||
        FrameCrc::calc(frame.data(), FrameSize - 2) != received_crc ||
        frame[8] != 0 || frame[9] != 0 || frame[10] != 0 || frame[11] != 0)
    {
        return false;
    }
    const float voltage = ReadFloat(frame.data() + 4);
    if (!std::isfinite(voltage) || voltage < 0.0f || voltage > max_voltage)
    {
        return false;
    }
    target = voltage;
    return true;
}

inline bool EncodeFeedback(float voltage, float torque, uint8_t sequence, Frame& frame)
{
    if (!std::isfinite(voltage) || !std::isfinite(torque))
    {
        return false;
    }
    frame = {};
    frame[0] = Header;
    frame[1] = Version;
    frame[2] = Feedback;
    frame[3] = sequence;
    WriteFloat(frame.data() + 4, voltage);
    WriteFloat(frame.data() + 8, torque);
    FinishFrame(frame);
    return true;
}
} // namespace host_protocol
