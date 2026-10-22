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
enum class CommandType : uint8_t
{
    SetVoltage = 0x01,
    SetTorque = 0x02,
    SetTorqueEnabled = 0x03,
    SetParameter = 0x10,
    GetParameter = 0x11,
    SaveParameters = 0x12,
    GetState = 0x13,
};

enum class ParameterId : uint32_t
{
    Kp = 1,
    Ki = 2,
    Kd = 3,
    ProcessNoise = 4,
    MeasurementNoise = 5,
    TorqueLimit = 6,
};

enum class Result : uint8_t
{
    Ok = 0,
    InvalidValue = 1,
    NotReady = 2,
    ModeConflict = 3,
    OutputActive = 4,
    StorageError = 5,
    HardwareError = 6,
};

struct Command
{
    CommandType type;
    uint8_t sequence;
    uint32_t argument;
    float value;
};

struct Response
{
    CommandType type;
    uint8_t sequence;
    Result result;
    float value;
    bool torque_enabled;
    bool feedback_valid;
};

constexpr uint8_t Feedback = 0x81;
constexpr uint8_t CommandResponse = 0x82;
constexpr std::size_t FrameSize = 14;
using Frame = std::array<uint8_t, FrameSize>;
using FrameCrc = crc::CRCX<16, 0x1021, 0xFFFF, false, false, 0x0000>;

static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
              "Host protocol requires IEEE 754 binary32");

// Explicit little-endian serialization: no packed structs or unaligned accesses.
inline void WriteUint32(uint8_t* data, uint32_t value)
{
    for (unsigned index = 0; index < 4; ++index)
    {
        data[index] = static_cast<uint8_t>(value >> (index * 8U));
    }
}

inline uint32_t ReadUint32(const uint8_t* data)
{
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8U) |
           (static_cast<uint32_t>(data[2]) << 16U) |
           (static_cast<uint32_t>(data[3]) << 24U);
}

inline void WriteFloat(uint8_t* data, float value)
{
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    WriteUint32(data, bits);
}

inline float ReadFloat(const uint8_t* data)
{
    const uint32_t bits = ReadUint32(data);
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

inline bool DecodeCommand(const Frame& frame, Command& command)
{
    const uint16_t received_crc = static_cast<uint16_t>(frame[12]) |
                                  (static_cast<uint16_t>(frame[13]) << 8U);
    if (frame[0] != Header || frame[1] != Version ||
        FrameCrc::calc(frame.data(), FrameSize - 2) != received_crc)
    {
        return false;
    }

    Command decoded{static_cast<CommandType>(frame[2]), frame[3], 0, 0.0f};
    const uint32_t first = ReadUint32(frame.data() + 4);
    const uint32_t second = ReadUint32(frame.data() + 8);
    switch (decoded.type)
    {
    case CommandType::SetVoltage:
    case CommandType::SetTorque:
        if (second != 0U)
        {
            return false;
        }
        decoded.value = ReadFloat(frame.data() + 4);
        break;
    case CommandType::SetTorqueEnabled:
        if (second != 0U)
        {
            return false;
        }
        decoded.argument = first;
        break;
    case CommandType::SetParameter:
    case CommandType::GetParameter:
        if (first < static_cast<uint32_t>(ParameterId::Kp) ||
            first > static_cast<uint32_t>(ParameterId::TorqueLimit) ||
            (decoded.type == CommandType::GetParameter && second != 0U))
        {
            return false;
        }
        decoded.argument = first;
        if (decoded.type == CommandType::SetParameter)
        {
            decoded.value = ReadFloat(frame.data() + 8);
        }
        break;
    case CommandType::SaveParameters:
    case CommandType::GetState:
        if (first != 0U || second != 0U)
        {
            return false;
        }
        break;
    default:
        return false;
    }
    // Numeric limits and enable argument validity belong to the application.
    command = decoded;
    return true;
}

inline bool EncodeResponse(const Response& response, Frame& frame)
{
    if (!std::isfinite(response.value))
    {
        return false;
    }
    frame = {};
    frame[0] = Header;
    frame[1] = Version;
    frame[2] = CommandResponse;
    frame[3] = response.sequence;
    const uint32_t status = static_cast<uint8_t>(response.result) |
                            (static_cast<uint32_t>(response.torque_enabled) << 8U) |
                            (static_cast<uint32_t>(response.feedback_valid) << 9U) |
                            (static_cast<uint32_t>(response.type) << 16U);
    WriteUint32(frame.data() + 4, status);
    WriteFloat(frame.data() + 8, response.value);
    FinishFrame(frame);
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
