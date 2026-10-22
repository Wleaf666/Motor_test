#include "ParameterStorage.hpp"

#include "main.h"
#include "crc.hpp"

#include <array>
#include <cstring>
#include <limits>

namespace torque
{
namespace
{
using Record = std::array<uint8_t, 48>;
using RecordCrc = crc::CRCX<16, 0x1021, 0xFFFF, false, false, 0x0000>;
constexpr uint32_t Magic = 0x51524F54U;
constexpr uint32_t Version = 2U;
constexpr uint32_t Commit = 0x44455653U;
constexpr uint32_t PayloadSize = 24U;
constexpr std::size_t PayloadEnd = 16U + PayloadSize;

static_assert(sizeof(Record) % sizeof(uint64_t) == 0);

static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
static_assert(FLASH_PAGE_SIZE == 2048U, "Storage layout requires G431 2 KB pages");
#if defined(FLASH_OPTR_DBANK)
#error "ParameterStorage is for the single-bank STM32G431 only"
#endif

void Write32(uint8_t* data, uint32_t value)
{
    for (unsigned index = 0; index < 4; ++index)
    {
        data[index] = static_cast<uint8_t>(value >> (index * 8U));
    }
}

uint32_t Read32(const uint8_t* data)
{
    return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8U) |
           (static_cast<uint32_t>(data[2]) << 16U) | (static_cast<uint32_t>(data[3]) << 24U);
}

void WriteFloat(uint8_t* data, float value)
{
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    Write32(data, bits);
}

float ReadFloat(const uint8_t* data)
{
    const uint32_t bits = Read32(data);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

uint32_t Checksum(const Record& record)
{
    const uint32_t crc = RecordCrc::calc(record.data(), PayloadEnd);
    return crc | ((crc ^ 0xFFFFU) << 16U);
}

Record Encode(const Parameters& parameters, uint32_t generation)
{
    Record record;
    Write32(record.data(), Magic);
    Write32(record.data() + 4, Version);
    Write32(record.data() + 8, generation);
    Write32(record.data() + 12, PayloadSize);
    WriteFloat(record.data() + 16, parameters.kp);
    WriteFloat(record.data() + 20, parameters.ki);
    WriteFloat(record.data() + 24, parameters.kd);
    WriteFloat(record.data() + 28, parameters.process_noise);
    WriteFloat(record.data() + 32, parameters.measurement_noise);
    WriteFloat(record.data() + 36, parameters.torque_limit);
    Write32(record.data() + PayloadEnd, Checksum(record));
    Write32(record.data() + PayloadEnd + 4U, Commit);
    return record;
}
} // namespace

bool ParameterStorage::CheckLayout() const
{
    const std::uintptr_t flash_end = static_cast<std::uintptr_t>(FLASH_BASE) + FLASH_SIZE;
    return RegionSize_ == 2U * FLASH_PAGE_SIZE && FLASH_SIZE >= RegionSize_ &&
           RegionStart_ == flash_end - RegionSize_;
}

bool ParameterStorage::ReadSlot(unsigned slot, Parameters& parameters, uint32_t& generation) const
{
    Record record;
    std::memcpy(record.data(), reinterpret_cast<const void*>(RegionStart_ + slot * FLASH_PAGE_SIZE),
                record.size());
    if (Read32(record.data()) != Magic || Read32(record.data() + 4) != Version ||
        Read32(record.data() + 12) != PayloadSize ||
        Read32(record.data() + PayloadEnd + 4U) != Commit ||
        Read32(record.data() + PayloadEnd) != Checksum(record))
    {
        return false;
    }
    const Parameters candidate{ReadFloat(record.data() + 16), ReadFloat(record.data() + 20),
                               ReadFloat(record.data() + 24), ReadFloat(record.data() + 28),
                               ReadFloat(record.data() + 32), ReadFloat(record.data() + 36)};
    if (!IsValid(candidate))
    {
        return false;
    }
    parameters = candidate;
    generation = Read32(record.data() + 8);
    return true;
}

int ParameterStorage::FindLatest(Parameters& parameters, uint32_t& generation) const
{
    int latest = -1;
    for (unsigned slot = 0; slot < 2; ++slot)
    {
        Parameters candidate;
        uint32_t candidate_generation;
        if (!ReadSlot(slot, candidate, candidate_generation))
        {
            continue;
        }
        // Only adjacent successful saves coexist; unsigned comparison handles wraparound.
        const uint32_t distance = candidate_generation - generation;
        if (latest < 0 || (distance != 0U && distance < 0x80000000U))
        {
            latest = static_cast<int>(slot);
            parameters = candidate;
            generation = candidate_generation;
        }
    }
    return latest;
}

ParameterStorage::Status ParameterStorage::Load(Parameters& parameters) const
{
    if (!CheckLayout())
    {
        return Status::InvalidLayout;
    }
    uint32_t generation = 0;
    return FindLatest(parameters, generation) < 0 ? Status::NotFound : Status::Ok;
}

ParameterStorage::Status ParameterStorage::Save(const Parameters& parameters, bool output_stopped)
{
    if (!output_stopped)
    {
        return Status::OutputActive;
    }
    if (!CheckLayout())
    {
        return Status::InvalidLayout;
    }
    if (!IsValid(parameters))
    {
        return Status::InvalidParameters;
    }

    Parameters previous;
    uint32_t generation = 0;
    const int latest = FindLatest(previous, generation);
    const unsigned destination = latest < 0 ? 0U : static_cast<unsigned>(latest) ^ 1U;
    const std::uintptr_t address = RegionStart_ + destination * FLASH_PAGE_SIZE;
    const Record record = Encode(parameters, generation + 1U);

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return Status::UnlockFailed;
    }

    FLASH_EraseInitTypeDef erase{};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = static_cast<uint32_t>((address - FLASH_BASE) / FLASH_PAGE_SIZE);
    erase.NbPages = 1;
    uint32_t page_error = 0xFFFFFFFFU;
    Status result = Status::Ok;
    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK || page_error != 0xFFFFFFFFU)
    {
        result = Status::EraseFailed;
    }
    else
    {
        // The final aligned doubleword contains both the checksum and commit marker.
        for (std::size_t offset = 0; offset < record.size(); offset += sizeof(uint64_t))
        {
            uint64_t word;
            std::memcpy(&word, record.data() + offset, sizeof(word));
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                  static_cast<uint32_t>(address + offset), word) != HAL_OK)
            {
                result = Status::ProgramFailed;
                break;
            }
        }
        if (result == Status::Ok &&
            std::memcmp(reinterpret_cast<const void*>(address), record.data(), record.size()) != 0)
        {
            result = Status::VerifyFailed;
        }
    }

    // Relock on every path after a successful unlock, including erase/program errors.
    if (HAL_FLASH_Lock() != HAL_OK && result == Status::Ok)
    {
        result = Status::LockFailed;
    }
    return result;
}
} // namespace torque
