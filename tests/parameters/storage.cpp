#include "main.h"
#include "ParameterStorage.hpp"
#include "crc.hpp"

#include <array>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>

alignas(8) uint8_t TestFlash[128U * 1024U];

namespace
{
using Storage = torque::ParameterStorage;
using Status = Storage::Status;
constexpr unsigned RegionOffset = 124U * 1024U;
constexpr unsigned RegionSize = 4096U;
constexpr unsigned RecordSize = 48U;
constexpr unsigned PayloadEnd = 40U;
constexpr unsigned RecordWords = RecordSize / sizeof(uint64_t);
const torque::Parameters A{1.0f, 0.1f, 0.01f, 0.02f, 0.3f, 2.0f};
const torque::Parameters B{2.0f, 0.2f, 0.02f, 0.04f, 0.5f, 3.0f};
const torque::Parameters C{3.0f, 0.3f, 0.03f, 0.06f, 0.7f, 4.0f};
using ParameterField = float torque::Parameters::*;
constexpr std::array<ParameterField, 6> Fields{
    &torque::Parameters::kp, &torque::Parameters::ki, &torque::Parameters::kd,
    &torque::Parameters::process_noise, &torque::Parameters::measurement_noise,
    &torque::Parameters::torque_limit};
bool locked;
bool fail_unlock;
bool fail_erase;
bool fail_lock;
bool corrupt_write;
int cut_word;
unsigned cut_bytes;
unsigned program_word;

Storage MakeStorage()
{
    return Storage(reinterpret_cast<std::uintptr_t>(TestFlash + RegionOffset), RegionSize);
}

void ResetFaults()
{
    locked = true;
    fail_unlock = false;
    fail_erase = false;
    fail_lock = false;
    corrupt_write = false;
    cut_word = -1;
    cut_bytes = 0;
    program_word = 0;
}

void Reset()
{
    std::memset(TestFlash, 0xFF, sizeof(TestFlash));
    ResetFaults();
}

bool Equal(const torque::Parameters& left, const torque::Parameters& right)
{
    return left.kp == right.kp && left.ki == right.ki && left.kd == right.kd &&
           left.process_noise == right.process_noise &&
           left.measurement_noise == right.measurement_noise &&
           left.torque_limit == right.torque_limit;
}

void ExpectLoaded(const torque::Parameters& expected)
{
    torque::Parameters loaded{};
    // A fresh object has no cached state, modelling restart of the application.
    assert(MakeStorage().Load(loaded) == Status::Ok);
    assert(Equal(loaded, expected));
}

void SeedTwoSaves()
{
    Reset();
    assert(MakeStorage().Save(A, true) == Status::Ok);
    assert(MakeStorage().Save(B, true) == Status::Ok);
    ResetFaults();
}

void TestRoundTripAndFallback()
{
    Reset();
    auto loaded = A;
    assert(MakeStorage().Load(loaded) == Status::NotFound);
    assert(Equal(loaded, A));
    assert(MakeStorage().Save(A, true) == Status::Ok);
    ExpectLoaded(A);
    assert(MakeStorage().Save(B, true) == Status::Ok);
    ExpectLoaded(B);
    assert(MakeStorage().Save(C, true) == Status::Ok);
    ExpectLoaded(C);
    assert(locked);

    // Every bit in the latest record is covered by format, CRC or commit checks.
    for (unsigned byte = 0; byte < RecordSize; ++byte)
    {
        for (unsigned bit = 0; bit < 8; ++bit)
        {
            TestFlash[RegionOffset + byte] ^= static_cast<uint8_t>(1U << bit);
            ExpectLoaded(B);
            TestFlash[RegionOffset + byte] ^= static_cast<uint8_t>(1U << bit);
        }
    }
    ExpectLoaded(C);
}

void TestRejectedSavesPreserveParameters()
{
    Reset();
    auto storage = MakeStorage();
    assert(storage.Save(A, true) == Status::Ok);
    assert(storage.Save(B, false) == Status::OutputActive);
    const float invalid[] = {-1.0f, std::numeric_limits<float>::infinity(),
                             std::numeric_limits<float>::quiet_NaN()};
    for (auto field : Fields)
    {
        for (float value : invalid)
        {
            auto parameters = B;
            parameters.*field = value;
            assert(storage.Save(parameters, true) == Status::InvalidParameters);
            ExpectLoaded(A);
        }
    }
    for (unsigned field = 4; field < Fields.size(); ++field)
    {
        auto parameters = B;
        parameters.*Fields[field] = 0.0f;
        assert(storage.Save(parameters, true) == Status::InvalidParameters);
        ExpectLoaded(A);
    }
    const Storage invalid_layouts[] = {
        Storage(reinterpret_cast<std::uintptr_t>(TestFlash + RegionOffset + 8U), RegionSize),
        Storage(reinterpret_cast<std::uintptr_t>(TestFlash + RegionOffset), 2048U),
    };
    for (auto invalid_storage : invalid_layouts)
    {
        auto loaded = B;
        assert(invalid_storage.Load(loaded) == Status::InvalidLayout);
        assert(Equal(loaded, B));
        assert(invalid_storage.Save(B, true) == Status::InvalidLayout);
    }
    ExpectLoaded(A);
}

void TestInterruptedWrites()
{
    // Interrupt every aligned program operation after 0..7 bytes. The last fully
    // committed page must remain loadable, even when reusing a previously valid page.
    for (unsigned word = 0; word < RecordWords; ++word)
    {
        for (unsigned bytes = 0; bytes < 8; ++bytes)
        {
            SeedTwoSaves();
            cut_word = static_cast<int>(word);
            cut_bytes = bytes;
            assert(MakeStorage().Save(C, true) == Status::ProgramFailed);
            assert(locked);
            ResetFaults();
            ExpectLoaded(B);
            assert(MakeStorage().Save(C, true) == Status::Ok);
            ExpectLoaded(C);
        }
    }
}

void TestHalFailures()
{
    SeedTwoSaves();
    fail_unlock = true;
    assert(MakeStorage().Save(C, true) == Status::UnlockFailed);
    ExpectLoaded(B);

    SeedTwoSaves();
    fail_erase = true;
    assert(MakeStorage().Save(C, true) == Status::EraseFailed);
    assert(locked);
    ExpectLoaded(B);

    SeedTwoSaves();
    corrupt_write = true;
    assert(MakeStorage().Save(C, true) == Status::VerifyFailed);
    assert(locked);
    ExpectLoaded(B);

    SeedTwoSaves();
    fail_lock = true;
    assert(MakeStorage().Save(C, true) == Status::LockFailed);
    // The data was committed before relocking failed; report the lock failure,
    // but a subsequent load must still select the newly committed parameters.
    ExpectLoaded(C);
}

void Write32(uint8_t* data, uint32_t value)
{
    for (unsigned byte = 0; byte < 4; ++byte)
    {
        data[byte] = static_cast<uint8_t>(value >> (8U * byte));
    }
}

void RewriteChecksum(uint8_t* record, unsigned payload_end = PayloadEnd)
{
    using Crc = crc::CRCX<16, 0x1021, 0xFFFF, false, false, 0x0000>;
    const uint32_t crc = Crc::calc(record, payload_end);
    Write32(record + payload_end, crc | ((crc ^ 0xFFFFU) << 16U));
}

void RewriteGeneration(unsigned slot, uint32_t generation)
{
    uint8_t* record = TestFlash + RegionOffset + slot * FLASH_PAGE_SIZE;
    Write32(record + 8U, generation);
    RewriteChecksum(record);
}

void TestRecordLayoutAndChecksumBounds()
{
    Reset();
    assert(MakeStorage().Save(A, true) == Status::Ok);
    assert(program_word == RecordWords && locked);
    const uint8_t* record = TestFlash + RegionOffset;
    assert(record[4] == 2U && record[5] == 0U && record[6] == 0U && record[7] == 0U);
    assert(record[12] == 24U && record[13] == 0U && record[14] == 0U && record[15] == 0U);
    for (unsigned field = 0; field < Fields.size(); ++field)
    {
        float restored;
        std::memcpy(&restored, record + 16U + field * sizeof(float), sizeof(restored));
        assert(restored == A.*Fields[field]);
    }
    for (unsigned byte = RecordSize; byte < FLASH_PAGE_SIZE; ++byte)
    {
        assert(record[byte] == 0xFFU);
    }
    // Neither page padding nor unrelated Flash bytes belong to the record CRC.
    TestFlash[RegionOffset + RecordSize] = 0;
    TestFlash[RegionOffset + FLASH_PAGE_SIZE - 1U] = 0;
    TestFlash[RegionOffset - 1U] = 0;
    ExpectLoaded(A);
}

void TestObsoleteRecordsRejected()
{
    Reset();
    uint8_t* record = TestFlash + RegionOffset;
    Write32(record, 0x51524F54U);
    Write32(record + 4U, 1U);
    Write32(record + 8U, 100U);
    Write32(record + 12U, 16U);
    const std::array<float, 4> old_parameters{A.kp, A.ki, A.kd, 2.0f};
    std::memcpy(record + 16U, old_parameters.data(), sizeof(old_parameters));
    RewriteChecksum(record, 32U);
    Write32(record + 36U, 0x44455653U);
    auto loaded = B;
    assert(MakeStorage().Load(loaded) == Status::NotFound);
    assert(Equal(loaded, B));
    // A valid old record cannot displace a current record, even with a higher generation.
    std::memcpy(record + FLASH_PAGE_SIZE, record, 40U);
    assert(MakeStorage().Save(C, true) == Status::Ok);
    ExpectLoaded(C);
    // Reject version 1 even when all other version-2 fields and CRC are valid.
    Write32(record + 4U, 1U);
    RewriteChecksum(record);
    loaded = A;
    assert(MakeStorage().Load(loaded) == Status::NotFound);
    assert(Equal(loaded, A));
}

void TestNumericBoundaries()
{
    for (unsigned field = 0; field < Fields.size(); ++field)
    {
        Reset();
        auto parameters = A;
        parameters.*Fields[field] = field < 4U ? 0.0f : std::numeric_limits<float>::min();
        assert(MakeStorage().Save(parameters, true) == Status::Ok);
        ExpectLoaded(parameters);
    }
    // A correct CRC does not make numerically invalid persisted parameters usable.
    const float invalid[] = {-1.0f, std::numeric_limits<float>::infinity(),
                             std::numeric_limits<float>::quiet_NaN(), 0.0f};
    for (unsigned field = 0; field < Fields.size(); ++field)
    {
        for (float value : invalid)
        {
            if (field < 4U && value == 0.0f) continue;
            SeedTwoSaves();
            uint8_t* record = TestFlash + RegionOffset + FLASH_PAGE_SIZE;
            std::memcpy(record + 16U + field * sizeof(float), &value, sizeof(value));
            RewriteChecksum(record);
            ExpectLoaded(A);
        }
    }
}

void TestGenerationWraparound()
{
    SeedTwoSaves();
    RewriteGeneration(0, 0xFFFFFFFEU);
    RewriteGeneration(1, 0xFFFFFFFFU);
    ExpectLoaded(B);
    assert(MakeStorage().Save(C, true) == Status::Ok);
    ExpectLoaded(C);
    assert(MakeStorage().Save(A, true) == Status::Ok);
    ExpectLoaded(A);
}

} // namespace

HAL_StatusTypeDef HAL_FLASH_Unlock()
{
    if (fail_unlock) return HAL_ERROR;
    locked = false;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FLASH_Lock()
{
    if (fail_lock) return HAL_ERROR;
    locked = true;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef* erase, uint32_t* page_error)
{
    assert(!locked && erase->TypeErase == FLASH_TYPEERASE_PAGES && erase->Banks == FLASH_BANK_1);
    assert(erase->NbPages == 1 && (erase->Page == 62 || erase->Page == 63));
    // Even an erase that fails after destroying half the inactive page must not
    // affect the last committed page.
    std::memset(TestFlash + erase->Page * FLASH_PAGE_SIZE, 0xFF,
                fail_erase ? FLASH_PAGE_SIZE / 2U : FLASH_PAGE_SIZE);
    *page_error = fail_erase ? erase->Page : 0xFFFFFFFFU;
    return fail_erase ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address, uint64_t data)
{
    assert(!locked && type == FLASH_TYPEPROGRAM_DOUBLEWORD);
    const uint32_t base = static_cast<uint32_t>(reinterpret_cast<std::uintptr_t>(TestFlash));
    const uint32_t offset = address - base;
    assert(offset >= RegionOffset && offset + 8U <= sizeof(TestFlash) && offset % 8U == 0U);
    const bool cut = static_cast<int>(program_word++) == cut_word;
    const unsigned count = cut ? cut_bytes : 8U;
    for (unsigned byte = 0; byte < count; ++byte)
    {
        assert(TestFlash[offset + byte] == 0xFF);
        TestFlash[offset + byte] = static_cast<uint8_t>(data >> (8U * byte));
    }
    if (corrupt_write && program_word == RecordWords)
    {
        TestFlash[offset - 16U] ^= 1U;
    }
    return cut ? HAL_ERROR : HAL_OK;
}

int main()
{
    TestRoundTripAndFallback();
    TestRejectedSavesPreserveParameters();
    TestInterruptedWrites();
    TestHalFailures();
    TestGenerationWraparound();
    TestRecordLayoutAndChecksumBounds();
    TestObsoleteRecordsRejected();
    TestNumericBoundaries();
    std::puts("parameter storage regression scenarios passed");
}
