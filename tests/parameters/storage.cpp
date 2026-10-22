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
const torque::Parameters A{1.0f, 0.1f, 0.01f, 2.0f};
const torque::Parameters B{2.0f, 0.2f, 0.02f, 3.0f};
const torque::Parameters C{3.0f, 0.3f, 0.03f, 4.0f};
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
           left.cutoff_hz == right.cutoff_hz;
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
    for (unsigned byte = 0; byte < 40; ++byte)
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
    for (unsigned field = 0; field < 4; ++field)
    {
        for (float value : invalid)
        {
            auto parameters = B;
            switch (field)
            {
            case 0: parameters.kp = value; break;
            case 1: parameters.ki = value; break;
            case 2: parameters.kd = value; break;
            default: parameters.cutoff_hz = value; break;
            }
            assert(storage.Save(parameters, true) == Status::InvalidParameters);
            ExpectLoaded(A);
        }
    }
    auto zero_cutoff = B;
    zero_cutoff.cutoff_hz = 0.0f;
    assert(storage.Save(zero_cutoff, true) == Status::InvalidParameters);
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
    for (int word = 0; word < 5; ++word)
    {
        for (unsigned bytes = 0; bytes < 8; ++bytes)
        {
            SeedTwoSaves();
            cut_word = word;
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

void RewriteGeneration(unsigned slot, uint32_t generation)
{
    uint8_t* record = TestFlash + RegionOffset + slot * FLASH_PAGE_SIZE;
    for (unsigned byte = 0; byte < 4; ++byte)
    {
        record[8U + byte] = static_cast<uint8_t>(generation >> (8U * byte));
    }
    using Crc = crc::CRCX<16, 0x1021, 0xFFFF, false, false, 0x0000>;
    const uint32_t crc = Crc::calc(record, 32);
    const uint32_t checksum = crc | ((crc ^ 0xFFFFU) << 16U);
    for (unsigned byte = 0; byte < 4; ++byte)
    {
        record[32U + byte] = static_cast<uint8_t>(checksum >> (8U * byte));
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
    if (corrupt_write && program_word == 5)
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
    std::puts("parameter storage regression scenarios passed");
}
