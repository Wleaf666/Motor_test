// Host regression suite for the production ADS1220 driver; no hardware or real-time sleeps.
// Build from repository root:
// g++ -std=c++17 -Wall -Wextra -Werror -Itests/ads1220 -IUserCode/ADS1220 UserCode/ADS1220/ADS1220.cpp tests/ads1220/calibration.cpp -o build/ads1220-calibration.exe
// Run build/ads1220-calibration.exe.
#include "ADS1220.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace
{
struct AdcModel
{
    std::array<uint8_t, 4> registers{};
    uint64_t now = 0;
    uint64_t ready_at = 0;
    int32_t input_code = 123;
    int32_t offset_per_gain = 8;
    int32_t output = 5000;
    int32_t latched = 5000;
    int32_t temperature_code = 800; // 25 Celsius, 0.03125 Celsius/LSB.
    unsigned short_reads = 0;
    unsigned fail_short_read = 0;
    unsigned partial_write_failures = 0;
    unsigned data_timeouts = 0;
    bool mismatch_once = false;
    bool fail_all_writes = false;
    bool fail_reset_once = false;
    bool running = false;
    uint8_t command = 0;
} chip;

void Require(bool condition, const char* message)
{
    if (!condition)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(1);
    }
}

bool Near(float actual, float expected)
{
    return std::fabs(actual - expected) < 0.000001f;
}

uint32_t ConversionPeriodMs()
{
    // Independent chip timing: nominal periods rounded upwards plus start-up latency.
    static constexpr uint32_t normal_us[] = {50000, 22223, 11112, 5715, 3031, 1667, 1000};
    static constexpr uint32_t duty_us[] = {200000, 88889, 44445, 22728, 12122, 6667, 4000};
    static constexpr uint32_t turbo_us[] = {25000, 11112, 5556, 2858, 1516, 834, 500};
    const uint8_t dr = chip.registers[1] >> 5U;
    const uint8_t mode = (chip.registers[1] >> 3U) & 3U;
    const uint32_t us = mode == 1 ? duty_us[dr] : mode == 2 ? turbo_us[dr] : normal_us[dr];
    return (us + 999U) / 1000U + 1U;
}

void Advance()
{
    if (!chip.running || chip.now < chip.ready_at)
    {
        return;
    }
    const uint8_t input = chip.registers[0] >> 4U;
    const int32_t gain = 1 << ((chip.registers[0] >> 1U) & 7U);
    const int32_t offset = gain * chip.offset_per_gain + ((chip.registers[0] & 1U) ? 100 : 0);
    if (chip.registers[1] & 2U)
    {
        // The unused low ten bits must not influence temperature decoding.
        chip.output = static_cast<int32_t>((static_cast<uint32_t>(chip.temperature_code) & 0x3FFFU) << 10U) | 0x3FF;
    }
    else if (input == 12)
    {
        chip.output = 2097152; // External reference 2.048 V divided by four, internal reference.
    }
    else if (input == 13)
    {
        chip.output = 5120000; // Supply 5 V divided by four, internal reference.
    }
    else if (input == 14)
    {
        chip.output = offset;
    }
    else
    {
        chip.output = (chip.input_code + input * 20) * gain + offset;
    }
    chip.running = (chip.registers[1] & 4U) != 0;
    chip.ready_at = chip.now + ConversionPeriodMs();
}

ADS1220::Config MakeConfig()
{
    ADS1220::Config config{};
    config.data_rate = ADS1220::DataRate::Normal20;
    config.filter = ADS1220::FirFilter::Reject50And60;
    return config;
}

float ExpectedInput(float reference = 5.0f, int32_t channel = 0)
{
    return (chip.input_code + channel * 20) * reference / 8388608.0f;
}

// Waiting belongs to the caller, never to Update(). Keep existing conversion/error
// scenarios while checking that every individual poll leaves simulated time unchanged.
ADS1220::Status Poll(ADS1220& adc)
{
    const uint64_t before = chip.now;
    const auto status = adc.Update();
    Require(chip.now == before, "Update must not delay for ADC conversion");
    return status;
}

ADS1220::Status ReadNext(ADS1220& adc)
{
    for (unsigned elapsed = 0; elapsed < 500; ++elapsed)
    {
        const auto status = Poll(adc);
        if (status != ADS1220::Status::NotReady)
        {
            return status;
        }
        HAL_Delay(1); // Test caller advances time between polls.
    }
    return ADS1220::Status::NotReady;
}

void ExpectPending(ADS1220& adc)
{
    const int32_t raw = adc.GetRawValue();
    const float voltage = adc.GetVoltage();
    const float gain_voltage = adc.GetGainVoltage();
    const float monitor_voltage = adc.GetMonitorVoltage();
    const float temperature = adc.GetTemperature();
    const bool valid = adc.HasSample();
    const uint8_t last_command = chip.command;
    Require(Poll(adc) == ADS1220::Status::NotReady, "pending conversion must return immediately");
    Require(chip.command == last_command, "pending update does not issue SPI commands");
    Require(adc.GetRawValue() == raw && adc.GetVoltage() == voltage &&
            adc.GetGainVoltage() == gain_voltage && adc.GetMonitorVoltage() == monitor_voltage &&
            adc.GetTemperature() == temperature && adc.HasSample() == valid,
            "NotReady must preserve all cached measurements and validity");
}

void TestNonblockingUpdate()
{
    chip = {};
    SPI_HandleTypeDef spi;
    ADS1220 adc(&spi, MakeConfig());
    Require(adc.Init() == ADS1220::Status::Ok, "nonblocking startup still calibrates");
    for (unsigned poll = 0; poll < 100; ++poll)
    {
        ExpectPending(adc);
    }
    HAL_Delay(50); // 20 SPS plus startup latency is not complete yet.
    ExpectPending(adc);
    HAL_Delay(10);
    Require(Poll(adc) == ADS1220::Status::Ok && Near(adc.GetVoltage(), ExpectedInput()),
            "caller can read ready conversion without waiting inside Update");
    ExpectPending(adc);

    auto config = adc.GetConfig();
    config.input = ADS1220::InputMux::Ain0Ain2;
    Require(adc.Configure(config) == ADS1220::Status::Ok, "switch channel for pending sample");
    ExpectPending(adc);
    HAL_Delay(60);
    Require(Poll(adc) == ADS1220::Status::Ok && Near(adc.GetVoltage(), ExpectedInput(5, 1)),
            "channel switch never exposes previous channel as a new sample");

    config.conversion_mode = ADS1220::ConversionMode::SingleShot;
    Require(adc.Configure(config) == ADS1220::Status::Ok, "single-shot pending setup");
    ExpectPending(adc);
    HAL_Delay(60);
    Require(Poll(adc) == ADS1220::Status::Ok, "single-shot result ready");
    ExpectPending(adc); // A completed single shot cannot be consumed twice.
    Require(adc.PowerDown() == ADS1220::Status::Ok, "sleep while polling");
    ExpectPending(adc);
    chip.now = static_cast<uint64_t>(std::numeric_limits<uint32_t>::max()) - 10;
    Require(adc.Start() == ADS1220::Status::Ok, "start near timer wraparound");
    HAL_Delay(20);
    ExpectPending(adc); // Tick wrapped, but the full conversion period has not elapsed.
    HAL_Delay(40);
    Require(Poll(adc) == ADS1220::Status::Ok && Near(adc.GetVoltage(), ExpectedInput(5, 1)),
            "nonblocking readiness works across tick wraparound");
    std::puts("PASS nonblocking polls, cache preservation, channel changes and tick wraparound");
}

void TestCalibration()
{
    chip = {};
    SPI_HandleTypeDef spi;
    ADS1220 adc(&spi, MakeConfig());
    Require(adc.Init() == ADS1220::Status::Ok, "startup calibration");
    Require(!adc.HasSample(), "shorted calibration data is not a published input sample");
    Require(ReadNext(adc) == ADS1220::Status::Ok, "first input conversion");
    Require(adc.GetRawValue() == chip.input_code * 128, "ADC offset removed from raw code");
    Require(Near(adc.GetVoltage(), ExpectedInput()), "ADC offset removed from input voltage");
    Require(Near(adc.GetGainVoltage(), ExpectedInput() * 128), "gain voltage matches corrected input");
    Require(adc.HasSample(), "successful sample valid");
    Require(adc.Calibrate() == ADS1220::Status::Ok, "repeat calibration");
    Require(ReadNext(adc) == ADS1220::Status::Ok && adc.GetRawValue() == chip.input_code * 128,
            "repeat calibration does not subtract previous offset twice");
    const int32_t raw = adc.GetRawValue();
    chip.fail_short_read = chip.short_reads + 3;
    Require(adc.Calibrate() == ADS1220::Status::SpiTimeout, "calibration read error propagated");
    Require(adc.HasSample() && adc.GetRawValue() == raw, "failed calibration retains previous valid sample");
    Require(ReadNext(adc) == ADS1220::Status::Ok && adc.GetRawValue() == raw,
            "failed calibration restores original path and offset");
    chip.fail_short_read = 0;
    chip.data_timeouts = 1;
    Require(ReadNext(adc) == ADS1220::Status::SpiTimeout && adc.GetRawValue() == raw,
            "ordinary SPI read failure does not publish partial sample");
    Require(ReadNext(adc) == ADS1220::Status::Ok, "SPI retry after read failure");
    chip.input_code = -123;
    Require(ReadNext(adc) == ADS1220::Status::Ok && adc.GetRawValue() == -123 * 128,
            "negative input sign extension and correction");
    std::puts("PASS calibration, repeat, errors and signed input");
}

void TestConfigRecovery()
{
    chip = {};
    SPI_HandleTypeDef spi;
    ADS1220 adc(&spi, MakeConfig());
    Require(adc.Init() == ADS1220::Status::Ok && ReadNext(adc) == ADS1220::Status::Ok, "recovery setup");
    const float voltage = adc.GetVoltage();
    auto candidate = adc.GetConfig();
    candidate.gain = ADS1220::Gain::X64;
    chip.partial_write_failures = 1;
    Require(adc.Configure(candidate) == ADS1220::Status::SpiTimeout, "partial register write reports failure");
    Require(adc.IsInitialized() && adc.HasSample() && Near(adc.GetVoltage(), voltage),
            "recovered configuration retains previous sample");
    Require(adc.GetConfig().gain == ADS1220::Gain::X128, "failed config not committed");
    Require(ReadNext(adc) == ADS1220::Status::Ok && Near(adc.GetVoltage(), voltage),
            "hardware restored after partial write, no doubled voltage");
    chip.mismatch_once = true;
    Require(adc.Configure(candidate) == ADS1220::Status::ConfigurationMismatch, "readback mismatch reports failure");
    Require(ReadNext(adc) == ADS1220::Status::Ok && Near(adc.GetVoltage(), voltage),
            "readback failure restores actual hardware config");
    chip.fail_short_read = chip.short_reads + 3;
    Require(adc.Configure(candidate) == ADS1220::Status::SpiTimeout, "candidate calibration error reports failure");
    Require(adc.IsInitialized() && adc.GetConfig().gain == ADS1220::Gain::X128,
            "candidate calibration failure recovers original config");
    Require(ReadNext(adc) == ADS1220::Status::Ok && Near(adc.GetVoltage(), voltage),
            "candidate calibration failure recovers old offset as well");
    chip.fail_short_read = 0;
    Require(adc.PowerDown() == ADS1220::Status::Ok, "sleep before failed config");
    chip.partial_write_failures = 1;
    Require(adc.Configure(candidate) == ADS1220::Status::SpiTimeout, "failed config while sleeping");
    Require(adc.Update() == ADS1220::Status::NotReady, "rollback preserves sleeping state");
    chip.fail_all_writes = true;
    Require(adc.Configure(candidate) == ADS1220::Status::SpiTimeout, "unrecoverable configuration error");
    Require(!adc.IsInitialized() && !adc.HasSample(), "uncertain hardware configuration invalidates driver");
    Require(adc.Update() == ADS1220::Status::NotInitialized, "sampling prohibited after recovery failure");
    chip.fail_all_writes = false;
    Require(adc.Init() == ADS1220::Status::Ok && ReadNext(adc) == ADS1220::Status::Ok,
            "explicit reinitialization recovers disabled driver");
    std::puts("PASS partial-write recovery, readback recovery and failed recovery");
}

void TestFreshSamplesAndCalibrationKeys()
{
    chip = {};
    SPI_HandleTypeDef spi;
    ADS1220 adc(&spi, MakeConfig());
    Require(adc.Init() == ADS1220::Status::Ok && ReadNext(adc) == ADS1220::Status::Ok, "fresh sample setup");
    auto config = adc.GetConfig();
    config.input = ADS1220::InputMux::Ain0Ain2;
    Require(adc.Configure(config) == ADS1220::Status::Ok && !adc.HasSample(), "channel switch invalidates sample");
    ExpectPending(adc);
    Require(ReadNext(adc) == ADS1220::Status::Ok && Near(adc.GetVoltage(), ExpectedInput(5, 1)),
            "caller polls until the new channel result is ready");
    chip.input_code += 10;
    ExpectPending(adc);
    Require(ReadNext(adc) == ADS1220::Status::Ok && Near(adc.GetVoltage(), ExpectedInput(5, 1)),
            "consecutive successful samples must be separate conversions");
    Require(adc.SetGain(ADS1220::Gain::X4) == ADS1220::Status::Ok, "gain change automatically recalibrates");
    Require(ReadNext(adc) == ADS1220::Status::Ok && Near(adc.GetVoltage(), ExpectedInput(5, 1)),
            "new gain has new offset calibration");
    config = adc.GetConfig();
    config.bypass_pga = true;
    config.input = ADS1220::InputMux::Ain0Avss;
    Require(adc.Configure(config) == ADS1220::Status::Ok, "single-ended path configured");
    Require(ReadNext(adc) == ADS1220::Status::Ok && Near(adc.GetVoltage(), ExpectedInput(5, 8)),
            "PGA bypass change recalibrates its distinct offset");
    config.voltage_reference = ADS1220::VoltageReference::Internal2048;
    Require(adc.Configure(config) == ADS1220::Status::Ok, "internal reference configured");
    Require(ReadNext(adc) == ADS1220::Status::Ok && Near(adc.GetVoltage(), ExpectedInput(2.048f, 8)),
            "internal reference ignores external reference voltage");
    std::puts("PASS fresh channels, fresh continuous samples, gain and bypass recalibration");
}

void TestMonitorAndTemperature()
{
    chip = {};
    SPI_HandleTypeDef spi;
    ADS1220 adc(&spi, MakeConfig());
    Require(adc.Init() == ADS1220::Status::Ok, "monitor setup with existing PGA calibration");
    auto config = adc.GetConfig();
    config.input = ADS1220::InputMux::SupplyDiv4;
    Require(adc.Configure(config) == ADS1220::Status::Ok && ReadNext(adc) == ADS1220::Status::Ok,
            "supply monitor conversion");
    Require(Near(adc.GetVoltage(), 1.25f) && Near(adc.GetMonitorVoltage(), 5.0f),
            "supply monitor ignores PGA gain, old offset and external reference");
    Require(adc.GetGainVoltage() == 0.0f, "monitor result not exposed as PGA gain voltage");
    config.input = ADS1220::InputMux::ReferenceDiv4;
    Require(adc.Configure(config) == ADS1220::Status::Ok && ReadNext(adc) == ADS1220::Status::Ok,
            "external reference monitor conversion");
    Require(Near(adc.GetMonitorVoltage(), 2.048f), "external reference monitor restores divide-by-four");
    config.temperature_sensor = true;
    chip.temperature_code = -800;
    Require(adc.Configure(config) == ADS1220::Status::Ok && ReadNext(adc) == ADS1220::Status::Ok,
            "temperature conversion after monitor");
    Require(adc.GetTemperature() == -25.0f && adc.GetMonitorVoltage() == 0.0f,
            "signed temperature ignores unused low ten bits and clears monitor result");
    Require(adc.Calibrate() == ADS1220::Status::InvalidArgument, "temperature offset calibration rejected");
    config = MakeConfig();
    Require(adc.Configure(config) == ADS1220::Status::Ok && ReadNext(adc) == ADS1220::Status::Ok,
            "return from temperature to analog recalibrates");
    Require(Near(adc.GetVoltage(), ExpectedInput()) && adc.GetTemperature() == 0.0f && adc.GetMonitorVoltage() == 0.0f,
            "mode-specific getters cannot expose stale other-mode results");
    std::puts("PASS independent monitor scaling, signed temperature and mode transitions");
}

void TestPowerAndTiming()
{
    chip = {};
    chip.offset_per_gain = -8;
    SPI_HandleTypeDef spi;
    auto config = MakeConfig();
    config.conversion_mode = ADS1220::ConversionMode::SingleShot;
    config.data_rate = ADS1220::DataRate::Duty5;
    config.filter = ADS1220::FirFilter::None;
    ADS1220 adc(&spi, config);
    Require(adc.Init() == ADS1220::Status::Ok && ReadNext(adc) == ADS1220::Status::Ok,
            "slowest single-shot conversion and negative offset");
    Require(Near(adc.GetVoltage(), ExpectedInput()), "negative ADC offset correction");
    Require(adc.Update() == ADS1220::Status::NotReady, "single-shot cannot be consumed twice");
    Require(adc.Start() == ADS1220::Status::Ok, "start another single shot");
    Require(!adc.HasSample() && ReadNext(adc) == ADS1220::Status::Ok, "new shot replaces invalidated cache");
    Require(adc.PowerDown() == ADS1220::Status::Ok && adc.Update() == ADS1220::Status::NotReady,
            "sleep prevents stale data reported as new sample");
    chip.now = static_cast<uint64_t>(std::numeric_limits<uint32_t>::max()) - 10;
    Require(adc.Start() == ADS1220::Status::Ok && ReadNext(adc) == ADS1220::Status::Ok,
            "wait survives millisecond tick wraparound");
    Require(Near(adc.GetVoltage(), ExpectedInput()), "wrapped timer still waits for complete conversion");
    chip.fail_reset_once = true;
    Require(adc.Reset() == ADS1220::Status::SpiTimeout && !adc.IsInitialized() && !adc.HasSample(),
            "ambiguous reset failure invalidates cached data and initialization");
    std::puts("PASS single-shot, low-power, negative offset, sleep and tick wraparound");
}

void TestConfigurationContract()
{
    chip = {};
    SPI_HandleTypeDef spi;
    ADS1220 adc(&spi, ADS1220::Config{});
    auto config = MakeConfig();
    config.gain = ADS1220::Gain::X16;
    Require(adc.Configure(config) == ADS1220::Status::Ok, "configuration can be staged before Init");
    Require(adc.Init() == ADS1220::Status::Ok && ReadNext(adc) == ADS1220::Status::Ok,
            "staged configuration used for first measurement");
    Require(Near(adc.GetVoltage(), ExpectedInput()), "staged gain used correctly");
    const float voltage = adc.GetVoltage();
    for (const float value : {0.0f, -1.0f, std::numeric_limits<float>::quiet_NaN(),
                              std::numeric_limits<float>::infinity()})
    {
        Require(adc.SetVref(value) == ADS1220::Status::InvalidArgument,
                "setter rejects nonpositive and nonfinite reference");
        config.reference_voltage = value;
        Require(adc.Configure(config) == ADS1220::Status::InvalidArgument,
                "full configuration rejects nonpositive and nonfinite reference");
        Require(adc.HasSample() && Near(adc.GetVoltage(), voltage), "invalid config preserves measurement");
    }
    Require(adc.SetVref(2.5f) == ADS1220::Status::Ok && !adc.HasSample(), "reference setter invalidates old scale");
    Require(ReadNext(adc) == ADS1220::Status::Ok && Near(adc.GetVoltage(), ExpectedInput(2.5f)),
            "updated external reference applied consistently");
    config = adc.GetConfig();
    config.input = ADS1220::InputMux::Ain0Avss;
    Require(adc.Configure(config) == ADS1220::Status::InvalidArgument, "invalid single-ended PGA config rejected");
    std::puts("PASS pre-init configuration, finite reference validation and scaling");
}

void TestSoftwareReferenceUpdate()
{
    SPI_HandleTypeDef spi;
    for (const auto rate : {ADS1220::DataRate::Normal20, ADS1220::DataRate::Duty5})
    {
        for (const auto reference : {ADS1220::VoltageReference::ExternalRef0,
                                     ADS1220::VoltageReference::Internal2048})
        {
            chip = {};
            auto config = MakeConfig();
            config.data_rate = rate;
            config.voltage_reference = reference;
            ADS1220 adc(&spi, config);
            Require(adc.Init() == ADS1220::Status::Ok && ReadNext(adc) == ADS1220::Status::Ok,
                    "software reference setup includes valid ADC offset calibration");
            const int32_t corrected_code = adc.GetRawValue();
            const unsigned calibration_reads = chip.short_reads;

            const uint64_t before_setter = chip.now;
            Require(adc.SetVref(2.5f) == ADS1220::Status::Ok, "software reference setter succeeds");
            Require(chip.now == before_setter && chip.short_reads == calibration_reads,
                    "SetVref does not wait for calibration or acquire shorted samples");
            Require(adc.GetConfig().reference_voltage == 2.5f, "new software reference saved");
            const float setter_reference = reference == ADS1220::VoltageReference::Internal2048
                                               ? 2.048f : 2.5f;
            Require(ReadNext(adc) == ADS1220::Status::Ok && adc.GetRawValue() == corrected_code &&
                    Near(adc.GetVoltage(), ExpectedInput(setter_reference)),
                    "setter preserves offset code and changes only applicable voltage scale");

            config = adc.GetConfig();
            config.reference_voltage = 4.75f;
            const uint64_t before_configure = chip.now;
            Require(adc.Configure(config) == ADS1220::Status::Ok, "direct software reference update succeeds");
            Require(chip.now == before_configure && chip.short_reads == calibration_reads,
                    "Configure reference-only change does not recalibrate at either data rate");
            const float configured_reference = reference == ADS1220::VoltageReference::Internal2048
                                                   ? 2.048f : 4.75f;
            Require(ReadNext(adc) == ADS1220::Status::Ok && adc.GetRawValue() == corrected_code &&
                    Near(adc.GetVoltage(), ExpectedInput(configured_reference)),
                    "direct update retains offset and internal reference ignores software coefficient");
        }
    }
    std::puts("PASS reference-only updates: no calibration delay, preserved offset, Normal20/Duty5, external/internal reference");
}

} // namespace

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*, uint8_t* data, uint16_t size, uint32_t)
{
    Advance();
    if (size == 5 && data[0] == 0x43)
    {
        if (chip.fail_all_writes)
        {
            return HAL_TIMEOUT;
        }
        if (chip.partial_write_failures != 0)
        {
            --chip.partial_write_failures;
            // Simulate an actual partial hardware change, not a simple rejected write.
            chip.registers[0] = data[1];
            chip.registers[1] = data[2];
            return HAL_TIMEOUT;
        }
        for (unsigned i = 0; i < 4; ++i)
        {
            chip.registers[i] = data[i + 1];
        }
        chip.running = true;
        chip.ready_at = chip.now + ConversionPeriodMs();
    }
    else if (size == 1)
    {
        chip.command = data[0];
        if (chip.command == 0x08)
        {
            chip.running = true;
            chip.ready_at = chip.now + ConversionPeriodMs();
        }
        else if (chip.command == 0x06)
        {
            chip.registers = {};
            chip.running = false;
            if (chip.fail_reset_once)
            {
                chip.fail_reset_once = false;
                return HAL_TIMEOUT;
            }
        }
        else if (chip.command == 0x02)
        {
            chip.running = false;
        }
        else if (chip.command == 0x10)
        {
            chip.latched = chip.output; // RDATA latches at command time, not receive time.
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef*, uint8_t* data, uint16_t size, uint32_t)
{
    Advance();
    if (chip.command == 0x23 && size == 4)
    {
        for (unsigned i = 0; i < 4; ++i)
        {
            data[i] = chip.registers[i];
        }
        if (chip.mismatch_once)
        {
            chip.mismatch_once = false;
            data[3] ^= 2U;
        }
    }
    else if (chip.command == 0x10 && size == 3)
    {
        if (chip.data_timeouts != 0)
        {
            --chip.data_timeouts;
            return HAL_TIMEOUT;
        }
        const bool shorted = (chip.registers[0] >> 4U) == 14;
        if (shorted && ++chip.short_reads == chip.fail_short_read)
        {
            return HAL_TIMEOUT;
        }
        const int32_t noise = shorted ? (chip.short_reads % 2 == 0 ? -1 : 1) : 0;
        const uint32_t packed = static_cast<uint32_t>(chip.latched + noise) & 0xFFFFFFU;
        data[0] = static_cast<uint8_t>(packed >> 16U);
        data[1] = static_cast<uint8_t>(packed >> 8U);
        data[2] = static_cast<uint8_t>(packed);
    }
    else
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

void HAL_Delay(uint32_t ms)
{
    chip.now += ms;
    Advance();
}

uint32_t HAL_GetTick()
{
    return static_cast<uint32_t>(chip.now);
}

int main()
{
    TestNonblockingUpdate();
    TestCalibration();
    TestConfigRecovery();
    TestFreshSamplesAndCalibrationKeys();
    TestMonitorAndTemperature();
    TestPowerAndTiming();
    TestConfigurationContract();
    TestSoftwareReferenceUpdate();
    std::puts("All ADS1220 regression scenarios passed.");
}
