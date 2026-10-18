// Runs the production driver against a deterministic, time-driven ADC model.
// From the repository root:
// g++ -std=c++17 -Wall -Wextra -Werror -Itests/ads1220 -IUserCode/ADS1220
//     UserCode/ADS1220/ADS1220.cpp tests/ads1220/calibration.cpp -o build/ads1220-calibration.exe
// Then run build/ads1220-calibration.exe. No board or real-time sleeps required.

#include "ADS1220.hpp"
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {
std::array<uint8_t, 4> regs{};
uint64_t now = 0, ready_at = 0;
int32_t offset = 1024, signal = 12345, output = 5000;
int samples = 0, fail_sample = 0;
uint8_t command = 0;
bool running = false, fail_restore = false, fail_init = false;
void Require(bool condition, const char* message) {
    if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
}
uint32_t Period() {
    const uint32_t normal_us[] = {50000, 22223, 11112, 5715, 3031, 1667, 1000};
    const uint32_t duty_us[] = {200000, 88889, 44445, 22728, 12122, 6667, 4000};
    const uint32_t turbo_us[] = {25000, 11112, 5556, 2858, 1516, 834, 500};
    unsigned dr = regs[1] >> 5;
    unsigned mode = (regs[1] >> 3) & 3;
    const uint32_t us = mode == 1 ? duty_us[dr] : mode == 2 ? turbo_us[dr] : normal_us[dr];
    return (us + 999) / 1000 + 1;
}
void Advance() {
    if (running && now >= ready_at) {
        if ((regs[1] & 2) != 0) output = 25 * 32 * 1024;
        else output = (regs[0] >> 4) == 14 ? offset : signal + offset;
        running = (regs[1] & 4) != 0;
        ready_at = now + Period();
    }
}
ADS1220::Config MakeConfig() {
    ADS1220::Config cfg{};
    cfg.input = ADS1220::InputMux::Ain0Ain1;
    cfg.gain = ADS1220::Gain::X128;
    cfg.data_rate = ADS1220::DataRate::Normal20;
    cfg.conversion_mode = ADS1220::ConversionMode::Continuous;
    cfg.voltage_reference = ADS1220::VoltageReference::ExternalRef0;
    cfg.reference_voltage = 5.0f;
    cfg.filter = ADS1220::FirFilter::Reject50And60;
    return cfg;
}
void ResetModel() {
    regs = {}; now = ready_at = 0; output = 5000; offset = 1024;
    samples = fail_sample = 0; command = 0; running = fail_restore = fail_init = false;
}
}
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*, uint8_t* data, uint16_t size, uint32_t) {
    if (size == 5 && data[0] == 0x43) {
        if (fail_init) return HAL_TIMEOUT;
        if (fail_restore && (regs[0] >> 4) == 14 && (data[1] >> 4) != 14) return HAL_TIMEOUT;
        for (unsigned i = 0; i < 4; ++i) regs[i] = data[i + 1];
        running = true; ready_at = now + Period();
    } else if (size == 1) {
        command = data[0];
        if (command == 0x08) { running = true; ready_at = now + Period(); }
        if (command == 0x06) { regs = {}; running = false; }
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef*, uint8_t* data, uint16_t size, uint32_t) {
    Advance();
    if (command == 0x23 && size == 4) {
        for (unsigned i = 0; i < 4; ++i) data[i] = regs[i];
    } else if (command == 0x10 && size == 3) {
        if ((regs[0] >> 4) == 14 && ++samples == fail_sample) return HAL_TIMEOUT;
        // Alternating noise has zero mean over an even number of fresh samples.
        const int32_t noise = (regs[0] >> 4) == 14 ? (samples % 2 == 0 ? -1 : 1) : 0;
        uint32_t packed = static_cast<uint32_t>(output + noise) & 0xFFFFFF;
        data[0] = packed >> 16; data[1] = packed >> 8; data[2] = packed;
    } else return HAL_ERROR;
    return HAL_OK;
}
void HAL_Delay(uint32_t ms) { now += ms; Advance(); }
int main() {
    SPI_HandleTypeDef spi;
    auto cfg = MakeConfig();
    ADS1220 adc(&spi, cfg);
    Require(adc.Init() == ADS1220::Status::Ok, "startup calibration succeeds");
    HAL_Delay(60);
    Require(adc.Update() == ADS1220::Status::Ok, "normal sample succeeds");
    std::printf("After startup: corrected code=%ld, expected=%ld, virtual time=%lu ms\n",
                static_cast<long>(adc.GetRawValue()), static_cast<long>(signal),
                static_cast<unsigned long>(now));
    Require(adc.GetRawValue() == signal, "startup removes offset using fresh conversions");
    Require(std::fabs(adc.GetVoltage() - signal * 5.0f / (128 * 8388608.0f)) < 1e-8f,
            "input voltage uses corrected code");
    Require(std::fabs(adc.GetGainVoltage() - signal * 5.0f / 8388608.0f) < 1e-7f,
            "gain voltage uses corrected code");
    Require(adc.Calibrate() == ADS1220::Status::Ok, "repeat calibration succeeds");
    Require(adc.Update() == ADS1220::Status::Ok && adc.GetRawValue() == signal,
            "repeat calibration does not subtract previous offset twice");
    const int32_t cached = adc.GetRawValue();
    fail_sample = samples + 3;
    Require(adc.Calibrate() == ADS1220::Status::SpiTimeout, "sample error propagates");
    Require(adc.GetRawValue() == cached, "failed calibration preserves measurement cache");
    Require(adc.Update() == ADS1220::Status::Ok && adc.GetRawValue() == signal,
            "failed calibration retains previous offset");
    fail_sample = 0; fail_restore = true;
    Require(adc.Calibrate() == ADS1220::Status::SpiTimeout, "restore error propagates");
    Require(!adc.IsInitialized(), "restore failure prohibits sampling with wrong mux");
    ResetModel(); fail_init = true;
    ADS1220 failed(&spi, cfg);
    Require(failed.Init() == ADS1220::Status::SpiTimeout && !failed.IsInitialized(),
            "init failure not masked by calibration");
    ResetModel(); fail_sample = 3;
    ADS1220 failed_calibration(&spi, cfg);
    Require(failed_calibration.Init() == ADS1220::Status::SpiTimeout &&
            !failed_calibration.IsInitialized(), "startup calibration failure blocks business sampling");

    ResetModel(); cfg.conversion_mode = ADS1220::ConversionMode::SingleShot;
    cfg.filter = ADS1220::FirFilter::None; cfg.data_rate = ADS1220::DataRate::Duty5;
    offset = -2000;
    ADS1220 slow(&spi, cfg);
    Require(slow.Init() == ADS1220::Status::Ok, "single shot low power calibration succeeds");
    Require(slow.Update() == ADS1220::Status::Ok && slow.GetRawValue() == signal,
            "negative offset at slowest rate and single shot handled");
    ResetModel(); cfg.temperature_sensor = true;
    ADS1220 temp(&spi, cfg);
    Require(temp.Init() == ADS1220::Status::Ok && samples == 0, "temperature startup skips ADC offset");
    Require(temp.Calibrate() == ADS1220::Status::InvalidArgument, "temperature calibration rejected");
    std::puts("PASS: startup, fresh samples, restoration, repeat, failures, negative offset, single shot, temperature");
}
