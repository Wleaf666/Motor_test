#pragma once

#include "main.h"

class ADS1220
{
public:
    enum class Gain : uint8_t
    {
        X1   = 0,
        X2   = 1,
        X4   = 2,
        X8   = 3,
        X16  = 4,
        X32  = 5,
        X64  = 6,
        X128 = 7,
    };

    ADS1220(SPI_HandleTypeDef* hspi, Gain gain, float Vref);
    void Init();
    void Update();

    int32_t GetRawValue() const { return RawValue_; }
    float   GetVoltage() const { return Voltage_; }

    void SetVref(float Vref) { Vref_ = Vref; }
    void SetGain(Gain gain) { gain_ = gain; }

private:
    enum class Cmd : uint8_t
    {
        RESET     = 0x06,
        START     = 0x08,
        POWERDOWN = 0x02,
        RDATA     = 0x10,
        RREG      = 0x20,
        WREG      = 0x40,
    };

    HAL_StatusTypeDef SendCommand(Cmd cmd);

    int32_t            scale_ = 1 << 24;
    SPI_HandleTypeDef* hspi_  = nullptr;
    Gain               gain_;
    float              Vref_;
    int32_t            RawValue_ = 0;
    float              Voltage_  = 0.0f;
};