#include "ADS1220.hpp"

ADS1220::ADS1220(SPI_HandleTypeDef* hspi, Gain gain, float Vref) :
    hspi_(hspi), gain_(gain), Vref_(Vref)
{
}

HAL_StatusTypeDef ADS1220::SendCommand(Cmd cmd)
{
    uint8_t tmp = static_cast<uint8_t>(cmd);
    return HAL_SPI_Transmit(hspi_, &tmp, 1, 100);
}

void ADS1220::Init()
{
    uint8_t messages[5];

    // 复位 ADS1220
    SendCommand(Cmd::RESET);
    HAL_Delay(1);

    // 配置寄存器 0：GAIN[2:0] 位于 bit3:1，bit0 为 PGA_BYPASS。
    // 当前使用差分输入并启用 PGA，因此增益编码需要左移 1 位。
    messages[0] = static_cast<uint8_t>(Cmd::WREG) | 0x03;
    messages[1] = static_cast<uint8_t>(gain_) << 1;
    messages[2] = 0x84; // 20 SPS，正常模式，连续转换
    messages[3] = 0x40; // 外部 REFP0/REFN0，50/60Hz 抑制
    messages[4] = 0x00;
    HAL_SPI_Transmit(hspi_, messages, 5, 100);

    HAL_Delay(1);

    // 启动连续转换
    SendCommand(Cmd::START);
}

void ADS1220::Update()
{
    uint8_t cmd = static_cast<uint8_t>(Cmd::RDATA);
    uint8_t messages[3];

    HAL_SPI_Transmit(hspi_, &cmd, 1, 10);
    HAL_SPI_Receive(hspi_, messages, 3, 10);

    int32_t value = ((int32_t)messages[0] << 16) | ((int32_t)messages[1] << 8) | messages[2];
    // 24位补码符号扩展
    if (value & 0x800000)
    {
        value |= 0xFF000000;
    }

    RawValue_ = value;
    Voltage_  = RawValue_ * Vref_ / scale_;
}
