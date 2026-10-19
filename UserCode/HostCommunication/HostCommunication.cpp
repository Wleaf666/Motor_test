#include "HostCommunication.hpp"

HostCommunication* HostCommunication::Instance_ = nullptr;

HostCommunication::HostCommunication(UART_HandleTypeDef* huart, float max_voltage) :
    Receiver(huart, RxBuffer_), MaxVoltage_(max_voltage)
{
}

bool HostCommunication::Init()
{
    auto* uart = huart();
    if (Initialized_ || (Instance_ != nullptr && Instance_ != this) || uart == nullptr ||
        uart->hdmarx == nullptr || uart->hdmatx == nullptr || !std::isfinite(MaxVoltage_) ||
        MaxVoltage_ <= 0.0f || uart->hdmarx->Init.Mode != DMA_CIRCULAR ||
        uart->hdmatx->Init.Mode != DMA_NORMAL ||
        uart->hdmarx->Init.Direction != DMA_PERIPH_TO_MEMORY ||
        uart->hdmatx->Init.Direction != DMA_MEMORY_TO_PERIPH ||
        uart->hdmarx->Init.MemDataAlignment != DMA_MDATAALIGN_BYTE ||
        uart->hdmatx->Init.MemDataAlignment != DMA_MDATAALIGN_BYTE ||
        uart->hdmarx->Init.PeriphDataAlignment != DMA_PDATAALIGN_BYTE ||
        uart->hdmatx->Init.PeriphDataAlignment != DMA_PDATAALIGN_BYTE ||
        uart->hdmarx->Init.MemInc != DMA_MINC_ENABLE ||
        uart->hdmatx->Init.MemInc != DMA_MINC_ENABLE ||
        uart->hdmarx->Init.PeriphInc != DMA_PINC_DISABLE ||
        uart->hdmatx->Init.PeriphInc != DMA_PINC_DISABLE)
    {
        return false;
    }

    Instance_ = this;
    const auto rx_status = HAL_UART_RegisterCallback(uart, HAL_UART_RX_COMPLETE_CB_ID,
        [](UART_HandleTypeDef* handle) {
            if (Instance_ != nullptr && Instance_->huart() == handle)
            {
                Instance_->receiveCallback();
            }
        });
    const auto tx_status = HAL_UART_RegisterCallback(uart, HAL_UART_TX_COMPLETE_CB_ID,
        [](UART_HandleTypeDef* handle) {
            if (Instance_ != nullptr && Instance_->huart() == handle)
            {
                Instance_->TxBusy_.store(false, std::memory_order_release);
            }
        });
    const auto error_status = HAL_UART_RegisterCallback(uart, HAL_UART_ERROR_CB_ID,
        [](UART_HandleTypeDef* handle) {
            if (Instance_ != nullptr && Instance_->huart() == handle)
            {
                Instance_->ErrorCallback();
            }
        });
    if (rx_status != HAL_OK || tx_status != HAL_OK || error_status != HAL_OK || !startReceive())
    {
        HAL_UART_AbortReceive(uart);
        Instance_ = nullptr;
        return false;
    }
    Initialized_ = true;
    return true;
}

bool HostCommunication::TakeTarget(float& voltage)
{
    const uint32_t bits = PendingTarget_.exchange(NoTarget, std::memory_order_acq_rel);
    if (bits == NoTarget)
    {
        return false;
    }
    std::memcpy(&voltage, &bits, sizeof(voltage));
    return true;
}

HAL_StatusTypeDef HostCommunication::SendFeedback(float voltage, float torque)
{
    if (!Initialized_)
    {
        return HAL_ERROR;
    }
    bool expected = false;
    if (!TxBusy_.compare_exchange_strong(expected, true, std::memory_order_acq_rel))
    {
        return HAL_BUSY;
    }
    if (!host_protocol::EncodeFeedback(voltage, torque, TxSequence_, TxBuffer_))
    {
        TxBusy_.store(false, std::memory_order_release);
        return HAL_ERROR;
    }
    const auto status = HAL_UART_Transmit_DMA(huart(), TxBuffer_.data(), TxBuffer_.size());
    if (status == HAL_OK)
    {
        ++TxSequence_;
    }
    else
    {
        TxBusy_.store(false, std::memory_order_release);
    }
    return status;
}

const std::array<uint8_t, 1>& HostCommunication::header() const
{
    static constexpr std::array<uint8_t, 1> value{host_protocol::Header};
    return value;
}

bool HostCommunication::decode(const uint8_t* data)
{
    // DMA 已开始接下一帧，先复制到固定大小栈快照，禁止保存 DMA 数据指针。
    host_protocol::Frame frame;
    std::memcpy(frame.data(), data, frame.size());
    float voltage;
    if (!host_protocol::DecodeTarget(frame, MaxVoltage_, voltage))
    {
        return false;
    }
    uint32_t bits;
    std::memcpy(&bits, &voltage, sizeof(bits));
    // 多条命令到达时保留最新值；无效帧不能覆盖上一条合法待处理命令。
    PendingTarget_.store(bits, std::memory_order_release);
    return true;
}

void HostCommunication::ErrorCallback()
{
    auto* uart = huart();
    const bool tx_dma_error = (uart->ErrorCode & HAL_UART_ERROR_DMA) != 0U &&
        uart->hdmatx != nullptr && uart->hdmatx->ErrorCode != HAL_DMA_ERROR_NONE;
    if (tx_dma_error && HAL_UART_AbortTransmit(uart) == HAL_OK)
    {
        // 只有确认 DMA 已停止后，才允许重用发送缓冲区。
        TxBusy_.store(false, std::memory_order_release);
    }
    errorHandler(); // 复用接收器的 RX 错误恢复，不干预正常 TX。
}
