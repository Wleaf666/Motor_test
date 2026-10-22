#include "HostCommunication.hpp"

HostCommunication* HostCommunication::Instance_ = nullptr;

HostCommunication::HostCommunication(UART_HandleTypeDef* huart, float max_voltage) :
    Receiver(huart, RxBuffer_), MaxVoltage_(max_voltage)
{
}

bool HostCommunication::Init()
{
    if (Initialized_ || (Instance_ != nullptr && Instance_ != this) || !CheckConfig())
    {
        return false;
    }

    Instance_ = this;
    if (!RegisterCallbacks() || !startReceive())
    {
        HAL_UART_AbortReceive(huart());
        Instance_ = nullptr;
        return false;
    }
    Initialized_ = true;
    return true;
}

bool HostCommunication::TakeCommand(host_protocol::Command& command)
{
    if (HasPendingResponse())
    {
        return false;
    }
    const uint32_t read = CommandRead_.load(std::memory_order_relaxed);
    if (read == CommandWrite_.load(std::memory_order_acquire))
    {
        return false;
    }
    command = Commands_[read % CommandCapacity];
    CommandRead_.store(read + 1U, std::memory_order_release);
    return true;
}

bool HostCommunication::PublishResponse(const host_protocol::Response& response)
{
    if (HasPendingResponse() || !host_protocol::EncodeResponse(response, PendingResponse_))
    {
        return false;
    }
    ResponsePending_.store(true, std::memory_order_release);
    return true;
}

bool HostCommunication::HasPendingResponse() const
{
    return ResponsePending_.load(std::memory_order_acquire);
}

HAL_StatusTypeDef HostCommunication::SendPendingResponse()
{
    if (!Initialized_)
    {
        return HAL_ERROR;
    }
    if (!HasPendingResponse())
    {
        return HAL_OK;
    }
    bool expected = false;
    if (!TxBusy_.compare_exchange_strong(expected, true, std::memory_order_acq_rel))
    {
        return HAL_BUSY;
    }
    TxBuffer_ = PendingResponse_;
    const auto status = HAL_UART_Transmit_DMA(huart(), TxBuffer_.data(), TxBuffer_.size());
    if (status == HAL_OK)
    {
        ResponsePending_.store(false, std::memory_order_release);
    }
    else
    {
        TxBusy_.store(false, std::memory_order_release);
    }
    return status;
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
    host_protocol::Command command;
    if (!host_protocol::DecodeCommand(frame, command) ||
        (command.type == host_protocol::CommandType::SetVoltage &&
         (!std::isfinite(command.value) || command.value < 0.0f || command.value > MaxVoltage_)))
    {
        return false;
    }
    const uint32_t write = CommandWrite_.load(std::memory_order_relaxed);
    if (write - CommandRead_.load(std::memory_order_acquire) == CommandCapacity)
    {
        return false; // Preserve all eight older commands; drop the incoming frame.
    }
    Commands_[write % CommandCapacity] = command;
    CommandWrite_.store(write + 1U, std::memory_order_release);
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

bool HostCommunication::CheckConfig() const
{
    const auto* uart = huart();
    if (uart == nullptr || uart->hdmarx == nullptr || uart->hdmatx == nullptr ||
        !std::isfinite(MaxVoltage_) || MaxVoltage_ <= 0.0f)
    {
        return false;
    }

    const auto& rx = uart->hdmarx->Init;
    const auto& tx = uart->hdmatx->Init;
    // CubeMX 负责配置；这里只确认当前协议所需的 RX 循环 / TX 单次字节传输。
    return rx.Mode == DMA_CIRCULAR && tx.Mode == DMA_NORMAL &&
           rx.Direction == DMA_PERIPH_TO_MEMORY && tx.Direction == DMA_MEMORY_TO_PERIPH &&
           rx.MemDataAlignment == DMA_MDATAALIGN_BYTE && tx.MemDataAlignment == DMA_MDATAALIGN_BYTE &&
           rx.PeriphDataAlignment == DMA_PDATAALIGN_BYTE && tx.PeriphDataAlignment == DMA_PDATAALIGN_BYTE &&
           rx.MemInc == DMA_MINC_ENABLE && tx.MemInc == DMA_MINC_ENABLE &&
           rx.PeriphInc == DMA_PINC_DISABLE && tx.PeriphInc == DMA_PINC_DISABLE;
}

bool HostCommunication::RegisterCallbacks()
{
    auto* uart = huart();
    return HAL_UART_RegisterCallback(uart, HAL_UART_RX_COMPLETE_CB_ID, RxCompleteCallback) == HAL_OK &&
           HAL_UART_RegisterCallback(uart, HAL_UART_TX_COMPLETE_CB_ID, TxCompleteCallback) == HAL_OK &&
           HAL_UART_RegisterCallback(uart, HAL_UART_ERROR_CB_ID, UartErrorCallback) == HAL_OK;
}

HostCommunication* HostCommunication::FromHandle(UART_HandleTypeDef* uart)
{
    return Instance_ != nullptr && Instance_->huart() == uart ? Instance_ : nullptr;
}

void HostCommunication::RxCompleteCallback(UART_HandleTypeDef* uart)
{
    if (auto* host = FromHandle(uart))
    {
        host->receiveCallback();
    }
}

void HostCommunication::TxCompleteCallback(UART_HandleTypeDef* uart)
{
    if (auto* host = FromHandle(uart))
    {
        host->TxBusy_.store(false, std::memory_order_release);
    }
}

void HostCommunication::UartErrorCallback(UART_HandleTypeDef* uart)
{
    if (auto* host = FromHandle(uart))
    {
        host->ErrorCallback();
    }
}
