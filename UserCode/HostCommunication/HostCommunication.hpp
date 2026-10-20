#pragma once

#include "HostProtocol.hpp"
#include "UartRxSync.hpp"

#include <atomic>

/**
 * @brief 固定帧上位机通信，接收电压目标，DMA 发送电压/扭矩测量值。
 *
 * 一个静态对象独占 USART 及 RX/TX 回调，Init 在启动任务前调用。
 * TakeTarget 由接收任务消费，SendFeedback 由单个发送任务调用。
 * ISR 只验证帧和发布最新目标，不直接调用 PID；不分配堆内存。
 */
class HostCommunication final : private protocol::UartRxSync<1, host_protocol::FrameSize, true>
{
public:
    HostCommunication(UART_HandleTypeDef* huart, float max_voltage);
    HostCommunication(const HostCommunication&) = delete;
    HostCommunication& operator=(const HostCommunication&) = delete;

    bool Init();
    bool TakeTarget(float& voltage);
    // HAL_BUSY 表示上一帧仍在发送；调用方跳过本周期，不覆盖 DMA 缓冲。
    HAL_StatusTypeDef SendFeedback(float voltage, float torque);

private:
    using Receiver = protocol::UartRxSync<1, host_protocol::FrameSize, true>;

    bool CheckConfig() const;
    bool RegisterCallbacks();
    static HostCommunication* FromHandle(UART_HandleTypeDef* huart);
    static void RxCompleteCallback(UART_HandleTypeDef* huart);
    static void TxCompleteCallback(UART_HandleTypeDef* huart);
    static void UartErrorCallback(UART_HandleTypeDef* huart);

    const std::array<uint8_t, 1>& header() const override;
    bool decode(const uint8_t* data) override;
    void ErrorCallback();

    static_assert(std::atomic<uint32_t>::is_always_lock_free,
                  "ISR target mailbox requires lock-free 32-bit atomic operations");
    // 0xFFFFFFFF 为 NaN 位模式，合法有限电压不会与“无待处理目标”冲突。
    static constexpr uint32_t NoTarget = 0xFFFFFFFFU;
    static HostCommunication* Instance_;

    Receiver::Buffer RxBuffer_;
    host_protocol::Frame TxBuffer_{};
    float MaxVoltage_;
    std::atomic<uint32_t> PendingTarget_{NoTarget};
    std::atomic<bool> TxBusy_{false};
    bool Initialized_ = false;
    uint8_t TxSequence_ = 0;
};
