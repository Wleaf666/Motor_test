#pragma once

#include "HostProtocol.hpp"
#include "UartRxSync.hpp"

#include <atomic>

/**
 * @brief 固定帧上位机通信：有序命令、单待发应答及电压/扭矩反馈。
 *
 * 一个长期存活的对象独占 USART 及 RX/TX 回调，Init 在启动任务前调用。
 * 一个任务顺序调用 TakeCommand/PublishResponse，一个发送任务独占两种 Send 方法。
 * ISR 只验证帧并入队，不执行控制，不分配堆内存；FIFO 满时丢弃新命令。
 */
class HostCommunication final : private protocol::UartRxSync<1, host_protocol::FrameSize, true>
{
public:
    HostCommunication(UART_HandleTypeDef* huart, float max_voltage);
    HostCommunication(const HostCommunication&) = delete;
    HostCommunication& operator=(const HostCommunication&) = delete;

    bool Init();
    bool TakeCommand(host_protocol::Command& command);
    // 单生产者；已有待发应答或 value 非有限数时返回 false，不覆盖原应答。
    bool PublishResponse(const host_protocol::Response& response);
    bool HasPendingResponse() const;
    // 无应答时返回 HAL_OK；提交失败保留应答，成功提交即释放应答槽位。
    HAL_StatusTypeDef SendPendingResponse();
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
                  "ISR command FIFO requires lock-free 32-bit atomic operations");
    static_assert(std::atomic<bool>::is_always_lock_free,
                  "DMA ownership requires lock-free boolean atomic operations");
    static constexpr uint32_t CommandCapacity = 8;
    static HostCommunication* Instance_;

    Receiver::Buffer RxBuffer_;
    host_protocol::Frame TxBuffer_{};
    float MaxVoltage_;
    std::array<host_protocol::Command, CommandCapacity> Commands_{};
    // ISR owns write, command task owns read. Unsigned distance stays in [0, 8].
    std::atomic<uint32_t> CommandWrite_{0};
    std::atomic<uint32_t> CommandRead_{0};
    host_protocol::Frame PendingResponse_{};
    std::atomic<bool> ResponsePending_{false};
    std::atomic<bool> TxBusy_{false};
    bool Initialized_ = false;
    uint8_t TxSequence_ = 0;
};
