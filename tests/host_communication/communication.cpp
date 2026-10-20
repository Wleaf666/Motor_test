#include "HostCommunication.hpp"

#include <cstdio>
#include <cstdlib>
#include <limits>

namespace
{
void Check(bool condition, const char* reason)
{
    if (!condition)
    {
        std::fprintf(stderr, "FAIL: %s\n", reason);
        std::exit(1);
    }
}

struct UartModel
{
    uint8_t* rx_data = nullptr;
    uint16_t rx_size = 0;
    uint16_t rx_position = 0;
    bool rx_dma = false;
    const uint8_t* tx_data = nullptr;
    uint16_t tx_size = 0;
    HAL_StatusTypeDef next_tx_status = HAL_OK;
    HAL_StatusTypeDef abort_tx_status = HAL_OK;
} model;

constexpr host_protocol::Frame CommandGolden{
    0xA5, 0x01, 0x01, 0x07, 0x00, 0x00, 0xD0, 0x40, 0, 0, 0, 0, 0x22, 0xFD};
constexpr host_protocol::Frame FeedbackGolden{
    0xA5, 0x01, 0x81, 0x00, 0x00, 0x00, 0xC8, 0x40, 0x00, 0x00, 0xC0, 0xBF, 0x1B, 0xFF};

host_protocol::Frame MakeCommand(float target)
{
    auto frame = CommandGolden;
    host_protocol::WriteFloat(frame.data() + 4, target);
    host_protocol::FinishFrame(frame);
    return frame;
}

// Feed actual UartRxSync IT/header and circular DMA states byte by byte.
void Feed(UART_HandleTypeDef& uart, const uint8_t* data, std::size_t size)
{
    for (std::size_t index = 0; index < size; ++index)
    {
        Check(model.rx_data != nullptr && model.rx_size != 0, "receiver is armed");
        model.rx_data[model.rx_position++] = data[index];
        if (model.rx_position == model.rx_size)
        {
            const bool circular = model.rx_dma;
            model.rx_position = 0;
            if (!circular)
            {
                model.rx_data = nullptr;
                model.rx_size = 0;
            }
            uart.callbacks[HAL_UART_RX_COMPLETE_CB_ID](&uart);
        }
    }
}

void Feed(UART_HandleTypeDef& uart, const host_protocol::Frame& frame)
{
    Feed(uart, frame.data(), frame.size());
}

void TxComplete(UART_HandleTypeDef& uart)
{
    model.tx_data = nullptr;
    model.tx_size = 0;
    uart.callbacks[HAL_UART_TX_COMPLETE_CB_ID](&uart);
}

void TestCodec()
{
    const uint8_t vector[] = {'1','2','3','4','5','6','7','8','9'};
    Check(host_protocol::FrameCrc::calc(vector, sizeof(vector)) == 0x29B1, "CCITT-FALSE known check");
    float target = -1;
    Check(host_protocol::DecodeTarget(CommandGolden, 18, target) && target == 6.5f,
          "Python-generated little-endian command golden frame");
    for (std::size_t byte = 0; byte < CommandGolden.size(); ++byte)
    {
        for (unsigned bit = 0; bit < 8; ++bit)
        {
            auto corrupt = CommandGolden;
            corrupt[byte] ^= static_cast<uint8_t>(1U << bit);
            target = -1;
            Check(!host_protocol::DecodeTarget(corrupt, 18, target) && target == -1,
                  "single-bit corruption never changes target");
        }
    }
    for (const float value : {-0.1f, 18.1f, std::numeric_limits<float>::infinity(),
                              std::numeric_limits<float>::quiet_NaN()})
    {
        target = -1;
        Check(!host_protocol::DecodeTarget(MakeCommand(value), 18, target) && target == -1,
              "CRC-valid invalid target rejected");
    }
    for (const float value : {0.0f, 18.0f})
    {
        Check(host_protocol::DecodeTarget(MakeCommand(value), 18, target) && target == value,
              "voltage range endpoints accepted");
    }
    auto reserved = CommandGolden;
    reserved[8] = 1;
    host_protocol::FinishFrame(reserved);
    Check(!host_protocol::DecodeTarget(reserved, 18, target), "nonzero reserved payload rejected");
    auto wrong_type = CommandGolden;
    wrong_type[2] = host_protocol::Feedback;
    host_protocol::FinishFrame(wrong_type);
    Check(!host_protocol::DecodeTarget(wrong_type, 18, target), "feedback cannot set output voltage");
    auto wrong_version = CommandGolden;
    wrong_version[1] = 2;
    host_protocol::FinishFrame(wrong_version);
    Check(!host_protocol::DecodeTarget(wrong_version, 18, target), "unsupported version rejected");
    host_protocol::Frame feedback;
    Check(host_protocol::EncodeFeedback(6.25f, -1.5f, 0, feedback) && feedback == FeedbackGolden,
          "feedback matches independent Python CRC and float serialization");
    Check(!host_protocol::EncodeFeedback(0, std::numeric_limits<float>::quiet_NaN(), 0, feedback),
          "nonfinite sensor result not transmitted");
    std::puts("PASS codec golden vectors, bit corruption, range, nonfinite and reserved fields");
}

void TestTransport()
{
    DMA_HandleTypeDef rx, tx;
    rx.Init.Mode = DMA_CIRCULAR;
    tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    UART_HandleTypeDef uart{&rx, &tx};
    static HostCommunication host(&uart, 18.0f);
    Check(host.Init(), "UART callback registration and initial IT receive");
    const uint8_t noise[] = {0, 0x33, 0x5A};
    Feed(uart, noise, sizeof(noise));
    Feed(uart, CommandGolden.data(), 5); // Fragmented USB/UART writes.
    float target = -1;
    Check(!host.TakeTarget(target) && target == -1, "partial command cannot update target");
    Feed(uart, CommandGolden.data() + 5, CommandGolden.size() - 5);
    Check(host.TakeTarget(target) && target == 6.5f, "first frame decoded after header and DMA remainder");
    Check(!host.TakeTarget(target), "command mailbox consumed once");
    Feed(uart, MakeCommand(8));
    Feed(uart, MakeCommand(12));
    auto bad = MakeCommand(16);
    bad[4] ^= 1;
    Feed(uart, bad);
    Check(host.TakeTarget(target) && target == 12, "latest valid target survives later corrupt frame");
    Feed(uart, MakeCommand(19));
    Check(!host.TakeTarget(target), "out-of-range DMA command ignored");
    const uint8_t slip = 0x13;
    Feed(uart, &slip, 1);
    Feed(uart, CommandGolden); // One extra byte breaks frame alignment.
    Feed(uart, CommandGolden);
    Check(host.TakeTarget(target) && target == 6.5f, "header search recovers after stream misalignment");

    Check(host.SendFeedback(6.25f, -1.5f) == HAL_OK && model.tx_size == FeedbackGolden.size(), "TX DMA submitted");
    Check(std::memcmp(model.tx_data, FeedbackGolden.data(), FeedbackGolden.size()) == 0,
          "TX DMA reads persistent correctly encoded feedback");
    const auto* dma_pointer = model.tx_data;
    Check(host.SendFeedback(10, 20) == HAL_BUSY && model.tx_data == dma_pointer &&
          std::memcmp(model.tx_data, FeedbackGolden.data(), FeedbackGolden.size()) == 0,
          "busy sender cannot overwrite active DMA storage");
    uart.ErrorCode = HAL_UART_ERROR_ORE;
    uart.callbacks[HAL_UART_ERROR_CB_ID](&uart);
    Check(host.SendFeedback(0, 0) == HAL_BUSY, "RX error recovery leaves active TX alone");
    Feed(uart, MakeCommand(9));
    Check(host.TakeTarget(target) && target == 9, "receive error restarts header synchronization");
    TxComplete(uart);
    model.next_tx_status = HAL_BUSY;
    Check(host.SendFeedback(1, 2) == HAL_BUSY, "HAL start failure releases unsubmitted TX buffer");
    Check(host.SendFeedback(1, 2) == HAL_OK && model.tx_data[3] == 1,
          "next successful DMA uses next sequence without advancing on failed start");
    uart.ErrorCode = HAL_UART_ERROR_DMA;
    tx.ErrorCode = 1;
    model.abort_tx_status = HAL_ERROR;
    uart.callbacks[HAL_UART_ERROR_CB_ID](&uart);
    Check(host.SendFeedback(0, 0) == HAL_BUSY, "failed TX abort cannot release DMA buffer");
    model.abort_tx_status = HAL_OK;
    uart.callbacks[HAL_UART_ERROR_CB_ID](&uart);
    Check(host.SendFeedback(2, 3) == HAL_OK, "TX DMA error can release buffer after successful abort");
    TxComplete(uart);
    Check(host.SendFeedback(0, std::numeric_limits<float>::infinity()) == HAL_ERROR,
          "invalid feedback rejected without arming DMA");
    Check(host.SendFeedback(0, 0) == HAL_OK, "invalid feedback does not wedge sender");
    TxComplete(uart);
    std::puts("PASS real IT/DMA RX state machine, stream recovery, target mailbox and TX ownership");
}
} // namespace

HAL_StatusTypeDef HAL_UART_RegisterCallback(UART_HandleTypeDef* uart, HAL_UART_CallbackIDTypeDef id, UartCallback callback)
{
    uart->callbacks[id] = callback;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef*, uint8_t* data, uint16_t size)
{
    model.rx_data = data; model.rx_size = size; model.rx_position = 0; model.rx_dma = false;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef*, uint8_t* data, uint16_t size)
{
    model.rx_data = data; model.rx_size = size; model.rx_position = 0; model.rx_dma = true;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef*)
{
    model.rx_data = nullptr; model.rx_size = model.rx_position = 0;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef*)
{
    if (model.abort_tx_status == HAL_OK) { model.tx_data = nullptr; model.tx_size = 0; }
    return model.abort_tx_status;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef*, const uint8_t* data, uint16_t size)
{
    const auto status = model.next_tx_status;
    model.next_tx_status = HAL_OK;
    if (status == HAL_OK) { model.tx_data = data; model.tx_size = size; }
    return status;
}
int main()
{
    TestCodec();
    TestTransport();
}
