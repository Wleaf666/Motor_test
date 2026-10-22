#include "HostCommunication.hpp"

#include <cstdio>
#include <cstdlib>
#include <limits>

namespace
{
using host_protocol::Command;
using host_protocol::CommandType;
using host_protocol::Frame;
using host_protocol::ParameterId;
using host_protocol::Response;
using host_protocol::Result;

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

// Generated independently with Python struct.pack and binascii.crc_hqx, not this codec.
struct GoldenCommand
{
    Frame frame;
    Command command;
};
constexpr std::array<GoldenCommand, 8> CommandGoldens{{
    {{0xA5, 0x01, 0x01, 0x07, 0x00, 0x00, 0xD0, 0x40, 0x00, 0x00, 0x00, 0x00, 0x22, 0xFD},
     {CommandType::SetVoltage, 7, 0, 6.5f}},
    {{0xA5, 0x01, 0x02, 0x08, 0x00, 0x00, 0x10, 0x40, 0x00, 0x00, 0x00, 0x00, 0x89, 0x6A},
     {CommandType::SetTorque, 8, 0, 2.25f}},
    {{0xA5, 0x01, 0x03, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xA2},
     {CommandType::SetTorqueEnabled, 9, 1, 0.0f}},
    {{0xA5, 0x01, 0x03, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0xC8},
     {CommandType::SetTorqueEnabled, 10, 0, 0.0f}},
    {{0xA5, 0x01, 0x10, 0x0B, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3D, 0x7D, 0xA9},
     {CommandType::SetParameter, 11, 5, 0.03125f}},
    {{0xA5, 0x01, 0x11, 0x0C, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6D, 0x90},
     {CommandType::GetParameter, 12, 5, 0.0f}},
    {{0xA5, 0x01, 0x12, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x82},
     {CommandType::SaveParameters, 13, 0, 0.0f}},
    {{0xA5, 0x01, 0x13, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0xC0},
     {CommandType::GetState, 14, 0, 0.0f}},
}};
constexpr Frame FeedbackGolden{
    0xA5, 0x01, 0x81, 0x00, 0x00, 0x00, 0xC8, 0x40, 0x00, 0x00, 0xC0, 0xBF, 0x1B, 0xFF};
struct GoldenResponse
{
    Frame frame;
    Response response;
};
constexpr std::array<GoldenResponse, 7> ResponseGoldens{{
    {{0xA5, 0x01, 0x82, 0x50, 0x00, 0x03, 0x10, 0x00, 0x00, 0x00, 0x00, 0x3D, 0x71, 0x85},
     {CommandType::SetParameter, 0x50, Result::Ok, 0.03125f, true, true}},
    {{0xA5, 0x01, 0x82, 0x51, 0x01, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDA, 0xE7},
     {CommandType::SetTorque, 0x51, Result::InvalidValue, 0.0f, false, true}},
    {{0xA5, 0x01, 0x82, 0x52, 0x02, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA8, 0x27},
     {CommandType::SetTorqueEnabled, 0x52, Result::NotReady, 0.0f, false, false}},
    {{0xA5, 0x01, 0x82, 0x53, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x10, 0x40, 0x2D, 0x93},
     {CommandType::SetVoltage, 0x53, Result::ModeConflict, 2.25f, true, true}},
    {{0xA5, 0x01, 0x82, 0x54, 0x04, 0x03, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4D, 0x7A},
     {CommandType::SaveParameters, 0x54, Result::OutputActive, 0.0f, true, true}},
    {{0xA5, 0x01, 0x82, 0x55, 0x05, 0x02, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDC, 0x6E},
     {CommandType::SaveParameters, 0x55, Result::StorageError, 0.0f, false, true}},
    {{0xA5, 0x01, 0x82, 0x56, 0x06, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAE, 0xAE},
     {CommandType::GetState, 0x56, Result::HardwareError, 0.0f, false, false}},
}};

bool SameCommand(const Command& left, const Command& right)
{
    return left.type == right.type && left.sequence == right.sequence &&
           left.argument == right.argument && left.value == right.value;
}

Frame MakeCommand(CommandType type, uint8_t sequence = 0, uint32_t argument = 0, float value = 0.0f)
{
    Frame frame{};
    frame[0] = host_protocol::Header;
    frame[1] = host_protocol::Version;
    frame[2] = static_cast<uint8_t>(type);
    frame[3] = sequence;
    switch (type)
    {
    case CommandType::SetVoltage:
    case CommandType::SetTorque:
        host_protocol::WriteFloat(frame.data() + 4, value);
        break;
    case CommandType::SetTorqueEnabled:
    case CommandType::GetParameter:
        host_protocol::WriteUint32(frame.data() + 4, argument);
        break;
    case CommandType::SetParameter:
        host_protocol::WriteUint32(frame.data() + 4, argument);
        host_protocol::WriteFloat(frame.data() + 8, value);
        break;
    default:
        break;
    }
    host_protocol::FinishFrame(frame);
    return frame;
}

void CheckRejected(const Frame& frame)
{
    const Command sentinel{CommandType::GetState, 0xEE, 0x12345678, -7.0f};
    Command command = sentinel;
    Check(!host_protocol::DecodeCommand(frame, command) && SameCommand(command, sentinel),
          "rejected frame leaves the caller's command unchanged");
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

void Feed(UART_HandleTypeDef& uart, const Frame& frame)
{
    Feed(uart, frame.data(), frame.size());
}

void TxComplete(UART_HandleTypeDef& uart)
{
    model.tx_data = nullptr;
    model.tx_size = 0;
    uart.callbacks[HAL_UART_TX_COMPLETE_CB_ID](&uart);
}

void CheckTx(const Frame& frame)
{
    Check(model.tx_data != nullptr && model.tx_size == frame.size() &&
          std::memcmp(model.tx_data, frame.data(), frame.size()) == 0,
          "DMA owns the expected persistent frame bytes");
}

void Acknowledge(HostCommunication& host, UART_HandleTypeDef& uart, const Command& command)
{
    const Response response{command.type, command.sequence, Result::Ok, 0.0f, false, true};
    Check(host.PublishResponse(response), "command publishes one response");
    Check(host.SendPendingResponse() == HAL_OK && !host.HasPendingResponse(),
          "successful response submission releases the pending slot");
    Check(model.tx_data[2] == host_protocol::CommandResponse && model.tx_data[3] == command.sequence &&
          model.tx_data[6] == static_cast<uint8_t>(command.type), "response echoes request opcode and sequence");
    TxComplete(uart);
}

void TestCodec()
{
    const uint8_t vector[] = {'1','2','3','4','5','6','7','8','9'};
    Check(host_protocol::FrameCrc::calc(vector, sizeof(vector)) == 0x29B1, "CCITT-FALSE known check");
    for (const auto& golden : CommandGoldens)
    {
        Command command{};
        Check(host_protocol::DecodeCommand(golden.frame, command) && SameCommand(command, golden.command),
              "command opcode, sequence, argument and value match independent golden frame");
        for (std::size_t byte = 0; byte < golden.frame.size(); ++byte)
        {
            for (unsigned bit = 0; bit < 8; ++bit)
            {
                auto corrupt = golden.frame;
                corrupt[byte] ^= static_cast<uint8_t>(1U << bit);
                CheckRejected(corrupt);
            }
        }
        const std::size_t reserved_start =
            golden.command.type == CommandType::SetParameter ? 12 :
            (golden.command.type == CommandType::SaveParameters || golden.command.type == CommandType::GetState ? 4 : 8);
        for (std::size_t byte = reserved_start; byte < 12; ++byte)
        {
            auto malformed = golden.frame;
            malformed[byte] = 1;
            host_protocol::FinishFrame(malformed);
            CheckRejected(malformed);
        }
    }
    for (const uint8_t opcode : {uint8_t{0}, uint8_t{0x04}, uint8_t{0x14},
                                 host_protocol::Feedback, host_protocol::CommandResponse, uint8_t{0xFF}})
    {
        CheckRejected(MakeCommand(static_cast<CommandType>(opcode)));
    }
    for (const std::size_t offset : {std::size_t{0}, std::size_t{1}})
    {
        auto malformed = CommandGoldens[0].frame;
        malformed[offset] ^= 1;
        host_protocol::FinishFrame(malformed);
        CheckRejected(malformed);
    }
    for (const auto type : {CommandType::SetParameter, CommandType::GetParameter})
    {
        for (const uint32_t id : {0U, 7U, 0xFFFFFFFFU})
        {
            CheckRejected(MakeCommand(type, 0, id, 1.0f));
        }
        for (uint32_t id = static_cast<uint32_t>(ParameterId::Kp);
             id <= static_cast<uint32_t>(ParameterId::TorqueLimit); ++id)
        {
            Command command{};
            const float value = type == CommandType::SetParameter ? 0.25f * static_cast<float>(id) : 0.0f;
            Check(host_protocol::DecodeCommand(MakeCommand(type, 255, id, value), command) &&
                  command.argument == id && command.value == value && command.sequence == 255,
                  "all six tuning parameter IDs retain argument and value");
        }
    }
    for (const float value : {-1.0f, 0.0f, 18.0f, 19.0f, std::numeric_limits<float>::infinity(),
                              -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()})
    {
        for (const auto type : {CommandType::SetVoltage, CommandType::SetTorque, CommandType::SetParameter})
        {
            Command command{};
            Check(host_protocol::DecodeCommand(MakeCommand(type, 0, 1, value), command) &&
                  ((std::isnan(value) && std::isnan(command.value)) || command.value == value),
                  "codec leaves numeric validity to the application");
        }
    }
    Command enable{};
    Check(host_protocol::DecodeCommand(MakeCommand(CommandType::SetTorqueEnabled, 0, 2), enable) &&
          enable.argument == 2, "invalid enable argument reaches application for InvalidValue response");
    for (const auto& golden : ResponseGoldens)
    {
        Frame frame{};
        Check(host_protocol::EncodeResponse(golden.response, frame) && frame == golden.frame,
              "all response result codes, flags and command echo match independent golden frames");
        Check((host_protocol::ReadUint32(frame.data() + 4) & 0xFF00FC00U) == 0U,
              "response status reserved bits remain zero");
        CheckRejected(frame);
    }
    Frame feedback{};
    Check(host_protocol::EncodeFeedback(6.25f, -1.5f, 0, feedback) && feedback == FeedbackGolden,
          "legacy feedback golden frame is unchanged");
    for (const float invalid : {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()})
    {
        auto response = ResponseGoldens[0].response;
        response.value = invalid;
        auto unchanged = FeedbackGolden;
        Check(!host_protocol::EncodeResponse(response, unchanged) && unchanged == FeedbackGolden,
              "nonfinite response cannot modify a frame");
        Check(!host_protocol::EncodeFeedback(invalid, 0, 0, unchanged) && unchanged == FeedbackGolden,
              "nonfinite feedback voltage cannot modify a frame");
        Check(!host_protocol::EncodeFeedback(0, invalid, 0, unchanged) && unchanged == FeedbackGolden,
              "nonfinite feedback torque cannot modify a frame");
    }
    std::puts("PASS command/response golden frames, CRC bit corruption, reserved fields and tuning IDs");
}

void TestFifo(HostCommunication& host, UART_HandleTypeDef& uart)
{
    const auto& first = CommandGoldens[0];
    const uint8_t noise[] = {0, 0x33, 0x5A};
    Feed(uart, noise, sizeof(noise));
    Feed(uart, first.frame.data(), 5);
    Command command{CommandType::GetState, 0xEE, 0, -1.0f};
    const auto sentinel = command;
    Check(!host.TakeCommand(command) && SameCommand(command, sentinel), "partial input cannot publish a command");
    Feed(uart, first.frame.data() + 5, first.frame.size() - 5);
    Check(host.TakeCommand(command) && SameCommand(command, first.command), "fragmented first command survives DMA transition");
    Acknowledge(host, uart, command);
    Check(!host.TakeCommand(command), "FIFO command is consumed once");

    for (const auto& golden : CommandGoldens)
    {
        Feed(uart, golden.frame);
    }
    Feed(uart, MakeCommand(CommandType::SetVoltage, 0xE0, 0, 17.0f));
    auto corrupt = first.frame;
    corrupt[4] ^= 1;
    Feed(uart, corrupt);
    for (const auto& golden : CommandGoldens)
    {
        Check(host.TakeCommand(command) && SameCommand(command, golden.command),
              "FIFO8 preserves voltage, torque, enable/disable, tuning, query, save and state order");
        Acknowledge(host, uart, command);
    }
    Check(!host.TakeCommand(command), "overflow drops the ninth/newest frame rather than any queued command");
    for (unsigned round = 0; round < 4; ++round)
    {
        for (uint8_t index = 0; index < 8; ++index)
        {
            Feed(uart, MakeCommand(CommandType::GetState, static_cast<uint8_t>(round * 8U + index)));
        }
        for (uint8_t index = 0; index < 8; ++index)
        {
            Check(host.TakeCommand(command) && command.type == CommandType::GetState &&
                  command.sequence == round * 8U + index, "FIFO slots wrap and reuse without reordering");
            Acknowledge(host, uart, command);
        }
    }
    for (const float invalid : {-0.1f, 18.1f, std::numeric_limits<float>::infinity(),
                                std::numeric_limits<float>::quiet_NaN()})
    {
        Feed(uart, MakeCommand(CommandType::SetVoltage, 0, 0, invalid));
        Check(!host.TakeCommand(command), "legacy voltage transport bounds remain enforced");
    }
    for (const float endpoint : {0.0f, 18.0f})
    {
        Feed(uart, MakeCommand(CommandType::SetVoltage, 0, 0, endpoint));
        Check(host.TakeCommand(command) && command.value == endpoint, "legacy voltage range endpoints accepted");
        Acknowledge(host, uart, command);
    }
    for (const auto type : {CommandType::SetTorque, CommandType::SetParameter})
    {
        Feed(uart, MakeCommand(type, 0, 1, std::numeric_limits<float>::quiet_NaN()));
        Check(host.TakeCommand(command) && command.type == type && std::isnan(command.value),
              "numeric-invalid torque/tuning requests are queued for application error responses");
        Acknowledge(host, uart, command);
    }
    Feed(uart, MakeCommand(CommandType::SetTorqueEnabled, 0, 2));
    Check(host.TakeCommand(command) && command.argument == 2, "invalid enable value is not silently dropped");
    Acknowledge(host, uart, command);
    for (const auto type : {CommandType::SetParameter, CommandType::GetParameter})
    {
        Feed(uart, MakeCommand(type, 0, 7, 1.0f));
    }
    Feed(uart, MakeCommand(static_cast<CommandType>(0xFF)));
    Check(!host.TakeCommand(command), "unknown parameter IDs and opcodes cannot enter the FIFO");
    std::puts("PASS actual FIFO8 order, overflow, slot reuse and legacy voltage bounds");
}

void TestBackpressure(HostCommunication& host, UART_HandleTypeDef& uart)
{
    Feed(uart, CommandGoldens[6].frame);
    Feed(uart, CommandGoldens[7].frame);
    Command command{};
    Check(host.TakeCommand(command) && command.type == CommandType::SaveParameters, "save precedes state query");
    Check(host.SendFeedback(6.25f, -1.5f) == HAL_OK, "feedback occupies shared DMA buffer");
    CheckTx(FeedbackGolden);
    Check(host.PublishResponse(ResponseGoldens[4].response) && host.HasPendingResponse(), "one save error response becomes pending");
    Check(!host.PublishResponse(ResponseGoldens[0].response), "pending response cannot be overwritten");
    const auto saved_command = command;
    Check(!host.TakeCommand(command) && SameCommand(command, saved_command), "pending response blocks command consumption");
    Check(host.SendPendingResponse() == HAL_BUSY && host.HasPendingResponse(), "active feedback backpressures response submission");
    CheckTx(FeedbackGolden);
    TxComplete(uart);
    for (const auto failure : {HAL_BUSY, HAL_ERROR, HAL_TIMEOUT})
    {
        model.next_tx_status = failure;
        Check(host.SendPendingResponse() == failure && host.HasPendingResponse() && model.tx_data == nullptr,
              "HAL submission failure retains the same response and releases unsubmitted DMA ownership");
        Check(!host.TakeCommand(command), "HAL failure cannot let a later command bypass the pending response");
    }
    Check(host.SendPendingResponse() == HAL_OK && !host.HasPendingResponse(), "successful DMA submission releases backpressure");
    CheckTx(ResponseGoldens[4].frame);
    const auto* dma_pointer = model.tx_data;
    Check(host.TakeCommand(command) && command.type == CommandType::GetState, "next command resumes after submission, not completion");
    Check(host.PublishResponse(ResponseGoldens[6].response), "next response can wait while earlier response is in flight");
    Check(host.SendPendingResponse() == HAL_BUSY && host.SendFeedback(10, 20) == HAL_BUSY &&
          model.tx_data == dma_pointer, "both send methods respect the same DMA owner");
    CheckTx(ResponseGoldens[4].frame);
    TxComplete(uart);
    Check(host.SendPendingResponse() == HAL_OK && !host.HasPendingResponse(), "second pending response submits after completion");
    CheckTx(ResponseGoldens[6].frame);
    TxComplete(uart);
    Check(host.SendPendingResponse() == HAL_OK && model.tx_data == nullptr, "empty response send is a no-op");
    auto invalid = ResponseGoldens[0].response;
    invalid.value = std::numeric_limits<float>::quiet_NaN();
    Check(!host.PublishResponse(invalid) && !host.HasPendingResponse(), "invalid response cannot wedge the pending slot");
    std::puts("PASS single response slot, command backpressure, HAL retry and shared DMA ownership");
}

void TestRecovery(HostCommunication& host, UART_HandleTypeDef& uart)
{
    const uint8_t slip = 0x13;
    Feed(uart, &slip, 1);
    Feed(uart, CommandGoldens[0].frame);
    Feed(uart, CommandGoldens[0].frame);
    Command command{};
    Check(host.TakeCommand(command) && SameCommand(command, CommandGoldens[0].command), "header search recovers after stream misalignment");
    Acknowledge(host, uart, command);
    Check(!host.TakeCommand(command), "misalignment did not produce an extra command");

    Check(host.PublishResponse(ResponseGoldens[0].response) && host.SendPendingResponse() == HAL_OK,
          "response starts before RX fault");
    for (const uint32_t error : {HAL_UART_ERROR_PE, HAL_UART_ERROR_FE, HAL_UART_ERROR_NE, HAL_UART_ERROR_ORE})
    {
        Feed(uart, CommandGoldens[2].frame.data(), 5);
        uart.ErrorCode = error;
        uart.callbacks[HAL_UART_ERROR_CB_ID](&uart);
        Check(host.SendFeedback(0, 0) == HAL_BUSY, "RX error does not release active response DMA storage");
        CheckTx(ResponseGoldens[0].frame);
        Feed(uart, CommandGoldens[3].frame);
        Check(host.TakeCommand(command) && SameCommand(command, CommandGoldens[3].command),
              "RX UART faults discard partial input and restart header search");
    }
    uart.hdmarx->ErrorCode = 1;
    uart.ErrorCode = HAL_UART_ERROR_DMA;
    uart.callbacks[HAL_UART_ERROR_CB_ID](&uart);
    Check(uart.hdmarx->ErrorCode == HAL_DMA_ERROR_NONE, "RX DMA fault is cleared independently");
    CheckTx(ResponseGoldens[0].frame);
    Feed(uart, CommandGoldens[7].frame);
    Check(host.TakeCommand(command) && command.type == CommandType::GetState, "RX DMA error restarts receive without affecting TX");

    Check(host.PublishResponse(ResponseGoldens[5].response), "next response remains pending during active TX");
    uart.ErrorCode = HAL_UART_ERROR_DMA;
    uart.hdmatx->ErrorCode = 1;
    model.abort_tx_status = HAL_ERROR;
    uart.callbacks[HAL_UART_ERROR_CB_ID](&uart);
    Check(host.SendPendingResponse() == HAL_BUSY && host.HasPendingResponse(), "failed TX abort cannot release buffer or pending response");
    CheckTx(ResponseGoldens[0].frame);
    model.abort_tx_status = HAL_OK;
    uart.callbacks[HAL_UART_ERROR_CB_ID](&uart);
    uart.hdmatx->ErrorCode = HAL_DMA_ERROR_NONE;
    uart.ErrorCode = 0;
    Check(host.SendPendingResponse() == HAL_OK && !host.HasPendingResponse(), "confirmed TX abort permits next pending submission");
    CheckTx(ResponseGoldens[5].frame);
    TxComplete(uart);

    model.next_tx_status = HAL_BUSY;
    Check(host.SendFeedback(1, 2) == HAL_BUSY, "failed feedback submission releases unused DMA ownership");
    Check(host.SendFeedback(1, 2) == HAL_OK && model.tx_data[3] == 1,
          "responses and failed feedback starts do not advance legacy feedback sequence");
    TxComplete(uart);
    for (unsigned sequence = 2; sequence <= 256; ++sequence)
    {
        Check(host.SendFeedback(1, 2) == HAL_OK && model.tx_data[3] == static_cast<uint8_t>(sequence),
              "legacy feedback sequence advances and wraps only on successful submission");
        TxComplete(uart);
    }
    Check(host.SendFeedback(0, std::numeric_limits<float>::infinity()) == HAL_ERROR,
          "invalid feedback is rejected without arming DMA");
    Check(host.SendFeedback(0, 0) == HAL_OK, "invalid feedback does not wedge sender");
    TxComplete(uart);
    std::puts("PASS RX stream/UART/DMA recovery, TX abort safety and legacy feedback sequence");
}

void TestTransport()
{
    DMA_HandleTypeDef rx, tx;
    rx.Init.Mode = DMA_CIRCULAR;
    tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    UART_HandleTypeDef uart{&rx, &tx};
    static HostCommunication host(&uart, 18.0f);
    Check(host.SendFeedback(0, 0) == HAL_ERROR && host.SendPendingResponse() == HAL_ERROR,
          "uninitialized transport cannot send");
    Check(host.Init(), "UART callback registration and initial IT receive");
    Check(!host.Init(), "reinitialization is rejected");
    TestFifo(host, uart);
    TestBackpressure(host, uart);
    TestRecovery(host, uart);
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
