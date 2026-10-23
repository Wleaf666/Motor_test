// Executes the real application, PID, Kalman, protocol, ADS driver and parameter store.
// Only hardware and scheduling are simulated. Run --exercise then --restore in separate processes.
#include "main.h"
#include "cmsis_os2.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <sys/mman.h>

#include "../../UserCode/app.cpp"

SPI_HandleTypeDef hspi1;
ADC_HandleTypeDef hadc2;
TIM_HandleTypeDef htim1, htim2, htim3;
UART_HandleTypeDef huart3;
uint32_t SimulatedPrimask = 0;

namespace
{
using Type = host_protocol::CommandType;
using Result = host_protocol::Result;
using Id = host_protocol::ParameterId;
uint32_t tick = 0;
unsigned delays_until_pause = 0;
struct TaskPaused {};
struct ThreadExited {};
bool pwm_started = false;
bool flash_locked = true;
bool spi_fail_next = false;
uint8_t registers[4]{};
uint8_t spi_command = 0;
int32_t sensor_raw = 1000;
uint8_t* rx_data = nullptr;
uint16_t rx_size = 0;
uint16_t rx_position = 0;
const uint8_t* tx_data = nullptr;
uint16_t tx_size = 0;
uint8_t sequence = 0;

void RunTask(osThreadFunc_t task, unsigned delay_calls)
{
    delays_until_pause = delay_calls;
    try { task(nullptr); }
    catch (const TaskPaused&) {}
    delays_until_pause = 0;
}

void Initialize()
{
    static DMA_HandleTypeDef rx;
    static DMA_HandleTypeDef tx;
    rx.Init.Mode = DMA_CIRCULAR;
    tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    huart3.hdmarx = &rx;
    huart3.hdmatx = &tx;
    try { Init(nullptr); }
    catch (const ThreadExited&) {}
    assert(Host_ != nullptr && Torque_pid != nullptr && Torque_filter != nullptr);
    assert(reinterpret_cast<std::uintptr_t>(Host_) % alignof(HostCommunication) == 0);
    assert(!Torque_enabled && !Output_running.load() && !pwm_started && V_target.load() == 0.0f);
}

void Write32(uint8_t* data, uint32_t value)
{
    for (unsigned index = 0; index < 4; ++index) data[index] = static_cast<uint8_t>(value >> (8U * index));
}

uint32_t Read32(const uint8_t* data)
{
    return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8U) |
           (static_cast<uint32_t>(data[2]) << 16U) | (static_cast<uint32_t>(data[3]) << 24U);
}

host_protocol::Frame Send(Type type, float value = 0.0f, uint32_t argument = 0)
{
    host_protocol::Frame frame{};
    frame[0] = host_protocol::Header;
    frame[1] = host_protocol::Version;
    frame[2] = static_cast<uint8_t>(type);
    frame[3] = sequence++;
    if (type == Type::SetTorque || type == Type::SetVoltage)
        host_protocol::WriteFloat(frame.data() + 4, value);
    else
    {
        Write32(frame.data() + 4, argument);
        if (type == Type::SetParameter) host_protocol::WriteFloat(frame.data() + 8, value);
    }
    host_protocol::FinishFrame(frame);
    for (uint8_t byte : frame)
    {
        assert(rx_data != nullptr && rx_position < rx_size);
        rx_data[rx_position++] = byte;
        if (rx_position == rx_size)
        {
            rx_position = 0;
            huart3.callbacks[HAL_UART_RX_COMPLETE_CB_ID](&huart3);
        }
    }
    Host_update(MAX_VOUT);
    RunTask(Task_host_communication, 2); // One actual send iteration, then pause at the next delay.
    assert(tx_data != nullptr && tx_size == frame.size() && tx_data[2] == 0x82);
    host_protocol::Frame reply;
    std::memcpy(reply.data(), tx_data, reply.size());
    assert(reply[3] == frame[3] && ((Read32(reply.data() + 4) >> 16U) & 0xFFU) == frame[2]);
    tx_data = nullptr;
    huart3.callbacks[HAL_UART_TX_COMPLETE_CB_ID](&huart3);
    return reply;
}

void ExpectResult(const host_protocol::Frame& reply, Result expected)
{
    assert((Read32(reply.data() + 4) & 0xFFU) == static_cast<uint8_t>(expected));
}

void SetParameter(Id id, float value)
{
    ExpectResult(Send(Type::SetParameter, value, static_cast<uint32_t>(id)), Result::Ok);
}

void SampleTorque(float signed_torque)
{
    sensor_raw = 1000 + static_cast<int32_t>(signed_torque * Vrange / TorqueRange / 5.0f * 8388608.0f);
    tick += 60;
    RunTask(Task_get_torque, 1);
}

void Exercise(const char* image)
{
    Initialize();
    assert(Torque_target == 0.0f);
    ExpectResult(Send(Type::SetTorqueEnabled, 0.0f, 1), Result::NotReady);
    assert(!pwm_started && !Torque_enabled);
    SampleTorque(-3.0f);
    assert(Torque_feedback_valid && std::fabs(Torque_feedback.load() - 3.0f) < 0.001f);
    assert(!pwm_started);

    ExpectResult(Send(Type::SetTorque, 3.5f), Result::InvalidValue);
    assert(Torque_target == 0.0f);
    SetParameter(Id::Kp, 2.0f);
    SetParameter(Id::Ki, 0.1f);
    ExpectResult(Send(Type::SetTorque, 2.0f), Result::Ok);
    ExpectResult(Send(Type::SetTorqueEnabled, 0.0f, 1), Result::Ok);
    assert(pwm_started && Torque_enabled && V_target.load() == 0.0f);
    htim3.callback(&htim3);
    const float first = V_target.load();
    htim3.callback(&htim3);
    const float second = V_target.load();
    // 4 V feedforward minus discrete PI correction for 2 - 3 N.m error.
    assert(std::fabs(first - 1.9f) < 0.003f && std::fabs(second - 1.8f) < 0.003f);
    htim2.callback(&htim2);
    assert(htim1.compare <= static_cast<uint32_t>(PWM_CNT_MAX));
    std::printf("TIM3 feedforward plus discrete PID: first=%.3f V second=%.3f V; negative sensor reports %.3f magnitude\n",
                first, second, Torque_feedback.load());
    // With zero measured torque, P/I/D=0 still produces the driver's feedforward.
    SetParameter(Id::Kp, 0.0f);
    SetParameter(Id::Ki, 0.0f);
    SetParameter(Id::ProcessNoise, 0.02f);
    SampleTorque(0.0f);
    ExpectResult(Send(Type::SetTorque, 1.25f), Result::Ok);
    htim3.callback(&htim3);
    assert(V_target.load() == 2.5f);
    SetParameter(Id::Kp, 2.0f);
    SetParameter(Id::Ki, 0.5f);
    ExpectResult(Send(Type::SetTorque, 3.0f), Result::Ok);
    for (unsigned cycle = 0; cycle < 10000; ++cycle)
    {
        htim3.callback(&htim3);
        assert(V_target.load() == 6.0f);
    }
    // Lowering the selected target or its runtime limit constrains the next TIM3 output.
    ExpectResult(Send(Type::SetTorque, 0.25f), Result::Ok);
    htim3.callback(&htim3);
    assert(V_target.load() == 0.5f);
    SetParameter(Id::TorqueLimit, 0.1f);
    htim3.callback(&htim3);
    assert(Torque_target == 0.1f && V_target.load() == 0.2f);
    ExpectResult(Send(Type::SetTorque, 0.0f), Result::Ok);
    htim3.callback(&htim3);
    assert(V_target.load() == 0.0f);
    // The absolute voltage ceiling also survives very large user-supplied finite limits.
    SetParameter(Id::Kp, 0.0f);
    SetParameter(Id::Ki, 0.0f);
    SetParameter(Id::TorqueLimit, std::numeric_limits<float>::max());
    ExpectResult(Send(Type::SetTorque, std::numeric_limits<float>::max()), Result::Ok);
    htim3.callback(&htim3);
    assert(V_target.load() == 18.0f && Torque_enabled);
    SetParameter(Id::TorqueLimit, 3.0f);
    SetParameter(Id::Kp, 2.0f);
    SetParameter(Id::Ki, 0.1f);
    SetParameter(Id::ProcessNoise, 0.01f);
    ExpectResult(Send(Type::SetTorque, 2.0f), Result::Ok);
    SampleTorque(-1.0f);
    std::puts("zero-feedback voltage envelope: 3 N.m held at 6 V for 10000 TIM3 cycles; target decrease, zero and 18 V ceiling passed");
    spi_fail_next = true;
    tick += 60;
    RunTask(Task_get_torque, 1);
    assert(!Torque_enabled && !Torque_feedback_valid && !pwm_started && V_target.load() == 0.0f);
    ExpectResult(Send(Type::SetTorqueEnabled, 0.0f, 1), Result::NotReady);
    SampleTorque(-1.0f);
    assert(Torque_feedback_valid && !Torque_enabled && !pwm_started);
    ExpectResult(Send(Type::SetTorqueEnabled, 0.0f, 1), Result::Ok);

    ExpectResult(Send(Type::SetVoltage, 5.0f), Result::ModeConflict);
    ExpectResult(Send(Type::SaveParameters), Result::OutputActive);
    torque::Parameters stored;
    assert(Parameter_store->Load(stored) == torque::ParameterStorage::Status::NotFound);
    ExpectResult(Send(Type::SetVoltage, 0.0f), Result::Ok);
    assert(!Torque_enabled && !pwm_started && V_target.load() == 0.0f);
    ExpectResult(Send(Type::SetTorqueEnabled, 0.0f, 1), Result::Ok);
    SetParameter(Id::TorqueLimit, 5.0f);
    ExpectResult(Send(Type::SetTorque, 4.0f), Result::Ok);
    SetParameter(Id::TorqueLimit, 1.5f);
    assert(Torque_target == 1.5f);
    auto state = Send(Type::GetState);
    assert(host_protocol::ReadFloat(state.data() + 8) == 1.5f && (Read32(state.data() + 4) & 0x100U));
    const float previous_q = Torque_parameters.process_noise;
    ExpectResult(Send(Type::SetParameter, std::numeric_limits<float>::quiet_NaN(),
                      static_cast<uint32_t>(Id::ProcessNoise)), Result::InvalidValue);
    assert(Torque_parameters.process_noise == previous_q);
    SetParameter(Id::ProcessNoise, 0.005f);
    SetParameter(Id::MeasurementNoise, 0.1f);
    ExpectResult(Send(Type::SetTorqueEnabled, 0.0f, 0), Result::Ok);
    assert(!Torque_enabled && !pwm_started && Output_stopped && V_target.load() == 0.0f);
    htim3.callback(&htim3);
    htim2.callback(&htim2);
    assert(V_target.load() == 0.0f && htim1.compare == 0U);
    // Manual voltage is not constrained by the retained 1.5 N.m torque target (3 V cap).
    ExpectResult(Send(Type::SetVoltage, 5.0f), Result::Ok);
    spi_fail_next = true;
    tick += 60;
    RunTask(Task_get_torque, 1);
    assert(!Torque_feedback_valid && pwm_started && V_target.load() == 5.0f);
    ExpectResult(Send(Type::SetVoltage, 0.0f), Result::Ok);
    SampleTorque(-1.0f);
    assert(Torque_feedback_valid && !Torque_enabled && !pwm_started);
    ExpectResult(Send(Type::SaveParameters), Result::Ok);
    assert(Parameter_store->Load(stored) == torque::ParameterStorage::Status::Ok);
    assert(stored.kp == 2.0f && stored.ki == 0.1f && stored.process_noise == 0.005f &&
           stored.measurement_noise == 0.1f && stored.torque_limit == 1.5f);
    SetParameter(Id::Kp, 9.0f);
    assert(Parameter_store->Load(stored) == torque::ParameterStorage::Status::Ok && stored.kp == 2.0f);
    auto* file = std::fopen(image, "wb");
    assert(file != nullptr);
    assert(std::fwrite(reinterpret_cast<void*>(0x0801F000U), 1, 4096, file) == 4096);
    std::fclose(file);
    std::puts("wire commands: readiness, enable/disable, tuning, dynamic limit, mode conflict and explicit save passed");
}

void Restore(const char* image)
{
    auto* file = std::fopen(image, "rb");
    assert(file != nullptr);
    assert(std::fread(reinterpret_cast<void*>(0x0801F000U), 1, 4096, file) == 4096);
    std::fclose(file);
    Initialize();
    assert(Torque_parameters.kp == 2.0f && Torque_parameters.ki == 0.1f &&
           Torque_parameters.process_noise == 0.005f && Torque_parameters.measurement_noise == 0.1f &&
           Torque_parameters.torque_limit == 1.5f);
    assert(Torque_target == 0.0f && !Torque_enabled && !pwm_started);
    std::puts("fresh process Init restored PID/Q/R/limit; target=0 and torque loop disabled");
    std::remove(image);
}
} // namespace

uint32_t osKernelGetTickFreq() { return 1000; }
void osDelay(uint32_t ticks)
{
    tick += ticks;
    if (delays_until_pause != 0 && --delays_until_pause == 0) throw TaskPaused{};
}
[[noreturn]] void osThreadExit() { throw ThreadExited{}; }
osThreadId_t osThreadNew(osThreadFunc_t, void*, const osThreadAttr_t*)
{
    static std::uintptr_t id = 1;
    return reinterpret_cast<void*>(id++);
}
void osThreadTerminate(osThreadId_t) {}
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef*, uint32_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef*, uint32_t* data, uint32_t size)
{
    assert(size == 2);
    auto* values = reinterpret_cast<uint16_t*>(data);
    values[0] = values[1] = 0;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_RegisterCallback(TIM_HandleTypeDef* timer, uint32_t, TimCallback callback)
{
    timer->callback = callback;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef*) { return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef*, uint32_t) { pwm_started = true; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef*, uint32_t) { pwm_started = false; return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*, uint8_t* bytes, uint16_t size, uint32_t)
{
    if (spi_fail_next)
    {
        spi_fail_next = false;
        return HAL_ERROR;
    }
    assert(size > 0);
    spi_command = bytes[0];
    if (spi_command == 0x06) std::memset(registers, 0, sizeof(registers));
    if ((spi_command & 0xE0U) == 0x40U)
    {
        const unsigned first = (spi_command >> 2U) & 3U;
        const unsigned count = (spi_command & 3U) + 1U;
        assert(size == count + 1U && first + count <= 4);
        std::memcpy(registers + first, bytes + 1, count);
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef*, uint8_t* bytes, uint16_t size, uint32_t)
{
    if ((spi_command & 0xE0U) == 0x20U)
    {
        const unsigned first = (spi_command >> 2U) & 3U;
        assert(first + size <= 4);
        std::memcpy(bytes, registers + first, size);
    }
    else
    {
        assert(spi_command == 0x10 && size == 3);
        const int32_t raw = (registers[0] & 0xF0U) == 0xE0U ? 0 : sensor_raw;
        for (unsigned byte = 0; byte < 3; ++byte)
            bytes[byte] = static_cast<uint8_t>(static_cast<uint32_t>(raw) >> (16U - byte * 8U));
    }
    return HAL_OK;
}
void HAL_Delay(uint32_t ticks) { tick += ticks; }
uint32_t HAL_GetTick() { return tick; }
HAL_StatusTypeDef HAL_UART_RegisterCallback(UART_HandleTypeDef* uart, HAL_UART_CallbackIDTypeDef id, UartCallback cb)
{
    uart->callbacks[id] = cb;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef*, uint8_t* bytes, uint16_t size)
{
    rx_data = bytes; rx_size = size; rx_position = 0; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef* uart, uint8_t* bytes, uint16_t size)
{
    return HAL_UART_Receive_IT(uart, bytes, size);
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef*) { rx_data = nullptr; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef* uart)
{
    tx_data = nullptr;
    uart->hdmatx->Remaining = 0;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef* uart, const uint8_t* bytes, uint16_t size)
{
    if (tx_data != nullptr) return HAL_BUSY;
    uart->hdmatx->Remaining = size;
    uart->Flags &= ~UART_FLAG_TC;
    tx_data = bytes; tx_size = size; return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASH_Unlock() { flash_locked = false; return HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_Lock() { flash_locked = true; return HAL_OK; }
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef* erase, uint32_t* error)
{
    assert(!flash_locked && erase->NbPages == 1 && (erase->Page == 62 || erase->Page == 63));
    std::memset(reinterpret_cast<void*>(static_cast<std::uintptr_t>(FLASH_BASE + erase->Page * FLASH_PAGE_SIZE)),
                0xFF, FLASH_PAGE_SIZE);
    *error = 0xFFFFFFFFU;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t, uint32_t address, uint64_t value)
{
    assert(!flash_locked && address >= 0x0801F000U && address + 8U <= 0x08020000U);
    auto* bytes = reinterpret_cast<uint8_t*>(static_cast<std::uintptr_t>(address));
    for (unsigned byte = 0; byte < 8; ++byte)
    {
        assert(bytes[byte] == 0xFF);
        bytes[byte] = static_cast<uint8_t>(value >> (8U * byte));
    }
    return HAL_OK;
}

int main(int argc, char** argv)
{
    assert(argc == 3);
    void* flash = mmap(reinterpret_cast<void*>(FLASH_BASE), FLASH_SIZE, PROT_READ | PROT_WRITE,
                       MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
    assert(flash != MAP_FAILED);
    std::memset(flash, 0xFF, FLASH_SIZE);
    if (std::strcmp(argv[1], "--exercise") == 0) Exercise(argv[2]);
    else { assert(std::strcmp(argv[1], "--restore") == 0); Restore(argv[2]); }
    munmap(flash, FLASH_SIZE);
}
