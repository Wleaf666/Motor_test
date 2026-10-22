#include "app_control.hpp"

#include "HostCommunication/HostCommunication.hpp"
#include "Parameters/ParameterStorage.hpp"
#include "Torque/TorqueFeedback.hpp"
#include "pid_motor.hpp"
#include "tim.h"
#include <algorithm>
#include <cmath>

bool StopOutput()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    Torque_enabled = false;
    Torque_pid->reset();
    V_target.store(0.0f, std::memory_order_relaxed);
    Output_running.store(false, std::memory_order_release);
    __set_PRIMASK(primask);
    const bool stopped = Output_stopped || HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1) == HAL_OK;
    Output_stopped = stopped;
    V_pid->reset(); // TIM2 cannot access the controller while Output_running is false.
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    return stopped;
}

namespace
{
bool StartOutput()
{
    if (Output_running.load(std::memory_order_acquire))
    {
        return true;
    }
    V_pid->reset();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    // A failed HAL operation is not proof that the physical output is stopped.
    Output_stopped = false;
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        return false;
    }
    Output_running.store(true, std::memory_order_release);
    return true;
}

void ApplyParameters(const torque::Parameters& parameters, float max_voltage)
{
    const bool noise_changed = parameters.process_noise != Torque_parameters.process_noise ||
                               parameters.measurement_noise != Torque_parameters.measurement_noise;
    Torque_parameters = parameters; // Same task owns configuration and the Kalman estimator.
    if (noise_changed)
    {
        Torque_filter->Configure(parameters.process_noise, parameters.measurement_noise);
    }
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    Torque_pid->setConfig({parameters.kp, parameters.ki, parameters.kd, max_voltage});
    Torque_pid->reset();
    Torque_target = std::min(Torque_target, parameters.torque_limit);
    __set_PRIMASK(primask);
}

bool ParameterValue(torque::Parameters& parameters, uint32_t id, float*& value)
{
    using Id = host_protocol::ParameterId;
    switch (static_cast<Id>(id))
    {
    case Id::Kp: value = &parameters.kp; break;
    case Id::Ki: value = &parameters.ki; break;
    case Id::Kd: value = &parameters.kd; break;
    case Id::ProcessNoise: value = &parameters.process_noise; break;
    case Id::MeasurementNoise: value = &parameters.measurement_noise; break;
    case Id::TorqueLimit: value = &parameters.torque_limit; break;
    default: return false;
    }
    return true;
}
} // namespace

void InvalidateTorqueFeedback()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool active = Torque_enabled;
    Torque_feedback_valid = false;
    __set_PRIMASK(primask);
    // Invalid torque must stop its own loop, not change the existing manual voltage mode.
    if (active) StopOutput();
}

void UpdateTorqueFeedback(float signed_torque)
{
    float filtered;
    if (Torque_filter->Update(signed_torque, filtered))
    {
        Torque_feedback.store(filtered, std::memory_order_relaxed);
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        Torque_feedback_valid = true;
        __set_PRIMASK(primask);
    }
    else
    {
        InvalidateTorqueFeedback();
    }
}

void Host_update(float max_voltage)
{
    using Type = host_protocol::CommandType;
    using Result = host_protocol::Result;
    host_protocol::Command command;
    if (!Host_->TakeCommand(command))
    {
        return;
    }
    host_protocol::Response response{command.type, command.sequence, Result::Ok, 0.0f, false, false};
    const uint32_t initial_primask = __get_PRIMASK();
    __disable_irq();
    response.torque_enabled = Torque_enabled;
    response.feedback_valid = Torque_feedback_valid;
    __set_PRIMASK(initial_primask);
    switch (command.type)
    {
    case Type::SetVoltage:
        if (!std::isfinite(command.value) || command.value < 0.0f || command.value > max_voltage)
        {
            response.result = Result::InvalidValue;
        }
        else if (command.value == 0.0f)
        {
            if (!StopOutput()) response.result = Result::HardwareError;
        }
        else if (response.torque_enabled)
        {
            response.result = Result::ModeConflict;
        }
        else if (!StartOutput())
        {
            response.result = Result::HardwareError;
        }
        else
        {
            V_target.store(command.value, std::memory_order_relaxed);
            response.value = command.value;
        }
        break;
    case Type::SetTorque:
        if (!std::isfinite(command.value) || command.value < 0.0f ||
            command.value > Torque_parameters.torque_limit)
        {
            response.result = Result::InvalidValue;
        }
        else
        {
            const uint32_t primask = __get_PRIMASK();
            __disable_irq();
            Torque_target = command.value;
            __set_PRIMASK(primask);
            response.value = command.value;
        }
        break;
    case Type::SetTorqueEnabled:
        if (command.argument > 1U)
        {
            response.result = Result::InvalidValue;
        }
        else if (command.argument == 0U)
        {
            if (!StopOutput()) response.result = Result::HardwareError;
        }
        else if (!response.feedback_valid)
        {
            response.result = Result::NotReady;
        }
        else if (!StopOutput() || !StartOutput())
        {
            response.result = Result::HardwareError;
        }
        else
        {
            const uint32_t primask = __get_PRIMASK();
            __disable_irq();
            Torque_enabled = true;
            __set_PRIMASK(primask);
            response.value = 1.0f;
        }
        break;
    case Type::SetParameter:
    case Type::GetParameter:
    {
        auto parameters = Torque_parameters;
        float* value = nullptr;
        if (!ParameterValue(parameters, command.argument, value))
        {
            response.result = Result::InvalidValue;
        }
        else if (command.type == Type::GetParameter)
        {
            response.value = *value;
        }
        else
        {
            *value = command.value;
            if (!torque::IsValid(parameters))
            {
                response.result = Result::InvalidValue;
            }
            else
            {
                ApplyParameters(parameters, max_voltage);
                response.value = *value;
            }
        }
        break;
    }
    case Type::SaveParameters:
        if (response.torque_enabled || !Output_stopped || V_target.load(std::memory_order_relaxed) != 0.0f)
        {
            response.result = Result::OutputActive;
        }
        else
        {
            const auto status = Parameter_store->Save(Torque_parameters, true);
            response.value = static_cast<float>(status);
            if (status != torque::ParameterStorage::Status::Ok) response.result = Result::StorageError;
        }
        break;
    case Type::GetState:
        response.value = Torque_target;
        break;
    default:
        response.result = Result::InvalidValue;
        break;
    }
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    response.torque_enabled = Torque_enabled;
    response.feedback_valid = Torque_feedback_valid;
    __set_PRIMASK(primask);
    // TakeCommand reserves this one response slot until DMA accepts the reply.
    Host_->PublishResponse(response);
}
