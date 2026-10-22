#include "app_tasks.hpp"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ADS1220/ADS1220.hpp"
#include "HostCommunication/HostCommunication.hpp"
#include "Parameters/ParameterStorage.hpp"
#include "Torque/TorqueFeedback.hpp"
#include "pid_motor.hpp"

constexpr uint32_t Torque_task_stack_bytes = 1024U;
constexpr uint32_t Host_task_stack_bytes = 512U;

// Objects and both existing business tasks' stacks/control blocks share the boot arena.
// This avoids exhausting the separate 3 KB RTOS heap while Init is still alive.
static_assert(sizeof(ADS1220) + 2U * sizeof(PIDMotor) + sizeof(HostCommunication) +
                  sizeof(TorqueFeedback) + sizeof(torque::ParameterStorage) +
                  Torque_task_stack_bytes + Host_task_stack_bytes + 2U * sizeof(StaticTask_t) +
                  2U * sizeof(osThreadAttr_t) + alignof(HostCommunication) - 1U +
                  12U * (alignof(std::max_align_t) - 1U) <= 4096U);

bool Tasks_init()
{
    // Keep the descriptors in the boot arena too; Init retains its existing small stack.
    auto* sample_attributes = new osThreadAttr_t{};
    sample_attributes->name = "torque";
    sample_attributes->stack_mem = new StackType_t[Torque_task_stack_bytes / sizeof(StackType_t)];
    sample_attributes->stack_size = Torque_task_stack_bytes;
    sample_attributes->cb_mem = new StaticTask_t;
    sample_attributes->cb_size = sizeof(StaticTask_t);
    const auto sample_task = osThreadNew(Task_get_torque, nullptr, sample_attributes);
    auto* host_attributes = new osThreadAttr_t{};
    host_attributes->name = "host";
    host_attributes->stack_mem = new StackType_t[Host_task_stack_bytes / sizeof(StackType_t)];
    host_attributes->stack_size = Host_task_stack_bytes;
    host_attributes->cb_mem = new StaticTask_t;
    host_attributes->cb_size = sizeof(StaticTask_t);
    const auto host_task = osThreadNew(Task_host_communication, nullptr, host_attributes);
    if (sample_task == nullptr || host_task == nullptr)
    {
        if (sample_task != nullptr) osThreadTerminate(sample_task);
        if (host_task != nullptr) osThreadTerminate(host_task);
        return false;
    }
    return true;
}
