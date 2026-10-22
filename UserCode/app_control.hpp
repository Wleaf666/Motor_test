#pragma once

#include <atomic>

class PIDMotor;
class HostCommunication;
class TorqueFeedback;
namespace torque
{
struct Parameters;
class ParameterStorage;
}

// Application-private bindings. Definitions and ownership remain in app.cpp.
// The sampling task owns commands/filtering; shared PID state keeps its IRQ guards.
extern PIDMotor* V_pid;
extern PIDMotor* Torque_pid;
extern HostCommunication* Host_;
extern TorqueFeedback* Torque_filter;
extern torque::ParameterStorage* Parameter_store;
extern torque::Parameters Torque_parameters;
extern std::atomic<float> V_target;
extern std::atomic<float> Torque_feedback;
extern std::atomic<bool> Output_running;
extern float Torque_target;
extern bool Torque_enabled;
extern bool Torque_feedback_valid;
extern bool Output_stopped;

bool StopOutput();
void Host_update(float max_voltage);
void InvalidateTorqueFeedback();
void UpdateTorqueFeedback(float signed_torque);
