#pragma once

#include <cmath>

namespace torque
{
/** PID gains and low-pass cutoff in Hz; never includes a running target. */
struct Parameters
{
    float kp;
    float ki;
    float kd;
    float cutoff_hz;
};

// Sampling-rate and actuator-dependent limits must also be checked when applying parameters.
inline bool IsValid(const Parameters& parameters)
{
    return std::isfinite(parameters.kp) && parameters.kp >= 0.0f &&
           std::isfinite(parameters.ki) && parameters.ki >= 0.0f &&
           std::isfinite(parameters.kd) && parameters.kd >= 0.0f &&
           std::isfinite(parameters.cutoff_hz) && parameters.cutoff_hz > 0.0f;
}
} // namespace torque
