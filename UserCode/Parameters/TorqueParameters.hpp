#pragma once

#include <cmath>

namespace torque
{
/** Persistent tuning: discrete PIDMotor gains, per-sample Kalman noise and torque limit. */
struct Parameters
{
    float kp = 1.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float process_noise = 0.01f;     ///< Q variance per new sensor sample, torque-unit squared.
    float measurement_noise = 0.01f; ///< R variance, torque-unit squared per sample.
    float torque_limit = 3.0f;       ///< Runtime-adjustable target magnitude limit.
};

// Hardware-dependent safe limits and stability are not established by this numeric check.
inline bool IsValid(const Parameters& parameters)
{
    return std::isfinite(parameters.kp) && parameters.kp >= 0.0f &&
           std::isfinite(parameters.ki) && parameters.ki >= 0.0f &&
           std::isfinite(parameters.kd) && parameters.kd >= 0.0f &&
           std::isfinite(parameters.process_noise) && parameters.process_noise >= 0.0f &&
           std::isfinite(parameters.measurement_noise) && parameters.measurement_noise > 0.0f &&
           std::isfinite(parameters.torque_limit) && parameters.torque_limit > 0.0f;
}
} // namespace torque
