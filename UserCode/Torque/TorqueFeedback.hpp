#pragma once

#include <cmath>
#include <utility>
#include "EKF.hpp"

/** Task-owned magnitude estimation using the cpkg one-dimensional EKF core.
 * Q is process variance per new sample; R is measurement variance, not Hz.
 * No timer/PID work is performed here. Configure and Update have one task owner.
 */
class TorqueFeedback final
{
public:
    bool Configure(float process_noise, float measurement_noise)
    {
        if (!std::isfinite(process_noise) || process_noise < 0.0f ||
            !std::isfinite(measurement_noise) || measurement_noise <= 0.0f)
        {
            return false;
        }
        Filter_.Q[0][0] = process_noise;
        MeasurementNoise_[0][0] = measurement_noise;
        Initialized_ = false;
        return true;
    }

    // Call only once per successful ADC reading, never repeatedly on cached data.
    // Invalid input or numerical failure leaves the caller's published value unchanged.
    bool Update(float signed_torque, float& magnitude)
    {
        if (!std::isfinite(signed_torque) || MeasurementNoise_[0][0] <= 0.0f)
        {
            Initialized_ = false;
            return false;
        }
        const float sample = std::fabs(signed_torque);
        if (!Initialized_)
        {
            Filter_.x[0] = sample;
            Filter_.P[0][0] = MeasurementNoise_[0][0];
            Initialized_ = true;
        }
        else
        {
            const float variance = Filter_.P[0][0] + Filter_.Q[0][0] + MeasurementNoise_[0][0];
            if (!std::isfinite(variance) || variance <= 0.0f || !std::isfinite(1.0f / variance))
            {
                Initialized_ = false;
                return false;
            }
            Filter_.predict(Filter_.x, Identity_);
            math::Vec<float, 1> innovation{};
            innovation[0] = sample - Filter_.x[0];
            Filter_.update(innovation, Identity_, MeasurementNoise_);
            if (!std::isfinite(Filter_.x[0]) || Filter_.x[0] < 0.0f ||
                !std::isfinite(Filter_.P[0][0]) || Filter_.P[0][0] < 0.0f)
            {
                Initialized_ = false;
                return false;
            }
        }
        magnitude = Filter_.x[0];
        return true;
    }

private:
    static constexpr auto Identity_ = math::Mat<float, 1, 1>::identity();
    math::ekf::EKF<float, 1> Filter_{};
    math::Mat<float, 1, 1> MeasurementNoise_{};
    bool Initialized_ = false;
};
