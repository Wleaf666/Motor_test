#include "Torque/TorqueFeedback.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

int main()
{
    TorqueFeedback filter;
    assert(filter.Configure(0.001f, 0.1f));
    float magnitude = -1.0f;
    for (unsigned index = 0; index < 100; ++index)
    {
        assert(filter.Update(index % 2U == 0U ? -2.0f : 2.0f, magnitude));
        assert(std::fabs(magnitude - 2.0f) < 0.0001f);
    }

    double filtered_energy = 0.0;
    double raw_energy = 0.0;
    for (unsigned index = 0; index < 400; ++index)
    {
        const float sample = index % 2U == 0U ? 2.4f : 1.6f;
        assert(filter.Update(-sample, magnitude));
        if (index >= 100)
        {
            filtered_energy += (magnitude - 2.0f) * (magnitude - 2.0f);
            raw_energy += (sample - 2.0f) * (sample - 2.0f);
        }
    }
    assert(filtered_energy < raw_energy * 0.1);

    TorqueFeedback fast;
    TorqueFeedback smooth;
    assert(fast.Configure(1.0f, 0.001f));
    assert(smooth.Configure(0.001f, 1.0f));
    float fast_value = 0.0f;
    float smooth_value = 0.0f;
    assert(fast.Update(0.0f, fast_value));
    assert(smooth.Update(0.0f, smooth_value));
    for (unsigned index = 0; index < 5; ++index)
    {
        assert(fast.Update(2.0f, fast_value));
        assert(smooth.Update(2.0f, smooth_value));
    }
    assert(fast_value > 1.99f && smooth_value < 1.9f);

    const float last_valid = magnitude;
    assert(!filter.Update(std::numeric_limits<float>::quiet_NaN(), magnitude));
    assert(magnitude == last_valid);
    assert(!filter.Update(std::numeric_limits<float>::infinity(), magnitude));
    assert(magnitude == last_valid);
    assert(!filter.Configure(-1.0f, 1.0f));
    assert(!filter.Configure(1.0f, 0.0f));
    assert(filter.Update(-3.0f, magnitude) && magnitude == 3.0f);

    const float maximum = std::numeric_limits<float>::max();
    assert(filter.Configure(maximum, maximum));
    assert(filter.Update(1.0f, magnitude));
    assert(!filter.Update(2.0f, magnitude));
    assert(magnitude == 1.0f);
    std::puts("Kalman magnitude, noise attenuation, tuning and invalid-data scenarios passed");
}
