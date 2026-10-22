#pragma once

#include "TorqueParameters.hpp"

#include <cstddef>
#include <cstdint>

namespace torque
{
/**
 * Two-page STM32G431 parameter storage, using the existing HAL Flash driver.
 *
 * Construct with the linker-reserved region start and size. No hardware is touched
 * by construction. One task exclusively owns Load/Save and all Flash operations.
 * Save is blocking and must never run in an ISR. The caller must physically stop
 * output and prevent concurrent restart before passing output_stopped=true, and
 * keep it stopped until Save returns. A zero target alone is not proof of this.
 */
class ParameterStorage final
{
public:
    enum class Status : uint8_t
    {
        Ok,
        NotFound,
        InvalidLayout,
        InvalidParameters,
        OutputActive,
        UnlockFailed,
        EraseFailed,
        ProgramFailed,
        VerifyFailed,
        LockFailed,
    };

    ParameterStorage(std::uintptr_t region_start, std::size_t region_size) :
        RegionStart_(region_start), RegionSize_(region_size)
    {
    }

    // Failure leaves the caller's parameters unchanged. Never supplies guessed defaults.
    Status Load(Parameters& parameters) const;
    // Writes only the inactive page, commits last, verifies, then relocks Flash.
    Status Save(const Parameters& parameters, bool output_stopped);

private:
    bool CheckLayout() const;
    bool ReadSlot(unsigned slot, Parameters& parameters, uint32_t& generation) const;
    int FindLatest(Parameters& parameters, uint32_t& generation) const;

    std::uintptr_t RegionStart_;
    std::size_t RegionSize_;
};
} // namespace torque
