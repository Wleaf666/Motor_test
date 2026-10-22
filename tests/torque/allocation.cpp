#include <cassert>
#include <cstdint>
#include <cstdio>
#include <new>

struct alignas(32) DmaObject { uint8_t bytes[32]{}; };
struct alignas(64) WideObject { uint8_t bytes[64]{}; };

int main()
{
    auto* ordinary = new uint8_t[3];
    ordinary[0] = 0xA5;
    auto* dma = new DmaObject;
    auto* wide = new WideObject[2];
    assert(reinterpret_cast<std::uintptr_t>(dma) % alignof(DmaObject) == 0);
    assert(reinterpret_cast<std::uintptr_t>(wide) % alignof(WideObject) == 0);
    const auto base = reinterpret_cast<std::uintptr_t>(ordinary);
    assert(reinterpret_cast<std::uintptr_t>(dma) >= base &&
           reinterpret_cast<std::uintptr_t>(dma) - base < 4096U);
    assert(reinterpret_cast<std::uintptr_t>(wide) >= base &&
           reinterpret_cast<std::uintptr_t>(wide) - base < 4096U);
    dma->bytes[31] = 0x5A;
    wide[0].bytes[63] = 0x31;
    wide[1].bytes[0] = 0x64;
    assert(ordinary[0] == 0xA5 && dma->bytes[31] == 0x5A &&
           wide[0].bytes[63] == 0x31 && wide[1].bytes[0] == 0x64);
    delete dma;
    delete[] wide;
    std::puts("boot arena handles ordinary and over-aligned object allocations");
}
