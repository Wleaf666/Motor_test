#include "static_arena.hpp"
#include <cstdio>
#include <memory>

// 1. 定义一个足够大的静态分配器（例如 64KB）
// 放在静态区 (.bss)
static StaticArena<4 * 1024> g_boot_arena;

// 2. 覆盖全局 operator new
void* operator new(const std::size_t size)
{
    void* ptr = g_boot_arena.allocate(size);
    if (!ptr)
    {
        // 嵌入式调试：打印错误并死循环
        // printf("Error: Global Arena Out of Memory! Request: %zu bytes\n", size);
        while (true)
        { /* 触发看门狗或挂起 */
        }
    }
    return ptr;
}

// 必须实现对应的 delete (即便它什么都不做)
void operator delete(void*) noexcept
{
    // 线性分配器无法回收单块内存，故此处留空
}

// 数组版本
void* operator new[](const std::size_t size)
{
    return ::operator new(size);
}

void operator delete[](void* p) noexcept
{
    ::operator delete(p);
}

// C++17 over-aligned classes (including DMA buffers) use these overloads.
// Align the absolute address: StaticArena's alignment argument only aligns its offset.
void* operator new(const std::size_t size, const std::align_val_t alignment)
{
    const auto align = static_cast<std::size_t>(alignment);
    if (align > 4096U || size > 4096U - align + 1U)
    {
        while (true) {}
    }
    std::size_t available = size + align - 1U;
    void* storage = ::operator new(available);
    return std::align(align, size, storage, available);
}

void* operator new[](const std::size_t size, const std::align_val_t alignment)
{
    return ::operator new(size, alignment);
}

void operator delete(void* pointer, std::align_val_t) noexcept
{
    ::operator delete(pointer);
}

void operator delete[](void* pointer, std::align_val_t) noexcept
{
    ::operator delete(pointer);
}

void operator delete(void* pointer, std::size_t) noexcept
{
    ::operator delete(pointer);
}

void operator delete[](void* pointer, std::size_t) noexcept
{
    ::operator delete(pointer);
}

void operator delete(void* pointer, std::size_t, std::align_val_t alignment) noexcept
{
    ::operator delete(pointer, alignment);
}

void operator delete[](void* pointer, std::size_t, std::align_val_t alignment) noexcept
{
    ::operator delete(pointer, alignment);
}
