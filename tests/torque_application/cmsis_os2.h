#pragma once
#include <cstdint>
using osThreadId_t = void*;
using osThreadFunc_t = void (*)(void*);
struct osThreadAttr_t { const char* name = nullptr; void* cb_mem = nullptr; uint32_t cb_size = 0; void* stack_mem = nullptr; uint32_t stack_size = 0; };
uint32_t osKernelGetTickFreq();
void osDelay(uint32_t);
[[noreturn]] void osThreadExit();
osThreadId_t osThreadNew(osThreadFunc_t, void*, const osThreadAttr_t*);
void osThreadTerminate(osThreadId_t);
