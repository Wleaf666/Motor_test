#pragma once

#include "../host_communication/main.h"

struct SPI_HandleTypeDef {};
struct ADC_HandleTypeDef {};
struct TIM_HandleTypeDef;
using TimCallback = void (*)(TIM_HandleTypeDef*);
struct TIM_HandleTypeDef { uint32_t compare = 0; TimCallback callback = nullptr; };
extern SPI_HandleTypeDef hspi1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim1, htim2, htim3;
extern UART_HandleTypeDef huart3;
extern uint32_t SimulatedPrimask;

inline uint32_t __get_PRIMASK() { return SimulatedPrimask; }
inline void __disable_irq() { SimulatedPrimask = 1; }
inline void __set_PRIMASK(uint32_t value) { SimulatedPrimask = value; }
constexpr uint32_t ADC_SINGLE_ENDED = 0;
constexpr uint32_t TIM_CHANNEL_1 = 0;
constexpr uint32_t HAL_TIM_PERIOD_ELAPSED_CB_ID = 0;
#define __HAL_TIM_SET_COMPARE(timer, channel, value) ((timer)->compare = (value))
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef*, uint32_t);
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef*, uint32_t*, uint32_t);
HAL_StatusTypeDef HAL_TIM_RegisterCallback(TIM_HandleTypeDef*, uint32_t, TimCallback);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef*);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef*, uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef*, uint32_t);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*, uint8_t*, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef*, uint8_t*, uint16_t, uint32_t);
void HAL_Delay(uint32_t);
uint32_t HAL_GetTick();

#define FLASH_BASE 0x08000000U
#define FLASH_SIZE (128U * 1024U)
#define FLASH_PAGE_SIZE 2048U
#define FLASH_TYPEERASE_PAGES 2U
#define FLASH_BANK_1 1U
#define FLASH_TYPEPROGRAM_DOUBLEWORD 0U
struct FLASH_EraseInitTypeDef
{
    uint32_t TypeErase;
    uint32_t Banks;
    uint32_t Page;
    uint32_t NbPages;
};
HAL_StatusTypeDef HAL_FLASH_Unlock();
HAL_StatusTypeDef HAL_FLASH_Lock();
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef*, uint32_t*);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t, uint32_t, uint64_t);
