#include "cmsis_os2.h"

#include "adc.h"

uint16_t ADC_Value[2];

void ADC_init()
{
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC_Value, 2) != HAL_OK)
    {
        Error_Handler();
    }
}

extern "C" void Init(void* argument)
{
    /* 初始化代码 */
    ADC_init();

    /* 初始化完成后退出线程 */
    osThreadExit();
}


