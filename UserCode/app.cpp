#include "cmsis_os2.h"

#include "adc.h"
#include "tim.h"
#include "algorithm"

#include "pid_motor.hpp"

volatile uint16_t  ADC_Value[2];
float     Raw_V_target = 12.0;
PIDMotor* V_pid        = nullptr;

const int Rtop=82;
const int Rbottom = 10;

const float Vref = 3.3f;
const float ADC_MAX = 4095.0f;

void ADC_init()
{
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC_Value, 2);
}

void PID_init()
{
    PIDMotor::Config cfg = { .Kp = 0.01, .Ki = 0.01, .Kd = 0.0, .abs_output_max = 199.0 };
    V_pid                = new PIDMotor(cfg);
}

extern "C" void TIM2_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    uint16_t raw_V    = ADC_Value[1];
    float    adc_V        = raw_V / ADC_MAX * Vref;
    float    V_target = Raw_V_target * Rbottom / (Rtop + Rbottom);
    float    output_voltage     = V_pid->calc(V_target, adc_V);
    output_voltage              = std::clamp(output_voltage, 0.0f, 199.0f);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, static_cast<uint32_t>(output_voltage));
}

void TIM_init()
{
    HAL_TIM_RegisterCallback(&htim2, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM2_PeriodElapsedCallback);
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

extern "C" void Init(void* argument)
{
    /* 初始化代码 */
    ADC_init();
    PID_init();
    TIM_init();

    /* 初始化完成后退出线程 */
    osThreadExit();
}
