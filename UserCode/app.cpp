#include "cmsis_os2.h"

#include "adc.h"
#include "tim.h"
#include "algorithm"

#include "pid_motor.hpp"

extern DMA_HandleTypeDef hdma_adc2;

constexpr float V_in        = 24.0;
constexpr float PWM_CNT_MAX = 3599 * 0.9;

volatile uint16_t ADC_Value[2];
float             V_target = 12.0;
PIDMotor*         V_pid    = nullptr;

constexpr int Rtop    = 82;
constexpr int Rbottom = 10;

constexpr float Vref    = 3.3f;
constexpr float ADC_MAX = 4095.0f;

PIDMotor::Config cfg = { .Kp = 5000.0, .Ki = 20.0, .Kd = 0.0, .abs_output_max = PWM_CNT_MAX };

void ADC_init()
{
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC_Value, 2);

}

void PID_init()
{
    V_pid = new PIDMotor(cfg);
}

float adc_V;
float V_feedback;
float output;
float pwm_output;

extern "C" void TIM2_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    uint16_t raw_V = ADC_Value[1];
    adc_V          = raw_V / ADC_MAX * Vref;
    V_feedback     = adc_V * (Rbottom + Rtop) / Rbottom;
    pwm_output     = V_pid->calc(V_target, V_feedback);
    pwm_output     = std::clamp(pwm_output, 0.0f, PWM_CNT_MAX);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, static_cast<uint32_t>(pwm_output));
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
