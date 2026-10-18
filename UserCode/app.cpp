#include "cmsis_os2.h"

#include "adc.h"
#include "tim.h"
#include "algorithm"
#include "spi.h"

#include "pid_motor.hpp"

#include "ADS1220/ADS1220.hpp"

extern DMA_HandleTypeDef hdma_adc2;

constexpr float V_in        = 24.0;
constexpr float PWM_CNT_MAX = 3599 * 0.9;
constexpr float Vref        = 3.3f;

uint16_t ADC_Value[2];
float    V_target = 8.0;

PIDMotor* V_pid      = nullptr;
PIDMotor* Torque_pid = nullptr;

float Torque_feedback  = 0.0f;
float Torque_Vfeedback = 0.0f;
float Torque_target    = 4.0f;

ADS1220* ADS1220_ = nullptr;

constexpr int Rtop    = 82;
constexpr int Rbottom = 10;

constexpr float MAX_VOUT    = 18.0;
constexpr float ADC_MAX     = 4095.0f;
constexpr float Torque_rate = 0.0015;

PIDMotor::Config V_cfg = { .Kp = 5000.0, .Ki = 20.0, .Kd = 0.0, .abs_output_max = PWM_CNT_MAX };
PIDMotor::Config Torque_cfg = { .Kp = 0.1, .Ki = 0.0, .Kd = 0.0, .abs_output_max = MAX_VOUT };

void ADC_init()
{
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC_Value, 2);
}

void PID_init()
{
    V_pid      = new PIDMotor(V_cfg);
    Torque_pid = new PIDMotor(Torque_cfg);
}

float adc_V;
float V_feedback;
float output;
float pwm_output;

void TIM2_Vout_pid_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    int32_t raw_V = ADC_Value[1];
    adc_V         = raw_V / ADC_MAX * Vref;
    V_feedback    = adc_V * (Rbottom + Rtop) / Rbottom;
    pwm_output    = V_pid->calc(V_target, V_feedback);
    pwm_output    = std::clamp(pwm_output, 0.0f, PWM_CNT_MAX);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, static_cast<uint32_t>(pwm_output));
}

constexpr float Vrange      = 0.0075*128;
constexpr float TorqueRange = 20.0f;

void TIM3_Torque_pid_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    Torque_feedback = Torque_Vfeedback / Vrange * TorqueRange;
    // V_target        = Torque_pid->calc(Torque_target, Torque_feedback);
    // V_target        = std::clamp(V_target, 0.0f, MAX_VOUT);
}

void TIM_init()
{
    HAL_TIM_RegisterCallback(&htim2,
                             HAL_TIM_PERIOD_ELAPSED_CB_ID,
                             TIM2_Vout_pid_PeriodElapsedCallback);
    HAL_TIM_Base_Start_IT(&htim2);

    HAL_TIM_RegisterCallback(&htim3,
                             HAL_TIM_PERIOD_ELAPSED_CB_ID,
                             TIM3_Torque_pid_PeriodElapsedCallback);
    HAL_TIM_Base_Start_IT(&htim3);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void ADS1220_init()
{
    ADS1220::Config cfg{};

    cfg.input              = ADS1220::InputMux::Ain0Ain1;
    cfg.gain               = ADS1220::Gain::X128;
    cfg.data_rate          = ADS1220::DataRate::Normal20;
    cfg.conversion_mode    = ADS1220::ConversionMode::Continuous;
    cfg.voltage_reference  = ADS1220::VoltageReference::ExternalRef0;
    cfg.reference_voltage  = 5.0f;
    cfg.filter             = ADS1220::FirFilter::Reject50And60;
    cfg.bypass_pga         = false;
    cfg.temperature_sensor = false;
    cfg.burn_out_sources   = false;
    cfg.low_side_switch    = false;
    cfg.idac_current       = ADS1220::IdacCurrent::Off;
    cfg.idac1_route        = ADS1220::IdacRoute::Disabled;
    cfg.idac2_route        = ADS1220::IdacRoute::Disabled;
    cfg.dout_drdy_enabled  = false;

    ADS1220_ = new ADS1220(&hspi1, cfg);
    ADS1220_->Init();
}

float Vref_5V = 5.0;

void Task_get_torque(void* argument)
{
    for (;;)
    {
        Vref_5V = ADC_Value[0] / ADC_MAX * Vref;
        ADS1220_->Update();
        Torque_Vfeedback = ADS1220_->GetGainVoltage();
    }
}

extern "C" void Init(void* argument)
{
    /* 初始化代码 */
    ADS1220_init();
    ADC_init();

    PID_init();
    TIM_init();

    osThreadNew(Task_get_torque, nullptr, nullptr);

    /* 初始化完成后退出线程 */
    osThreadExit();
}
