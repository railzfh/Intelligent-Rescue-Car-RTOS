#include "dvc_servo.h"

#include <algorithm>
#include <cmath>

static constexpr float Servo_Default_Slew_Us_Per_S = 1500.0f;

uint16_t Class_Servo_Controller::Clamp_Pulse(uint16_t pulse_us) const
{
    if (pulse_us < Min_Us)
    {
        return Min_Us;
    }
    if (pulse_us > Max_Us)
    {
        return Max_Us;
    }
    return pulse_us;
}

void Class_Servo_Controller::Init(const Struct_Servo_Config &config)
{
    const uint16_t lo = std::clamp(config.min_us, SERVO_HARD_MIN_US, SERVO_HARD_MAX_US);
    const uint16_t hi = std::clamp(config.max_us, SERVO_HARD_MIN_US, SERVO_HARD_MAX_US);
    Min_Us = std::min(lo, hi);
    Max_Us = std::max(lo, hi);

    const uint16_t initial = Clamp_Pulse(config.init_us);
    Current_Us = static_cast<float>(initial);
    Target_Us = initial;

    const float slew = (std::isfinite(config.slew_us_per_s) && config.slew_us_per_s > 0.0f)
                           ? config.slew_us_per_s
                           : Servo_Default_Slew_Us_Per_S;
    Slew_Us_Per_Ms = slew * 0.001f;
    Settle_Margin_Ms = config.settle_margin_ms;
    Settle_Remaining_Ms = 0U;
    Enabled = true;
    Settled = true;
}

void Class_Servo_Controller::Set_Target_Us(uint16_t pulse_us)
{
    const uint16_t clamped = Clamp_Pulse(pulse_us);
    if (clamped != Target_Us)
    {
        Target_Us = clamped;
        Settled = false;
        Settle_Remaining_Ms = Settle_Margin_Ms;
    }
}

bool Class_Servo_Controller::Set_Target_Angle_Deg(float angle_deg, float min_deg, float max_deg)
{
    if (!std::isfinite(angle_deg) || !std::isfinite(min_deg) || !std::isfinite(max_deg) ||
        max_deg <= min_deg)
    {
        return false;
    }
    const float ratio = std::clamp((angle_deg - min_deg) / (max_deg - min_deg), 0.0f, 1.0f);
    const float us = static_cast<float>(Min_Us) + ratio * static_cast<float>(Max_Us - Min_Us);
    Set_Target_Us(static_cast<uint16_t>(std::lround(us)));
    return true;
}

uint16_t Class_Servo_Controller::Update_1ms()
{
    if (!Enabled)
    {
        return 0U;
    }

    const float target_f = static_cast<float>(Target_Us);
    const float delta = target_f - Current_Us;
    if (std::fabs(delta) <= Slew_Us_Per_Ms)
    {
        const bool just_arrived = (Current_Us != target_f);
        Current_Us = target_f;
        if (!Settled)
        {
            if (Settle_Remaining_Ms == 0U)
            {
                Settled = true;
            }
            else if (!just_arrived)
            {
                --Settle_Remaining_Ms;
                if (Settle_Remaining_Ms == 0U)
                {
                    Settled = true;
                }
            }
        }
    }
    else
    {
        Current_Us += (delta > 0.0f) ? Slew_Us_Per_Ms : -Slew_Us_Per_Ms;
        Settled = false;
        Settle_Remaining_Ms = Settle_Margin_Ms;
    }

    return Get_Current_Us();
}

void Class_Servo_Controller::Disable()
{
    Enabled = false;
    Settled = false;
    Settle_Remaining_Ms = Settle_Margin_Ms;
}

void Class_Servo_Controller::Enable_At_Current()
{
    const uint16_t cur = Get_Current_Us();
    Current_Us = static_cast<float>(cur);
    Target_Us = cur;
    Enabled = true;
    Settled = false;
    Settle_Remaining_Ms = Settle_Margin_Ms;
}

uint16_t Class_Servo_Controller::Get_Current_Us() const
{
    return Clamp_Pulse(static_cast<uint16_t>(std::lround(Current_Us)));
}

#ifdef USE_HAL_DRIVER
bool Class_PWM_Servo::Init(TIM_HandleTypeDef *htim, uint32_t channel, const Struct_Servo_Config &config)
{
    if (htim == nullptr)
    {
        return false;
    }
    Htim = htim;
    Channel = channel;
    Controller.Init(config);

    // Stop channel, set 50 Hz period (20000 us at 1 MHz tick), enable CCR preload
    // so pulse updates only latch at timer update events without glitches.
    HAL_TIM_PWM_Stop(Htim, Channel);
    __HAL_TIM_SET_AUTORELOAD(Htim, SERVO_PWM_PERIOD_US - 1U);
    __HAL_TIM_ENABLE_OCxPRELOAD(Htim, Channel);
    __HAL_TIM_SET_COMPARE(Htim, Channel, Controller.Get_Current_Us());
    HAL_TIM_GenerateEvent(Htim, TIM_EVENTSOURCE_UPDATE);
    return HAL_TIM_PWM_Start(Htim, Channel) == HAL_OK;
}

void Class_PWM_Servo::Update_1ms()
{
    if (Htim == nullptr)
    {
        return;
    }
    const uint16_t pulse_us = Controller.Update_1ms();
    __HAL_TIM_SET_COMPARE(Htim, Channel, pulse_us);
}

void Class_PWM_Servo::Disable()
{
    Controller.Disable();
    if (Htim != nullptr)
    {
        __HAL_TIM_SET_COMPARE(Htim, Channel, 0U);
        HAL_TIM_PWM_Stop(Htim, Channel);
    }
}

void Class_PWM_Servo::Enable()
{
    if (Htim == nullptr || Controller.Is_Enabled())
    {
        return;
    }
    Controller.Enable_At_Current();
    __HAL_TIM_SET_COMPARE(Htim, Channel, Controller.Get_Current_Us());
    HAL_TIM_PWM_Start(Htim, Channel);
}
#endif
