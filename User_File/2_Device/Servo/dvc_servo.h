#ifndef DVC_SERVO_H
#define DVC_SERVO_H

#include <stdint.h>

#ifdef USE_HAL_DRIVER
#include "tim.h"
#endif

// Hard safety bounds for standard 3-wire 50 Hz hobby servos (microseconds).
static constexpr uint16_t SERVO_HARD_MIN_US = 500U;
static constexpr uint16_t SERVO_HARD_MAX_US = 2500U;
static constexpr uint16_t SERVO_PWM_PERIOD_US = 20000U; // 50 Hz at 1 MHz timer tick

struct Struct_Servo_Config
{
    uint16_t min_us;
    uint16_t max_us;
    uint16_t init_us;
    float slew_us_per_s;
    uint16_t settle_margin_ms;
};

// Pure slew-rate limited 1 ms servo pulse controller with soft limits and
// open-loop settling timer (3-wire PWM servos have no position feedback).
class Class_Servo_Controller
{
public:
    void Init(const Struct_Servo_Config &config);
    void Set_Target_Us(uint16_t pulse_us);
    bool Set_Target_Angle_Deg(float angle_deg, float min_deg = 0.0f, float max_deg = 180.0f);
    uint16_t Update_1ms();

    void Disable();
    void Enable_At_Current();

    bool Is_Enabled() const { return Enabled; }
    bool Is_Settled() const { return Enabled && Settled; }
    uint16_t Get_Current_Us() const;
    uint16_t Get_Target_Us() const { return Target_Us; }
    uint16_t Get_Min_Us() const { return Min_Us; }
    uint16_t Get_Max_Us() const { return Max_Us; }

private:
    uint16_t Clamp_Pulse(uint16_t pulse_us) const;

    uint16_t Min_Us = 1000U;
    uint16_t Max_Us = 2000U;
    float Current_Us = 1500.0f;
    uint16_t Target_Us = 1500U;
    float Slew_Us_Per_Ms = 1.5f;
    uint16_t Settle_Margin_Ms = 120U;
    uint16_t Settle_Remaining_Ms = 0U;
    bool Enabled = false;
    bool Settled = false;
};

#ifdef USE_HAL_DRIVER
// Thin STM32 HAL 50 Hz PWM timer binding for a single 3-wire servo channel.
// Assumes timer counter clock is 1 MHz (PSC = 240 - 1 on 240 MHz TIM1/TIM2).
class Class_PWM_Servo
{
public:
    bool Init(TIM_HandleTypeDef *htim, uint32_t channel, const Struct_Servo_Config &config);
    void Set_Target_Us(uint16_t pulse_us) { Controller.Set_Target_Us(pulse_us); }
    bool Set_Target_Angle_Deg(float angle_deg, float min_deg = 0.0f, float max_deg = 180.0f)
    {
        return Controller.Set_Target_Angle_Deg(angle_deg, min_deg, max_deg);
    }
    void Update_1ms();
    void Disable();
    void Enable();

    bool Is_Enabled() const { return Controller.Is_Enabled(); }
    bool Is_Settled() const { return Controller.Is_Settled(); }
    uint16_t Get_Current_Us() const { return Controller.Get_Current_Us(); }
    uint16_t Get_Target_Us() const { return Controller.Get_Target_Us(); }

private:
    TIM_HandleTypeDef *Htim = nullptr;
    uint32_t Channel = 0U;
    Class_Servo_Controller Controller;
};
#endif

#endif
