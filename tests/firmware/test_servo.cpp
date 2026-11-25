#include "2_Device/Servo/dvc_servo.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

static void Test_Limits_And_Hard_Bounds()
{
    Class_Servo_Controller servo;
    // Out-of-range config must be clamped inside [500, 2500] us and ordered min <= max.
    servo.Init({3000U, 100U, 200U, 2000.0f, 100U});
    assert(servo.Get_Min_Us() == SERVO_HARD_MIN_US);
    assert(servo.Get_Max_Us() == SERVO_HARD_MAX_US);
    assert(servo.Get_Current_Us() == SERVO_HARD_MIN_US);
    assert(servo.Is_Enabled());
    assert(servo.Is_Settled());

    // Soft limits respected.
    servo.Init({900U, 2100U, 1500U, 2000.0f, 120U});
    servo.Set_Target_Us(500U);
    assert(servo.Get_Target_Us() == 900U);
    servo.Set_Target_Us(2500U);
    assert(servo.Get_Target_Us() == 2100U);
    assert(!servo.Set_Target_Angle_Deg(std::numeric_limits<float>::quiet_NaN()));
    assert(servo.Set_Target_Angle_Deg(90.0f, 0.0f, 180.0f));
    assert(servo.Get_Target_Us() == 1500U);
}

static void Test_Slew_And_Settle_Margin()
{
    Class_Servo_Controller servo;
    // 1000 -> 1900 us at 2000 us/s (2 us/ms) takes 450 ms slew + 120 ms settle margin = 570 ms.
    servo.Init({900U, 2100U, 1000U, 2000.0f, 120U});
    servo.Set_Target_Us(1900U);
    assert(!servo.Is_Settled());

    uint16_t prev = servo.Get_Current_Us();
    for (int ms = 1; ms <= 449; ++ms)
    {
        const uint16_t out = servo.Update_1ms();
        assert(out >= prev && static_cast<unsigned>(out - prev) <= 2U);
        assert(!servo.Is_Settled());
        prev = out;
    }

    // At ms=450 pulse reaches 1900 us, but settle margin must hold Is_Settled() false.
    assert(servo.Update_1ms() == 1900U);
    assert(!servo.Is_Settled());

    for (int ms = 1; ms < 120; ++ms)
    {
        assert(servo.Update_1ms() == 1900U);
        assert(!servo.Is_Settled());
    }
    // After 120 ms margin elapses, servo reports settled.
    assert(servo.Update_1ms() == 1900U);
    assert(servo.Is_Settled());
}

static void Test_Disable_And_Reenable()
{
    Class_Servo_Controller servo;
    servo.Init({900U, 2100U, 1500U, 1500.0f, 50U});
    servo.Disable();
    assert(!servo.Is_Enabled());
    assert(!servo.Is_Settled());
    assert(servo.Update_1ms() == 0U);

    servo.Enable_At_Current();
    assert(servo.Is_Enabled());
    assert(servo.Get_Current_Us() == 1500U);
    assert(!servo.Is_Settled());
    for (int ms = 0; ms < 50; ++ms)
    {
        assert(servo.Update_1ms() == 1500U);
    }
    assert(servo.Is_Settled());
}

int main()
{
    Test_Limits_And_Hard_Bounds();
    Test_Slew_And_Settle_Margin();
    Test_Disable_And_Reenable();
    std::puts("PASS: servo bounds, slew rate, open-loop settle timer, fault disable");
}
