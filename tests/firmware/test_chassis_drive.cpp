#include "3_Robot/Drive/chassis_drive.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

static constexpr Struct_Chassis_Config Default_Config{
    0.0261f, // wheel_radius_m (58mm * 0.90 / 2)
    0.22f,   // track_width_m
    5.236f,  // max_wheel_rad_s (50 rpm)
    6.283f,  // wheel_ramp_rad_s2 (60 rpm/s)
    0.9f,    // torque_limit_nm
    0.78f,   // stall_torque_ratio
    0.35f,   // stall_speed_rad_s
    600U,    // stall_duration_ms
};

static void Test_Twist_Signs_And_Curvature_Scaling()
{
    Class_Chassis_Drive chassis(Default_Config, 40.0f, 40.0f);
    std::array<float, 2> angle{};
    std::array<float, 2> speed{};

    // Positive omega (CCW / left turn): right wheel must turn forward faster than left wheel.
    assert(chassis.Set_Velocity(0.05f, 0.8f, speed, 0));
    for (uint32_t ms = 1; ms <= 1500; ++ms)
    {
        const auto torque = chassis.Update(angle, speed, 0.0f, 0.0f, ms);
        for (unsigned i = 0; i < 2; ++i)
        {
            angle[i] += speed[i] * 0.001f;
            speed[i] += 40.0f * torque[i] * 0.001f;
        }
    }
    assert(speed[0] > speed[1]);
    assert(chassis.Get_Angular_Speed_Rad_S(speed) > 0.5f);

    // Overspeed twist should scale both wheels proportionally while respecting max_wheel_rad_s.
    assert(chassis.Set_Velocity(1.0f, 5.0f, speed, 1501));
    for (uint32_t ms = 1502; ms <= 3000; ++ms)
    {
        const auto torque = chassis.Update(angle, speed, 0.0f, 0.0f, ms);
        for (unsigned i = 0; i < 2; ++i)
        {
            angle[i] += speed[i] * 0.001f;
            speed[i] += 40.0f * torque[i] * 0.001f;
        }
    }
    assert(std::fabs(speed[0]) <= Default_Config.max_wheel_rad_s + 0.1f);
    assert(std::fabs(speed[1]) <= Default_Config.max_wheel_rad_s + 0.1f);
    assert(speed[0] > speed[1]);
}

static void Test_Signed_Forward_And_Reverse_Distance()
{
    // Forward 0.55 m (speed-bump crossing)
    {
        Class_Chassis_Drive chassis(Default_Config, 40.0f, 40.0f);
        std::array<float, 2> angle{};
        std::array<float, 2> speed{};
        float heading = 0.0f;
        assert(chassis.Start_Distance(0.55f, 0.12f, angle, speed, heading, 0));
        for (uint32_t ms = 1; ms < 12000 && chassis.Is_Active(); ++ms)
        {
            const auto torque = chassis.Update(angle, speed, heading, 0.0f, ms);
            for (unsigned i = 0; i < 2; ++i)
            {
                angle[i] += speed[i] * 0.001f;
                speed[i] += 40.0f * torque[i] * 0.001f;
            }
        }
        assert(!chassis.Is_Active());
        assert(chassis.Is_Motion_Done());
        assert(std::fabs(chassis.Get_Travelled_M() - 0.55f) < 0.03f);
    }

    // Reverse -0.25 m (post-drop backout) with left-drift heading correction check
    {
        Class_Chassis_Drive chassis(Default_Config, 40.0f, 40.0f);
        std::array<float, 2> angle{10.0f, -4.0f};
        std::array<float, 2> speed{};
        float heading = 0.2f;
        assert(chassis.Start_Distance(-0.25f, 0.10f, angle, speed, heading, 100));
        // Left heading drift (+0.05 rad) must make right wheel reverse faster (more negative torque) than left.
        const auto steer_check = chassis.Update(angle, speed, heading + 0.05f, 0.0f, 101);
        assert(steer_check[0] < steer_check[1]);

        for (uint32_t ms = 102; ms < 10000 && chassis.Is_Active(); ++ms)
        {
            const auto torque = chassis.Update(angle, speed, heading, 0.0f, ms);
            for (unsigned i = 0; i < 2; ++i)
            {
                angle[i] += speed[i] * 0.001f;
                speed[i] += 40.0f * torque[i] * 0.001f;
            }
        }
        assert(!chassis.Is_Active());
        assert(chassis.Is_Motion_Done());
        assert(std::fabs(chassis.Get_Travelled_M() - (-0.25f)) < 0.03f);
    }
}

static void Test_Closed_Loop_Turn_Left_And_Right()
{
    for (float delta_yaw : {1.5707963f, -0.7853981f})
    {
        Class_Chassis_Drive chassis(Default_Config, 40.0f, 40.0f);
        std::array<float, 2> angle{};
        std::array<float, 2> speed{};
        float heading = 0.5f;
        float yaw_rate = 0.0f;
        assert(chassis.Start_Turn(delta_yaw, 1.0f, speed, heading, 0));
        for (uint32_t ms = 1; ms < 8000 && chassis.Is_Active(); ++ms)
        {
            const auto torque = chassis.Update(angle, speed, heading, yaw_rate, ms);
            for (unsigned i = 0; i < 2; ++i)
            {
                angle[i] += speed[i] * 0.001f;
                speed[i] += 40.0f * torque[i] * 0.001f;
            }
            yaw_rate = chassis.Get_Angular_Speed_Rad_S(speed);
            heading += yaw_rate * 0.001f;
        }
        assert(!chassis.Is_Active());
        assert(chassis.Is_Motion_Done());
        assert(std::fabs((heading - 0.5f) - delta_yaw) < 0.05f);
    }
}

static void Test_Stall_Detection_And_Nan_Safety()
{
    Class_Chassis_Drive chassis(Default_Config, 40.0f, 40.0f);
    const std::array<float, 2> zero{};
    assert(chassis.Set_Velocity(0.10f, 0.0f, zero, 0));
    bool triggered = false;
    for (uint32_t ms = 1; ms <= 1500; ++ms)
    {
        chassis.Update(zero, zero, 0.0f, 0.0f, ms);
        if (chassis.Is_Stalled())
        {
            // Must trigger around stall_duration_ms after ramp crosses stall cmd threshold, well under 1.5 s.
            assert(ms >= Default_Config.stall_duration_ms);
            triggered = true;
            break;
        }
    }
    assert(triggered);
    chassis.Clear_Stall();
    assert(!chassis.Is_Stalled());

    // NaN sensor input must immediately emergency-stop with zero torque.
    const float nan_v = std::numeric_limits<float>::quiet_NaN();
    assert(!chassis.Set_Velocity(nan_v, 0.0f, zero, 2000));
    const auto safe_torque = chassis.Update(zero, zero, nan_v, 0.0f, 2001);
    assert(!chassis.Is_Active());
    assert(safe_torque[0] == 0.0f && safe_torque[1] == 0.0f);
    // Mode switch from active reverse DISTANCE directly to TURN must re-seed from measured speed
    // and never retain old negative wheel targets from the prior reverse.
    Class_Chassis_Drive switch_chassis(Default_Config, 40.0f, 40.0f);
    std::array<float, 2> angle{};
    std::array<float, 2> rev_speed{};
    assert(switch_chassis.Start_Distance(-0.20f, 0.10f, angle, rev_speed, 0.0f, 0));
    for (uint32_t ms = 1; ms <= 250; ++ms)
    {
        const auto tq = switch_chassis.Update(angle, rev_speed, 0.0f, 0.0f, ms);
        for (unsigned i = 0; i < 2; ++i)
        {
            angle[i] += rev_speed[i] * 0.001f;
            rev_speed[i] += 40.0f * tq[i] * 0.001f;
        }
    }
    assert(rev_speed[0] < -0.5f && rev_speed[1] < -0.5f);
    // Switch to left turn (+yaw) from standstill speeds: right wheel torque must be positive immediately.
    assert(switch_chassis.Start_Turn(0.5f, 0.7f, zero, 0.0f, 251, 4000));
    const auto turn_tq = switch_chassis.Update(angle, zero, 0.0f, 0.0f, 252);
    assert(turn_tq[0] > 0.0f && turn_tq[1] < 0.0f);
}

int main()
{
    Test_Twist_Signs_And_Curvature_Scaling();
    Test_Signed_Forward_And_Reverse_Distance();
    Test_Closed_Loop_Turn_Left_And_Right();
    Test_Stall_Detection_And_Nan_Safety();
    std::puts("PASS: chassis twist, signed distance, closed-loop turn, stall detection, mode-switch reset, NaN guards");
}
