#include "straight_drive.h"

#include <algorithm>
#include <cmath>

static constexpr float Control_Period_S = 0.001f;
static constexpr float Distance_Tolerance_M = 0.005f;
static constexpr float Max_Heading_Error_Rad = 0.44f;
static constexpr uint32_t Drive_Timeout_Ms = 20000;
static constexpr uint32_t Stop_Timeout_Ms = 2000;

void Class_Wheel_LADRC::Reset(float speed_rad_s)
{
    Estimated_Speed = speed_rad_s;
    Estimated_Disturbance = 0.0f;
    Command_Torque = 0.0f;
}

float Class_Wheel_LADRC::Update(float speed_rad_s, float target_rad_s, float torque_limit_nm)
{
    constexpr float observer_bandwidth = 24.0f;
    constexpr float control_bandwidth = 6.0f;
    const float error = speed_rad_s - Estimated_Speed;
    // The ESO uses the preceding command after saturation.
    Estimated_Speed += Control_Period_S *
        (Estimated_Disturbance + Model_Gain * Command_Torque + 2.0f * observer_bandwidth * error);
    Estimated_Disturbance += Control_Period_S * observer_bandwidth * observer_bandwidth * error;
    const float torque = (control_bandwidth * (target_rad_s - Estimated_Speed) - Estimated_Disturbance) / Model_Gain;
    Command_Torque = std::clamp(torque, -torque_limit_nm, torque_limit_nm);
    return Command_Torque;
}

bool Class_Straight_Drive::Start(const std::array<float, 2> &angle, const std::array<float, 2> &speed,
                                 float heading_rad, uint32_t now_ms)
{
    if (!(Config.wheel_radius_m > 0.0f && Config.target_distance_m > Distance_Tolerance_M &&
          Config.max_speed_rad_s > 0.0f && Config.ramp_rad_s2 > 0.0f && Config.torque_limit_nm > 0.0f))
    {
        return false;
    }

    Start_Angle = angle;
    Start_Ms = now_ms;
    Stopped_Sample_Count = 0;
    Distance_M = 0.0f;
    Start_Heading_Rad = heading_rad;
    Forward_Rad_S = 0.0f;
    Stopping = false;
    for (uint8_t i = 0; i < 2; ++i)
    {
        Wheel[i].Reset(speed[i]);
    }
    Running = true;
    return true;
}

void Class_Straight_Drive::Request_Stop(uint32_t now_ms)
{
    if (Running && !Stopping)
    {
        Stop_Start_Ms = now_ms;
        Stopping = true;
    }
}

void Class_Straight_Drive::Stop()
{
    Running = false;
    Stopping = false;
}

std::array<float, 2> Class_Straight_Drive::Update(const std::array<float, 2> &angle, const std::array<float, 2> &speed,
                                                float heading_rad, float yaw_rate_rad_s, uint32_t now_ms)
{
    if (!Running)
    {
        return {};
    }

    const float right_delta = angle[0] - Start_Angle[0];
    const float left_delta = angle[1] - Start_Angle[1];
    Distance_M = 0.5f * Config.wheel_radius_m * (right_delta + left_delta);
    const float heading_error = heading_rad - Start_Heading_Rad;
    if (std::fabs(heading_error) >= Max_Heading_Error_Rad || now_ms - Start_Ms >= Drive_Timeout_Ms ||
        (Stopping && now_ms - Stop_Start_Ms >= Stop_Timeout_Ms))
    {
        Stop();
        return {};
    }

    const float remaining_m = Config.target_distance_m - Distance_M;
    if (remaining_m <= Distance_Tolerance_M)
    {
        Request_Stop(now_ms);
    }

    if (Stopping)
    {
        Forward_Rad_S = std::max(0.0f, Forward_Rad_S - Config.ramp_rad_s2 * Control_Period_S);
    }
    else
    {
        const float stop_speed = std::sqrt(Config.ramp_rad_s2 * remaining_m / Config.wheel_radius_m);
        Forward_Rad_S = std::min({Config.max_speed_rad_s, stop_speed, Forward_Rad_S + Config.ramp_rad_s2 * Control_Period_S});
    }
    const float correction_limit = std::min(0.8f, 0.5f * Forward_Rad_S);
    const float correction = std::clamp(2.0f * heading_error + 0.4f * yaw_rate_rad_s,
                                        -correction_limit, correction_limit);
    // Positive yaw is a left turn. Slow the right wheel to steer back to the right.
    const float right_target = std::min(Forward_Rad_S - correction, Config.max_speed_rad_s);
    const float left_target = std::min(Forward_Rad_S + correction, Config.max_speed_rad_s);
    const std::array<float, 2> torque = {
        Wheel[0].Update(speed[0], right_target, Config.torque_limit_nm),
        Wheel[1].Update(speed[1], left_target, Config.torque_limit_nm)};
    if (Stopping && Forward_Rad_S == 0.0f && std::fabs(speed[0]) < 0.1f && std::fabs(speed[1]) < 0.1f)
    {
        if (++Stopped_Sample_Count >= 30U)
        {
            Stop();
            return {};
        }
    }
    else
    {
        Stopped_Sample_Count = 0;
    }
    return torque;
}
