#include "chassis_drive.h"

#include <algorithm>
#include <cmath>

static constexpr float Chassis_Control_Period_S = 0.001f;
static constexpr float Chassis_Distance_Tol_M = 0.008f;
static constexpr float Chassis_Turn_Tol_Rad = 0.04f;
static constexpr float Chassis_Max_Heading_Drift_Rad = 0.60f;
static constexpr float Chassis_Wheel_Stop_Rad_S = 0.12f;
static constexpr float Chassis_Yaw_Stop_Rad_S = 0.15f;
static constexpr float Chassis_Stall_Cmd_Min_Rad_S = 0.8f;
static constexpr uint16_t Chassis_Settle_Sample_Target = 30U;
static constexpr uint32_t Chassis_Stop_Timeout_Ms = 2000U;

bool Class_Chassis_Drive::Config_Valid() const
{
    return std::isfinite(Config.wheel_radius_m) && Config.wheel_radius_m > 0.0f &&
           std::isfinite(Config.track_width_m) && Config.track_width_m > 0.0f &&
           std::isfinite(Config.max_wheel_rad_s) && Config.max_wheel_rad_s > 0.0f &&
           std::isfinite(Config.wheel_ramp_rad_s2) && Config.wheel_ramp_rad_s2 > 0.0f &&
           std::isfinite(Config.torque_limit_nm) && Config.torque_limit_nm > 0.0f &&
           std::isfinite(Config.stall_torque_ratio) && Config.stall_torque_ratio > 0.0f &&
           std::isfinite(Config.stall_speed_rad_s) && Config.stall_speed_rad_s >= 0.0f &&
           Config.stall_duration_ms > 0U;
}

void Class_Chassis_Drive::Prepare_Active_Mode(const std::array<float, 2> &measured_speed,
                                              Enum_Chassis_Mode next_mode)
{
    if (Mode == CHASSIS_MODE_IDLE || Mode != next_mode)
    {
        for (uint8_t i = 0; i < 2; ++i)
        {
            const float seed = std::isfinite(measured_speed[i])
                                   ? std::clamp(measured_speed[i], -Config.max_wheel_rad_s, Config.max_wheel_rad_s)
                                   : 0.0f;
            Wheel[i].Reset(seed);
            Wheel_Target_Rad_S[i] = seed;
        }
        Forward_Profile_Rad_S = 0.5f * (Wheel_Target_Rad_S[0] + Wheel_Target_Rad_S[1]);
    }
    Motion_Done = false;
    Motion_Aborted = false;
    Settled_Samples = 0U;
}

std::array<float, 2> Class_Chassis_Drive::Mix_Wheel_Speeds(float linear_mps, float angular_rad_s) const
{
    const float half_track = 0.5f * Config.track_width_m;
    const float right_mps = linear_mps + half_track * angular_rad_s;
    const float left_mps = linear_mps - half_track * angular_rad_s;
    float right_rad_s = right_mps / Config.wheel_radius_m;
    float left_rad_s = left_mps / Config.wheel_radius_m;

    // Preserve curvature when either wheel exceeds the speed limit.
    const float peak = std::max(std::fabs(right_rad_s), std::fabs(left_rad_s));
    if (peak > Config.max_wheel_rad_s)
    {
        const float scale = Config.max_wheel_rad_s / peak;
        right_rad_s *= scale;
        left_rad_s *= scale;
    }
    return {right_rad_s, left_rad_s};
}

void Class_Chassis_Drive::Ramp_Wheel_Targets(const std::array<float, 2> &desired_rad_s)
{
    const float max_step = Config.wheel_ramp_rad_s2 * Chassis_Control_Period_S;
    for (uint8_t i = 0; i < 2; ++i)
    {
        const float clamped = std::clamp(desired_rad_s[i], -Config.max_wheel_rad_s, Config.max_wheel_rad_s);
        const float delta = clamped - Wheel_Target_Rad_S[i];
        if (delta > max_step)
        {
            Wheel_Target_Rad_S[i] += max_step;
        }
        else if (delta < -max_step)
        {
            Wheel_Target_Rad_S[i] -= max_step;
        }
        else
        {
            Wheel_Target_Rad_S[i] = clamped;
        }
    }
}

void Class_Chassis_Drive::Update_Stall_Detector(const std::array<float, 2> &speed,
                                                const std::array<float, 2> &torque)
{
    const float torque_threshold = Config.stall_torque_ratio * Config.torque_limit_nm;
    bool wheel_stalled = false;
    for (uint8_t i = 0; i < 2; ++i)
    {
        if (std::fabs(Wheel_Target_Rad_S[i]) >= Chassis_Stall_Cmd_Min_Rad_S &&
            std::fabs(speed[i]) <= Config.stall_speed_rad_s &&
            std::fabs(torque[i]) >= torque_threshold)
        {
            wheel_stalled = true;
        }
    }

    if (wheel_stalled)
    {
        if (Stall_Counter_Ms < Config.stall_duration_ms)
        {
            ++Stall_Counter_Ms;
        }
        if (Stall_Counter_Ms >= Config.stall_duration_ms)
        {
            Stalled = true;
        }
    }
    else
    {
        Stall_Counter_Ms = (Stall_Counter_Ms >= 2U) ? (Stall_Counter_Ms - 2U) : 0U;
    }
}

bool Class_Chassis_Drive::Set_Velocity(float linear_mps, float angular_rad_s,
                                       const std::array<float, 2> &measured_speed, uint32_t now_ms)
{
    if (!Config_Valid() || !std::isfinite(linear_mps) || !std::isfinite(angular_rad_s) ||
        !std::isfinite(measured_speed[0]) || !std::isfinite(measured_speed[1]))
    {
        return false;
    }

    Prepare_Active_Mode(measured_speed, CHASSIS_MODE_VELOCITY);
    Cmd_Linear_Mps = linear_mps;
    Cmd_Angular_Rad_S = angular_rad_s;
    Mode_Start_Ms = now_ms;
    Mode_Timeout_Ms = 0U;
    Mode = CHASSIS_MODE_VELOCITY;
    return true;
}

bool Class_Chassis_Drive::Start_Distance(float distance_m, float max_speed_mps,
                                         const std::array<float, 2> &angle,
                                         const std::array<float, 2> &speed,
                                         float heading_rad, uint32_t now_ms, uint32_t timeout_ms)
{
    if (!Config_Valid() || !std::isfinite(distance_m) || !std::isfinite(max_speed_mps) ||
        max_speed_mps <= 0.0f || timeout_ms == 0U ||
        !std::isfinite(angle[0]) || !std::isfinite(angle[1]) ||
        !std::isfinite(speed[0]) || !std::isfinite(speed[1]) || !std::isfinite(heading_rad))
    {
        return false;
    }

    if (std::fabs(distance_m) <= Chassis_Distance_Tol_M)
    {
        Travelled_M = 0.0f;
        Motion_Done = true;
        Mode = CHASSIS_MODE_IDLE;
        return true;
    }

    Prepare_Active_Mode(speed, CHASSIS_MODE_DISTANCE);
    Clear_Stall();
    Start_Angle = angle;
    Target_Distance_M = distance_m;
    Distance_Max_Wheel_Rad_S = std::min(max_speed_mps / Config.wheel_radius_m, Config.max_wheel_rad_s);
    Travelled_M = 0.0f;
    Start_Heading_Rad = heading_rad;
    Forward_Profile_Rad_S = 0.5f * (Wheel_Target_Rad_S[0] + Wheel_Target_Rad_S[1]);
    Mode_Start_Ms = now_ms;
    Mode_Timeout_Ms = timeout_ms;
    Mode = CHASSIS_MODE_DISTANCE;
    return true;
}

bool Class_Chassis_Drive::Start_Turn(float delta_yaw_rad, float max_omega_rad_s,
                                     const std::array<float, 2> &speed, float heading_rad,
                                     uint32_t now_ms, uint32_t timeout_ms)
{
    if (!Config_Valid() || !std::isfinite(delta_yaw_rad) || !std::isfinite(max_omega_rad_s) ||
        max_omega_rad_s <= 0.0f || timeout_ms == 0U ||
        !std::isfinite(speed[0]) || !std::isfinite(speed[1]) || !std::isfinite(heading_rad))
    {
        return false;
    }

    if (std::fabs(delta_yaw_rad) <= Chassis_Turn_Tol_Rad)
    {
        Motion_Done = true;
        Mode = CHASSIS_MODE_IDLE;
        return true;
    }

    Prepare_Active_Mode(speed, CHASSIS_MODE_TURN);
    Clear_Stall();
    Target_Heading_Rad = heading_rad + delta_yaw_rad;
    const float max_physical_omega = (2.0f * Config.wheel_radius_m * Config.max_wheel_rad_s) / Config.track_width_m;
    Turn_Max_Omega_Rad_S = std::min(max_omega_rad_s, max_physical_omega);
    Mode_Start_Ms = now_ms;
    Mode_Timeout_Ms = timeout_ms;
    Mode = CHASSIS_MODE_TURN;
    return true;
}

void Class_Chassis_Drive::Request_Stop(uint32_t now_ms)
{
    if (Mode == CHASSIS_MODE_IDLE)
    {
        return;
    }
    if (Mode != CHASSIS_MODE_STOPPING)
    {
        Stop_Start_Ms = now_ms;
        Settled_Samples = 0U;
        Mode = CHASSIS_MODE_STOPPING;
    }
}

void Class_Chassis_Drive::Emergency_Stop()
{
    Mode = CHASSIS_MODE_IDLE;
    Motion_Done = false;
    Motion_Aborted = false;
    Settled_Samples = 0U;
    Wheel_Target_Rad_S = {};
    Forward_Profile_Rad_S = 0.0f;
    Cmd_Linear_Mps = 0.0f;
    Cmd_Angular_Rad_S = 0.0f;
    Clear_Stall();
}

void Class_Chassis_Drive::Clear_Stall()
{
    Stalled = false;
    Stall_Counter_Ms = 0U;
}

float Class_Chassis_Drive::Get_Linear_Speed_Mps(const std::array<float, 2> &speed) const
{
    return 0.5f * Config.wheel_radius_m * (speed[0] + speed[1]);
}

float Class_Chassis_Drive::Get_Angular_Speed_Rad_S(const std::array<float, 2> &speed) const
{
    if (!(Config.track_width_m > 0.0f))
    {
        return 0.0f;
    }
    return Config.wheel_radius_m * (speed[0] - speed[1]) / Config.track_width_m;
}

std::array<float, 2> Class_Chassis_Drive::Update(const std::array<float, 2> &angle,
                                                 const std::array<float, 2> &speed,
                                                 float heading_rad, float yaw_rate_rad_s, uint32_t now_ms)
{
    if (Mode == CHASSIS_MODE_IDLE)
    {
        return {};
    }

    if (!Config_Valid() || !std::isfinite(angle[0]) || !std::isfinite(angle[1]) ||
        !std::isfinite(speed[0]) || !std::isfinite(speed[1]) ||
        !std::isfinite(heading_rad) || !std::isfinite(yaw_rate_rad_s))
    {
        Emergency_Stop();
        return {};
    }

    std::array<float, 2> desired_wheels = {};

    if (Mode == CHASSIS_MODE_VELOCITY)
    {
        // Light gyro damping reduces yaw oscillation on tracked skid-steer turns.
        const float damped_omega = Cmd_Angular_Rad_S + 0.15f * (Cmd_Angular_Rad_S - yaw_rate_rad_s);
        desired_wheels = Mix_Wheel_Speeds(Cmd_Linear_Mps, damped_omega);
        Ramp_Wheel_Targets(desired_wheels);
    }
    else if (Mode == CHASSIS_MODE_DISTANCE)
    {
        const float right_delta = angle[0] - Start_Angle[0];
        const float left_delta = angle[1] - Start_Angle[1];
        Travelled_M = 0.5f * Config.wheel_radius_m * (right_delta + left_delta);

        const float heading_error = heading_rad - Start_Heading_Rad;
        if (std::fabs(heading_error) >= Chassis_Max_Heading_Drift_Rad ||
            (uint32_t)(now_ms - Mode_Start_Ms) >= Mode_Timeout_Ms)
        {
            Emergency_Stop();
            Motion_Aborted = true;
            return {};
        }

        const float dir = (Target_Distance_M >= 0.0f) ? 1.0f : -1.0f;
        const float remaining_m = dir * (Target_Distance_M - Travelled_M);
        if (remaining_m <= Chassis_Distance_Tol_M)
        {
            Motion_Done = true;
            Request_Stop(now_ms);
            desired_wheels = {0.0f, 0.0f};
            Ramp_Wheel_Targets(desired_wheels);
        }
        else
        {
            const float stop_speed = std::sqrt(Config.wheel_ramp_rad_s2 * remaining_m / Config.wheel_radius_m);
            const float target_mag = std::min(Distance_Max_Wheel_Rad_S, stop_speed);
            const float signed_target = dir * target_mag;
            const float max_step = Config.wheel_ramp_rad_s2 * Chassis_Control_Period_S;
            const float step = std::clamp(signed_target - Forward_Profile_Rad_S, -max_step, max_step);
            Forward_Profile_Rad_S += step;

            const float corr_limit = std::min(0.9f, 0.5f * std::fabs(Forward_Profile_Rad_S) + 0.15f);
            const float correction = std::clamp(2.0f * heading_error + 0.4f * yaw_rate_rad_s,
                                                -corr_limit, corr_limit);
            // Positive heading_error means nose drifted left; always steer right (slow right, speed left).
            Wheel_Target_Rad_S[0] = std::clamp(Forward_Profile_Rad_S - correction,
                                               -Config.max_wheel_rad_s, Config.max_wheel_rad_s);
            Wheel_Target_Rad_S[1] = std::clamp(Forward_Profile_Rad_S + correction,
                                               -Config.max_wheel_rad_s, Config.max_wheel_rad_s);
        }
    }
    else if (Mode == CHASSIS_MODE_TURN)
    {
        if ((uint32_t)(now_ms - Mode_Start_Ms) >= Mode_Timeout_Ms)
        {
            Emergency_Stop();
            Motion_Aborted = true;
            return {};
        }

        const float yaw_error = Target_Heading_Rad - heading_rad;
        if (std::fabs(yaw_error) <= Chassis_Turn_Tol_Rad && std::fabs(yaw_rate_rad_s) <= Chassis_Yaw_Stop_Rad_S)
        {
            Ramp_Wheel_Targets({0.0f, 0.0f});
            if (std::fabs(speed[0]) <= Chassis_Wheel_Stop_Rad_S &&
                std::fabs(speed[1]) <= Chassis_Wheel_Stop_Rad_S)
            {
                if (++Settled_Samples >= Chassis_Settle_Sample_Target)
                {
                    Mode = CHASSIS_MODE_IDLE;
                    Motion_Done = true;
                    Wheel_Target_Rad_S = {};
                    return {};
                }
            }
            else
            {
                Settled_Samples = 0U;
            }
        }
        else
        {
            Settled_Samples = 0U;
            const float alpha_max =
                (2.0f * Config.wheel_radius_m * Config.wheel_ramp_rad_s2) / Config.track_width_m;
            const float stop_omega = std::sqrt(alpha_max * std::fabs(yaw_error));
            const float prop_omega = 3.5f * std::fabs(yaw_error);
            const float mag = std::min({Turn_Max_Omega_Rad_S, stop_omega, prop_omega});
            const float desired_omega = std::clamp((yaw_error >= 0.0f ? mag : -mag) - 0.12f * yaw_rate_rad_s,
                                                   -Turn_Max_Omega_Rad_S, Turn_Max_Omega_Rad_S);
            desired_wheels = Mix_Wheel_Speeds(0.0f, desired_omega);
            Ramp_Wheel_Targets(desired_wheels);
        }
    }
    else if (Mode == CHASSIS_MODE_STOPPING)
    {
        if ((uint32_t)(now_ms - Stop_Start_Ms) >= Chassis_Stop_Timeout_Ms)
        {
            Mode = CHASSIS_MODE_IDLE;
            Motion_Aborted = !Motion_Done;
            Wheel_Target_Rad_S = {};
            return {};
        }

        Ramp_Wheel_Targets({0.0f, 0.0f});
        if (Wheel_Target_Rad_S[0] == 0.0f && Wheel_Target_Rad_S[1] == 0.0f &&
            std::fabs(speed[0]) <= Chassis_Wheel_Stop_Rad_S &&
            std::fabs(speed[1]) <= Chassis_Wheel_Stop_Rad_S)
        {
            if (++Settled_Samples >= Chassis_Settle_Sample_Target)
            {
                Mode = CHASSIS_MODE_IDLE;
                Wheel_Target_Rad_S = {};
                return {};
            }
        }
        else
        {
            Settled_Samples = 0U;
        }
    }

    const std::array<float, 2> torque = {
        Wheel[0].Update(speed[0], Wheel_Target_Rad_S[0], Config.torque_limit_nm),
        Wheel[1].Update(speed[1], Wheel_Target_Rad_S[1], Config.torque_limit_nm),
    };

    if (Mode == CHASSIS_MODE_VELOCITY || Mode == CHASSIS_MODE_DISTANCE || Mode == CHASSIS_MODE_TURN)
    {
        Update_Stall_Detector(speed, torque);
    }

    return torque;
}
