#ifndef CHASSIS_DRIVE_H
#define CHASSIS_DRIVE_H

#include "3_Robot/Drive/straight_drive.h"

#include <array>
#include <stdint.h>

enum Enum_Chassis_Mode : uint8_t
{
    CHASSIS_MODE_IDLE = 0,
    CHASSIS_MODE_VELOCITY,
    CHASSIS_MODE_DISTANCE,
    CHASSIS_MODE_TURN,
    CHASSIS_MODE_STOPPING,
};

struct Struct_Chassis_Config
{
    float wheel_radius_m;
    float track_width_m;
    float max_wheel_rad_s;
    float wheel_ramp_rad_s2;
    float torque_limit_nm;
    float stall_torque_ratio = 0.78f;
    float stall_speed_rad_s = 0.35f;
    uint32_t stall_duration_ms = 600U;
};

// Tracked differential chassis controller built on per-wheel LADRC loops.
// Body frame: +x forward, +y left, +omega counter-clockwise (left turn).
// Wheel index: [0] = right wheel, [1] = left wheel (positive = forward).
class Class_Chassis_Drive
{
public:
    Class_Chassis_Drive(const Struct_Chassis_Config &config, float right_gain, float left_gain)
        : Config(config), Wheel{Class_Wheel_LADRC(right_gain), Class_Wheel_LADRC(left_gain)} {}

    bool Set_Velocity(float linear_mps, float angular_rad_s,
                      const std::array<float, 2> &measured_speed, uint32_t now_ms);

    bool Start_Distance(float distance_m, float max_speed_mps,
                        const std::array<float, 2> &angle, const std::array<float, 2> &speed,
                        float heading_rad, uint32_t now_ms, uint32_t timeout_ms = 12000U);

    bool Start_Turn(float delta_yaw_rad, float max_omega_rad_s,
                    const std::array<float, 2> &speed, float heading_rad,
                    uint32_t now_ms, uint32_t timeout_ms = 8000U);

    void Request_Stop(uint32_t now_ms);
    void Emergency_Stop();
    void Clear_Stall();

    std::array<float, 2> Update(const std::array<float, 2> &angle,
                                const std::array<float, 2> &speed,
                                float heading_rad, float yaw_rate_rad_s, uint32_t now_ms);

    Enum_Chassis_Mode Get_Mode() const { return Mode; }
    bool Is_Active() const { return Mode != CHASSIS_MODE_IDLE; }
    bool Is_Motion_Done() const { return Motion_Done; }
    bool Is_Motion_Aborted() const { return Motion_Aborted; }
    bool Is_Stalled() const { return Stalled; }
    float Get_Travelled_M() const { return Travelled_M; }
    float Get_Linear_Speed_Mps(const std::array<float, 2> &speed) const;
    float Get_Angular_Speed_Rad_S(const std::array<float, 2> &speed) const;

private:
    bool Config_Valid() const;
    void Prepare_Active_Mode(const std::array<float, 2> &measured_speed);
    std::array<float, 2> Mix_Wheel_Speeds(float linear_mps, float angular_rad_s) const;
    void Ramp_Wheel_Targets(const std::array<float, 2> &desired_rad_s);
    void Update_Stall_Detector(const std::array<float, 2> &speed, const std::array<float, 2> &torque);

    const Struct_Chassis_Config Config;
    Class_Wheel_LADRC Wheel[2];

    Enum_Chassis_Mode Mode = CHASSIS_MODE_IDLE;
    bool Motion_Done = false;
    bool Motion_Aborted = false;
    bool Stalled = false;

    std::array<float, 2> Wheel_Target_Rad_S = {};
    std::array<float, 2> Start_Angle = {};

    float Cmd_Linear_Mps = 0.0f;
    float Cmd_Angular_Rad_S = 0.0f;

    float Target_Distance_M = 0.0f;
    float Distance_Max_Wheel_Rad_S = 0.0f;
    float Travelled_M = 0.0f;
    float Start_Heading_Rad = 0.0f;
    float Forward_Profile_Rad_S = 0.0f;

    float Target_Heading_Rad = 0.0f;
    float Turn_Max_Omega_Rad_S = 0.0f;

    uint32_t Mode_Start_Ms = 0U;
    uint32_t Mode_Timeout_Ms = 0U;
    uint32_t Stop_Start_Ms = 0U;
    uint16_t Settled_Samples = 0U;
    uint32_t Stall_Counter_Ms = 0U;
};

#endif
