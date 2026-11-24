#ifndef STRAIGHT_DRIVE_H
#define STRAIGHT_DRIVE_H

#include <array>
#include <stdint.h>

class Class_Wheel_LADRC
{
public:
    explicit Class_Wheel_LADRC(float model_gain) : Model_Gain(model_gain) {}

    void Reset(float speed_rad_s);
    float Update(float speed_rad_s, float target_rad_s, float torque_limit_nm);

private:
    const float Model_Gain;
    float Estimated_Speed = 0.0f;
    float Estimated_Disturbance = 0.0f;
    float Command_Torque = 0.0f;
};

struct Struct_Straight_Drive_Config
{
    float wheel_radius_m;
    float target_distance_m;
    float max_speed_rad_s;
    float ramp_rad_s2;
    float torque_limit_nm;
};

// Wheel angles, speeds and torques are positive in the vehicle's forward direction.
class Class_Straight_Drive
{
public:
    Class_Straight_Drive(const Struct_Straight_Drive_Config &config, float right_gain, float left_gain)
        : Config(config), Wheel{Class_Wheel_LADRC(right_gain), Class_Wheel_LADRC(left_gain)} {}

    bool Start(const std::array<float, 2> &angle, const std::array<float, 2> &speed,
               float heading_rad, uint32_t now_ms);
    void Request_Stop(uint32_t now_ms);
    void Stop();
    bool Is_Running() const { return Running; }
    float Get_Distance_M() const { return Distance_M; }

    std::array<float, 2> Update(const std::array<float, 2> &angle, const std::array<float, 2> &speed,
                                float heading_rad, float yaw_rate_rad_s, uint32_t now_ms);

private:
    const Struct_Straight_Drive_Config Config;
    Class_Wheel_LADRC Wheel[2];
    std::array<float, 2> Start_Angle = {};
    uint32_t Start_Ms = 0;
    uint32_t Stop_Start_Ms = 0;
    uint16_t Stopped_Sample_Count = 0;
    bool Running = false;
    bool Stopping = false;
    float Distance_M = 0.0f;
    float Start_Heading_Rad = 0.0f;
    float Forward_Rad_S = 0.0f;
};

#endif
