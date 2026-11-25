#ifndef RESCUE_MISSION_H
#define RESCUE_MISSION_H

#include "1_Middleware/Protocol/rescue_protocol.h"

#include <stdint.h>

enum Enum_Mission_State : uint8_t
{
    MISSION_PRE_MATCH = 0U,
    MISSION_BENCH_STRAIGHT = 1U,
    MISSION_LAUNCH_CROSS_BUMPS = 2U,
    MISSION_SEARCH_TARGET = 3U,
    MISSION_APPROACH_TARGET = 4U,
    MISSION_INSPECT_CORRAL = 5U,
    MISSION_BACK_OUT_SEPARATE = 6U,
    MISSION_CLOSE_GATE_VERIFY = 7U,
    MISSION_FIND_HOME = 8U,
    MISSION_APPROACH_HOME = 9U,
    MISSION_DROP_OPEN_AND_PUSH = 10U,
    MISSION_DROP_BACKOUT_VERIFY = 11U,
    MISSION_ESCAPE_RECOVERY = 12U,
    MISSION_MATCH_FINISHED = 13U,
    MISSION_FAULT_STOP = 14U,
};

enum Enum_Mission_Drive_Action : uint8_t
{
    DRIVE_ACTION_IDLE = 0U,
    DRIVE_ACTION_KEEP = 1U,
    DRIVE_ACTION_VELOCITY = 2U,
    DRIVE_ACTION_START_DISTANCE = 3U,
    DRIVE_ACTION_START_TURN = 4U,
    DRIVE_ACTION_SOFT_STOP = 5U,
    DRIVE_ACTION_EMERGENCY_STOP = 6U,
    DRIVE_ACTION_BENCH_STRAIGHT = 7U,
};

enum Enum_Corral_Verdict : uint8_t
{
    CORRAL_EMPTY = 0U,
    CORRAL_VALID_MATERIALS = 1U,
    CORRAL_VALID_CASUALTY = 2U,
    CORRAL_ILLEGAL_HAZARD_OR_EXCESS = 3U,
};

struct Struct_Mission_Config
{
    uint32_t match_duration_ms = 180000U;      // 3-minute competition round
    uint32_t vision_timeout_ms = 300U;         // Vision freshness window
    uint32_t heartbeat_timeout_ms = 1000U;     // Link liveness window (never refreshes vision)
    uint32_t visual_stall_ms = 800U;           // Track-slip visual stall window (< 15 s DQ)
    uint32_t inspect_timeout_ms = 2500U;
    uint32_t drop_verify_timeout_ms = 2500U;
    uint8_t confirm_frames_required = 3U;

    float bump_cross_distance_m = 0.55f;       // Cross 3x 10 mm speed bumps out of start zone
    float bump_cross_speed_mps = 0.11f;
    float approach_max_speed_mps = 0.10f;
    float approach_min_speed_mps = 0.035f;
    float search_omega_rad_s = 0.45f;
    float align_kp_omega = 1.6f;
    float align_max_omega_rad_s = 0.70f;
    uint16_t target_capture_dist_mm = 210U;
    uint16_t home_drop_dist_mm = 230U;

    float separate_backout_m = -0.18f;
    float separate_turn_rad = 0.45f;
    float drop_push_distance_m = 0.16f;
    float drop_push_speed_mps = 0.07f;
    float drop_backout_distance_m = -0.24f;
    float drop_backout_speed_mps = 0.09f;
    float escape_backout_m = -0.14f;
    float escape_turn_rad = 0.785398f;         // 45 deg alternating escape turn
};

struct Struct_Mission_Inputs
{
    uint32_t now_ms = 0U;
    bool hardware_ready = false;
    bool can_launch = false;
    bool button_pressed = false;
    bool chassis_active = false;
    bool chassis_motion_done = false;
    bool chassis_stalled = false;
    bool gate_settled = false;
    bool camera_settled = false;
};

struct Struct_Mission_Output
{
    Enum_Mission_Drive_Action drive_action = DRIVE_ACTION_IDLE;
    float linear_mps = 0.0f;
    float angular_rad_s = 0.0f;
    float distance_m = 0.0f;
    float speed_mps = 0.0f;
    float delta_yaw_rad = 0.0f;
    uint32_t motion_timeout_ms = 0U;
    bool gate_closed = false;
    bool camera_near = false;
    bool launch_rejected_cue = false;
};

class Class_Rescue_Mission
{
public:
    explicit Class_Rescue_Mission(const Struct_Mission_Config &config = Struct_Mission_Config())
        : Config(config) {}

    void On_Heartbeat(const Struct_Vision_Heartbeat &hb, uint32_t now_ms);
    bool On_Set_Config(const Struct_Match_Config_Cmd &cfg);
    bool On_Vision_State(const Struct_Vision_State &vis, uint32_t now_ms);
    bool On_Bench_Actuator(const Struct_Bench_Actuator_Cmd &cmd);

    Struct_Mission_Output Update(const Struct_Mission_Inputs &in);

    Enum_Corral_Verdict Validate_Corral(const Struct_Vision_State &vis) const;
    bool Is_Target_Eligible(uint8_t target_type) const;
    bool Is_Vision_Fresh(uint32_t now_ms) const;
    bool Is_Link_Alive(uint32_t now_ms) const;

    Enum_Mission_State Get_State() const { return State; }
    uint8_t Get_Team() const { return Team; }
    uint8_t Get_Start_Zone() const { return Start_Zone; }
    uint8_t Get_Run_Mode() const { return Run_Mode; }
    uint8_t Get_Max_Materials_Per_Trip() const { return Max_Materials_Per_Trip; }
    bool Is_Config_Locked() const { return Config_Locked; }
    bool Is_First_Green_Unlocked() const { return First_Green_Unlocked; }
    bool Is_Carrying_Casualty() const { return Cargo_Is_Casualty; }
    bool Is_Gate_Closed_Cmd() const { return Gate_Closed_Cmd; }
    bool Is_Camera_Near_Cmd() const { return Camera_Near_Cmd; }
    uint32_t Get_Match_Elapsed_Ms(uint32_t now_ms) const;
    uint8_t Get_Delivered_Green() const { return Delivered_Green; }
    uint8_t Get_Delivered_Black() const { return Delivered_Black; }
    uint8_t Get_Delivered_Orange() const { return Delivered_Orange; }

private:
    void Enter_State(Enum_Mission_State next, uint32_t now_ms);
    void Start_Escape(uint32_t now_ms, bool allow_reverse, Struct_Mission_Output &out);

    const Struct_Mission_Config Config;

    Enum_Mission_State State = MISSION_PRE_MATCH;
    uint32_t State_Enter_Ms = 0U;
    uint8_t Sub_Phase = 0U;

    uint8_t Team = RESCUE_TEAM_UNSET;
    uint8_t Start_Zone = 0U;
    uint8_t Run_Mode = RESCUE_MODE_COMPETITION;
    uint8_t Max_Materials_Per_Trip = 1U;
    bool Config_Locked = false;
    bool First_Green_Unlocked = false;

    bool Gate_Closed_Cmd = false;
    bool Camera_Near_Cmd = false;

    uint32_t Match_Start_Ms = 0U;
    uint32_t Match_Final_Elapsed_Ms = 0U;

    bool Heartbeat_Received = false;
    uint32_t Last_Heartbeat_Ms = 0U;

    bool Vision_Received = false;
    uint16_t Last_Vision_Frame_Seq = 0U;
    uint32_t Last_Vision_Ms = 0U;
    Struct_Vision_State Latest_Vision = {};

    bool Has_Confirm_Frame = false;
    uint16_t Last_Confirm_Frame_Seq = 0U;
    uint8_t Confirm_Count = 0U;
    uint32_t Visual_Stall_Counter_Ms = 0U;
    float Escape_Sign = 1.0f;

    // Cargo locked upon gate closure for safe-zone half matching and score counting.
    bool Cargo_Is_Casualty = false;
    uint8_t Cargo_Green = 0U;
    uint8_t Cargo_Black = 0U;
    uint8_t Cargo_Orange = 0U;

    uint8_t Delivered_Green = 0U;
    uint8_t Delivered_Black = 0U;
    uint8_t Delivered_Orange = 0U;
};

#endif
