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
    uint32_t vision_abort_ms = 4000U;          // Sustained vision outage -> fault stop (< 15 s DQ)
    uint32_t camera_restart_resync_ms = 1000U; // Minimum outage before accepting seq restart from 0..5
    uint32_t heartbeat_timeout_ms = 1000U;     // Link liveness window (never refreshes vision)
    uint32_t visual_stall_ms = 800U;           // Track-slip visual stall window (< 15 s DQ)
    uint32_t progress_watchdog_ms = 10000U;    // Cumulative no-progress fault limit (< 15 s DQ)
    uint8_t max_consecutive_escapes = 3U;      // Bounded escape attempts before latching fault
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

    float min_proven_reverse_m = 0.06f;        // Minimum proven swept corridor before reverse is authorized
    float reverse_stop_margin_m = 0.04f;       // Reserved stopping buffer behind authorized reverse distance
    float max_proven_reverse_m = 0.26f;        // Maximum tracked forward swept corridor
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
    bool uart_rx_healthy = true;
    bool button_pressed = false;
    bool chassis_active = false;
    bool wheels_still = false;
    bool chassis_motion_done = false;
    bool chassis_motion_aborted = false;
    bool chassis_stalled = false;
    bool gate_settled = false;
    bool camera_settled = false;
    float forward_delta_m = 0.0f;
    float yaw_delta_rad = 0.0f;
    uint32_t motor_feedback_seq[2] = {};
    uint32_t motor_feedback_ms[2] = {};
    bool use_motor_feedback_seq = false;
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
    void On_Uart_Rx_Fault();
    void Latch_Standstill_Boundary(const uint32_t motor_seq[2], const uint32_t motor_ms[2]);
    void Fault_Stop(uint32_t now_ms);

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
    bool Is_Drop_Pushing() const { return State == MISSION_DROP_OPEN_AND_PUSH && Sub_Phase == 2U; }
    bool Is_Standstill_Gate_Pending_Boundary() const { return Require_Standstill_Gate && Standstill_Boundary_Pending; }
    uint32_t Get_Match_Elapsed_Ms(uint32_t now_ms) const;
    uint8_t Get_Delivered_Green() const { return Delivered_Green; }
    uint8_t Get_Delivered_Black() const { return Delivered_Black; }
    uint8_t Get_Delivered_Orange() const { return Delivered_Orange; }
    uint8_t Get_Consecutive_Escapes() const { return Consecutive_Escapes; }
    float Get_Proven_Rear_Clear_M() const { return Proven_Rear_Clear_M; }

private:
    void Enter_State(Enum_Mission_State next, uint32_t now_ms);
    void Reset_Confirm_Streak();
    void Clear_Cargo();
    void Record_Progress(uint32_t now_ms, bool reset_escapes);
    float Available_Reverse_M(bool vision_fresh, bool rear_sensor_clear,
                              bool opponent_alert, float requested_abs_m) const;
    bool Is_Reverse_Clearance_Lost(bool rear_sensor_clear) const;
    bool Is_Active_Reverse_State() const;
    bool Is_Fresh_Standstill(const Struct_Mission_Inputs &in) const;
    void Engage_Standstill_Gate(uint32_t now_ms, const Struct_Mission_Inputs &in);
    void Start_Escape(const Struct_Mission_Inputs &in, float reverse_abs_m, Struct_Mission_Output &out);

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
    uint32_t Vision_Accept_Counter = 0U;
    Struct_Vision_State Latest_Vision = {};

    bool Vision_Outage_Active = false;
    uint32_t Vision_Outage_Start_Ms = 0U;
    bool Interrupted_By_Stale_Vision = false;
    bool Vision_Invalidated_By_Rx_Fault = false;
    bool Pending_Non_Near_View_Event = false;
    bool Pending_Non_Far_View_Event = false;
    bool Pending_Near_Hazard_Event = false;
    bool Pending_Near_Non_Valid_Corral_Event = false;
    bool Pending_Opponent_Alert_Event = false;
    bool Pending_Rear_Clear_Lost_Event = false;
    bool Pending_Drop_Path_Blocked_Event = false;
    bool Pending_Drop_Baseline_Not_Ready_Event = false;
    bool Pending_Home_Half_Mismatch_Event = false;
    bool Pending_Near_Cargo_Mismatch_Event = false;
    bool Pending_Drop_Unconfirmed_Event = false;

    bool Pre_Match_Wait_Servo_Apply = false;
    bool Pre_Match_Settled_Latched = false;
    bool Pre_Match_Need_Post_Settle_Far = false;
    uint32_t Pre_Match_Settle_Accept_Id = 0U;

    uint32_t Confirm_Boundary_Accept_Id = 0U;
    uint8_t Confirm_Count = 0U;
    uint32_t Visual_Stall_Counter_Ms = 0U;
    float Escape_Sign = 1.0f;

    // Recovery budget & cumulative progress watchdog (< 15 s DQ limit).
    uint8_t Consecutive_Escapes = 0U;
    uint32_t Last_Progress_Ms = 0U;
    float Progress_Dist_Accum_M = 0.0f;
    float Progress_Yaw_Accum_Rad = 0.0f;

    // Swept forward corridor proven clear behind the robot (never fabricated).
    float Proven_Rear_Clear_M = 0.0f;
    float Proven_Rear_Yaw_Accum_Rad = 0.0f;
    uint32_t Last_Forward_Clear_Ms = 0U;
    float Pending_Escape_Reverse_M = 0.0f;
    bool Standstill_Abrupt_Stop = false;

    // Post-settle gate check latch & global standstill interlock tracking reverse through braking.
    bool Gate_Open_Settled_Latched = false;
    uint16_t Gate_Settled_Frame_Seq = 0U;
    bool Reverse_Motion_In_Flight = false;
    bool Require_Standstill_Gate = false;
    bool Standstill_Boundary_Pending = false;
    uint32_t Standstill_Gate_Start_Ms = 0U;
    uint32_t Standstill_Gate_Motor_Seq[2] = {};
    uint32_t Standstill_Gate_Motor_Ms[2] = {};

    // Cargo locked upon gate closure for safe-zone half matching and score counting.
    bool Cargo_Already_Confirmed = false;
    bool Cargo_Is_Casualty = false;
    uint8_t Cargo_Green = 0U;
    uint8_t Cargo_Black = 0U;
    uint8_t Cargo_Orange = 0U;

    uint8_t Delivered_Green = 0U;
    uint8_t Delivered_Black = 0U;
    uint8_t Delivered_Orange = 0U;
};

#endif
