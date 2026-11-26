#include "rescue_mission.h"

#include <algorithm>
#include <cmath>

static constexpr float Millirad_To_Rad = 0.001f;
static constexpr uint32_t Proven_Corridor_Expiry_Ms = 3000U;
static constexpr float Proven_Corridor_Max_Yaw_Rad = 0.10f; // ~5.7 deg cumulative turn invalidates straight corridor

void Class_Rescue_Mission::On_Heartbeat(const Struct_Vision_Heartbeat &hb, uint32_t now_ms)
{
    (void)hb;
    // Heartbeat confirms UART link liveness ONLY; it never extends vision observation freshness.
    Heartbeat_Received = true;
    Last_Heartbeat_Ms = now_ms;
}

bool Class_Rescue_Mission::On_Set_Config(const Struct_Match_Config_Cmd &cfg)
{
    // Configuration is frozen once the one-button competition launch occurs.
    if (Config_Locked || State != MISSION_PRE_MATCH)
    {
        return false;
    }
    if (cfg.team > RESCUE_TEAM_BLUE || cfg.start_zone > 4U || cfg.run_mode > RESCUE_MODE_BENCH_STRAIGHT)
    {
        return false;
    }
    Team = cfg.team;
    Start_Zone = cfg.start_zone;
    Run_Mode = cfg.run_mode;
    Max_Materials_Per_Trip = std::clamp<uint8_t>(cfg.max_materials_per_trip, 1U, 3U);
    return true;
}

bool Class_Rescue_Mission::On_Vision_State(const Struct_Vision_State &vis, uint32_t now_ms)
{
    if (vis.camera_view > CAMERA_VIEW_NEAR || vis.target_type > TARGET_CYAN_HAZARD || vis.confidence > 100U)
    {
        return false;
    }

    if (Vision_Received)
    {
        // Enforce forward uint16 sequence progression with wraparound handling.
        const uint16_t delta = static_cast<uint16_t>(vis.frame_seq - Last_Vision_Frame_Seq);
        const bool forward_step = (delta >= 1U && delta < 0x8000U);
        // Explicit camera-restart policy: after a sustained outage, allow rebooting seq 0..5.
        const bool camera_restarted =
            (uint32_t)(now_ms - Last_Vision_Ms) >= Config.camera_restart_resync_ms && vis.frame_seq <= 5U;
        if (!forward_step && !camera_restarted)
        {
            return false;
        }
    }

    Latest_Vision = vis;
    Last_Vision_Frame_Seq = vis.frame_seq;
    Last_Vision_Ms = now_ms;
    Vision_Received = true;
    Vision_Invalidated_By_Rx_Fault = false;
    return true;
}

void Class_Rescue_Mission::On_Uart_Rx_Fault()
{
    Vision_Invalidated_By_Rx_Fault = true;
}

void Class_Rescue_Mission::Latch_Standstill_Boundary(const uint32_t motor_seq[2], const uint32_t motor_ms[2])
{
    if (!Require_Standstill_Gate || motor_seq == nullptr || motor_ms == nullptr)
    {
        return;
    }
    Standstill_Gate_Motor_Seq[0] = motor_seq[0];
    Standstill_Gate_Motor_Seq[1] = motor_seq[1];
    Standstill_Gate_Motor_Ms[0] = motor_ms[0];
    Standstill_Gate_Motor_Ms[1] = motor_ms[1];
    Standstill_Boundary_Pending = false;
}

bool Class_Rescue_Mission::On_Bench_Actuator(const Struct_Bench_Actuator_Cmd &cmd)
{
    // Actuator bench overrides are strictly limited to pre-match bench mode.
    if (Config_Locked || State != MISSION_PRE_MATCH || Run_Mode != RESCUE_MODE_BENCH_STRAIGHT)
    {
        return false;
    }
    if (cmd.gate_cmd == BENCH_SERVO_POS_A)
    {
        Gate_Closed_Cmd = false;
    }
    else if (cmd.gate_cmd == BENCH_SERVO_POS_B)
    {
        Gate_Closed_Cmd = true;
    }

    if (cmd.pitch_cmd == BENCH_SERVO_POS_A)
    {
        Camera_Near_Cmd = false;
    }
    else if (cmd.pitch_cmd == BENCH_SERVO_POS_B)
    {
        Camera_Near_Cmd = true;
    }
    return true;
}

void Class_Rescue_Mission::Fault_Stop(uint32_t now_ms)
{
    const bool is_competition_active =
        State >= MISSION_LAUNCH_CROSS_BUMPS && State <= MISSION_ESCAPE_RECOVERY;
    Clear_Cargo();
    Proven_Rear_Clear_M = 0.0f;
    Proven_Rear_Yaw_Accum_Rad = 0.0f;
    Gate_Closed_Cmd = false;
    Camera_Near_Cmd = false;
    if (is_competition_active)
    {
        Match_Final_Elapsed_Ms = now_ms - Match_Start_Ms;
        Reverse_Motion_In_Flight = false;
        Require_Standstill_Gate = false;
        Standstill_Boundary_Pending = false;
        Enter_State(MISSION_FAULT_STOP, now_ms);
    }
    else if (State == MISSION_BENCH_STRAIGHT)
    {
        Reverse_Motion_In_Flight = false;
        Require_Standstill_Gate = false;
        Standstill_Boundary_Pending = false;
        Enter_State(MISSION_PRE_MATCH, now_ms);
    }
}

bool Class_Rescue_Mission::Is_Vision_Fresh(uint32_t now_ms) const
{
    return Vision_Received && !Vision_Invalidated_By_Rx_Fault &&
           (uint32_t)(now_ms - Last_Vision_Ms) < Config.vision_timeout_ms;
}

bool Class_Rescue_Mission::Is_Link_Alive(uint32_t now_ms) const
{
    const bool hb_fresh = Heartbeat_Received && (uint32_t)(now_ms - Last_Heartbeat_Ms) < Config.heartbeat_timeout_ms;
    return hb_fresh || Is_Vision_Fresh(now_ms);
}

uint32_t Class_Rescue_Mission::Get_Match_Elapsed_Ms(uint32_t now_ms) const
{
    if (!Config_Locked)
    {
        return 0U;
    }
    if (State == MISSION_MATCH_FINISHED || State == MISSION_FAULT_STOP)
    {
        return Match_Final_Elapsed_Ms;
    }
    return now_ms - Match_Start_Ms;
}

bool Class_Rescue_Mission::Is_Target_Eligible(uint8_t target_type) const
{
    if (target_type == TARGET_NONE || target_type == TARGET_CYAN_HAZARD)
    {
        return false;
    }
    if (!First_Green_Unlocked)
    {
        return target_type == TARGET_GREEN_MATERIAL;
    }
    return target_type == TARGET_GREEN_MATERIAL ||
           target_type == TARGET_BLACK_CORE ||
           target_type == TARGET_ORANGE_CASUALTY;
}

Enum_Corral_Verdict Class_Rescue_Mission::Validate_Corral(const Struct_Vision_State &vis) const
{
    const bool safe_flags = (vis.flags & VIS_FLAG_CORRAL_SAFE) != 0U &&
                            (vis.flags & VIS_FLAG_SWEEP_CLEAR) != 0U;
    if (!safe_flags || vis.corral_cyan_cnt > 0U || vis.corral_unknown_cnt > 0U)
    {
        return CORRAL_ILLEGAL_HAZARD_OR_EXCESS;
    }

    const uint16_t total = static_cast<uint16_t>(vis.corral_green_cnt) +
                           static_cast<uint16_t>(vis.corral_black_cnt) +
                           static_cast<uint16_t>(vis.corral_orange_cnt);
    if (total == 0U)
    {
        return CORRAL_EMPTY;
    }

    // Rule redline: first valid delivery must be exactly 1 green ordinary material.
    if (!First_Green_Unlocked)
    {
        if (vis.corral_green_cnt == 1U && vis.corral_black_cnt == 0U && vis.corral_orange_cnt == 0U)
        {
            return CORRAL_VALID_MATERIALS;
        }
        return CORRAL_ILLEGAL_HAZARD_OR_EXCESS;
    }

    // Rule redline: orange casualty must always be transported alone (count == 1).
    if (vis.corral_orange_cnt > 0U)
    {
        if (vis.corral_orange_cnt == 1U && vis.corral_green_cnt == 0U && vis.corral_black_cnt == 0U)
        {
            return CORRAL_VALID_CASUALTY;
        }
        return CORRAL_ILLEGAL_HAZARD_OR_EXCESS;
    }

    // Materials (green/black) may be mixed up to configured trip limit (hard max 3).
    const uint8_t limit = std::min<uint8_t>(Max_Materials_Per_Trip, 3U);
    if (total <= limit)
    {
        return CORRAL_VALID_MATERIALS;
    }
    return CORRAL_ILLEGAL_HAZARD_OR_EXCESS;
}

void Class_Rescue_Mission::Enter_State(Enum_Mission_State next, uint32_t now_ms)
{
    State = next;
    State_Enter_Ms = now_ms;
    Sub_Phase = 0U;
    Confirm_Count = 0U;
    Has_Confirm_Frame = false;
    Last_Confirm_Frame_Seq = 0U;
    Visual_Stall_Counter_Ms = 0U;
    Pending_Escape_Reverse_M = 0.0f;
    Standstill_Abrupt_Stop = false;
    Gate_Open_Settled_Latched = false;
    Gate_Settled_Frame_Seq = 0U;
}

void Class_Rescue_Mission::Clear_Cargo()
{
    Cargo_Already_Confirmed = false;
    Cargo_Is_Casualty = false;
    Cargo_Green = 0U;
    Cargo_Black = 0U;
    Cargo_Orange = 0U;
}

void Class_Rescue_Mission::Record_Progress(uint32_t now_ms, bool reset_escapes)
{
    Last_Progress_Ms = now_ms;
    Progress_Dist_Accum_M = 0.0f;
    Progress_Yaw_Accum_Rad = 0.0f;
    if (reset_escapes)
    {
        Consecutive_Escapes = 0U;
    }
}

float Class_Rescue_Mission::Available_Reverse_M(bool vision_fresh, bool rear_sensor_clear,
                                                bool opponent_alert, float requested_abs_m) const
{
    if (!vision_fresh || opponent_alert || requested_abs_m <= 0.0f)
    {
        return 0.0f;
    }
    if (rear_sensor_clear)
    {
        return requested_abs_m;
    }
    const float usable = Proven_Rear_Clear_M - Config.reverse_stop_margin_m;
    if (usable >= Config.min_proven_reverse_m)
    {
        return std::min(requested_abs_m, usable);
    }
    return 0.0f;
}

bool Class_Rescue_Mission::Is_Reverse_Clearance_Lost(bool rear_sensor_clear) const
{
    if (rear_sensor_clear)
    {
        return false;
    }
    const float floor_m = std::max(0.001f, Config.reverse_stop_margin_m - 0.001f);
    return Proven_Rear_Clear_M < floor_m;
}

bool Class_Rescue_Mission::Is_Active_Reverse_State() const
{
    return Reverse_Motion_In_Flight ||
           (State == MISSION_BACK_OUT_SEPARATE && (Sub_Phase == 1U || Sub_Phase == 2U)) ||
           (State == MISSION_DROP_BACKOUT_VERIFY && Sub_Phase == 1U) ||
           (State == MISSION_ESCAPE_RECOVERY && (Sub_Phase == 0U || Sub_Phase == 1U));
}

bool Class_Rescue_Mission::Is_Fresh_Standstill(const Struct_Mission_Inputs &in) const
{
    if (in.chassis_active || !in.wheels_still)
    {
        return false;
    }
    if (!in.use_motor_feedback_seq)
    {
        return true;
    }
    if (Standstill_Boundary_Pending)
    {
        return false;
    }
    const bool m0_fresh = ((uint32_t)(in.motor_feedback_seq[0] - Standstill_Gate_Motor_Seq[0]) != 0U) ||
                          ((uint32_t)(in.motor_feedback_ms[0] - Standstill_Gate_Motor_Ms[0]) != 0U);
    const bool m1_fresh = ((uint32_t)(in.motor_feedback_seq[1] - Standstill_Gate_Motor_Seq[1]) != 0U) ||
                          ((uint32_t)(in.motor_feedback_ms[1] - Standstill_Gate_Motor_Ms[1]) != 0U);
    return m0_fresh && m1_fresh;
}

void Class_Rescue_Mission::Engage_Standstill_Gate(uint32_t now_ms, const Struct_Mission_Inputs &in)
{
    if (!Require_Standstill_Gate)
    {
        Require_Standstill_Gate = true;
        Standstill_Gate_Start_Ms = now_ms;
        Standstill_Gate_Motor_Seq[0] = in.motor_feedback_seq[0];
        Standstill_Gate_Motor_Seq[1] = in.motor_feedback_seq[1];
        Standstill_Gate_Motor_Ms[0] = in.motor_feedback_ms[0];
        Standstill_Gate_Motor_Ms[1] = in.motor_feedback_ms[1];
        // When motor CAN feedback sequence tracking is active, require the control task to latch
        // the post-stop feedback boundary AFTER applying Emergency_Stop and transmitting zero torque.
        Standstill_Boundary_Pending = in.use_motor_feedback_seq;
    }
}

void Class_Rescue_Mission::Start_Escape(const Struct_Mission_Inputs &in, float reverse_abs_m,
                                        Struct_Mission_Output &out)
{
    if (++Consecutive_Escapes > Config.max_consecutive_escapes)
    {
        Fault_Stop(in.now_ms);
        out.drive_action = DRIVE_ACTION_EMERGENCY_STOP;
        out.gate_closed = Gate_Closed_Cmd;
        out.camera_near = Camera_Near_Cmd;
        return;
    }

    const bool was_reversing = Is_Active_Reverse_State();
    Enter_State(MISSION_ESCAPE_RECOVERY, in.now_ms);
    Pending_Escape_Reverse_M = (reverse_abs_m >= Config.min_proven_reverse_m) ? reverse_abs_m : 0.0f;
    if (was_reversing || in.chassis_active || !in.wheels_still)
    {
        // Cancel active motion immediately and confirm fresh standstill before reversing or turning.
        if (was_reversing || in.use_motor_feedback_seq)
        {
            Engage_Standstill_Gate(in.now_ms, in);
        }
        Sub_Phase = (Pending_Escape_Reverse_M > 0.0f) ? 3U : 1U;
        Standstill_Abrupt_Stop = true;
        out.drive_action = DRIVE_ACTION_EMERGENCY_STOP;
        return;
    }
    if (Pending_Escape_Reverse_M >= Config.min_proven_reverse_m)
    {
        Sub_Phase = 0U;
        out.drive_action = DRIVE_ACTION_START_DISTANCE;
        out.distance_m = -Pending_Escape_Reverse_M;
        out.speed_mps = Config.drop_backout_speed_mps;
        out.motion_timeout_ms = 3500U;
    }
    else
    {
        Sub_Phase = 2U;
        out.drive_action = DRIVE_ACTION_START_TURN;
        out.delta_yaw_rad = Escape_Sign * Config.escape_turn_rad;
        out.angular_rad_s = Config.align_max_omega_rad_s;
        out.motion_timeout_ms = 4000U;
        Escape_Sign = -Escape_Sign;
    }
}

Struct_Mission_Output Class_Rescue_Mission::Update(const Struct_Mission_Inputs &in)
{
    Struct_Mission_Output out{};
    out.gate_closed = Gate_Closed_Cmd;
    out.camera_near = Camera_Near_Cmd;

    const bool is_competition_active =
        State >= MISSION_LAUNCH_CROSS_BUMPS && State <= MISSION_ESCAPE_RECOVERY;

    // Hard safety: hardware fault stops immediately and never auto-restarts.
    if (!in.hardware_ready)
    {
        Fault_Stop(in.now_ms);
        out.gate_closed = Gate_Closed_Cmd;
        out.camera_near = Camera_Near_Cmd;
        out.drive_action = DRIVE_ACTION_EMERGENCY_STOP;
        return out;
    }

    // Match 3-minute timer enforcement.
    if (is_competition_active && (uint32_t)(in.now_ms - Match_Start_Ms) >= Config.match_duration_ms)
    {
        const bool was_reversing = Is_Active_Reverse_State() || Require_Standstill_Gate;
        Match_Final_Elapsed_Ms = Config.match_duration_ms;
        Clear_Cargo();
        Reverse_Motion_In_Flight = false;
        Require_Standstill_Gate = false;
        Standstill_Boundary_Pending = false;
        Gate_Closed_Cmd = false;
        Camera_Near_Cmd = false;
        Enter_State(MISSION_MATCH_FINISHED, in.now_ms);
        out.gate_closed = Gate_Closed_Cmd;
        out.camera_near = Camera_Near_Cmd;
        out.drive_action = was_reversing ? DRIVE_ACTION_EMERGENCY_STOP : DRIVE_ACTION_SOFT_STOP;
        return out;
    }

    // Manual button press while running cleanly aborts the run (immediate emergency stop if reversing).
    if (in.button_pressed && is_competition_active)
    {
        const bool was_reversing = Is_Active_Reverse_State() || Require_Standstill_Gate;
        Match_Final_Elapsed_Ms = in.now_ms - Match_Start_Ms;
        Clear_Cargo();
        Reverse_Motion_In_Flight = false;
        Require_Standstill_Gate = false;
        Standstill_Boundary_Pending = false;
        Gate_Closed_Cmd = false;
        Camera_Near_Cmd = false;
        Enter_State(MISSION_MATCH_FINISHED, in.now_ms);
        out.gate_closed = Gate_Closed_Cmd;
        out.camera_near = Camera_Near_Cmd;
        out.drive_action = was_reversing ? DRIVE_ACTION_EMERGENCY_STOP : DRIVE_ACTION_SOFT_STOP;
        return out;
    }

    // Any UART RX fault/unhealthy observation invalidates cached pre-fault vision so rearming
    // the UART can never resume motion or VISION_FRESH until a newly accepted frame arrives.
    if (!in.uart_rx_healthy)
    {
        Vision_Invalidated_By_Rx_Fault = true;
    }

    if (Reverse_Motion_In_Flight && !Require_Standstill_Gate && !in.use_motor_feedback_seq &&
        !in.chassis_active && in.wheels_still)
    {
        Reverse_Motion_In_Flight = false;
    }

    // Global standstill interlock after any cancelled/interrupted/completed reverse transition:
    // inhibit ALL motion across ALL states until fresh post-stop feedback from EACH motor confirms
    // standstill, or latch a fault.
    if (Require_Standstill_Gate && is_competition_active)
    {
        if (Is_Fresh_Standstill(in))
        {
            Require_Standstill_Gate = false;
            Standstill_Boundary_Pending = false;
            Reverse_Motion_In_Flight = false;
        }
        else
        {
            if ((uint32_t)(in.now_ms - Standstill_Gate_Start_Ms) >= 1500U)
            {
                Fault_Stop(in.now_ms);
            }
            out.gate_closed = Gate_Closed_Cmd;
            out.camera_near = Camera_Near_Cmd;
            out.drive_action = DRIVE_ACTION_EMERGENCY_STOP;
            return out;
        }
    }

    const bool vision_fresh = Is_Vision_Fresh(in.now_ms) && in.uart_rx_healthy;
    const bool rear_sensor_clear = vision_fresh && ((Latest_Vision.flags & VIS_FLAG_REAR_CLEAR) != 0U);
    const bool opponent_alert =
        vision_fresh && ((Latest_Vision.flags & (VIS_FLAG_OPPONENT_NEAR | VIS_FLAG_OPPONENT_ZONE_NEAR)) != 0U);

    // Update proven rear swept corridor and cumulative progress watchdog during active competition.
    if (is_competition_active)
    {
        const float fwd_d = std::isfinite(in.forward_delta_m) ? in.forward_delta_m : 0.0f;
        const float yaw_d = std::isfinite(in.yaw_delta_rad) ? in.yaw_delta_rad : 0.0f;

        if (!vision_fresh || opponent_alert || in.chassis_stalled)
        {
            Proven_Rear_Clear_M = 0.0f;
            Proven_Rear_Yaw_Accum_Rad = 0.0f;
        }
        else
        {
            if (Proven_Rear_Clear_M > 0.0f || fwd_d > 0.0f)
            {
                Proven_Rear_Yaw_Accum_Rad += std::fabs(yaw_d);
            }
            if (Proven_Rear_Yaw_Accum_Rad >= Proven_Corridor_Max_Yaw_Rad)
            {
                // Any cumulative heading change >= ~5.7 deg invalidates the previous straight corridor.
                Proven_Rear_Clear_M = 0.0f;
                Proven_Rear_Yaw_Accum_Rad = 0.0f;
            }
            else if (fwd_d > 0.0f)
            {
                if (Proven_Rear_Clear_M == 0.0f)
                {
                    Proven_Rear_Yaw_Accum_Rad = std::fabs(yaw_d);
                }
                Proven_Rear_Clear_M = std::min(Config.max_proven_reverse_m, Proven_Rear_Clear_M + fwd_d);
                Last_Forward_Clear_Ms = in.now_ms;
            }
            else if (fwd_d < 0.0f)
            {
                Proven_Rear_Clear_M = std::max(0.0f, Proven_Rear_Clear_M + fwd_d);
                if (Proven_Rear_Clear_M == 0.0f)
                {
                    Proven_Rear_Yaw_Accum_Rad = 0.0f;
                }
            }
            else if (Proven_Rear_Clear_M > 0.0f &&
                     (uint32_t)(in.now_ms - Last_Forward_Clear_Ms) >= Proven_Corridor_Expiry_Ms)
            {
                Proven_Rear_Clear_M = 0.0f;
                Proven_Rear_Yaw_Accum_Rad = 0.0f;
            }
        }

        // Physical displacement/yaw progress feeds the < 15 s watchdog (only with fresh vision & no stall).
        if (vision_fresh && !in.chassis_stalled && State != MISSION_ESCAPE_RECOVERY)
        {
            Progress_Dist_Accum_M += std::fabs(fwd_d);
            Progress_Yaw_Accum_Rad += std::fabs(yaw_d);
            if (Progress_Dist_Accum_M >= 0.08f || Progress_Yaw_Accum_Rad >= 0.25f)
            {
                Record_Progress(in.now_ms, false);
            }
        }

        if ((uint32_t)(in.now_ms - Last_Progress_Ms) >= Config.progress_watchdog_ms)
        {
            Fault_Stop(in.now_ms);
            out.gate_closed = Gate_Closed_Cmd;
            out.camera_near = Camera_Near_Cmd;
            out.drive_action = DRIVE_ACTION_EMERGENCY_STOP;
            return out;
        }

        // Unified vision freshness interlock across ALL active competition states:
        // never continue blind open-loop motion on stale vision frames.
        if (!vision_fresh)
        {
            if (!Vision_Outage_Active)
            {
                Vision_Outage_Active = true;
                Vision_Outage_Start_Ms = in.now_ms;
            }
            Interrupted_By_Stale_Vision = true;
            if ((uint32_t)(in.now_ms - Vision_Outage_Start_Ms) >= Config.vision_abort_ms)
            {
                Fault_Stop(in.now_ms);
                out.gate_closed = Gate_Closed_Cmd;
                out.camera_near = Camera_Near_Cmd;
                out.drive_action = DRIVE_ACTION_EMERGENCY_STOP;
                return out;
            }
            const bool in_active_reverse = Is_Active_Reverse_State();
            if (in_active_reverse)
            {
                Engage_Standstill_Gate(in.now_ms, in);
            }
            out.drive_action = in_active_reverse ? DRIVE_ACTION_EMERGENCY_STOP : DRIVE_ACTION_SOFT_STOP;
            return out;
        }

        Vision_Outage_Active = false;
        if (Interrupted_By_Stale_Vision)
        {
            Interrupted_By_Stale_Vision = false;
            const bool need_standstill = Is_Active_Reverse_State() || in.chassis_active || !in.wheels_still;
            // After pausing an open-loop motion due to stale vision, re-evaluate safely instead of resuming blind,
            // and retain the global standstill interlock until fresh post-stop feedback confirms wheels have stopped.
            if (State == MISSION_LAUNCH_CROSS_BUMPS || State == MISSION_BACK_OUT_SEPARATE ||
                State == MISSION_ESCAPE_RECOVERY)
            {
                if (need_standstill)
                {
                    Engage_Standstill_Gate(in.now_ms, in);
                }
                if (Gate_Closed_Cmd)
                {
                    Camera_Near_Cmd = true;
                    Enter_State(MISSION_CLOSE_GATE_VERIFY, in.now_ms);
                }
                else
                {
                    Camera_Near_Cmd = false;
                    Enter_State(MISSION_SEARCH_TARGET, in.now_ms);
                }
                out.gate_closed = Gate_Closed_Cmd;
                out.camera_near = Camera_Near_Cmd;
                out.drive_action = DRIVE_ACTION_EMERGENCY_STOP;
                return out;
            }
            if (State == MISSION_DROP_OPEN_AND_PUSH && Sub_Phase == 2U)
            {
                if (need_standstill)
                {
                    Engage_Standstill_Gate(in.now_ms, in);
                }
                // Verify in place after interrupted push without blind reversing.
                Enter_State(MISSION_DROP_BACKOUT_VERIFY, in.now_ms);
                Sub_Phase = 2U;
                Standstill_Abrupt_Stop = true;
                out.gate_closed = Gate_Closed_Cmd;
                out.camera_near = Camera_Near_Cmd;
                out.drive_action = DRIVE_ACTION_EMERGENCY_STOP;
                return out;
            }
            if (State == MISSION_DROP_BACKOUT_VERIFY && (Sub_Phase == 0U || Sub_Phase == 1U || need_standstill))
            {
                if (need_standstill)
                {
                    Engage_Standstill_Gate(in.now_ms, in);
                }
                Sub_Phase = 2U;
                State_Enter_Ms = in.now_ms;
                Standstill_Abrupt_Stop = true;
                out.drive_action = DRIVE_ACTION_EMERGENCY_STOP;
                return out;
            }
        }

        // Unified opponent / opponent-zone interlock across ALL forward-moving states.
        const bool forward_motion_state =
            State == MISSION_LAUNCH_CROSS_BUMPS || State == MISSION_SEARCH_TARGET ||
            State == MISSION_APPROACH_TARGET || State == MISSION_FIND_HOME ||
            State == MISSION_APPROACH_HOME || State == MISSION_DROP_OPEN_AND_PUSH;
        if (opponent_alert && forward_motion_state)
        {
            // Never reverse blindly on opponent alert unless explicit rear sensor clears it (Available_Reverse_M returns 0).
            Start_Escape(in, 0.0f, out);
            out.gate_closed = Gate_Closed_Cmd;
            out.camera_near = Camera_Near_Cmd;
            return out;
        }

        // Visual slip/stall + chassis stall watchdog across forward/search states (< 1 s trigger).
        if (forward_motion_state)
        {
            const bool vis_stalled = (Latest_Vision.flags & VIS_FLAG_MOTION_STALLED) != 0U;
            if (vis_stalled)
            {
                if (Visual_Stall_Counter_Ms < Config.visual_stall_ms)
                {
                    ++Visual_Stall_Counter_Ms;
                }
            }
            else
            {
                Visual_Stall_Counter_Ms = (Visual_Stall_Counter_Ms >= 2U) ? (Visual_Stall_Counter_Ms - 2U) : 0U;
            }

            if (in.chassis_stalled || Visual_Stall_Counter_Ms >= Config.visual_stall_ms)
            {
                const float rev_m = Available_Reverse_M(
                    vision_fresh, rear_sensor_clear, opponent_alert, std::fabs(Config.escape_backout_m));
                Start_Escape(in, rev_m, out);
                out.gate_closed = Gate_Closed_Cmd;
                out.camera_near = Camera_Near_Cmd;
                return out;
            }
        }
    }

    switch (State)
    {
    case MISSION_PRE_MATCH:
        out.drive_action = DRIVE_ACTION_IDLE;
        if (in.button_pressed && in.can_launch)
        {
            if (Run_Mode == RESCUE_MODE_BENCH_STRAIGHT)
            {
                Enter_State(MISSION_BENCH_STRAIGHT, in.now_ms);
                out.drive_action = DRIVE_ACTION_BENCH_STRAIGHT;
            }
            else if ((Team == RESCUE_TEAM_RED || Team == RESCUE_TEAM_BLUE) && vision_fresh)
            {
                Config_Locked = true;
                Match_Start_Ms = in.now_ms;
                Vision_Outage_Active = false;
                Interrupted_By_Stale_Vision = false;
                Proven_Rear_Clear_M = 0.0f;
                Proven_Rear_Yaw_Accum_Rad = 0.0f;
                Last_Forward_Clear_Ms = in.now_ms;
                Record_Progress(in.now_ms, true);
                Gate_Closed_Cmd = false;
                Camera_Near_Cmd = false;
                Enter_State(MISSION_LAUNCH_CROSS_BUMPS, in.now_ms);
                out.gate_closed = Gate_Closed_Cmd;
                out.camera_near = Camera_Near_Cmd;
                out.drive_action = DRIVE_ACTION_START_DISTANCE;
                out.distance_m = Config.bump_cross_distance_m;
                out.speed_mps = Config.bump_cross_speed_mps;
                out.motion_timeout_ms = 10000U;
            }
            else
            {
                // Refuse blind competition launch if team not selected or MaixCAM offline.
                out.launch_rejected_cue = true;
            }
        }
        break;

    case MISSION_BENCH_STRAIGHT:
        out.drive_action = DRIVE_ACTION_BENCH_STRAIGHT;
        if (!in.chassis_active && (uint32_t)(in.now_ms - State_Enter_Ms) > 20U)
        {
            Enter_State(MISSION_PRE_MATCH, in.now_ms);
            out.drive_action = DRIVE_ACTION_IDLE;
        }
        break;

    case MISSION_LAUNCH_CROSS_BUMPS:
        Gate_Closed_Cmd = false;
        Camera_Near_Cmd = false;
        if (in.chassis_motion_done)
        {
            Record_Progress(in.now_ms, true);
            Enter_State(MISSION_SEARCH_TARGET, in.now_ms);
            out.drive_action = DRIVE_ACTION_SOFT_STOP;
        }
        else if (in.chassis_motion_aborted || (!in.chassis_active && (uint32_t)(in.now_ms - State_Enter_Ms) > 20U))
        {
            const float rev_m = Available_Reverse_M(
                vision_fresh, rear_sensor_clear, opponent_alert, std::fabs(Config.escape_backout_m));
            Start_Escape(in, rev_m, out);
        }
        else
        {
            out.drive_action = DRIVE_ACTION_KEEP;
        }
        break;

    case MISSION_SEARCH_TARGET:
        Gate_Closed_Cmd = false;
        Camera_Near_Cmd = false;
        if (!in.camera_settled || Latest_Vision.camera_view != CAMERA_VIEW_FAR)
        {
            out.drive_action = DRIVE_ACTION_SOFT_STOP;
            break;
        }
        if ((Latest_Vision.flags & VIS_FLAG_TARGET_VALID) != 0U &&
            Is_Target_Eligible(Latest_Vision.target_type))
        {
            Enter_State(MISSION_APPROACH_TARGET, in.now_ms);
            out.drive_action = DRIVE_ACTION_SOFT_STOP;
        }
        else
        {
            // Slow in-place yaw sweep with single-axis pitch camera held at FAR preset.
            out.drive_action = DRIVE_ACTION_VELOCITY;
            out.linear_mps = 0.0f;
            out.angular_rad_s = (Team == RESCUE_TEAM_BLUE) ? -Config.search_omega_rad_s : Config.search_omega_rad_s;
        }
        break;

    case MISSION_APPROACH_TARGET:
        Gate_Closed_Cmd = false;
        Camera_Near_Cmd = false;
        if (!in.camera_settled || Latest_Vision.camera_view != CAMERA_VIEW_FAR)
        {
            out.drive_action = DRIVE_ACTION_SOFT_STOP;
            break;
        }
        if ((Latest_Vision.flags & VIS_FLAG_TARGET_VALID) == 0U ||
            !Is_Target_Eligible(Latest_Vision.target_type))
        {
            Enter_State(MISSION_SEARCH_TARGET, in.now_ms);
            out.drive_action = DRIVE_ACTION_SOFT_STOP;
            break;
        }
        if (Latest_Vision.target_dist_mm > 0U && Latest_Vision.target_dist_mm <= Config.target_capture_dist_mm &&
            std::fabs(static_cast<float>(Latest_Vision.target_err_x_mrad) * Millirad_To_Rad) <= 0.14f)
        {
            Camera_Near_Cmd = true;
            Enter_State(MISSION_INSPECT_CORRAL, in.now_ms);
            out.drive_action = DRIVE_ACTION_SOFT_STOP;
            break;
        }
        {
            const float err_rad = static_cast<float>(Latest_Vision.target_err_x_mrad) * Millirad_To_Rad;
            const float omega = std::clamp(Config.align_kp_omega * err_rad,
                                           -Config.align_max_omega_rad_s, Config.align_max_omega_rad_s);
            const float align_scale = std::clamp(1.0f - std::fabs(err_rad) / 0.45f, 0.0f, 1.0f);
            const float linear = (align_scale > 0.2f)
                                     ? (Config.approach_min_speed_mps +
                                        align_scale * (Config.approach_max_speed_mps - Config.approach_min_speed_mps))
                                     : 0.0f;
            out.drive_action = DRIVE_ACTION_VELOCITY;
            out.linear_mps = linear;
            out.angular_rad_s = omega;
        }
        break;

    case MISSION_INSPECT_CORRAL:
        Gate_Closed_Cmd = false;
        Camera_Near_Cmd = true;
        out.drive_action = DRIVE_ACTION_SOFT_STOP;
        if (in.chassis_active || !in.camera_settled)
        {
            break;
        }
        if ((uint32_t)(in.now_ms - State_Enter_Ms) >= Config.inspect_timeout_ms)
        {
            Camera_Near_Cmd = false;
            Enter_State(MISSION_BACK_OUT_SEPARATE, in.now_ms);
            break;
        }
        if (Latest_Vision.camera_view != CAMERA_VIEW_NEAR ||
            (Has_Confirm_Frame && Latest_Vision.frame_seq == Last_Confirm_Frame_Seq))
        {
            break;
        }
        Has_Confirm_Frame = true;
        Last_Confirm_Frame_Seq = Latest_Vision.frame_seq;
        {
            const Enum_Corral_Verdict verdict = Validate_Corral(Latest_Vision);
            if (verdict == CORRAL_VALID_MATERIALS || verdict == CORRAL_VALID_CASUALTY)
            {
                if (++Confirm_Count >= Config.confirm_frames_required)
                {
                    Cargo_Is_Casualty = (verdict == CORRAL_VALID_CASUALTY);
                    Cargo_Green = Latest_Vision.corral_green_cnt;
                    Cargo_Black = Latest_Vision.corral_black_cnt;
                    Cargo_Orange = Latest_Vision.corral_orange_cnt;
                    Gate_Closed_Cmd = true;
                    Enter_State(MISSION_CLOSE_GATE_VERIFY, in.now_ms);
                }
            }
            else if (verdict == CORRAL_ILLEGAL_HAZARD_OR_EXCESS)
            {
                Camera_Near_Cmd = false;
                Enter_State(MISSION_BACK_OUT_SEPARATE, in.now_ms);
            }
            else
            {
                Confirm_Count = 0U;
            }
        }
        break;

    case MISSION_BACK_OUT_SEPARATE:
        Gate_Closed_Cmd = false;
        Camera_Near_Cmd = false;
        if (Sub_Phase == 0U)
        {
            if (in.chassis_active || !in.wheels_still)
            {
                if ((uint32_t)(in.now_ms - State_Enter_Ms) >= 1500U)
                {
                    Fault_Stop(in.now_ms);
                }
                out.drive_action = DRIVE_ACTION_EMERGENCY_STOP;
                break;
            }
            const float rev_m = Available_Reverse_M(
                vision_fresh, rear_sensor_clear, opponent_alert, std::fabs(Config.separate_backout_m));
            if (rev_m >= Config.min_proven_reverse_m)
            {
                Sub_Phase = 1U;
                State_Enter_Ms = in.now_ms;
                out.drive_action = DRIVE_ACTION_START_DISTANCE;
                out.distance_m = -rev_m;
                out.speed_mps = Config.drop_backout_speed_mps;
                out.motion_timeout_ms = 4000U;
            }
            else
            {
                Sub_Phase = 3U;
                State_Enter_Ms = in.now_ms;
                out.drive_action = DRIVE_ACTION_START_TURN;
                out.delta_yaw_rad = Escape_Sign * Config.separate_turn_rad;
                out.angular_rad_s = Config.align_max_omega_rad_s;
                out.motion_timeout_ms = 4000U;
                Escape_Sign = -Escape_Sign;
            }
        }
        else if (Sub_Phase == 1U)
        {
            const bool reverse_clearance_lost = Is_Reverse_Clearance_Lost(rear_sensor_clear);
            if (opponent_alert || in.chassis_stalled || in.chassis_motion_aborted || reverse_clearance_lost)
            {
                // Cancel reverse immediately (zero torque) and engage global standstill gate before turning.
                Sub_Phase = 2U;
                State_Enter_Ms = in.now_ms;
                Standstill_Abrupt_Stop = true;
                Engage_Standstill_Gate(in.now_ms, in);
                out.drive_action = DRIVE_ACTION_EMERGENCY_STOP;
            }
            else if (in.chassis_motion_done)
            {
                if (!in.use_motor_feedback_seq && !in.chassis_active && in.wheels_still)
                {
                    Reverse_Motion_In_Flight = false;
                    Sub_Phase = 3U;
                    State_Enter_Ms = in.now_ms;
                    out.drive_action = DRIVE_ACTION_START_TURN;
                    out.delta_yaw_rad = Escape_Sign * Config.separate_turn_rad;
                    out.angular_rad_s = Config.align_max_omega_rad_s;
                    out.motion_timeout_ms = 4000U;
                    Escape_Sign = -Escape_Sign;
                }
                else
                {
                    Sub_Phase = 2U;
                    State_Enter_Ms = in.now_ms;
                    Standstill_Abrupt_Stop = false;
                    if (in.use_motor_feedback_seq)
                    {
                        Engage_Standstill_Gate(in.now_ms, in);
                    }
                    out.drive_action = in.use_motor_feedback_seq ? DRIVE_ACTION_EMERGENCY_STOP : DRIVE_ACTION_SOFT_STOP;
                }
            }
            else if (!in.chassis_active)
            {
                Start_Escape(in, 0.0f, out);
            }
            else
            {
                out.drive_action = DRIVE_ACTION_KEEP;
            }
        }
        else if (Sub_Phase == 2U)
        {
            const bool reverse_clearance_lost = Is_Reverse_Clearance_Lost(rear_sensor_clear);
            if (in.chassis_active || !in.wheels_still)
            {
                if ((uint32_t)(in.now_ms - State_Enter_Ms) >= 1500U)
                {
                    Fault_Stop(in.now_ms);
                    out.drive_action = DRIVE_ACTION_EMERGENCY_STOP;
                }
                else
                {
                    const bool hard_stop =
                        Standstill_Abrupt_Stop || reverse_clearance_lost || opponent_alert || in.chassis_stalled;
                    out.drive_action = hard_stop ? DRIVE_ACTION_EMERGENCY_STOP : DRIVE_ACTION_SOFT_STOP;
                }
            }
            else
            {
                Sub_Phase = 3U;
                State_Enter_Ms = in.now_ms;
                out.drive_action = DRIVE_ACTION_START_TURN;
                out.delta_yaw_rad = Escape_Sign * Config.separate_turn_rad;
                out.angular_rad_s = Config.align_max_omega_rad_s;
                out.motion_timeout_ms = 4000U;
                Escape_Sign = -Escape_Sign;
            }
        }
        else
        {
            if (in.chassis_motion_done)
            {
                Enter_State(MISSION_SEARCH_TARGET, in.now_ms);
                out.drive_action = DRIVE_ACTION_SOFT_STOP;
            }
            else if (in.chassis_stalled || in.chassis_motion_aborted || !in.chassis_active)
            {
                Start_Escape(in, 0.0f, out);
            }
            else
            {
                out.drive_action = DRIVE_ACTION_KEEP;
            }
        }
        break;

    case MISSION_CLOSE_GATE_VERIFY:
        Gate_Closed_Cmd = true;
        Camera_Near_Cmd = true;
        out.drive_action = DRIVE_ACTION_SOFT_STOP;
        if ((uint32_t)(in.now_ms - State_Enter_Ms) >= Config.inspect_timeout_ms)
        {
            Clear_Cargo();
            Gate_Closed_Cmd = false;
            Camera_Near_Cmd = false;
            Enter_State(MISSION_BACK_OUT_SEPARATE, in.now_ms);
            break;
        }
        if (!in.camera_settled || Latest_Vision.camera_view != CAMERA_VIEW_NEAR ||
            (Has_Confirm_Frame && Latest_Vision.frame_seq == Last_Confirm_Frame_Seq))
        {
            break;
        }
        Has_Confirm_Frame = true;
        Last_Confirm_Frame_Seq = Latest_Vision.frame_seq;
        {
            const Enum_Corral_Verdict verdict = Validate_Corral(Latest_Vision);
            // Continuously inspect hazards EVEN WHILE gate is still moving; cancel immediately!
            if (verdict == CORRAL_ILLEGAL_HAZARD_OR_EXCESS)
            {
                Clear_Cargo();
                Gate_Closed_Cmd = false;
                Camera_Near_Cmd = false;
                Enter_State(MISSION_BACK_OUT_SEPARATE, in.now_ms);
                break;
            }
            // Only count confirmation frames after the gate has mechanically settled.
            if (!in.gate_settled)
            {
                Confirm_Count = 0U;
                break;
            }
            if (verdict == CORRAL_VALID_MATERIALS || verdict == CORRAL_VALID_CASUALTY)
            {
                if (++Confirm_Count >= Config.confirm_frames_required)
                {
                    Cargo_Is_Casualty = (verdict == CORRAL_VALID_CASUALTY);
                    Cargo_Green = Latest_Vision.corral_green_cnt;
                    Cargo_Black = Latest_Vision.corral_black_cnt;
                    Cargo_Orange = Latest_Vision.corral_orange_cnt;
                    // Only count initial pickup as new progress; re-verifying the same cargo after
                    // an escape must NEVER reset the escape budget or progress watchdog!
                    if (!Cargo_Already_Confirmed)
                    {
                        Cargo_Already_Confirmed = true;
                        Record_Progress(in.now_ms, true);
                    }
                    Camera_Near_Cmd = false;
                    Enter_State(MISSION_FIND_HOME, in.now_ms);
                }
            }
            else
            {
                // Cargo lost while closing gate: open gate and back out.
                Clear_Cargo();
                Gate_Closed_Cmd = false;
                Camera_Near_Cmd = false;
                Enter_State(MISSION_BACK_OUT_SEPARATE, in.now_ms);
            }
        }
        break;

    case MISSION_FIND_HOME:
        Gate_Closed_Cmd = true;
        Camera_Near_Cmd = false;
        if (!in.camera_settled || Latest_Vision.camera_view != CAMERA_VIEW_FAR)
        {
            out.drive_action = DRIVE_ACTION_SOFT_STOP;
            break;
        }
        if ((Latest_Vision.flags & VIS_FLAG_HOME_VALID) != 0U)
        {
            Enter_State(MISSION_APPROACH_HOME, in.now_ms);
            out.drive_action = DRIVE_ACTION_SOFT_STOP;
        }
        else
        {
            out.drive_action = DRIVE_ACTION_VELOCITY;
            out.linear_mps = 0.0f;
            out.angular_rad_s = Config.search_omega_rad_s;
        }
        break;

    case MISSION_APPROACH_HOME:
        Gate_Closed_Cmd = true;
        Camera_Near_Cmd = false;
        if (!in.camera_settled || Latest_Vision.camera_view != CAMERA_VIEW_FAR)
        {
            out.drive_action = DRIVE_ACTION_SOFT_STOP;
            break;
        }
        if ((Latest_Vision.flags & VIS_FLAG_HOME_VALID) == 0U)
        {
            Enter_State(MISSION_FIND_HOME, in.now_ms);
            out.drive_action = DRIVE_ACTION_SOFT_STOP;
            break;
        }
        {
            const float err_rad = static_cast<float>(Latest_Vision.home_err_x_mrad) * Millirad_To_Rad;
            const bool half_ok = (Latest_Vision.flags & VIS_FLAG_HOME_HALF_MATCH) != 0U;
            const bool at_ramp = Latest_Vision.home_dist_mm > 0U &&
                                 Latest_Vision.home_dist_mm <= Config.home_drop_dist_mm &&
                                 std::fabs(err_rad) <= 0.12f;
            if (at_ramp && half_ok)
            {
                Camera_Near_Cmd = true;
                Enter_State(MISSION_DROP_OPEN_AND_PUSH, in.now_ms);
                out.drive_action = DRIVE_ACTION_SOFT_STOP;
                break;
            }
            const float omega = std::clamp(Config.align_kp_omega * err_rad,
                                           -Config.align_max_omega_rad_s, Config.align_max_omega_rad_s);
            const float align_scale = std::clamp(1.0f - std::fabs(err_rad) / 0.40f, 0.0f, 1.0f);
            const bool stop_short_wrong_half =
                (!half_ok) && Latest_Vision.home_dist_mm > 0U &&
                Latest_Vision.home_dist_mm <= (Config.home_drop_dist_mm + 80U);
            const float linear = (!stop_short_wrong_half && align_scale > 0.25f)
                                     ? (Config.approach_min_speed_mps +
                                        align_scale * (Config.approach_max_speed_mps - Config.approach_min_speed_mps))
                                     : 0.0f;
            out.drive_action = DRIVE_ACTION_VELOCITY;
            out.linear_mps = linear;
            out.angular_rad_s = omega;
        }
        break;

    case MISSION_DROP_OPEN_AND_PUSH:
        Camera_Near_Cmd = true;
        if (Sub_Phase == 0U)
        {
            out.drive_action = DRIVE_ACTION_SOFT_STOP;
            if (in.chassis_active || !in.camera_settled)
            {
                break;
            }
            if ((uint32_t)(in.now_ms - State_Enter_Ms) >= Config.inspect_timeout_ms)
            {
                Camera_Near_Cmd = false;
                Enter_State(MISSION_FIND_HOME, in.now_ms);
                break;
            }
            if (Latest_Vision.camera_view != CAMERA_VIEW_NEAR ||
                (Has_Confirm_Frame && Latest_Vision.frame_seq == Last_Confirm_Frame_Seq))
            {
                break;
            }
            Has_Confirm_Frame = true;
            Last_Confirm_Frame_Seq = Latest_Vision.frame_seq;
            const Enum_Corral_Verdict verdict = Validate_Corral(Latest_Vision);
            const bool cargo_matches = (Cargo_Is_Casualty && verdict == CORRAL_VALID_CASUALTY) ||
                                       (!Cargo_Is_Casualty && verdict == CORRAL_VALID_MATERIALS);
            const bool half_ok = (Latest_Vision.flags & VIS_FLAG_HOME_HALF_MATCH) != 0U;
            const bool path_ok = (Latest_Vision.flags & VIS_FLAG_DROP_PATH_CLEAR) != 0U;
            const bool baseline_ready = (Latest_Vision.flags & VIS_FLAG_DROP_BASELINE_READY) != 0U;
            if (!cargo_matches)
            {
                Clear_Cargo();
                Gate_Closed_Cmd = false;
                Camera_Near_Cmd = false;
                Enter_State(MISSION_BACK_OUT_SEPARATE, in.now_ms);
                break;
            }
            if (!path_ok || !baseline_ready)
            {
                Start_Escape(in, 0.0f, out);
                break;
            }
            if (!half_ok)
            {
                Camera_Near_Cmd = false;
                Enter_State(MISSION_APPROACH_HOME, in.now_ms);
                break;
            }
            if (++Confirm_Count >= Config.confirm_frames_required)
            {
                Cargo_Green = Latest_Vision.corral_green_cnt;
                Cargo_Black = Latest_Vision.corral_black_cnt;
                Cargo_Orange = Latest_Vision.corral_orange_cnt;
                Gate_Closed_Cmd = false;
                Sub_Phase = 1U;
                Gate_Open_Settled_Latched = false;
                Gate_Settled_Frame_Seq = Latest_Vision.frame_seq;
                State_Enter_Ms = in.now_ms;
            }
        }
        else if (Sub_Phase == 1U)
        {
            Gate_Closed_Cmd = false;
            out.drive_action = DRIVE_ACTION_SOFT_STOP;
            if ((uint32_t)(in.now_ms - State_Enter_Ms) >= 3000U)
            {
                Clear_Cargo();
                Camera_Near_Cmd = false;
                Enter_State(MISSION_BACK_OUT_SEPARATE, in.now_ms);
                out.drive_action = DRIVE_ACTION_EMERGENCY_STOP;
                break;
            }
            // Continuously recheck latest NEAR frame for corral/sweep/path safety, exact cargo match,
            // and half alignment while gate opens; at the settled edge record frame_seq and require a
            // strictly later NEAR frame acknowledging open-and-settled telemetry before pushing.
            if (in.camera_settled && Latest_Vision.camera_view == CAMERA_VIEW_NEAR)
            {
                const Enum_Corral_Verdict verdict = Validate_Corral(Latest_Vision);
                const bool exact_cargo_match = Latest_Vision.corral_green_cnt == Cargo_Green &&
                                               Latest_Vision.corral_black_cnt == Cargo_Black &&
                                               Latest_Vision.corral_orange_cnt == Cargo_Orange;
                const bool cargo_matches = exact_cargo_match &&
                                           ((Cargo_Is_Casualty && verdict == CORRAL_VALID_CASUALTY) ||
                                            (!Cargo_Is_Casualty && verdict == CORRAL_VALID_MATERIALS));
                const bool half_ok = (Latest_Vision.flags & VIS_FLAG_HOME_HALF_MATCH) != 0U;
                const bool path_ok = (Latest_Vision.flags & VIS_FLAG_DROP_PATH_CLEAR) != 0U;
                const bool baseline_ready = (Latest_Vision.flags & VIS_FLAG_DROP_BASELINE_READY) != 0U;
                if (verdict == CORRAL_ILLEGAL_HAZARD_OR_EXCESS || !cargo_matches || !path_ok || !baseline_ready)
                {
                    Clear_Cargo();
                    Gate_Closed_Cmd = false;
                    Camera_Near_Cmd = false;
                    Enter_State(MISSION_BACK_OUT_SEPARATE, in.now_ms);
                    out.drive_action = DRIVE_ACTION_EMERGENCY_STOP;
                    break;
                }
                if (!half_ok)
                {
                    Gate_Closed_Cmd = true;
                    Camera_Near_Cmd = true;
                    Enter_State(MISSION_CLOSE_GATE_VERIFY, in.now_ms);
                    out.drive_action = DRIVE_ACTION_SOFT_STOP;
                    break;
                }
                if (!in.gate_settled)
                {
                    Gate_Open_Settled_Latched = false;
                }
                else if (!Gate_Open_Settled_Latched)
                {
                    Gate_Open_Settled_Latched = true;
                    Gate_Settled_Frame_Seq = Latest_Vision.frame_seq;
                }
                else
                {
                    const uint16_t seq_delta =
                        static_cast<uint16_t>(Latest_Vision.frame_seq - Gate_Settled_Frame_Seq);
                    const bool post_settle_frame = (seq_delta >= 1U && seq_delta < 0x8000U);
                    const bool open_settled_ack = (Latest_Vision.flags & VIS_FLAG_GATE_OPEN_SETTLED_ACK) != 0U;
                    if (!in.chassis_active && in.wheels_still && post_settle_frame && open_settled_ack)
                    {
                        Sub_Phase = 2U;
                        State_Enter_Ms = in.now_ms;
                        out.drive_action = DRIVE_ACTION_START_DISTANCE;
                        out.distance_m = Config.drop_push_distance_m;
                        out.speed_mps = Config.drop_push_speed_mps;
                        out.motion_timeout_ms = 5000U;
                    }
                }
            }
            else if (!in.gate_settled)
            {
                Gate_Open_Settled_Latched = false;
            }
        }
        else
        {
            Gate_Closed_Cmd = false;
            const bool near_view_ok = in.camera_settled && Latest_Vision.camera_view == CAMERA_VIEW_NEAR;
            const bool push_safe = near_view_ok &&
                                   (Latest_Vision.flags & VIS_FLAG_CORRAL_SAFE) != 0U &&
                                   (Latest_Vision.flags & VIS_FLAG_SWEEP_CLEAR) != 0U &&
                                   (Latest_Vision.flags & VIS_FLAG_DROP_PATH_CLEAR) != 0U &&
                                   Latest_Vision.corral_cyan_cnt == 0U &&
                                   Latest_Vision.corral_unknown_cnt == 0U;
            const bool half_ok = (Latest_Vision.flags & VIS_FLAG_HOME_HALF_MATCH) != 0U;
            if (!push_safe || !half_ok)
            {
                Clear_Cargo();
                Gate_Closed_Cmd = false;
                Camera_Near_Cmd = false;
                Enter_State(MISSION_BACK_OUT_SEPARATE, in.now_ms);
                out.drive_action = DRIVE_ACTION_EMERGENCY_STOP;
                break;
            }
            if (in.chassis_motion_done)
            {
                Enter_State(MISSION_DROP_BACKOUT_VERIFY, in.now_ms);
                if (!in.chassis_active && in.wheels_still)
                {
                    const float rev_m = Available_Reverse_M(
                        vision_fresh, rear_sensor_clear, opponent_alert, std::fabs(Config.drop_backout_distance_m));
                    if (rev_m >= Config.min_proven_reverse_m)
                    {
                        Sub_Phase = 1U;
                        out.drive_action = DRIVE_ACTION_START_DISTANCE;
                        out.distance_m = -rev_m;
                        out.speed_mps = Config.drop_backout_speed_mps;
                        out.motion_timeout_ms = 5000U;
                    }
                    else
                    {
                        Sub_Phase = 2U;
                        out.drive_action = DRIVE_ACTION_SOFT_STOP;
                    }
                }
                else
                {
                    // Confirm standstill from forward push before starting margin-bounded backout.
                    Sub_Phase = 0U;
                    out.drive_action = DRIVE_ACTION_SOFT_STOP;
                }
            }
            else if (in.chassis_motion_aborted || !in.chassis_active)
            {
                Start_Escape(in, 0.0f, out);
            }
            else
            {
                out.drive_action = DRIVE_ACTION_KEEP;
            }
        }
        break;

    case MISSION_DROP_BACKOUT_VERIFY:
        Gate_Closed_Cmd = false;
        Camera_Near_Cmd = true;
        if (Sub_Phase == 0U)
        {
            if (in.chassis_active || !in.wheels_still)
            {
                if ((uint32_t)(in.now_ms - State_Enter_Ms) >= 1500U)
                {
                    Fault_Stop(in.now_ms);
                    out.drive_action = DRIVE_ACTION_EMERGENCY_STOP;
                }
                else
                {
                    out.drive_action = (opponent_alert || in.chassis_stalled)
                                           ? DRIVE_ACTION_EMERGENCY_STOP
                                           : DRIVE_ACTION_SOFT_STOP;
                }
            }
            else
            {
                const float rev_m = Available_Reverse_M(
                    vision_fresh, rear_sensor_clear, opponent_alert, std::fabs(Config.drop_backout_distance_m));
                if (rev_m >= Config.min_proven_reverse_m)
                {
                    Sub_Phase = 1U;
                    State_Enter_Ms = in.now_ms;
                    out.drive_action = DRIVE_ACTION_START_DISTANCE;
                    out.distance_m = -rev_m;
                    out.speed_mps = Config.drop_backout_speed_mps;
                    out.motion_timeout_ms = 5000U;
                }
                else
                {
                    Sub_Phase = 2U;
                    State_Enter_Ms = in.now_ms;
                    out.drive_action = DRIVE_ACTION_SOFT_STOP;
                }
            }
        }
        else if (Sub_Phase == 1U)
        {
            const bool reverse_clearance_lost = Is_Reverse_Clearance_Lost(rear_sensor_clear);
            if (opponent_alert || in.chassis_stalled || in.chassis_motion_aborted || reverse_clearance_lost)
            {
                // Immediate zero-torque stop on abrupt clearance loss + global standstill gate.
                Sub_Phase = 2U;
                State_Enter_Ms = in.now_ms;
                Standstill_Abrupt_Stop = true;
                Confirm_Count = 0U;
                Has_Confirm_Frame = false;
                Last_Confirm_Frame_Seq = 0U;
                Engage_Standstill_Gate(in.now_ms, in);
                out.drive_action = DRIVE_ACTION_EMERGENCY_STOP;
            }
            else if (in.chassis_motion_done || !in.chassis_active)
            {
                Sub_Phase = 2U;
                State_Enter_Ms = in.now_ms;
                Standstill_Abrupt_Stop = false;
                Confirm_Count = 0U;
                Has_Confirm_Frame = false;
                Last_Confirm_Frame_Seq = 0U;
                if (in.use_motor_feedback_seq)
                {
                    Engage_Standstill_Gate(in.now_ms, in);
                }
                out.drive_action = in.use_motor_feedback_seq ? DRIVE_ACTION_EMERGENCY_STOP : DRIVE_ACTION_SOFT_STOP;
            }
            else
            {
                out.drive_action = DRIVE_ACTION_KEEP;
            }
        }
        else
        {
            const bool reverse_clearance_lost = Is_Reverse_Clearance_Lost(rear_sensor_clear);
            const bool hard_stop =
                Standstill_Abrupt_Stop || reverse_clearance_lost || opponent_alert || in.chassis_stalled;
            if (in.chassis_active || !in.wheels_still)
            {
                // Never leave drop verification into search/motion while wheels are still moving!
                if ((uint32_t)(in.now_ms - State_Enter_Ms) >= 1500U)
                {
                    Fault_Stop(in.now_ms);
                    out.drive_action = DRIVE_ACTION_EMERGENCY_STOP;
                }
                else
                {
                    out.drive_action = hard_stop ? DRIVE_ACTION_EMERGENCY_STOP : DRIVE_ACTION_SOFT_STOP;
                }
                break;
            }
            out.drive_action = DRIVE_ACTION_SOFT_STOP;
            if ((uint32_t)(in.now_ms - State_Enter_Ms) >= Config.drop_verify_timeout_ms)
            {
                // Drop NOT confirmed inside own correct half -> do NOT unlock first green.
                Clear_Cargo();
                Camera_Near_Cmd = false;
                Enter_State(MISSION_SEARCH_TARGET, in.now_ms);
                break;
            }
            if (!in.camera_settled || Latest_Vision.camera_view != CAMERA_VIEW_NEAR ||
                (Has_Confirm_Frame && Latest_Vision.frame_seq == Last_Confirm_Frame_Seq))
            {
                break;
            }
            Has_Confirm_Frame = true;
            Last_Confirm_Frame_Seq = Latest_Vision.frame_seq;
            if ((Latest_Vision.flags & VIS_FLAG_DROP_CONFIRMED) != 0U)
            {
                if (++Confirm_Count >= Config.confirm_frames_required)
                {
                    if (!First_Green_Unlocked && !Cargo_Is_Casualty &&
                        Cargo_Green == 1U && Cargo_Black == 0U && Cargo_Orange == 0U)
                    {
                        First_Green_Unlocked = true;
                    }
                    Delivered_Green = static_cast<uint8_t>(Delivered_Green + Cargo_Green);
                    Delivered_Black = static_cast<uint8_t>(Delivered_Black + Cargo_Black);
                    Delivered_Orange = static_cast<uint8_t>(Delivered_Orange + Cargo_Orange);
                    Clear_Cargo();
                    Record_Progress(in.now_ms, true);
                    Camera_Near_Cmd = false;
                    Enter_State(MISSION_SEARCH_TARGET, in.now_ms);
                }
            }
            else
            {
                Confirm_Count = 0U;
            }
        }
        break;

    case MISSION_ESCAPE_RECOVERY:
        if (Sub_Phase == 3U)
        {
            if (in.chassis_active || !in.wheels_still)
            {
                if ((uint32_t)(in.now_ms - State_Enter_Ms) >= 1500U)
                {
                    Fault_Stop(in.now_ms);
                    out.drive_action = DRIVE_ACTION_EMERGENCY_STOP;
                }
                else
                {
                    out.drive_action = DRIVE_ACTION_EMERGENCY_STOP;
                }
            }
            else
            {
                const float rev_m = Available_Reverse_M(
                    vision_fresh, rear_sensor_clear, opponent_alert, Pending_Escape_Reverse_M);
                if (rev_m >= Config.min_proven_reverse_m)
                {
                    Sub_Phase = 0U;
                    State_Enter_Ms = in.now_ms;
                    out.drive_action = DRIVE_ACTION_START_DISTANCE;
                    out.distance_m = -rev_m;
                    out.speed_mps = Config.drop_backout_speed_mps;
                    out.motion_timeout_ms = 3500U;
                }
                else
                {
                    Sub_Phase = 2U;
                    State_Enter_Ms = in.now_ms;
                    out.drive_action = DRIVE_ACTION_START_TURN;
                    out.delta_yaw_rad = Escape_Sign * Config.escape_turn_rad;
                    out.angular_rad_s = Config.align_max_omega_rad_s;
                    out.motion_timeout_ms = 4000U;
                    Escape_Sign = -Escape_Sign;
                }
            }
        }
        else if (Sub_Phase == 0U)
        {
            const bool reverse_clearance_lost = Is_Reverse_Clearance_Lost(rear_sensor_clear);
            if (opponent_alert || in.chassis_stalled || in.chassis_motion_aborted || reverse_clearance_lost)
            {
                // Cancel reverse immediately and engage global standstill gate before turning.
                Sub_Phase = 1U;
                State_Enter_Ms = in.now_ms;
                Standstill_Abrupt_Stop = true;
                Engage_Standstill_Gate(in.now_ms, in);
                out.drive_action = DRIVE_ACTION_EMERGENCY_STOP;
            }
            else if (in.chassis_motion_done || !in.chassis_active)
            {
                if (!in.use_motor_feedback_seq && !in.chassis_active && in.wheels_still)
                {
                    Reverse_Motion_In_Flight = false;
                    Sub_Phase = 2U;
                    State_Enter_Ms = in.now_ms;
                    out.drive_action = DRIVE_ACTION_START_TURN;
                    out.delta_yaw_rad = Escape_Sign * Config.escape_turn_rad;
                    out.angular_rad_s = Config.align_max_omega_rad_s;
                    out.motion_timeout_ms = 4000U;
                    Escape_Sign = -Escape_Sign;
                }
                else
                {
                    Sub_Phase = 1U;
                    State_Enter_Ms = in.now_ms;
                    Standstill_Abrupt_Stop = false;
                    if (in.use_motor_feedback_seq)
                    {
                        Engage_Standstill_Gate(in.now_ms, in);
                    }
                    out.drive_action = in.use_motor_feedback_seq ? DRIVE_ACTION_EMERGENCY_STOP : DRIVE_ACTION_SOFT_STOP;
                }
            }
            else
            {
                out.drive_action = DRIVE_ACTION_KEEP;
            }
        }
        else if (Sub_Phase == 1U)
        {
            const bool reverse_clearance_lost = Is_Reverse_Clearance_Lost(rear_sensor_clear);
            if (in.chassis_active || !in.wheels_still)
            {
                if ((uint32_t)(in.now_ms - State_Enter_Ms) >= 1500U)
                {
                    Fault_Stop(in.now_ms);
                    out.drive_action = DRIVE_ACTION_EMERGENCY_STOP;
                }
                else
                {
                    const bool hard_stop =
                        Standstill_Abrupt_Stop || reverse_clearance_lost || opponent_alert || in.chassis_stalled;
                    out.drive_action = hard_stop ? DRIVE_ACTION_EMERGENCY_STOP : DRIVE_ACTION_SOFT_STOP;
                }
            }
            else
            {
                Sub_Phase = 2U;
                State_Enter_Ms = in.now_ms;
                out.drive_action = DRIVE_ACTION_START_TURN;
                out.delta_yaw_rad = Escape_Sign * Config.escape_turn_rad;
                out.angular_rad_s = Config.align_max_omega_rad_s;
                out.motion_timeout_ms = 4000U;
                Escape_Sign = -Escape_Sign;
            }
        }
        else
        {
            if (in.chassis_motion_done)
            {
                if (Gate_Closed_Cmd)
                {
                    Camera_Near_Cmd = true;
                    Enter_State(MISSION_CLOSE_GATE_VERIFY, in.now_ms);
                }
                else
                {
                    Camera_Near_Cmd = false;
                    Enter_State(MISSION_SEARCH_TARGET, in.now_ms);
                }
                out.drive_action = DRIVE_ACTION_SOFT_STOP;
            }
            else if (in.chassis_stalled || in.chassis_motion_aborted || !in.chassis_active)
            {
                // Turn failed/stalled/timed out: consume escape budget rather than treating as success.
                Start_Escape(in, 0.0f, out);
            }
            else
            {
                out.drive_action = DRIVE_ACTION_KEEP;
            }
        }
        break;

    case MISSION_MATCH_FINISHED:
    case MISSION_FAULT_STOP:
    default:
        Gate_Closed_Cmd = false;
        Camera_Near_Cmd = false;
        out.drive_action = DRIVE_ACTION_IDLE;
        break;
    }

    if (out.drive_action == DRIVE_ACTION_START_DISTANCE && out.distance_m < 0.0f)
    {
        Reverse_Motion_In_Flight = true;
    }
    out.gate_closed = Gate_Closed_Cmd;
    out.camera_near = Camera_Near_Cmd;
    return out;
}
