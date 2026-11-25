#include "rescue_mission.h"

#include <algorithm>
#include <cmath>

static constexpr float Millirad_To_Rad = 0.001f;

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
    const uint8_t limit = std::clamp<uint8_t>(cfg.max_materials_per_trip, 1U, 3U);
    Max_Materials_Per_Trip = limit;
    return true;
}

bool Class_Rescue_Mission::On_Vision_State(const Struct_Vision_State &vis, uint32_t now_ms)
{
    if (vis.camera_view > CAMERA_VIEW_NEAR || vis.target_type > TARGET_CYAN_HAZARD || vis.confidence > 100U)
    {
        return false;
    }
    // Duplicate frame_seq cannot spoof fresh observations.
    if (Vision_Received && vis.frame_seq == Last_Vision_Frame_Seq)
    {
        return false;
    }
    Latest_Vision = vis;
    Last_Vision_Frame_Seq = vis.frame_seq;
    Last_Vision_Ms = now_ms;
    Vision_Received = true;
    return true;
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

bool Class_Rescue_Mission::Is_Vision_Fresh(uint32_t now_ms) const
{
    return Vision_Received && (uint32_t)(now_ms - Last_Vision_Ms) < Config.vision_timeout_ms;
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
}

void Class_Rescue_Mission::Start_Escape(uint32_t now_ms, bool allow_reverse, Struct_Mission_Output &out)
{
    Enter_State(MISSION_ESCAPE_RECOVERY, now_ms);
    if (allow_reverse)
    {
        Sub_Phase = 0U;
        out.drive_action = DRIVE_ACTION_START_DISTANCE;
        out.distance_m = Config.escape_backout_m;
        out.speed_mps = Config.drop_backout_speed_mps;
        out.motion_timeout_ms = 4000U;
    }
    else
    {
        Sub_Phase = 1U;
        out.drive_action = DRIVE_ACTION_START_TURN;
        out.delta_yaw_rad = Escape_Sign * Config.escape_turn_rad;
        out.angular_rad_s = Config.align_max_omega_rad_s;
        out.motion_timeout_ms = 4500U;
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
        if (is_competition_active)
        {
            Match_Final_Elapsed_Ms = in.now_ms - Match_Start_Ms;
            Enter_State(MISSION_FAULT_STOP, in.now_ms);
        }
        else if (State == MISSION_BENCH_STRAIGHT)
        {
            Enter_State(MISSION_PRE_MATCH, in.now_ms);
        }
        out.drive_action = DRIVE_ACTION_EMERGENCY_STOP;
        return out;
    }

    // Match 3-minute timer enforcement.
    if (is_competition_active && (uint32_t)(in.now_ms - Match_Start_Ms) >= Config.match_duration_ms)
    {
        Match_Final_Elapsed_Ms = Config.match_duration_ms;
        Gate_Closed_Cmd = false;
        Camera_Near_Cmd = false;
        Enter_State(MISSION_MATCH_FINISHED, in.now_ms);
        out.gate_closed = Gate_Closed_Cmd;
        out.camera_near = Camera_Near_Cmd;
        out.drive_action = DRIVE_ACTION_SOFT_STOP;
        return out;
    }

    // Manual button press while running cleanly aborts the run.
    if (in.button_pressed && is_competition_active)
    {
        Match_Final_Elapsed_Ms = in.now_ms - Match_Start_Ms;
        Gate_Closed_Cmd = false;
        Camera_Near_Cmd = false;
        Enter_State(MISSION_MATCH_FINISHED, in.now_ms);
        out.gate_closed = Gate_Closed_Cmd;
        out.camera_near = Camera_Near_Cmd;
        out.drive_action = DRIVE_ACTION_SOFT_STOP;
        return out;
    }

    const bool vision_fresh = Is_Vision_Fresh(in.now_ms);
    const bool rear_clear = vision_fresh && ((Latest_Vision.flags & VIS_FLAG_REAR_CLEAR) != 0U);
    const bool opponent_alert = vision_fresh &&
                                ((Latest_Vision.flags & (VIS_FLAG_OPPONENT_NEAR | VIS_FLAG_OPPONENT_ZONE_NEAR)) != 0U);

    // Visual slip/stall detection supplements motor stall detection (< 1 s escape vs 15 s DQ).
    const bool motion_state =
        State == MISSION_LAUNCH_CROSS_BUMPS || State == MISSION_SEARCH_TARGET ||
        State == MISSION_APPROACH_TARGET || State == MISSION_FIND_HOME ||
        State == MISSION_APPROACH_HOME || State == MISSION_DROP_OPEN_AND_PUSH;
    if (motion_state)
    {
        const bool vis_stalled = vision_fresh && ((Latest_Vision.flags & VIS_FLAG_MOTION_STALLED) != 0U);
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
            const bool can_reverse = rear_clear || (State == MISSION_LAUNCH_CROSS_BUMPS);
            Start_Escape(in.now_ms, can_reverse, out);
            out.gate_closed = Gate_Closed_Cmd;
            out.camera_near = Camera_Near_Cmd;
            return out;
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
        if (in.chassis_motion_done || !in.chassis_active)
        {
            Enter_State(MISSION_SEARCH_TARGET, in.now_ms);
            out.drive_action = DRIVE_ACTION_SOFT_STOP;
        }
        else
        {
            out.drive_action = DRIVE_ACTION_KEEP;
        }
        break;

    case MISSION_SEARCH_TARGET:
        Gate_Closed_Cmd = false;
        Camera_Near_Cmd = false;
        if (!vision_fresh || !in.camera_settled || Latest_Vision.camera_view != CAMERA_VIEW_FAR)
        {
            out.drive_action = DRIVE_ACTION_SOFT_STOP;
            break;
        }
        if (opponent_alert)
        {
            Start_Escape(in.now_ms, rear_clear, out);
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
        if (!vision_fresh)
        {
            // Never drive blind on stale vision frames.
            out.drive_action = DRIVE_ACTION_SOFT_STOP;
            break;
        }
        if (opponent_alert)
        {
            Start_Escape(in.now_ms, rear_clear, out);
            break;
        }
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
            // Slow linear advance when yaw error is large so target stays inside L-corral mouth.
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
        if (!vision_fresh || Latest_Vision.camera_view != CAMERA_VIEW_NEAR)
        {
            break;
        }
        if (Has_Confirm_Frame && Latest_Vision.frame_seq == Last_Confirm_Frame_Seq)
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
                // Hazard, unknown object, mixed casualty, or excess count: back out with gate OPEN.
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
            if (rear_clear)
            {
                Sub_Phase = 1U;
                out.drive_action = DRIVE_ACTION_START_DISTANCE;
                out.distance_m = Config.separate_backout_m;
                out.speed_mps = Config.drop_backout_speed_mps;
                out.motion_timeout_ms = 4000U;
            }
            else
            {
                Sub_Phase = 2U;
                out.drive_action = DRIVE_ACTION_START_TURN;
                out.delta_yaw_rad = Escape_Sign * Config.separate_turn_rad;
                out.angular_rad_s = Config.align_max_omega_rad_s;
                out.motion_timeout_ms = 4000U;
                Escape_Sign = -Escape_Sign;
            }
        }
        else if (Sub_Phase == 1U)
        {
            if (in.chassis_motion_done || !in.chassis_active)
            {
                Sub_Phase = 2U;
                out.drive_action = DRIVE_ACTION_START_TURN;
                out.delta_yaw_rad = Escape_Sign * Config.separate_turn_rad;
                out.angular_rad_s = Config.align_max_omega_rad_s;
                out.motion_timeout_ms = 4000U;
                Escape_Sign = -Escape_Sign;
            }
            else
            {
                out.drive_action = DRIVE_ACTION_KEEP;
            }
        }
        else
        {
            if (in.chassis_motion_done || !in.chassis_active)
            {
                Enter_State(MISSION_SEARCH_TARGET, in.now_ms);
                out.drive_action = DRIVE_ACTION_SOFT_STOP;
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
        if (!in.gate_settled || !in.camera_settled)
        {
            break;
        }
        if ((uint32_t)(in.now_ms - State_Enter_Ms) >= Config.inspect_timeout_ms)
        {
            Gate_Closed_Cmd = false;
            Camera_Near_Cmd = false;
            Enter_State(MISSION_BACK_OUT_SEPARATE, in.now_ms);
            break;
        }
        if (!vision_fresh || Latest_Vision.camera_view != CAMERA_VIEW_NEAR ||
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
                    Camera_Near_Cmd = false;
                    Enter_State(MISSION_FIND_HOME, in.now_ms);
                }
            }
            else
            {
                // Lost block or hazard swept in during gate closure: open gate and back out.
                Cargo_Is_Casualty = false;
                Cargo_Green = 0U;
                Cargo_Black = 0U;
                Cargo_Orange = 0U;
                Gate_Closed_Cmd = false;
                Camera_Near_Cmd = false;
                Enter_State(MISSION_BACK_OUT_SEPARATE, in.now_ms);
            }
        }
        break;

    case MISSION_FIND_HOME:
        Gate_Closed_Cmd = true;
        Camera_Near_Cmd = false;
        if (!vision_fresh || !in.camera_settled || Latest_Vision.camera_view != CAMERA_VIEW_FAR)
        {
            out.drive_action = DRIVE_ACTION_SOFT_STOP;
            break;
        }
        if (opponent_alert)
        {
            Start_Escape(in.now_ms, rear_clear, out);
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
        if (!vision_fresh)
        {
            out.drive_action = DRIVE_ACTION_SOFT_STOP;
            break;
        }
        if (opponent_alert)
        {
            Start_Escape(in.now_ms, rear_clear, out);
            break;
        }
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
            // Refuse to cross ramp line if half does not match cargo.
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
            // Pre-push near check: never push if hazard appeared or half mismatched.
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
            if (!vision_fresh || Latest_Vision.camera_view != CAMERA_VIEW_NEAR ||
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
            if (!cargo_matches)
            {
                // Abort delivery if cyan entered or cargo became invalid.
                Cargo_Is_Casualty = false;
                Cargo_Green = 0U;
                Cargo_Black = 0U;
                Cargo_Orange = 0U;
                Gate_Closed_Cmd = false;
                Camera_Near_Cmd = false;
                Enter_State(MISSION_BACK_OUT_SEPARATE, in.now_ms);
                break;
            }
            if (!half_ok)
            {
                // Re-align on own correct half before dropping (avoid -10 pt wrong-half penalty).
                Camera_Near_Cmd = false;
                Enter_State(MISSION_APPROACH_HOME, in.now_ms);
                break;
            }
            if (++Confirm_Count >= Config.confirm_frames_required)
            {
                Gate_Closed_Cmd = false;
                Sub_Phase = 1U;
                State_Enter_Ms = in.now_ms;
            }
        }
        else if (Sub_Phase == 1U)
        {
            // Wait until right inverted-L gate fully lifts out of the unloading path.
            Gate_Closed_Cmd = false;
            out.drive_action = DRIVE_ACTION_SOFT_STOP;
            if (in.gate_settled)
            {
                Sub_Phase = 2U;
                out.drive_action = DRIVE_ACTION_START_DISTANCE;
                out.distance_m = Config.drop_push_distance_m;
                out.speed_mps = Config.drop_push_speed_mps;
                out.motion_timeout_ms = 5000U;
            }
            else if ((uint32_t)(in.now_ms - State_Enter_Ms) >= 3000U)
            {
                Camera_Near_Cmd = false;
                Enter_State(MISSION_FIND_HOME, in.now_ms);
            }
        }
        else
        {
            Gate_Closed_Cmd = false;
            if (in.chassis_motion_done || !in.chassis_active)
            {
                Enter_State(MISSION_DROP_BACKOUT_VERIFY, in.now_ms);
                out.drive_action = DRIVE_ACTION_START_DISTANCE;
                out.distance_m = Config.drop_backout_distance_m;
                out.speed_mps = Config.drop_backout_speed_mps;
                out.motion_timeout_ms = 5000U;
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
            if (in.chassis_motion_done || !in.chassis_active)
            {
                Sub_Phase = 1U;
                State_Enter_Ms = in.now_ms;
                Confirm_Count = 0U;
                Has_Confirm_Frame = false;
                Last_Confirm_Frame_Seq = 0U;
                out.drive_action = DRIVE_ACTION_SOFT_STOP;
            }
            else
            {
                out.drive_action = DRIVE_ACTION_KEEP;
            }
        }
        else
        {
            out.drive_action = DRIVE_ACTION_SOFT_STOP;
            if ((uint32_t)(in.now_ms - State_Enter_Ms) >= Config.drop_verify_timeout_ms)
            {
                // Drop NOT confirmed inside safe zone -> do NOT unlock first green.
                Cargo_Is_Casualty = false;
                Cargo_Green = 0U;
                Cargo_Black = 0U;
                Cargo_Orange = 0U;
                Camera_Near_Cmd = false;
                Enter_State(MISSION_SEARCH_TARGET, in.now_ms);
                break;
            }
            if (!vision_fresh || !in.camera_settled || Latest_Vision.camera_view != CAMERA_VIEW_NEAR ||
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
                    Cargo_Is_Casualty = false;
                    Cargo_Green = 0U;
                    Cargo_Black = 0U;
                    Cargo_Orange = 0U;
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
        if (Sub_Phase == 0U)
        {
            if (in.chassis_motion_done || !in.chassis_active || in.chassis_stalled)
            {
                Sub_Phase = 1U;
                out.drive_action = DRIVE_ACTION_START_TURN;
                out.delta_yaw_rad = Escape_Sign * Config.escape_turn_rad;
                out.angular_rad_s = Config.align_max_omega_rad_s;
                out.motion_timeout_ms = 4500U;
                Escape_Sign = -Escape_Sign;
            }
            else
            {
                out.drive_action = DRIVE_ACTION_KEEP;
            }
        }
        else
        {
            if (in.chassis_motion_done || !in.chassis_active)
            {
                if (Gate_Closed_Cmd)
                {
                    // Re-verify corral contents after any escape while gate is closed.
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

    out.gate_closed = Gate_Closed_Cmd;
    out.camera_near = Camera_Near_Cmd;
    return out;
}
