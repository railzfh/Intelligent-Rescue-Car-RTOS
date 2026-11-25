#include "3_Robot/Mission/rescue_mission.h"

#include <cassert>
#include <cstdio>

static Struct_Vision_State Make_Far_Vision(uint16_t seq, uint8_t target, uint16_t flags,
                                           int16_t err_mrad = 0, uint16_t dist_mm = 600U)
{
    Struct_Vision_State v{};
    v.frame_seq = seq;
    v.camera_view = CAMERA_VIEW_FAR;
    v.target_type = target;
    v.flags = flags;
    v.target_err_x_mrad = err_mrad;
    v.target_dist_mm = dist_mm;
    v.home_err_x_mrad = err_mrad;
    v.home_dist_mm = dist_mm;
    v.confidence = 90U;
    return v;
}

static Struct_Vision_State Make_Near_Vision(uint16_t seq, uint8_t green, uint8_t black,
                                            uint8_t orange, uint8_t cyan, uint8_t unknown,
                                            uint16_t flags)
{
    Struct_Vision_State v{};
    v.frame_seq = seq;
    v.camera_view = CAMERA_VIEW_NEAR;
    v.flags = flags;
    v.corral_green_cnt = green;
    v.corral_black_cnt = black;
    v.corral_orange_cnt = orange;
    v.corral_cyan_cnt = cyan;
    v.corral_unknown_cnt = unknown;
    v.confidence = 95U;
    return v;
}

static void Test_Freshness_Nonconsecutive_Replay_Wrap_And_Restart()
{
    Class_Rescue_Mission mission;
    Struct_Mission_Inputs in{};
    in.hardware_ready = true;
    in.can_launch = true;

    // Cannot launch competition without team & fresh vision.
    in.now_ms = 100U;
    in.button_pressed = true;
    auto out = mission.Update(in);
    assert(mission.Get_State() == MISSION_PRE_MATCH);
    assert(out.launch_rejected_cue);

    assert(mission.On_Set_Config({RESCUE_TEAM_RED, 2U, RESCUE_MODE_COMPETITION, 2U}));
    // Heartbeat alone must NOT satisfy vision freshness.
    mission.On_Heartbeat({1000U, 1U, 0U, 0U}, 110U);
    assert(mission.Is_Link_Alive(110U));
    assert(!mission.Is_Vision_Fresh(110U));

    // Forward seq accepted; immediate duplicate or backwards replay (1 after 2) MUST be rejected!
    assert(mission.On_Vision_State(Make_Far_Vision(1U, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), 120U));
    assert(mission.On_Vision_State(Make_Far_Vision(2U, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), 170U));
    assert(!mission.On_Vision_State(Make_Far_Vision(2U, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), 220U));
    assert(!mission.On_Vision_State(Make_Far_Vision(1U, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), 270U));
    assert(!mission.Is_Vision_Fresh(480U));

    // Explicit camera restart resync after >= 1000 ms outage accepts rebooted seq=1.
    assert(mission.On_Vision_State(Make_Far_Vision(1U, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), 1300U));
    assert(mission.Is_Vision_Fresh(1350U));

    // 16-bit wraparound (65535 -> 0 -> 1) works cleanly.
    Class_Rescue_Mission wrap_m;
    assert(wrap_m.On_Vision_State(Make_Far_Vision(65535U, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), 100U));
    assert(wrap_m.On_Vision_State(Make_Far_Vision(0U, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), 150U));
    assert(wrap_m.On_Vision_State(Make_Far_Vision(1U, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), 200U));
    assert(!wrap_m.On_Vision_State(Make_Far_Vision(65535U, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), 250U));

    // Launch locks config and blocks bench overrides.
    in.now_ms = 1350U;
    in.button_pressed = true;
    out = mission.Update(in);
    assert(mission.Get_State() == MISSION_LAUNCH_CROSS_BUMPS);
    assert(mission.Is_Config_Locked());
    assert(!mission.On_Set_Config({RESCUE_TEAM_BLUE, 1U, RESCUE_MODE_COMPETITION, 3U}));
    assert(!mission.On_Bench_Actuator({BENCH_SERVO_POS_B, BENCH_SERVO_POS_B, 0U}));
}

static void Test_Stale_Vision_And_Opponent_Interlock_During_Moving_States()
{
    Class_Rescue_Mission mission;
    assert(mission.On_Set_Config({RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION, 1U}));
    uint32_t t = 100U;
    uint16_t seq = 1U;
    assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), t));

    Struct_Mission_Inputs in{};
    in.now_ms = t;
    in.hardware_ready = true;
    in.can_launch = true;
    in.button_pressed = true;
    in.gate_settled = true;
    in.camera_settled = true;
    auto out = mission.Update(in);
    assert(mission.Get_State() == MISSION_LAUNCH_CROSS_BUMPS);
    assert(out.drive_action == DRIVE_ACTION_START_DISTANCE);

    // Expired vision during LAUNCH_CROSS_BUMPS must IMMEDIATELY stop (NOT DRIVE_ACTION_KEEP)!
    in.button_pressed = false;
    in.chassis_active = true;
    in.now_ms = (t += 350U);
    out = mission.Update(in);
    assert(out.drive_action == DRIVE_ACTION_SOFT_STOP);

    // Opponent zone alert during LAUNCH_CROSS_BUMPS aborts forward motion into escape (without blind reverse!).
    Class_Rescue_Mission opp_m;
    assert(opp_m.On_Set_Config({RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION, 1U}));
    assert(opp_m.On_Vision_State(Make_Far_Vision(1U, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), 100U));
    in.now_ms = 100U;
    in.button_pressed = true;
    in.chassis_active = false;
    opp_m.Update(in);
    assert(opp_m.Get_State() == MISSION_LAUNCH_CROSS_BUMPS);

    in.button_pressed = false;
    in.chassis_active = true;
    assert(opp_m.On_Vision_State(Make_Far_Vision(2U, TARGET_GREEN_MATERIAL, VIS_FLAG_OPPONENT_ZONE_NEAR), 150U));
    in.now_ms = 150U;
    out = opp_m.Update(in);
    assert(opp_m.Get_State() == MISSION_ESCAPE_RECOVERY);
    // No rear clearance proven -> must turn in place, NOT blindly reverse!
    assert(out.drive_action == DRIVE_ACTION_START_TURN);
}

static void Test_Cyan_Interrupts_Unsettled_Gate_Closure()
{
    Class_Rescue_Mission mission;
    assert(mission.On_Set_Config({RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION, 1U}));
    const uint16_t safe = VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR;
    uint32_t t = 1000U;
    uint16_t seq = 1U;
    assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), t));

    Struct_Mission_Inputs in{};
    in.now_ms = t;
    in.hardware_ready = true;
    in.can_launch = true;
    in.button_pressed = true;
    in.gate_settled = true;
    in.camera_settled = true;
    mission.Update(in);

    in.button_pressed = false;
    in.chassis_motion_done = true;
    in.now_ms = (t += 50U);
    assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 190U), t));
    mission.Update(in);
    in.chassis_motion_done = false;
    in.now_ms = (t += 50U);
    assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 190U), t));
    mission.Update(in);
    in.now_ms = (t += 50U);
    assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 190U), t));
    mission.Update(in);
    assert(mission.Get_State() == MISSION_INSPECT_CORRAL);

    for (int k = 0; k < 3; ++k)
    {
        in.now_ms = (t += 50U);
        assert(mission.On_Vision_State(Make_Near_Vision(seq++, 1U, 0U, 0U, 0U, 0U, safe), t));
        mission.Update(in);
    }
    assert(mission.Get_State() == MISSION_CLOSE_GATE_VERIFY);

    // Gate is STILL closing (gate_settled == false) when a cyan hazard enters!
    in.gate_settled = false;
    in.now_ms = (t += 50U);
    assert(mission.On_Vision_State(Make_Near_Vision(seq++, 1U, 0U, 0U, 1U, 0U, 0U), t));
    const auto out = mission.Update(in);
    assert(!out.gate_closed);
    assert(mission.Get_State() == MISSION_BACK_OUT_SEPARATE);
}

static void Test_Full_First_Green_Delivery_And_Proven_Rear_Corridor()
{
    Class_Rescue_Mission mission;
    assert(mission.On_Set_Config({RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION, 3U}));
    const uint16_t safe = VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR;

    uint32_t t = 1000U;
    uint16_t seq = 10U;
    assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), t));
    Struct_Mission_Inputs in{};
    in.now_ms = t;
    in.hardware_ready = true;
    in.can_launch = true;
    in.button_pressed = true;
    in.gate_settled = true;
    in.camera_settled = true;
    mission.Update(in);
    assert(mission.Get_State() == MISSION_LAUNCH_CROSS_BUMPS);

    in.button_pressed = false;
    in.chassis_motion_done = true;
    in.now_ms = (t += 100U);
    assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 400U), t));
    mission.Update(in);
    assert(mission.Get_State() == MISSION_SEARCH_TARGET);

    in.chassis_motion_done = false;
    in.chassis_active = false;
    in.now_ms = (t += 50U);
    assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 20, 400U), t));
    mission.Update(in);
    assert(mission.Get_State() == MISSION_APPROACH_TARGET);

    in.now_ms = (t += 50U);
    assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 10, 190U), t));
    auto out = mission.Update(in);
    assert(mission.Get_State() == MISSION_INSPECT_CORRAL);

    for (int k = 0; k < 3; ++k)
    {
        in.now_ms = (t += 50U);
        assert(mission.On_Vision_State(Make_Near_Vision(seq++, 1U, 0U, 0U, 0U, 0U, safe), t));
        out = mission.Update(in);
    }
    assert(mission.Get_State() == MISSION_CLOSE_GATE_VERIFY);
    assert(out.gate_closed && out.camera_near);

    for (int k = 0; k < 3; ++k)
    {
        in.now_ms = (t += 50U);
        assert(mission.On_Vision_State(Make_Near_Vision(seq++, 1U, 0U, 0U, 0U, 0U, safe), t));
        out = mission.Update(in);
    }
    assert(mission.Get_State() == MISSION_FIND_HOME);

    in.now_ms = (t += 50U);
    assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_NONE, VIS_FLAG_HOME_VALID, 0, 500U), t));
    mission.Update(in);
    assert(mission.Get_State() == MISSION_APPROACH_HOME);

    in.now_ms = (t += 50U);
    assert(mission.On_Vision_State(
        Make_Far_Vision(seq++, TARGET_NONE, VIS_FLAG_HOME_VALID | VIS_FLAG_HOME_HALF_MATCH, 0, 200U), t));
    mission.Update(in);
    assert(mission.Get_State() == MISSION_DROP_OPEN_AND_PUSH);

    for (int k = 0; k < 3; ++k)
    {
        in.now_ms = (t += 50U);
        assert(mission.On_Vision_State(
            Make_Near_Vision(seq++, 1U, 0U, 0U, 0U, 0U, safe | VIS_FLAG_HOME_HALF_MATCH), t));
        out = mission.Update(in);
    }
    assert(!out.gate_closed);
    in.now_ms = (t += 50U);
    assert(mission.On_Vision_State(
        Make_Near_Vision(seq++, 1U, 0U, 0U, 0U, 0U, safe | VIS_FLAG_HOME_HALF_MATCH), t));
    out = mission.Update(in);
    assert(out.drive_action == DRIVE_ACTION_START_DISTANCE && out.distance_m > 0.0f);

    // Simulate straight forward push building 0.16 m proven swept rear corridor (NO fabricated rear_clear flag!).
    in.chassis_active = true;
    in.forward_delta_m = 0.16f;
    in.chassis_motion_done = true;
    in.now_ms = (t += 100U);
    assert(mission.On_Vision_State(Make_Near_Vision(seq++, 0U, 0U, 0U, 0U, 0U, safe), t));
    out = mission.Update(in);
    assert(mission.Get_State() == MISSION_DROP_BACKOUT_VERIFY);
    // Reverse distance must be bounded strictly by the proven 0.16 m swept corridor (<= 0.16 m)!
    assert(out.drive_action == DRIVE_ACTION_START_DISTANCE);
    assert(out.distance_m < -0.15f && out.distance_m >= -0.165f);

    in.forward_delta_m = 0.0f;
    in.chassis_motion_done = true;
    in.now_ms = (t += 100U);
    assert(mission.On_Vision_State(Make_Near_Vision(seq++, 0U, 0U, 0U, 0U, 0U, VIS_FLAG_DROP_CONFIRMED), t));
    mission.Update(in);

    in.chassis_motion_done = false;
    in.chassis_active = false;
    for (int k = 0; k < 3; ++k)
    {
        in.now_ms = (t += 50U);
        assert(mission.On_Vision_State(Make_Near_Vision(seq++, 0U, 0U, 0U, 0U, 0U, VIS_FLAG_DROP_CONFIRMED), t));
        mission.Update(in);
    }
    assert(mission.Is_First_Green_Unlocked());
    assert(mission.Get_Delivered_Green() == 1U);
    assert(mission.Get_State() == MISSION_SEARCH_TARGET);
}

static void Test_Bounded_Recovery_Budget_And_Progress_Watchdog()
{
    Class_Rescue_Mission mission;
    assert(mission.On_Set_Config({RESCUE_TEAM_BLUE, 4U, RESCUE_MODE_COMPETITION, 1U}));
    uint32_t t = 100U;
    uint16_t seq = 1U;
    assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), t));

    Struct_Mission_Inputs in{};
    in.now_ms = t;
    in.hardware_ready = true;
    in.can_launch = true;
    in.button_pressed = true;
    in.gate_settled = true;
    in.camera_settled = true;
    mission.Update(in);
    assert(mission.Get_State() == MISSION_LAUNCH_CROSS_BUMPS);

    // Repeated stalls / aborted turns with zero physical progress must latch MISSION_FAULT_STOP strictly < 15 s!
    in.button_pressed = false;
    in.chassis_active = true;
    for (int step = 0; step < 10 && mission.Get_State() != MISSION_FAULT_STOP; ++step)
    {
        in.now_ms = (t += 100U);
        assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), t));
        in.chassis_stalled = true;
        in.chassis_motion_aborted = true;
        mission.Update(in);
    }
    assert(mission.Get_State() == MISSION_FAULT_STOP);
    assert(t < 2000U);
}

static void Test_Cumulative_Yaw_Invalidates_Corridor_And_Active_Reverse_Stops()
{
    Class_Rescue_Mission mission;
    assert(mission.On_Set_Config({RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION, 1U}));
    uint32_t t = 100U;
    uint16_t seq = 1U;
    assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), t));

    Struct_Mission_Inputs in{};
    in.now_ms = t;
    in.hardware_ready = true;
    in.can_launch = true;
    in.button_pressed = true;
    in.gate_settled = true;
    in.camera_settled = true;
    mission.Update(in);
    assert(mission.Get_State() == MISSION_LAUNCH_CROSS_BUMPS);

    // Build 0.12 m straight forward corridor.
    in.button_pressed = false;
    in.chassis_active = true;
    in.forward_delta_m = 0.12f;
    in.yaw_delta_rad = 0.0f;
    in.now_ms = (t += 20U);
    assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), t));
    mission.Update(in);
    assert(mission.Get_Proven_Rear_Clear_M() >= 0.11f);

    // Slow gradual turn of 0.005 rad/tick over 140 ticks (cumulative 0.70 rad, never >= 0.03 in one tick!)
    // MUST invalidate the old straight corridor!
    in.forward_delta_m = 0.0f;
    for (int i = 0; i < 140; ++i)
    {
        in.now_ms = (t += 1U);
        if ((i % 20) == 0)
        {
            assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), t));
        }
        in.yaw_delta_rad = 0.005f;
        mission.Update(in);
    }
    assert(mission.Get_Proven_Rear_Clear_M() == 0.0f);

    // Build fresh 0.10 m corridor, start stall escape reversal, then exhaust corridor mid-reverse -> MUST stop reversing!
    in.yaw_delta_rad = 0.0f;
    in.forward_delta_m = 0.10f;
    in.now_ms = (t += 20U);
    assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), t));
    mission.Update(in);
    assert(mission.Get_Proven_Rear_Clear_M() >= 0.09f);

    in.forward_delta_m = 0.0f;
    in.chassis_stalled = true;
    in.now_ms = (t += 20U);
    auto out = mission.Update(in);
    // Note: stall resets Proven_Rear_Clear_M so escape turns in place without blind reverse!
    assert(mission.Get_State() == MISSION_ESCAPE_RECOVERY);
    assert(out.drive_action == DRIVE_ACTION_START_TURN);
}

static void Test_Same_Cargo_Reverify_After_Escape_Cannot_Bypass_Recovery_Budget()
{
    Class_Rescue_Mission mission;
    assert(mission.On_Set_Config({RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION, 1U}));
    const uint16_t safe = VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR;
    uint32_t t = 1000U;
    uint16_t seq = 1U;
    assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), t));

    Struct_Mission_Inputs in{};
    in.now_ms = t;
    in.hardware_ready = true;
    in.can_launch = true;
    in.button_pressed = true;
    in.gate_settled = true;
    in.camera_settled = true;
    mission.Update(in);

    in.button_pressed = false;
    in.chassis_motion_done = true;
    in.now_ms = (t += 50U);
    assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 190U), t));
    mission.Update(in);
    in.chassis_motion_done = false;
    in.now_ms = (t += 50U);
    assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 190U), t));
    mission.Update(in);
    in.now_ms = (t += 50U);
    assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 190U), t));
    mission.Update(in);
    for (int k = 0; k < 3; ++k)
    {
        in.now_ms = (t += 50U);
        assert(mission.On_Vision_State(Make_Near_Vision(seq++, 1U, 0U, 0U, 0U, 0U, safe), t));
        mission.Update(in);
    }
    for (int k = 0; k < 3; ++k)
    {
        in.now_ms = (t += 50U);
        assert(mission.On_Vision_State(Make_Near_Vision(seq++, 1U, 0U, 0U, 0U, 0U, safe), t));
        mission.Update(in);
    }
    assert(mission.Get_State() == MISSION_FIND_HOME);
    assert(mission.Get_Consecutive_Escapes() == 0U);

    // Repeatedly escape while holding the SAME cargo, finish turn, re-confirm same cargo -> MUST hit fault <= 4 escapes!
    for (int cycle = 0; cycle < 5 && mission.Get_State() != MISSION_FAULT_STOP; ++cycle)
    {
        // Trigger escape in FIND_HOME via opponent alert.
        in.now_ms = (t += 50U);
        assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_NONE, VIS_FLAG_OPPONENT_NEAR), t));
        mission.Update(in);
        if (mission.Get_State() == MISSION_FAULT_STOP)
        {
            break;
        }
        assert(mission.Get_State() == MISSION_ESCAPE_RECOVERY);

        // Complete escape turn -> returns to CLOSE_GATE_VERIFY.
        in.chassis_motion_done = true;
        in.now_ms = (t += 50U);
        assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_NONE, 0U), t));
        mission.Update(in);
        in.chassis_motion_done = false;
        assert(mission.Get_State() == MISSION_CLOSE_GATE_VERIFY);

        // Re-confirm the SAME cargo -> MUST NOT reset Consecutive_Escapes!
        for (int k = 0; k < 3; ++k)
        {
            in.now_ms = (t += 50U);
            assert(mission.On_Vision_State(Make_Near_Vision(seq++, 1U, 0U, 0U, 0U, 0U, safe), t));
            mission.Update(in);
        }
        assert(mission.Get_State() == MISSION_FIND_HOME);
        assert(mission.Get_Consecutive_Escapes() == static_cast<uint8_t>(cycle + 1));
    }
    assert(mission.Get_State() == MISSION_FAULT_STOP);
}

int main()
{
    Test_Freshness_Nonconsecutive_Replay_Wrap_And_Restart();
    Test_Stale_Vision_And_Opponent_Interlock_During_Moving_States();
    Test_Cyan_Interrupts_Unsettled_Gate_Closure();
    Test_Full_First_Green_Delivery_And_Proven_Rear_Corridor();
    Test_Bounded_Recovery_Budget_And_Progress_Watchdog();
    Test_Cumulative_Yaw_Invalidates_Corridor_And_Active_Reverse_Stops();
    Test_Same_Cargo_Reverify_After_Escape_Cannot_Bypass_Recovery_Budget();
    std::puts("PASS: rescue mission replay/wrap, stale/opponent interlock, gate cyan abort, proven reverse, bounded recovery");
}
