#include "3_Robot/Mission/rescue_mission.h"

#include <cassert>
#include <cstdio>

static Struct_Vision_State Make_Far_Vision(uint16_t seq, uint8_t target, uint16_t flags,
                                           int16_t err_mrad = 0, uint16_t dist_mm = 600U,
                                           uint8_t pre_match_gen = 1U)
{
    Struct_Vision_State v{};
    v.frame_seq = seq;
    v.camera_view = CAMERA_VIEW_FAR;
    v.target_type = target;
    v.flags = flags;
    if ((v.flags & VIS_FLAG_PRE_MATCH_GEN_MASK) == 0U && pre_match_gen != 0U)
    {
        v.flags |= static_cast<uint16_t>((pre_match_gen & 0x3U) << VIS_FLAG_PRE_MATCH_GEN_SHIFT);
    }
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
    in.wheels_still = true;
    in.gate_settled = true;
    in.camera_settled = true;

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
    in.wheels_still = true;
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
    in.wheels_still = false;
    assert(opp_m.On_Vision_State(Make_Far_Vision(2U, TARGET_GREEN_MATERIAL, VIS_FLAG_OPPONENT_ZONE_NEAR), 150U));
    in.now_ms = 150U;
    out = opp_m.Update(in);
    assert(opp_m.Get_State() == MISSION_ESCAPE_RECOVERY);
    // While still moving, must cancel immediately with EMERGENCY_STOP and confirm standstill before turning!
    assert(out.drive_action == DRIVE_ACTION_EMERGENCY_STOP);

    // Once stopped (chassis inactive and wheels still), escape starts bounded in-place turn (no blind reverse).
    in.chassis_active = false;
    in.wheels_still = true;
    in.now_ms = 160U;
    assert(opp_m.On_Vision_State(Make_Far_Vision(3U, TARGET_GREEN_MATERIAL, 0U), 160U));
    out = opp_m.Update(in);
    assert(opp_m.Get_State() == MISSION_ESCAPE_RECOVERY);
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
    in.wheels_still = true;
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

    // Also verify vision/UART loss while gate is closing (!gate_settled) IMMEDIATELY aborts blind closure
    // (commands gate open and returns to INSPECT_CORRAL to require fresh confirmation after recovery)!
    Class_Rescue_Mission blind_m;
    assert(blind_m.On_Set_Config({RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION, 1U}));
    uint32_t bt = 1000U;
    uint16_t bseq = 1U;
    assert(blind_m.On_Vision_State(Make_Far_Vision(bseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), bt));
    Struct_Mission_Inputs bin{};
    bin.now_ms = bt;
    bin.hardware_ready = true;
    bin.can_launch = true;
    bin.button_pressed = true;
    bin.wheels_still = true;
    bin.gate_settled = true;
    bin.camera_settled = true;
    blind_m.Update(bin);
    bin.button_pressed = false;
    bin.chassis_motion_done = true;
    bin.now_ms = (bt += 50U);
    assert(blind_m.On_Vision_State(Make_Far_Vision(bseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 190U), bt));
    blind_m.Update(bin);
    bin.chassis_motion_done = false;
    for (int i = 0; i < 2; ++i)
    {
        bin.now_ms = (bt += 50U);
        assert(blind_m.On_Vision_State(Make_Far_Vision(bseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 190U), bt));
        blind_m.Update(bin);
    }
    for (int k = 0; k < 3; ++k)
    {
        bin.now_ms = (bt += 50U);
        assert(blind_m.On_Vision_State(Make_Near_Vision(bseq++, 1U, 0U, 0U, 0U, 0U, safe), bt));
        blind_m.Update(bin);
    }
    assert(blind_m.Get_State() == MISSION_CLOSE_GATE_VERIFY);
    assert(blind_m.Is_Gate_Closed_Cmd());

    // Gate is mid-close (!gate_settled) when UART RX faults -> MUST immediately command gate open!
    bin.gate_settled = false;
    bin.uart_rx_healthy = false;
    bin.now_ms = (bt += 10U);
    auto bout = blind_m.Update(bin);
    assert(!bout.gate_closed);
    assert(!blind_m.Is_Gate_Closed_Cmd());
    assert(blind_m.Get_State() == MISSION_INSPECT_CORRAL);

    // Recover UART + 1 fresh frame -> MUST stay open until 3 fresh frames confirm!
    bin.uart_rx_healthy = true;
    bin.gate_settled = true;
    bin.now_ms = (bt += 40U);
    assert(blind_m.On_Vision_State(Make_Near_Vision(bseq++, 1U, 0U, 0U, 0U, 0U, safe), bt));
    bout = blind_m.Update(bin);
    assert(!bout.gate_closed);
    assert(blind_m.Get_State() == MISSION_INSPECT_CORRAL);

    // Finding 1 (R10c): Healthy UART + fresh advancing FAR / TRANSITION frames while gate is closing
    // (!gate_settled) MUST ALSO immediately abort blind closure (NEVER keep commanding closed for 600 ms!)
    // and reset any confirmation streak when near view is lost even after settled!
    for (int k = 0; k < 2; ++k)
    {
        bin.now_ms = (bt += 40U);
        assert(blind_m.On_Vision_State(Make_Near_Vision(bseq++, 1U, 0U, 0U, 0U, 0U, safe), bt));
        blind_m.Update(bin);
    }
    assert(blind_m.Get_State() == MISSION_CLOSE_GATE_VERIFY);
    assert(blind_m.Is_Gate_Closed_Cmd());

    // Fresh FAR frame arrives while gate is still closing (!gate_settled) -> immediate abort!
    bin.gate_settled = false;
    bin.now_ms = (bt += 20U);
    assert(blind_m.On_Vision_State(Make_Far_Vision(bseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), bt));
    bout = blind_m.Update(bin);
    assert(!bout.gate_closed);
    assert(!blind_m.Is_Gate_Closed_Cmd());
    assert(blind_m.Get_State() == MISSION_INSPECT_CORRAL);

    // Re-enter CLOSE_GATE_VERIFY, settle gate, build 2 NEAR confirmation frames, then lose NEAR view
    // (TRANSITION frame) -> gate stays safely closed (already settled), confirmation streak resets so
    // a single subsequent NEAR frame CANNOT immediately advance to FIND_HOME!
    bin.gate_settled = true;
    for (int k = 0; k < 3; ++k)
    {
        bin.now_ms = (bt += 40U);
        assert(blind_m.On_Vision_State(Make_Near_Vision(bseq++, 1U, 0U, 0U, 0U, 0U, safe), bt));
        blind_m.Update(bin);
    }
    assert(blind_m.Get_State() == MISSION_CLOSE_GATE_VERIFY);
    for (int k = 0; k < 2; ++k)
    {
        bin.now_ms = (bt += 40U);
        assert(blind_m.On_Vision_State(Make_Near_Vision(bseq++, 1U, 0U, 0U, 0U, 0U, safe), bt));
        blind_m.Update(bin);
    }
    assert(blind_m.Get_State() == MISSION_CLOSE_GATE_VERIFY);
    // Inject TRANSITION frame -> resets confirmation streak while keeping settled gate stationary!
    Struct_Vision_State trans{};
    trans.frame_seq = bseq++;
    trans.camera_view = CAMERA_VIEW_TRANSITION;
    bin.now_ms = (bt += 40U);
    assert(blind_m.On_Vision_State(trans, bt));
    bout = blind_m.Update(bin);
    assert(bout.gate_closed);
    assert(blind_m.Get_State() == MISSION_CLOSE_GATE_VERIFY);
    // Single NEAR frame after TRANSITION MUST NOT advance to FIND_HOME (needs full 3-frame streak again)!
    bin.now_ms = (bt += 40U);
    assert(blind_m.On_Vision_State(Make_Near_Vision(bseq++, 1U, 0U, 0U, 0U, 0U, safe), bt));
    blind_m.Update(bin);
    assert(blind_m.Get_State() == MISSION_CLOSE_GATE_VERIFY);

    // Finding 1 (R11): Same-drain FAR -> NEAR before Update() while gate is still closing (!gate_settled)
    // MUST NOT hide the wrong-view frame! Must abort closure immediately back to INSPECT_CORRAL!
    bin.gate_settled = false;
    bin.now_ms = (bt += 20U);
    assert(blind_m.On_Vision_State(Make_Far_Vision(bseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), bt));
    assert(blind_m.On_Vision_State(Make_Near_Vision(bseq++, 1U, 0U, 0U, 0U, 0U, safe), bt));
    bout = blind_m.Update(bin);
    assert(!bout.gate_closed);
    assert(!blind_m.Is_Gate_Closed_Cmd());
    assert(blind_m.Get_State() == MISSION_INSPECT_CORRAL);

    // Re-enter CLOSE_GATE_VERIFY; same-drain NEAR(cyan) -> NEAR(clean) while !gate_settled MUST abort to BACK_OUT_SEPARATE!
    bin.gate_settled = true;
    for (int k = 0; k < 3; ++k)
    {
        bin.now_ms = (bt += 40U);
        assert(blind_m.On_Vision_State(Make_Near_Vision(bseq++, 1U, 0U, 0U, 0U, 0U, safe), bt));
        blind_m.Update(bin);
    }
    assert(blind_m.Get_State() == MISSION_CLOSE_GATE_VERIFY);
    bin.gate_settled = false;
    bin.now_ms = (bt += 20U);
    assert(blind_m.On_Vision_State(Make_Near_Vision(bseq++, 0U, 0U, 0U, 1U, 0U, 0U), bt));
    assert(blind_m.On_Vision_State(Make_Near_Vision(bseq++, 1U, 0U, 0U, 0U, 0U, safe), bt));
    bout = blind_m.Update(bin);
    assert(!bout.gate_closed);
    assert(blind_m.Get_State() == MISSION_BACK_OUT_SEPARATE);
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
    in.wheels_still = true;
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

    const uint16_t safe_push = VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR | VIS_FLAG_HOME_HALF_MATCH |
                               VIS_FLAG_DROP_PATH_CLEAR | VIS_FLAG_DROP_BASELINE_READY;
    for (int k = 0; k < 3; ++k)
    {
        in.now_ms = (t += 50U);
        assert(mission.On_Vision_State(Make_Near_Vision(seq++, 1U, 0U, 0U, 0U, 0U, safe_push), t));
        out = mission.Update(in);
    }
    assert(!out.gate_closed);
    // Settled edge tick: latches Gate_Settled_Frame_Seq without starting push on pre-settle frame!
    in.now_ms = (t += 50U);
    out = mission.Update(in);
    assert(out.drive_action == DRIVE_ACTION_SOFT_STOP);
    // Strictly later NEAR frame acknowledging open-and-settled telemetry authorizes push!
    in.now_ms = (t += 50U);
    assert(mission.On_Vision_State(
        Make_Near_Vision(seq++, 1U, 0U, 0U, 0U, 0U, safe_push | VIS_FLAG_GATE_OPEN_SETTLED_ACK), t));
    out = mission.Update(in);
    assert(out.drive_action == DRIVE_ACTION_START_DISTANCE && out.distance_m > 0.0f);

    // Simulate straight forward push building 0.16 m proven swept rear corridor (NO fabricated rear_clear flag!).
    in.chassis_active = true;
    in.wheels_still = false;
    in.forward_delta_m = 0.16f;
    in.chassis_motion_done = true;
    in.now_ms = (t += 100U);
    assert(mission.On_Vision_State(Make_Near_Vision(seq++, 0U, 0U, 0U, 0U, 0U, safe_push), t));
    out = mission.Update(in);
    assert(mission.Get_State() == MISSION_DROP_BACKOUT_VERIFY);
    // Must confirm standstill from forward push before starting margin-bounded reverse!
    assert(out.drive_action == DRIVE_ACTION_SOFT_STOP);

    in.forward_delta_m = 0.0f;
    in.chassis_motion_done = false;
    in.chassis_active = false;
    in.wheels_still = true;
    in.now_ms = (t += 20U);
    assert(mission.On_Vision_State(Make_Near_Vision(seq++, 0U, 0U, 0U, 0U, 0U, safe), t));
    out = mission.Update(in);
    // Reverse distance must reserve reverse_stop_margin_m (0.04 m) out of 0.16 m -> <= 0.12 m!
    assert(out.drive_action == DRIVE_ACTION_START_DISTANCE);
    assert(out.distance_m <= -0.115f && out.distance_m >= -0.125f);

    in.chassis_active = true;
    in.wheels_still = false;
    in.forward_delta_m = -0.12f;
    in.chassis_motion_done = true;
    in.now_ms = (t += 100U);
    assert(mission.On_Vision_State(Make_Near_Vision(seq++, 0U, 0U, 0U, 0U, 0U, VIS_FLAG_DROP_CONFIRMED), t));
    mission.Update(in);

    in.forward_delta_m = 0.0f;
    in.chassis_motion_done = false;
    in.chassis_active = false;
    in.wheels_still = true;
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
    in.wheels_still = true;
    in.gate_settled = true;
    in.camera_settled = true;
    mission.Update(in);
    assert(mission.Get_State() == MISSION_LAUNCH_CROSS_BUMPS);

    // Repeated stalls / aborted turns with zero physical progress must latch MISSION_FAULT_STOP strictly < 15 s!
    in.button_pressed = false;
    in.chassis_active = true;
    in.wheels_still = false;
    for (int step = 0; step < 20 && mission.Get_State() != MISSION_FAULT_STOP; ++step)
    {
        in.now_ms = (t += 100U);
        assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), t));
        const auto out = mission.Update(in);
        if (out.drive_action == DRIVE_ACTION_EMERGENCY_STOP)
        {
            // Mirror real Chassis_Drive::Emergency_Stop(): becomes idle & wheels stop before next turn.
            in.chassis_active = false;
            in.wheels_still = true;
            in.chassis_stalled = false;
            in.chassis_motion_aborted = false;
        }
        else if (out.drive_action == DRIVE_ACTION_START_TURN)
        {
            // Next turn immediately stalls/aborts.
            in.chassis_active = true;
            in.wheels_still = false;
            in.chassis_stalled = true;
            in.chassis_motion_aborted = true;
        }
        else
        {
            in.chassis_stalled = true;
            in.chassis_motion_aborted = true;
        }
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
    in.wheels_still = true;
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
    in.wheels_still = false;
    in.now_ms = (t += 20U);
    auto out = mission.Update(in);
    // Stall resets Proven_Rear_Clear_M and immediately emergency-stops before confirming standstill to turn!
    assert(mission.Get_State() == MISSION_ESCAPE_RECOVERY);
    assert(out.drive_action == DRIVE_ACTION_EMERGENCY_STOP);

    in.chassis_stalled = false;
    in.chassis_active = false;
    in.wheels_still = true;
    in.now_ms = (t += 20U);
    assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), t));
    out = mission.Update(in);
    assert(out.drive_action == DRIVE_ACTION_START_TURN);
}

static void Test_Drop_Rechecks_Hazards_And_Alignment_While_Opening_And_Pushing()
{
    const uint16_t safe_aligned =
        VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR | VIS_FLAG_HOME_HALF_MATCH |
        VIS_FLAG_DROP_PATH_CLEAR | VIS_FLAG_DROP_BASELINE_READY;

    auto advance_to_gate_opening = [&](Class_Rescue_Mission &m, Struct_Mission_Inputs &in, uint32_t &t, uint16_t &seq) {
        assert(m.On_Set_Config({RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION, 2U}));
        assert(m.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), t));
        in = {};
        in.now_ms = t;
        in.hardware_ready = true;
        in.can_launch = true;
        in.button_pressed = true;
        in.wheels_still = true;
        in.gate_settled = true;
        in.camera_settled = true;
        m.Update(in);

        in.button_pressed = false;
        in.chassis_motion_done = true;
        in.now_ms = (t += 50U);
        assert(m.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 190U), t));
        m.Update(in);
        in.chassis_motion_done = false;
        for (int i = 0; i < 2; ++i)
        {
            in.now_ms = (t += 50U);
            assert(m.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 190U), t));
            m.Update(in);
        }
        for (int k = 0; k < 3; ++k)
        {
            in.now_ms = (t += 50U);
            assert(m.On_Vision_State(Make_Near_Vision(seq++, 1U, 0U, 0U, 0U, 0U, VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR), t));
            m.Update(in);
        }
        for (int k = 0; k < 3; ++k)
        {
            in.now_ms = (t += 50U);
            assert(m.On_Vision_State(Make_Near_Vision(seq++, 1U, 0U, 0U, 0U, 0U, VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR), t));
            m.Update(in);
        }
        in.now_ms = (t += 50U);
        assert(m.On_Vision_State(Make_Far_Vision(seq++, TARGET_NONE, VIS_FLAG_HOME_VALID, 0, 500U), t));
        m.Update(in);
        in.now_ms = (t += 50U);
        assert(m.On_Vision_State(Make_Far_Vision(seq++, TARGET_NONE, VIS_FLAG_HOME_VALID | VIS_FLAG_HOME_HALF_MATCH, 0, 200U), t));
        m.Update(in);
        assert(m.Get_State() == MISSION_DROP_OPEN_AND_PUSH);
        for (int k = 0; k < 3; ++k)
        {
            in.now_ms = (t += 50U);
            assert(m.On_Vision_State(Make_Near_Vision(seq++, 1U, 0U, 0U, 0U, 0U, safe_aligned), t));
            auto out = m.Update(in);
            if (k == 2)
            {
                assert(!out.gate_closed);
            }
        }
    };

    // Case 1A: Safe frame captured while gate is moving MUST NOT authorize push 120 ms later when gate settles!
    {
        Class_Rescue_Mission m;
        Struct_Mission_Inputs in{};
        uint32_t t = 1000U;
        uint16_t seq = 1U;
        advance_to_gate_opening(m, in, t, seq);

        in.gate_settled = false;
        in.now_ms = (t += 50U);
        assert(m.On_Vision_State(Make_Near_Vision(seq++, 1U, 0U, 0U, 0U, 0U, safe_aligned), t));
        auto out = m.Update(in);
        assert(out.drive_action == DRIVE_ACTION_SOFT_STOP);

        // 120 ms later gate settles with NO new frame -> MUST NOT start push!
        in.gate_settled = true;
        in.now_ms = (t += 120U);
        out = m.Update(in);
        assert(out.drive_action == DRIVE_ACTION_SOFT_STOP);

        // Later frame WITHOUT VIS_FLAG_GATE_OPEN_SETTLED_ACK -> still MUST NOT start push!
        in.now_ms = (t += 50U);
        assert(m.On_Vision_State(Make_Near_Vision(seq++, 1U, 0U, 0U, 0U, 0U, safe_aligned), t));
        out = m.Update(in);
        assert(out.drive_action == DRIVE_ACTION_SOFT_STOP);

        // Strictly later post-settle frame WITH VIS_FLAG_GATE_OPEN_SETTLED_ACK -> authorizes push!
        in.now_ms = (t += 50U);
        assert(m.On_Vision_State(
            Make_Near_Vision(seq++, 1U, 0U, 0U, 0U, 0U, safe_aligned | VIS_FLAG_GATE_OPEN_SETTLED_ACK), t));
        out = m.Update(in);
        assert(out.drive_action == DRIVE_ACTION_START_DISTANCE && out.distance_m > 0.0f);
    }

    // Case 1B: Fresh cyan injected while gate is opening MUST abort and NEVER emit START_DISTANCE!
    {
        Class_Rescue_Mission m;
        Struct_Mission_Inputs in{};
        uint32_t t = 1000U;
        uint16_t seq = 1U;
        advance_to_gate_opening(m, in, t, seq);

        in.gate_settled = false;
        in.now_ms = (t += 50U);
        assert(m.On_Vision_State(Make_Near_Vision(seq++, 1U, 0U, 0U, 1U, 0U, VIS_FLAG_HOME_HALF_MATCH), t));
        auto out = m.Update(in);
        assert(out.drive_action == DRIVE_ACTION_EMERGENCY_STOP);
        assert(m.Get_State() == MISSION_BACK_OUT_SEPARATE);
    }

    // Case 1C: Cyan on aligned ramp (VIS_FLAG_DROP_PATH_CLEAR == 0 while corral/sweep/half are all set) MUST abort!
    {
        Class_Rescue_Mission m;
        Struct_Mission_Inputs in{};
        uint32_t t = 1000U;
        uint16_t seq = 1U;
        advance_to_gate_opening(m, in, t, seq);

        in.gate_settled = true;
        in.now_ms = (t += 50U);
        const uint16_t path_blocked =
            VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR | VIS_FLAG_HOME_HALF_MATCH | VIS_FLAG_GATE_OPEN_SETTLED_ACK;
        assert(m.On_Vision_State(Make_Near_Vision(seq++, 1U, 0U, 0U, 0U, 0U, path_blocked), t));
        auto out = m.Update(in);
        assert(out.drive_action == DRIVE_ACTION_EMERGENCY_STOP);
        assert(m.Get_State() == MISSION_BACK_OUT_SEPARATE);
    }

    // Case 1D: Cargo tuple mutates between Sub_Phase 0 (1g) and post-settle frame (1g+1b) -> MUST abort!
    {
        Class_Rescue_Mission m;
        Struct_Mission_Inputs in{};
        uint32_t t = 1000U;
        uint16_t seq = 1U;
        advance_to_gate_opening(m, in, t, seq);

        in.gate_settled = true;
        in.now_ms = (t += 50U);
        m.Update(in); // latch settled edge
        in.now_ms = (t += 50U);
        assert(m.On_Vision_State(
            Make_Near_Vision(seq++, 1U, 1U, 0U, 0U, 0U, safe_aligned | VIS_FLAG_GATE_OPEN_SETTLED_ACK), t));
        auto out = m.Update(in);
        assert(out.drive_action == DRIVE_ACTION_EMERGENCY_STOP);
        assert(m.Get_State() == MISSION_BACK_OUT_SEPARATE);
    }

    // Case 1E: Half alignment OR drop-path clearance lost DURING active forward push -> immediate EMERGENCY_STOP!
    {
        Class_Rescue_Mission m;
        Struct_Mission_Inputs in{};
        uint32_t t = 1000U;
        uint16_t seq = 1U;
        advance_to_gate_opening(m, in, t, seq);

        in.gate_settled = true;
        in.now_ms = (t += 50U);
        m.Update(in); // latch settled edge
        in.now_ms = (t += 50U);
        assert(m.On_Vision_State(
            Make_Near_Vision(seq++, 1U, 0U, 0U, 0U, 0U, safe_aligned | VIS_FLAG_GATE_OPEN_SETTLED_ACK), t));
        auto out = m.Update(in);
        assert(out.drive_action == DRIVE_ACTION_START_DISTANCE && out.distance_m > 0.0f);

        in.chassis_active = true;
        in.wheels_still = false;
        in.forward_delta_m = 0.05f;
        in.now_ms = (t += 50U);
        // Finding 7 (R6): During active push, VIS_FLAG_DROP_BASELINE_READY clearing (e.g. cargo sliding 1 px
        // out of corral before TEL_FLAG_DROP_PUSHING arrives) MUST NOT abort as long as VIS_FLAG_DROP_PATH_CLEAR=1!
        assert(m.On_Vision_State(
            Make_Near_Vision(seq++, 0U, 0U, 0U, 0U, 0U,
                             VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR | VIS_FLAG_HOME_HALF_MATCH | VIS_FLAG_DROP_PATH_CLEAR),
            t));
        out = m.Update(in);
        assert(out.drive_action == DRIVE_ACTION_KEEP);
        assert(m.Get_State() == MISSION_DROP_OPEN_AND_PUSH);

        // Cyan enters ramp/push path (VIS_FLAG_DROP_PATH_CLEAR drops to 0) mid-push -> MUST abort immediately!
        in.now_ms = (t += 50U);
        assert(m.On_Vision_State(
            Make_Near_Vision(seq++, 1U, 0U, 0U, 0U, 0U, VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR | VIS_FLAG_HOME_HALF_MATCH), t));
        out = m.Update(in);
        assert(out.drive_action == DRIVE_ACTION_EMERGENCY_STOP);
        assert(m.Get_State() == MISSION_BACK_OUT_SEPARATE);
    }
}

static void Test_Active_Reverse_Abrupt_Clearance_Loss_Cancels_Immediately_And_Waits_Standstill()
{
    Class_Rescue_Mission m;
    assert(m.On_Set_Config({RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION, 1U}));
    const uint16_t safe = VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR;
    uint32_t t = 1000U;
    uint16_t seq = 1U;
    assert(m.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), t));

    Struct_Mission_Inputs in{};
    in.now_ms = t;
    in.hardware_ready = true;
    in.can_launch = true;
    in.button_pressed = true;
    in.wheels_still = true;
    in.gate_settled = true;
    in.camera_settled = true;
    m.Update(in);

    // Build 0.16 m straight forward corridor during LAUNCH_CROSS_BUMPS, then arrive at INSPECT_CORRAL.
    in.button_pressed = false;
    in.chassis_active = true;
    in.wheels_still = false;
    in.forward_delta_m = 0.16f;
    in.chassis_motion_done = true;
    in.now_ms = (t += 50U);
    assert(m.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 190U), t));
    m.Update(in);

    in.forward_delta_m = 0.0f;
    in.chassis_motion_done = false;
    in.chassis_active = false;
    in.wheels_still = true;
    for (int i = 0; i < 2; ++i)
    {
        in.now_ms = (t += 50U);
        assert(m.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 190U), t));
        m.Update(in);
    }
    assert(m.Get_State() == MISSION_INSPECT_CORRAL);
    assert(m.Get_Proven_Rear_Clear_M() >= 0.15f);

    // Inject cyan in corral -> enters MISSION_BACK_OUT_SEPARATE.
    in.now_ms = (t += 50U);
    assert(m.On_Vision_State(Make_Near_Vision(seq++, 0U, 0U, 0U, 1U, 0U, 0U), t));
    m.Update(in);
    assert(m.Get_State() == MISSION_BACK_OUT_SEPARATE);

    // First tick of BACK_OUT_SEPARATE while still -> starts margin-bounded reverse (-0.12 m out of 0.16 m).
    in.now_ms = (t += 20U);
    assert(m.On_Vision_State(Make_Far_Vision(seq++, TARGET_NONE, 0U), t));
    auto out = m.Update(in);
    assert(out.drive_action == DRIVE_ACTION_START_DISTANCE);
    assert(out.distance_m <= -0.115f && out.distance_m >= -0.125f);

    // Mid-reverse while wheels are moving: yaw drift >= 0.10 rad abruptly zeroes Proven_Rear_Clear_M!
    in.chassis_active = true;
    in.wheels_still = false;
    in.forward_delta_m = -0.02f;
    in.yaw_delta_rad = 0.12f;
    in.now_ms = (t += 20U);
    assert(m.On_Vision_State(Make_Far_Vision(seq++, TARGET_NONE, 0U), t));
    out = m.Update(in);
    assert(m.Get_Proven_Rear_Clear_M() == 0.0f);
    // MUST cancel immediately with EMERGENCY_STOP (NOT START_TURN while wheels still moving!).
    assert(out.drive_action == DRIVE_ACTION_EMERGENCY_STOP);

    // Next tick: chassis idle, but wheels still spinning (wheels_still == false) -> MUST keep waiting!
    in.chassis_active = false;
    in.wheels_still = false;
    in.forward_delta_m = 0.0f;
    in.yaw_delta_rad = 0.0f;
    in.now_ms = (t += 20U);
    assert(m.On_Vision_State(Make_Far_Vision(seq++, TARGET_NONE, 0U), t));
    out = m.Update(in);
    assert(out.drive_action == DRIVE_ACTION_EMERGENCY_STOP);

    // Standstill confirmed -> NOW starts turn cleanly!
    in.wheels_still = true;
    in.now_ms = (t += 20U);
    assert(m.On_Vision_State(Make_Far_Vision(seq++, TARGET_NONE, 0U), t));
    out = m.Update(in);
    assert(out.drive_action == DRIVE_ACTION_START_TURN);

    // Also verify abrupt clearance loss during MISSION_DROP_BACKOUT_VERIFY active reverse
    // cancels IMMEDIATELY with DRIVE_ACTION_EMERGENCY_STOP (never DRIVE_ACTION_SOFT_STOP!).
    {
        Class_Rescue_Mission dm;
        assert(dm.On_Set_Config({RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION, 1U}));
        const uint16_t safe_aligned =
            VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR | VIS_FLAG_HOME_HALF_MATCH |
            VIS_FLAG_DROP_PATH_CLEAR | VIS_FLAG_DROP_BASELINE_READY;
        uint32_t dt = 1000U;
        uint16_t dseq = 1U;
        assert(dm.On_Vision_State(Make_Far_Vision(dseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), dt));
        Struct_Mission_Inputs din{};
        din.now_ms = dt;
        din.hardware_ready = true;
        din.can_launch = true;
        din.button_pressed = true;
        din.wheels_still = true;
        din.gate_settled = true;
        din.camera_settled = true;
        dm.Update(din);
        din.button_pressed = false;
        din.chassis_motion_done = true;
        din.now_ms = (dt += 50U);
        assert(dm.On_Vision_State(Make_Far_Vision(dseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 190U), dt));
        dm.Update(din);
        din.chassis_motion_done = false;
        for (int i = 0; i < 2; ++i)
        {
            din.now_ms = (dt += 50U);
            assert(dm.On_Vision_State(Make_Far_Vision(dseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 190U), dt));
            dm.Update(din);
        }
        for (int k = 0; k < 6; ++k)
        {
            din.now_ms = (dt += 50U);
            assert(dm.On_Vision_State(Make_Near_Vision(dseq++, 1U, 0U, 0U, 0U, 0U, safe), dt));
            dm.Update(din);
        }
        din.now_ms = (dt += 50U);
        assert(dm.On_Vision_State(Make_Far_Vision(dseq++, TARGET_NONE, VIS_FLAG_HOME_VALID, 0, 500U), dt));
        dm.Update(din);
        din.now_ms = (dt += 50U);
        assert(dm.On_Vision_State(Make_Far_Vision(dseq++, TARGET_NONE, VIS_FLAG_HOME_VALID | VIS_FLAG_HOME_HALF_MATCH, 0, 200U), dt));
        dm.Update(din);
        for (int k = 0; k < 3; ++k)
        {
            din.now_ms = (dt += 50U);
            assert(dm.On_Vision_State(Make_Near_Vision(dseq++, 1U, 0U, 0U, 0U, 0U, safe_aligned), dt));
            dm.Update(din);
        }
        // Settled edge latch + strictly later post-settle frame with ACK.
        din.now_ms = (dt += 50U);
        dm.Update(din);
        din.now_ms = (dt += 50U);
        assert(dm.On_Vision_State(
            Make_Near_Vision(dseq++, 1U, 0U, 0U, 0U, 0U, safe_aligned | VIS_FLAG_GATE_OPEN_SETTLED_ACK), dt));
        dm.Update(din);
        // Complete 0.16 m push -> standstill -> starts margin-bounded backout reverse (-0.12 m).
        din.forward_delta_m = 0.16f;
        din.chassis_motion_done = true;
        din.chassis_active = false;
        din.wheels_still = true;
        din.now_ms = (dt += 50U);
        assert(dm.On_Vision_State(Make_Near_Vision(dseq++, 0U, 0U, 0U, 0U, 0U, safe_aligned), dt));
        auto dout = dm.Update(din);
        assert(dm.Get_State() == MISSION_DROP_BACKOUT_VERIFY);
        assert(dout.drive_action == DRIVE_ACTION_START_DISTANCE && dout.distance_m <= -0.115f && dout.distance_m >= -0.125f);

        // Mid-reverse abrupt clearance loss via yaw jump -> MUST emit DRIVE_ACTION_EMERGENCY_STOP immediately!
        din.chassis_motion_done = false;
        din.chassis_active = true;
        din.wheels_still = false;
        din.forward_delta_m = -0.02f;
        din.yaw_delta_rad = 0.12f;
        din.now_ms = (dt += 20U);
        assert(dm.On_Vision_State(Make_Near_Vision(dseq++, 0U, 0U, 0U, 0U, 0U, safe_aligned), dt));
        dout = dm.Update(din);
        assert(dm.Get_Proven_Rear_Clear_M() == 0.0f);
        assert(dout.drive_action == DRIVE_ACTION_EMERGENCY_STOP);

        // Finding 3: If wheels NEVER stop (wheels_still stays false) past standstill/drop timeout,
        // mission MUST latch MISSION_FAULT_STOP with EMERGENCY_STOP (never enter SEARCH_TARGET and turn!).
        din.chassis_active = false;
        din.wheels_still = false;
        din.forward_delta_m = 0.0f;
        din.yaw_delta_rad = 0.0f;
        for (int k = 0; k < 20 && dm.Get_State() != MISSION_FAULT_STOP; ++k)
        {
            din.now_ms = (dt += 100U);
            assert(dm.On_Vision_State(Make_Far_Vision(dseq++, TARGET_NONE, 0U), dt));
            dout = dm.Update(din);
            assert(dout.drive_action == DRIVE_ACTION_EMERGENCY_STOP);
        }
        assert(dm.Get_State() == MISSION_FAULT_STOP);
    }

    // Finding 3A: Operator button press during active reverse MUST emit DRIVE_ACTION_EMERGENCY_STOP (not SOFT_STOP!).
    {
        Class_Rescue_Mission bm;
        assert(bm.On_Set_Config({RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION, 1U}));
        uint32_t bt = 1000U;
        uint16_t bseq = 1U;
        assert(bm.On_Vision_State(Make_Far_Vision(bseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), bt));
        Struct_Mission_Inputs bin{};
        bin.now_ms = bt;
        bin.hardware_ready = true;
        bin.can_launch = true;
        bin.button_pressed = true;
        bin.wheels_still = true;
        bin.gate_settled = true;
        bin.camera_settled = true;
        bm.Update(bin);
        bin.button_pressed = false;
        bin.forward_delta_m = 0.16f;
        bin.chassis_motion_done = true;
        bin.now_ms = (bt += 50U);
        assert(bm.On_Vision_State(Make_Far_Vision(bseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 190U), bt));
        bm.Update(bin);
        bin.forward_delta_m = 0.0f;
        bin.chassis_motion_done = false;
        for (int i = 0; i < 2; ++i)
        {
            bin.now_ms = (bt += 50U);
            assert(bm.On_Vision_State(Make_Far_Vision(bseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 190U), bt));
            bm.Update(bin);
        }
        bin.now_ms = (bt += 50U);
        assert(bm.On_Vision_State(Make_Near_Vision(bseq++, 0U, 0U, 0U, 1U, 0U, 0U), bt));
        bm.Update(bin);
        assert(bm.Get_State() == MISSION_BACK_OUT_SEPARATE);
        bin.now_ms = (bt += 20U);
        assert(bm.On_Vision_State(Make_Far_Vision(bseq++, TARGET_NONE, 0U), bt));
        auto bout = bm.Update(bin);
        assert(bout.drive_action == DRIVE_ACTION_START_DISTANCE && bout.distance_m < 0.0f);

        // Press operator stop button during active reverse -> MUST emit EMERGENCY_STOP!
        bin.chassis_active = true;
        bin.wheels_still = false;
        bin.button_pressed = true;
        bin.now_ms = (bt += 20U);
        bout = bm.Update(bin);
        assert(bm.Get_State() == MISSION_MATCH_FINISHED);
        assert(bout.drive_action == DRIVE_ACTION_EMERGENCY_STOP);
    }
    (void)safe;
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
    in.wheels_still = true;
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
        // Trigger escape in FIND_HOME via opponent alert (from standstill).
        in.chassis_active = false;
        in.wheels_still = true;
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

static void Test_Braking_Phase_Vision_Outage_And_Uart_Rx_Unhealthy()
{
    // Finding 1 (R5): Vision outage & recovery AFTER reverse completes into braking
    // (BOTH BACK_OUT_SEPARATE Sub_Phase 2 AND ESCAPE_RECOVERY Sub_Phase 1) MUST retain
    // the global standstill interlock until fresh feedback confirms wheels_still=true,
    // NEVER commanding search velocity (0.45 rad/s) while wheels_still=false!
    {
        Class_Rescue_Mission m;
        assert(m.On_Set_Config({RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION, 1U}));
        uint32_t t = 1000U;
        uint16_t seq = 1U;
        assert(m.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), t));
        Struct_Mission_Inputs in{};
        in.now_ms = t;
        in.hardware_ready = true;
        in.can_launch = true;
        in.button_pressed = true;
        in.wheels_still = true;
        in.gate_settled = true;
        in.camera_settled = true;
        m.Update(in);
        in.button_pressed = false;
        in.forward_delta_m = 0.16f;
        in.chassis_motion_done = true;
        in.now_ms = (t += 50U);
        assert(m.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 190U), t));
        m.Update(in);
        in.forward_delta_m = 0.0f;
        in.chassis_motion_done = false;
        for (int i = 0; i < 2; ++i)
        {
            in.now_ms = (t += 50U);
            assert(m.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 190U), t));
            m.Update(in);
        }
        // Cyan in inspect -> BACK_OUT_SEPARATE -> start reverse (Sub_Phase 1).
        in.now_ms = (t += 50U);
        assert(m.On_Vision_State(Make_Near_Vision(seq++, 0U, 0U, 0U, 1U, 0U, 0U), t));
        m.Update(in);
        assert(m.Get_State() == MISSION_BACK_OUT_SEPARATE);
        in.now_ms = (t += 20U);
        assert(m.On_Vision_State(Make_Far_Vision(seq++, TARGET_NONE, 0U), t));
        auto out = m.Update(in);
        assert(out.drive_action == DRIVE_ACTION_START_DISTANCE && out.distance_m < 0.0f);

        // Reverse finishes distance into braking Sub_Phase 2 while wheels are STILL moving!
        in.chassis_motion_done = true;
        in.chassis_active = false;
        in.wheels_still = false;
        in.now_ms = (t += 40U);
        assert(m.On_Vision_State(Make_Far_Vision(seq++, TARGET_NONE, 0U), t));
        out = m.Update(in);
        assert(m.Get_State() == MISSION_BACK_OUT_SEPARATE);
        assert(out.drive_action == DRIVE_ACTION_SOFT_STOP);
        in.chassis_motion_done = false;

        // Vision outage (> 300 ms) during braking Sub_Phase 2 -> MUST engage standstill gate + EMERGENCY_STOP!
        in.now_ms = (t += 350U);
        out = m.Update(in);
        assert(out.drive_action == DRIVE_ACTION_EMERGENCY_STOP);

        // Vision recovers with fresh FAR frame while wheels_still is STILL false ->
        // MUST NEVER command 0.45 rad/s search velocity! Must stay EMERGENCY_STOP until wheels_still=true!
        in.now_ms = (t += 50U);
        assert(m.On_Vision_State(Make_Far_Vision(seq++, TARGET_NONE, 0U), t));
        out = m.Update(in);
        assert(out.drive_action == DRIVE_ACTION_EMERGENCY_STOP);

        for (int k = 0; k < 5; ++k)
        {
            in.now_ms = (t += 40U);
            assert(m.On_Vision_State(Make_Far_Vision(seq++, TARGET_NONE, 0U), t));
            out = m.Update(in);
            assert(out.drive_action == DRIVE_ACTION_EMERGENCY_STOP);
            assert(out.angular_rad_s == 0.0f && out.linear_mps == 0.0f);
        }

        // Fresh feedback confirms standstill -> first tick transitions cleanly to SEARCH_TARGET with EMERGENCY_STOP,
        // and the next tick in SEARCH_TARGET cleanly resumes search spin!
        in.wheels_still = true;
        in.now_ms = (t += 40U);
        assert(m.On_Vision_State(Make_Far_Vision(seq++, TARGET_NONE, 0U), t));
        out = m.Update(in);
        assert(m.Get_State() == MISSION_SEARCH_TARGET);
        assert(out.drive_action == DRIVE_ACTION_EMERGENCY_STOP);

        in.now_ms = (t += 40U);
        assert(m.On_Vision_State(Make_Far_Vision(seq++, TARGET_NONE, 0U), t));
        out = m.Update(in);
        assert(m.Get_State() == MISSION_SEARCH_TARGET);
        assert(out.drive_action == DRIVE_ACTION_VELOCITY && out.angular_rad_s > 0.1f);
    }

    // Part B: Same braking-phase outage/recovery probe in MISSION_ESCAPE_RECOVERY Sub_Phase 1!
    {
        Class_Rescue_Mission em;
        assert(em.On_Set_Config({RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION, 1U}));
        uint32_t et = 1000U;
        uint16_t eseq = 1U;
        assert(em.On_Vision_State(Make_Far_Vision(eseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), et));
        Struct_Mission_Inputs ein{};
        ein.now_ms = et;
        ein.hardware_ready = true;
        ein.can_launch = true;
        ein.button_pressed = true;
        ein.wheels_still = true;
        ein.gate_settled = true;
        ein.camera_settled = true;
        em.Update(ein);
        ein.button_pressed = false;
        ein.forward_delta_m = 0.20f;
        ein.chassis_motion_done = true;
        ein.now_ms = (et += 50U);
        assert(em.On_Vision_State(Make_Far_Vision(eseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), et));
        em.Update(ein);
        ein.forward_delta_m = 0.0f;
        ein.chassis_motion_done = false;
        assert(em.Get_State() == MISSION_SEARCH_TARGET);

        // Trigger stall escape with explicit rear clearance -> starts escape reverse (Sub_Phase 0).
        ein.chassis_stalled = true;
        ein.now_ms = (et += 40U);
        assert(em.On_Vision_State(Make_Far_Vision(eseq++, TARGET_NONE, VIS_FLAG_REAR_CLEAR), et));
        auto eout = em.Update(ein);
        assert(em.Get_State() == MISSION_ESCAPE_RECOVERY);
        assert(eout.drive_action == DRIVE_ACTION_START_DISTANCE && eout.distance_m < 0.0f);
        ein.chassis_stalled = false;

        // Escape reverse completes into braking Sub_Phase 1 while wheels_still=false!
        ein.chassis_motion_done = true;
        ein.chassis_active = false;
        ein.wheels_still = false;
        ein.now_ms = (et += 40U);
        assert(em.On_Vision_State(Make_Far_Vision(eseq++, TARGET_NONE, VIS_FLAG_REAR_CLEAR), et));
        eout = em.Update(ein);
        assert(em.Get_State() == MISSION_ESCAPE_RECOVERY);
        assert(eout.drive_action == DRIVE_ACTION_SOFT_STOP);
        ein.chassis_motion_done = false;

        // Vision outage (> 300 ms) during escape braking Sub_Phase 1, then recovery while wheels_still=false!
        ein.now_ms = (et += 350U);
        eout = em.Update(ein);
        assert(eout.drive_action == DRIVE_ACTION_EMERGENCY_STOP);

        // Even across recovery + 1500 ms with wheels never stopping, MUST stay EMERGENCY_STOP and latch FAULT_STOP!
        for (int k = 0; k < 20 && em.Get_State() != MISSION_FAULT_STOP; ++k)
        {
            ein.now_ms = (et += 100U);
            assert(em.On_Vision_State(Make_Far_Vision(eseq++, TARGET_NONE, 0U), et));
            eout = em.Update(ein);
            assert(eout.drive_action == DRIVE_ACTION_EMERGENCY_STOP);
            assert(eout.angular_rad_s == 0.0f && eout.linear_mps == 0.0f);
        }
        assert(em.Get_State() == MISSION_FAULT_STOP);
    }

    // Part C: Finding 6 (R5) & Finding 2 (R6) — in.uart_rx_healthy=false immediately inhibits motion
    // AND invalidates cached pre-fault vision so rearming UART without a newly accepted frame NEVER
    // restores VISION_FRESH or forward motion!
    {
        Class_Rescue_Mission um;
        assert(um.On_Set_Config({RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION, 1U}));
        uint32_t ut = 100U;
        uint16_t useq = 1U;
        assert(um.On_Vision_State(Make_Far_Vision(useq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 500U), ut));
        Struct_Mission_Inputs uin{};
        uin.now_ms = ut;
        uin.hardware_ready = true;
        uin.can_launch = true;
        uin.uart_rx_healthy = true;
        uin.button_pressed = true;
        uin.wheels_still = true;
        uin.gate_settled = true;
        uin.camera_settled = true;
        assert(um.Update(uin).drive_action == DRIVE_ACTION_START_DISTANCE);
        uin.button_pressed = false;
        uin.chassis_motion_done = true;
        uin.now_ms = (ut = 101U);
        um.Update(uin);
        uin.chassis_motion_done = false;
        uin.now_ms = (ut = 102U);
        um.Update(uin);
        assert(um.Get_State() == MISSION_APPROACH_TARGET);

        // RX becomes unhealthy at 110 ms -> immediate stop and invalidates cached pre-fault vision.
        uin.uart_rx_healthy = false;
        uin.now_ms = (ut = 110U);
        auto uout = um.Update(uin);
        assert(uout.drive_action == DRIVE_ACTION_SOFT_STOP);
        assert(!um.Is_Vision_Fresh(ut));

        // RX rearms at 120 ms with NO new frame -> MUST remain stopped and NOT vision-fresh!
        uin.uart_rx_healthy = true;
        uin.now_ms = (ut = 120U);
        uout = um.Update(uin);
        assert(uout.drive_action == DRIVE_ACTION_SOFT_STOP);
        assert(uout.linear_mps == 0.0f);
        assert(!um.Is_Vision_Fresh(ut));

        // Duplicate pre-fault frame_seq=1 MUST be rejected and keep vision invalidated!
        assert(!um.On_Vision_State(Make_Far_Vision(1U, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 500U), 125U));
        assert(!um.Is_Vision_Fresh(125U));

        // Brand-new post-recovery frame_seq=2 restores Is_Vision_Fresh and forward approach cleanly!
        uin.now_ms = (ut = 130U);
        assert(um.On_Vision_State(Make_Far_Vision(useq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 500U), ut));
        assert(um.Is_Vision_Fresh(ut));
        uout = um.Update(uin);
        assert(uout.drive_action == DRIVE_ACTION_VELOCITY && uout.linear_mps > 0.0f);
    }
}

static void Test_Drop_Verify_Requires_Consecutive_Near_Frames()
{
    // Finding 4 (R11): MISSION_DROP_BACKOUT_VERIFY Sub_Phase 2 MUST reset Confirm_Count on any
    // non-NEAR frame (TRANSITION/FAR), including same-drain TRANSITION -> CONFIRM or
    // NEAR(unconfirmed) -> NEAR(confirmed), so non-consecutive confirmations NEVER unlock
    // First_Green_Unlocked or credit Delivered_Green!
    Class_Rescue_Mission dm;
    assert(dm.On_Set_Config({RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION, 1U}));
    const uint16_t safe = VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR;
    const uint16_t safe_aligned =
        VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR | VIS_FLAG_HOME_HALF_MATCH |
        VIS_FLAG_DROP_PATH_CLEAR | VIS_FLAG_DROP_BASELINE_READY;
    uint32_t dt = 1000U;
    uint16_t dseq = 1U;
    assert(dm.On_Vision_State(Make_Far_Vision(dseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), dt));
    Struct_Mission_Inputs din{};
    din.now_ms = dt;
    din.hardware_ready = true;
    din.can_launch = true;
    din.button_pressed = true;
    din.wheels_still = true;
    din.gate_settled = true;
    din.camera_settled = true;
    dm.Update(din);
    din.button_pressed = false;
    din.chassis_motion_done = true;
    din.now_ms = (dt += 50U);
    assert(dm.On_Vision_State(Make_Far_Vision(dseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 190U), dt));
    dm.Update(din);
    din.chassis_motion_done = false;
    for (int i = 0; i < 2; ++i)
    {
        din.now_ms = (dt += 50U);
        assert(dm.On_Vision_State(Make_Far_Vision(dseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 190U), dt));
        dm.Update(din);
    }
    for (int k = 0; k < 6; ++k)
    {
        din.now_ms = (dt += 50U);
        assert(dm.On_Vision_State(Make_Near_Vision(dseq++, 1U, 0U, 0U, 0U, 0U, safe), dt));
        dm.Update(din);
    }
    din.now_ms = (dt += 50U);
    assert(dm.On_Vision_State(Make_Far_Vision(dseq++, TARGET_NONE, VIS_FLAG_HOME_VALID, 0, 500U), dt));
    dm.Update(din);
    din.now_ms = (dt += 50U);
    assert(dm.On_Vision_State(Make_Far_Vision(dseq++, TARGET_NONE, VIS_FLAG_HOME_VALID | VIS_FLAG_HOME_HALF_MATCH, 0, 200U), dt));
    dm.Update(din);
    for (int k = 0; k < 3; ++k)
    {
        din.now_ms = (dt += 50U);
        assert(dm.On_Vision_State(Make_Near_Vision(dseq++, 1U, 0U, 0U, 0U, 0U, safe_aligned), dt));
        dm.Update(din);
    }
    din.now_ms = (dt += 50U);
    dm.Update(din);
    din.now_ms = (dt += 50U);
    assert(dm.On_Vision_State(
        Make_Near_Vision(dseq++, 1U, 0U, 0U, 0U, 0U, safe_aligned | VIS_FLAG_GATE_OPEN_SETTLED_ACK), dt));
    dm.Update(din);
    // Complete push with no proven reverse -> enters MISSION_DROP_BACKOUT_VERIFY Sub_Phase 2 directly!
    din.chassis_motion_done = true;
    din.chassis_active = false;
    din.wheels_still = true;
    din.now_ms = (dt += 50U);
    assert(dm.On_Vision_State(Make_Near_Vision(dseq++, 0U, 0U, 0U, 0U, 0U, safe_aligned), dt));
    dm.Update(din);
    din.chassis_motion_done = false;
    assert(dm.Get_State() == MISSION_DROP_BACKOUT_VERIFY);

    auto make_trans = [&](uint16_t s) {
        Struct_Vision_State v{};
        v.frame_seq = s;
        v.camera_view = CAMERA_VIEW_TRANSITION;
        v.confidence = 90U;
        return v;
    };

    // Case 1: CONFIRM -> TRANSITION -> CONFIRM -> TRANSITION -> CONFIRM MUST NOT confirm!
    for (int step = 0; step < 3; ++step)
    {
        din.now_ms = (dt += 40U);
        assert(dm.On_Vision_State(Make_Near_Vision(dseq++, 0U, 0U, 0U, 0U, 0U, safe | VIS_FLAG_DROP_CONFIRMED), dt));
        dm.Update(din);
        assert(dm.Get_State() == MISSION_DROP_BACKOUT_VERIFY);
        assert(!dm.Is_First_Green_Unlocked());
        assert(dm.Get_Delivered_Green() == 0U);
        if (step < 2)
        {
            din.now_ms = (dt += 40U);
            assert(dm.On_Vision_State(make_trans(dseq++), dt));
            dm.Update(din);
            assert(dm.Get_State() == MISSION_DROP_BACKOUT_VERIFY);
        }
    }

    // Case 2: Currently Confirm_Count == 1; same-drain TRANSITION -> CONFIRM resets to 0, so a following CONFIRM
    // only reaches 1 (NOT 3!) and MUST NOT confirm!
    din.now_ms = (dt += 40U);
    assert(dm.On_Vision_State(make_trans(dseq++), dt));
    assert(dm.On_Vision_State(Make_Near_Vision(dseq++, 0U, 0U, 0U, 0U, 0U, safe | VIS_FLAG_DROP_CONFIRMED), dt));
    dm.Update(din);
    assert(dm.Get_State() == MISSION_DROP_BACKOUT_VERIFY);
    din.now_ms = (dt += 40U);
    assert(dm.On_Vision_State(Make_Near_Vision(dseq++, 0U, 0U, 0U, 0U, 0U, safe | VIS_FLAG_DROP_CONFIRMED), dt));
    dm.Update(din);
    assert(dm.Get_State() == MISSION_DROP_BACKOUT_VERIFY);
    assert(!dm.Is_First_Green_Unlocked());

    // Case 3: Currently Confirm_Count == 1; same-drain NEAR(unconfirmed) -> NEAR(confirmed) resets before counting
    // so Confirm_Count stays 1; one more CONFIRM reaches 2 (NOT 3!).
    din.now_ms = (dt += 40U);
    assert(dm.On_Vision_State(Make_Near_Vision(dseq++, 0U, 0U, 0U, 0U, 0U, safe), dt));
    assert(dm.On_Vision_State(Make_Near_Vision(dseq++, 0U, 0U, 0U, 0U, 0U, safe | VIS_FLAG_DROP_CONFIRMED), dt));
    dm.Update(din);
    assert(dm.Get_State() == MISSION_DROP_BACKOUT_VERIFY);
    din.now_ms = (dt += 40U);
    assert(dm.On_Vision_State(Make_Near_Vision(dseq++, 0U, 0U, 0U, 0U, 0U, safe | VIS_FLAG_DROP_CONFIRMED), dt));
    dm.Update(din);
    assert(dm.Get_State() == MISSION_DROP_BACKOUT_VERIFY);
    assert(!dm.Is_First_Green_Unlocked());

    // Case 4: Third consecutive settled NEAR(VIS_FLAG_DROP_CONFIRMED) frame cleanly confirms & unlocks!
    din.now_ms = (dt += 40U);
    assert(dm.On_Vision_State(Make_Near_Vision(dseq++, 0U, 0U, 0U, 0U, 0U, safe | VIS_FLAG_DROP_CONFIRMED), dt));
    dm.Update(din);
    assert(dm.Get_State() == MISSION_SEARCH_TARGET);
    assert(dm.Is_First_Green_Unlocked());
    assert(dm.Get_Delivered_Green() == 1U);
}

static void Test_Round12_Push_Same_Drain_Standstill_Bench_Leave_And_Interrupted_Drop()
{
    const uint16_t safe = VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR;
    const uint16_t aligned = safe | VIS_FLAG_HOME_VALID | VIS_FLAG_HOME_HALF_MATCH |
                             VIS_FLAG_DROP_PATH_CLEAR | VIS_FLAG_DROP_BASELINE_READY;

    struct Harness
    {
        Class_Rescue_Mission m;
        Struct_Mission_Inputs in{};
        uint16_t seq = 1U;
        uint32_t t = 1000U;
        Struct_Mission_Output tick(uint32_t dt = 10U)
        {
            in.now_ms = (t += dt);
            return m.Update(in);
        }
        void far(uint8_t type, uint16_t flags, uint16_t dist)
        {
            assert(m.On_Vision_State(Make_Far_Vision(seq++, type, flags, 0, dist), t));
        }
        void near(uint8_t g, uint16_t flags)
        {
            assert(m.On_Vision_State(Make_Near_Vision(seq++, g, 0U, 0U, 0U, 0U, flags), t));
        }
        void to_inspect()
        {
            assert(m.On_Set_Config({RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION, 1U}));
            in.hardware_ready = in.can_launch = in.wheels_still = in.gate_settled = in.camera_settled = true;
            in.button_pressed = true;
            far(TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 190U);
            assert(tick().drive_action == DRIVE_ACTION_START_DISTANCE);
            in.button_pressed = false;
            in.chassis_motion_done = true;
            tick();
            in.chassis_motion_done = false;
            tick();
            tick();
            assert(m.Get_State() == MISSION_INSPECT_CORRAL);
        }
        void to_opening()
        {
            to_inspect();
            for (unsigned i = 0; i < 6U; ++i)
            {
                near(1U, VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR);
                tick();
            }
            assert(m.Get_State() == MISSION_FIND_HOME);
            far(TARGET_NONE, VIS_FLAG_HOME_VALID, 500U);
            tick();
            far(TARGET_NONE, VIS_FLAG_HOME_VALID | VIS_FLAG_HOME_HALF_MATCH, 200U);
            tick();
            const uint16_t al = VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR | VIS_FLAG_HOME_VALID |
                                VIS_FLAG_HOME_HALF_MATCH | VIS_FLAG_DROP_PATH_CLEAR | VIS_FLAG_DROP_BASELINE_READY;
            for (unsigned i = 0; i < 3U; ++i)
            {
                near(1U, al);
                tick();
            }
            assert(m.Get_State() == MISSION_DROP_OPEN_AND_PUSH);
            assert(!tick().gate_closed);
        }
        void to_pushing()
        {
            to_opening();
            const uint16_t al = VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR | VIS_FLAG_HOME_VALID |
                                VIS_FLAG_HOME_HALF_MATCH | VIS_FLAG_DROP_PATH_CLEAR | VIS_FLAG_DROP_BASELINE_READY;
            near(1U, al | VIS_FLAG_GATE_OPEN_SETTLED_ACK);
            assert(tick().drive_action == DRIVE_ACTION_START_DISTANCE);
        }
        void to_verifying()
        {
            to_pushing();
            in.chassis_motion_done = true;
            near(0U, VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR | VIS_FLAG_HOME_VALID |
                         VIS_FLAG_HOME_HALF_MATCH | VIS_FLAG_DROP_PATH_CLEAR);
            tick();
            in.chassis_motion_done = false;
            assert(m.Get_State() == MISSION_DROP_BACKOUT_VERIFY);
        }
    };

    // 1. P1-3: Idle drive with moving wheels MUST NOT authorize gate closure, and losing standstill
    //    during unsettled gate closure MUST immediately retract gate to open!
    {
        Harness h;
        h.to_inspect();
        h.in.chassis_active = false;
        h.in.wheels_still = false;
        for (unsigned i = 0; i < 4U; ++i)
        {
            h.near(1U, safe);
            const auto out = h.tick();
            assert(!out.gate_closed);
            assert(h.m.Get_State() == MISSION_INSPECT_CORRAL);
        }
        // Once wheels settle, must still need 3 fresh post-standstill NEAR frames.
        h.in.wheels_still = true;
        assert(!h.tick().gate_closed); // no new frame -> cannot recount cached frame!
        for (unsigned i = 0; i < 2U; ++i)
        {
            h.near(1U, safe);
            assert(!h.tick().gate_closed);
        }
        h.near(1U, safe);
        assert(h.tick().gate_closed);
        assert(h.m.Get_State() == MISSION_CLOSE_GATE_VERIFY);
        // Losing standstill mid-closure (!gate_settled) immediately retracts!
        h.in.gate_settled = false;
        h.in.wheels_still = false;
        h.near(1U, safe);
        const auto retract = h.tick();
        assert(!retract.gate_closed);
        assert(h.m.Get_State() == MISSION_INSPECT_CORRAL);
    }

    // 2. P1-4: Leaving bench clears actuator overrides; competition launch requires settled open/FAR
    //    servos AND a fresh post-settle FAR frame (never cached NEAR or pre-settle FAR)!
    {
        Harness h;
        h.in.hardware_ready = h.in.can_launch = h.in.wheels_still = true;
        assert(h.m.On_Set_Config({RESCUE_TEAM_RED, 1U, RESCUE_MODE_BENCH_STRAIGHT, 1U}));
        assert(h.m.On_Bench_Actuator({BENCH_SERVO_POS_B, BENCH_SERVO_POS_B, 0U}));
        assert(h.m.Is_Gate_Closed_Cmd() && h.m.Is_Camera_Near_Cmd());
        assert(h.m.On_Set_Config({RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION, 1U}));
        assert(!h.m.Is_Gate_Closed_Cmd() && !h.m.Is_Camera_Near_Cmd());
        h.near(0U, safe);
        const auto pre = h.tick();
        assert(!pre.gate_closed && !pre.camera_near);
        h.in.button_pressed = true;
        auto out = h.tick();
        assert(out.drive_action == DRIVE_ACTION_IDLE && out.launch_rejected_cue);
        // Even a pre-settle FAR frame followed by settling with no new frame MUST still refuse launch!
        h.far(TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 500U);
        out = h.tick();
        assert(out.drive_action == DRIVE_ACTION_IDLE);
        h.in.gate_settled = true;
        h.in.camera_settled = true;
        out = h.tick();
        assert(out.drive_action == DRIVE_ACTION_IDLE && out.launch_rejected_cue);
        // Fresh post-settle FAR frame authorizes competition launch!
        h.far(TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 500U);
        out = h.tick();
        assert(out.drive_action == DRIVE_ACTION_START_DISTANCE);
        assert(h.m.Get_State() == MISSION_LAUNCH_CROSS_BUMPS);
    }

    // 3. P1-1: Same-drain path block / opponent alert / half mismatch MUST NOT be hidden by a clean tail frame!
    {
        Harness h;
        h.to_opening();
        h.near(1U, aligned & ~VIS_FLAG_DROP_PATH_CLEAR);
        h.near(1U, aligned | VIS_FLAG_GATE_OPEN_SETTLED_ACK);
        const auto out = h.tick();
        assert(out.drive_action == DRIVE_ACTION_EMERGENCY_STOP);
        assert(h.m.Get_State() == MISSION_BACK_OUT_SEPARATE);
    }
    {
        Harness h;
        h.to_pushing();
        h.in.chassis_active = true;
        h.in.wheels_still = false;
        h.near(0U, aligned & ~VIS_FLAG_DROP_PATH_CLEAR);
        h.near(0U, aligned);
        const auto out = h.tick();
        assert(out.drive_action == DRIVE_ACTION_EMERGENCY_STOP);
        assert(h.m.Get_State() == MISSION_BACK_OUT_SEPARATE);
    }
    {
        Harness h;
        h.to_opening();
        h.near(1U, aligned | VIS_FLAG_OPPONENT_ZONE_NEAR);
        h.near(1U, aligned | VIS_FLAG_GATE_OPEN_SETTLED_ACK);
        const auto out = h.tick();
        assert(out.drive_action != DRIVE_ACTION_START_DISTANCE);
        assert(h.m.Get_State() == MISSION_ESCAPE_RECOVERY);
    }
    {
        Harness h;
        h.to_opening();
        h.near(1U, aligned & ~VIS_FLAG_HOME_HALF_MATCH);
        h.near(1U, aligned | VIS_FLAG_GATE_OPEN_SETTLED_ACK);
        const auto out = h.tick();
        assert(!out.gate_closed);
        assert(out.drive_action == DRIVE_ACTION_SOFT_STOP);
        assert(h.m.Get_State() == MISSION_INSPECT_CORRAL);
    }

    // 5. Round 13, 14 & 15 invariants:
    //    5a. P1-1, R14 P1-3, R15 P1-2: Bench exit + FAR frame in one drain MUST command open first,
    //        refuse launch on that tick AND on the first tick observing open/FAR settled (raising the
    //        RX fence and advancing the settle generation), reject FAR frames lacking the current
    //        generation ack, and launch ONLY after a post-settle FAR frame echoes the new generation!
    {
        Harness h;
        h.in.hardware_ready = h.in.can_launch = h.in.wheels_still = h.in.gate_settled = h.in.camera_settled = true;
        // First settle in competition (gen 1) before entering bench mode.
        assert(h.m.On_Set_Config({RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION, 1U}));
        h.tick();
        const uint8_t gen_before_bench = h.m.Get_Pre_Match_Settle_Gen();
        assert(gen_before_bench == 1U);
        assert(h.m.On_Set_Config({RESCUE_TEAM_RED, 1U, RESCUE_MODE_BENCH_STRAIGHT, 1U}));
        assert(h.m.On_Bench_Actuator({BENCH_SERVO_POS_B, BENCH_SERVO_POS_A, 0U}));
        assert(h.tick().gate_closed);
        assert(h.m.Get_Pre_Match_Settle_Gen() == 0U);
        assert(h.m.On_Set_Config({RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION, 1U}));
        h.far(TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 190U);
        h.in.button_pressed = true;
        const auto out1 = h.tick();
        assert(!out1.gate_closed && out1.drive_action == DRIVE_ACTION_IDLE && out1.launch_rejected_cue);
        assert(h.m.Get_Pre_Match_Settle_Gen() == 0U);
        // First tick observing open/FAR settled -> advances generation (2 != 1), requests RX fence, and refuses!
        h.far(TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 190U);
        const auto out2 = h.tick();
        assert(out2.drive_action == DRIVE_ACTION_IDLE && out2.launch_rejected_cue);
        assert(h.m.Consume_Pre_Match_Rx_Fence());
        const uint8_t gen_after_bench = h.m.Get_Pre_Match_Settle_Gen();
        assert(gen_after_bench == 2U && gen_after_bench != gen_before_bench);
        // Pre-settle FAR frame echoing old gen=1 (or unacked gen=0) MUST STILL be rejected!
        assert(h.m.On_Vision_State(Make_Far_Vision(h.seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 190U, 0U), h.t));
        assert(h.tick().drive_action == DRIVE_ACTION_IDLE);
        assert(h.m.On_Vision_State(Make_Far_Vision(h.seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 190U, gen_before_bench), h.t));
        assert(h.tick().drive_action == DRIVE_ACTION_IDLE);
        // Strictly newer post-settle FAR frame echoing gen_after_bench (2) -> launch proceeds!
        assert(h.m.On_Vision_State(Make_Far_Vision(h.seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 0, 190U, gen_after_bench), h.t));
        const auto out3 = h.tick();
        assert(out3.drive_action == DRIVE_ACTION_START_DISTANCE);
    }
    //    5b. P1-2: Half-mismatch + FAR in one drain while moving/unsettled MUST keep gate open!
    {
        Harness h;
        h.to_opening();
        h.in.wheels_still = false;
        h.in.chassis_active = true;
        h.in.camera_settled = false;
        h.near(1U, aligned & ~VIS_FLAG_HOME_HALF_MATCH);
        h.far(TARGET_NONE, 0U, 0U);
        const auto out = h.tick();
        assert(!out.gate_closed);
        assert(h.m.Get_State() == MISSION_INSPECT_CORRAL);
    }
    //    5c. P1-3: Opponent alert + clean FAR in same drain MUST inhibit pre-match competition launch!
    {
        Harness h;
        h.in.hardware_ready = h.in.can_launch = h.in.wheels_still = h.in.gate_settled = h.in.camera_settled = true;
        assert(h.m.On_Set_Config({RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION, 1U}));
        h.far(TARGET_GREEN_MATERIAL, VIS_FLAG_OPPONENT_ZONE_NEAR, 500U);
        h.far(TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 500U);
        h.in.button_pressed = true;
        const auto out = h.tick();
        assert(out.drive_action == DRIVE_ACTION_IDLE && out.launch_rejected_cue);
        assert(!h.m.Is_Config_Locked());
    }
    //    5d. P1-4: Same-drain REAR_CLEAR=0 -> REAR_CLEAR=1 during unproven reverse MUST emergency-stop + standstill gate!
    {
        Harness h;
        h.to_pushing();
        h.in.chassis_motion_done = true;
        h.near(0U, (aligned & ~VIS_FLAG_DROP_BASELINE_READY) | VIS_FLAG_REAR_CLEAR);
        assert(h.tick().distance_m < 0.0f);
        h.in.chassis_motion_done = false;
        h.in.chassis_active = true;
        h.in.wheels_still = false;
        h.in.use_motor_feedback_seq = true;
        h.near(0U, aligned & ~VIS_FLAG_DROP_BASELINE_READY);
        h.near(0U, (aligned & ~VIS_FLAG_DROP_BASELINE_READY) | VIS_FLAG_REAR_CLEAR);
        const auto out = h.tick();
        assert(out.drive_action == DRIVE_ACTION_EMERGENCY_STOP);
        assert(h.m.Is_Standstill_Gate_Pending_Boundary());
    }
    //    5e. P1-5: Push emergency-stop abort MUST engage standstill gate and block reverse/turn until fresh feedback!
    {
        Harness h;
        h.to_pushing();
        h.in.use_motor_feedback_seq = true;
        h.in.motor_feedback_seq[0] = h.in.motor_feedback_seq[1] = 30U;
        h.in.motor_feedback_ms[0] = h.in.motor_feedback_ms[1] = 100U;
        h.in.chassis_active = true;
        h.in.wheels_still = true;
        h.near(1U, aligned & ~VIS_FLAG_DROP_PATH_CLEAR);
        assert(h.tick().drive_action == DRIVE_ACTION_EMERGENCY_STOP);
        assert(h.m.Is_Standstill_Gate_Pending_Boundary());
        h.in.chassis_active = false;
        h.near(1U, aligned | VIS_FLAG_REAR_CLEAR);
        const auto blocked = h.tick();
        assert(blocked.drive_action == DRIVE_ACTION_EMERGENCY_STOP);
        const uint32_t bseq[2] = {30U, 30U};
        const uint32_t bms[2] = {100U, 100U};
        h.m.Latch_Standstill_Boundary(bseq, bms);
        // Same pre-stop feedback seq/ms -> still blocked!
        assert(h.tick().drive_action == DRIVE_ACTION_EMERGENCY_STOP);
        // Fresh post-stop standstill samples from BOTH motors -> releases cleanly!
        h.in.motor_feedback_seq[0] = h.in.motor_feedback_seq[1] = 31U;
        h.in.motor_feedback_ms[0] = h.in.motor_feedback_ms[1] = 101U;
        const auto released = h.tick();
        assert(released.drive_action == DRIVE_ACTION_START_DISTANCE && released.distance_m < 0.0f);
    }
    //    5f. P2-2: Frame accepted while standstill gate is waiting MUST NOT count toward post-release drop confirm!
    {
        Harness h;
        h.to_pushing();
        h.in.use_motor_feedback_seq = true;
        h.in.chassis_motion_done = true;
        h.near(0U, safe | VIS_FLAG_HOME_HALF_MATCH | VIS_FLAG_DROP_PATH_CLEAR | VIS_FLAG_REAR_CLEAR);
        assert(h.tick().distance_m < 0.0f);
        h.in.wheels_still = false;
        h.in.chassis_active = true;
        h.in.chassis_motion_done = false;
        h.near(0U, safe | VIS_FLAG_DROP_CONFIRMED);
        assert(h.tick().drive_action == DRIVE_ACTION_EMERGENCY_STOP);
        assert(h.m.Is_Standstill_Gate_Pending_Boundary());
        h.in.wheels_still = true;
        h.in.chassis_active = false;
        h.near(0U, safe | VIS_FLAG_DROP_CONFIRMED);
        assert(h.tick().drive_action == DRIVE_ACTION_EMERGENCY_STOP);
        const uint32_t boundary[2] = {1U, 1U};
        h.m.Latch_Standstill_Boundary(boundary, boundary);
        h.in.motor_feedback_seq[0] = h.in.motor_feedback_seq[1] = 2U;
        h.in.motor_feedback_ms[0] = h.in.motor_feedback_ms[1] = 1U;
        h.tick();
        assert(h.m.Get_Delivered_Green() == 0U);
        h.near(0U, safe | VIS_FLAG_DROP_CONFIRMED);
        h.tick();
        h.near(0U, safe | VIS_FLAG_DROP_CONFIRMED);
        h.tick();
        assert(h.m.Get_Delivered_Green() == 0U);
        h.near(0U, safe | VIS_FLAG_DROP_CONFIRMED);
        h.tick();
        assert(h.m.Get_Delivered_Green() == 1U);
    }

    // 4. P2-1: Interrupted confirmation streaks (unsettled camera without new frame, OR moving FAR)
    //    MUST NOT recount cached frames or preserve Confirm_Count!
    {
        Harness h;
        h.to_verifying();
        h.near(0U, safe | VIS_FLAG_DROP_CONFIRMED);
        h.tick();
        h.in.camera_settled = false;
        h.tick();
        h.in.camera_settled = true;
        h.tick(); // No new frame since unsettled edge -> MUST NOT recount cached frame!
        for (unsigned i = 0; i < 2U; ++i)
        {
            h.near(0U, safe | VIS_FLAG_DROP_CONFIRMED);
            h.tick();
        }
        assert(h.m.Get_Delivered_Green() == 0U);
        assert(!h.m.Is_First_Green_Unlocked());
        h.near(0U, safe | VIS_FLAG_DROP_CONFIRMED);
        h.tick();
        assert(h.m.Get_Delivered_Green() == 1U);
        assert(h.m.Is_First_Green_Unlocked());
    }
    {
        Harness h;
        h.to_verifying();
        for (unsigned i = 0; i < 2U; ++i)
        {
            h.near(0U, safe | VIS_FLAG_DROP_CONFIRMED);
            h.tick();
        }
        h.in.wheels_still = false;
        h.far(TARGET_NONE, 0U, 0U);
        h.tick();
        h.in.wheels_still = true;
        h.near(0U, safe | VIS_FLAG_DROP_CONFIRMED);
        h.tick();
        assert(h.m.Get_Delivered_Green() == 0U);
        assert(!h.m.Is_First_Green_Unlocked());
    }
}

int main()
{
    Test_Freshness_Nonconsecutive_Replay_Wrap_And_Restart();
    Test_Stale_Vision_And_Opponent_Interlock_During_Moving_States();
    Test_Cyan_Interrupts_Unsettled_Gate_Closure();
    Test_Full_First_Green_Delivery_And_Proven_Rear_Corridor();
    Test_Bounded_Recovery_Budget_And_Progress_Watchdog();
    Test_Cumulative_Yaw_Invalidates_Corridor_And_Active_Reverse_Stops();
    Test_Drop_Rechecks_Hazards_And_Alignment_While_Opening_And_Pushing();
    Test_Active_Reverse_Abrupt_Clearance_Loss_Cancels_Immediately_And_Waits_Standstill();
    Test_Same_Cargo_Reverify_After_Escape_Cannot_Bypass_Recovery_Budget();
    Test_Braking_Phase_Vision_Outage_And_Uart_Rx_Unhealthy();
    Test_Drop_Verify_Requires_Consecutive_Near_Frames();
    Test_Round12_Push_Same_Drain_Standstill_Bench_Leave_And_Interrupted_Drop();
    std::puts("PASS: rescue mission replay/wrap, stale/opponent interlock, gate/push hazard abort, margin reverse + standstill, braking outage gate, drop consecutive confirm, R12 safety invariants, bounded recovery");
    return 0;
}
