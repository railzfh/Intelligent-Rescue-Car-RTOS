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

static void Test_Freshness_Config_Lock_And_Launch_Gates()
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

    // Select RED team + zone 2.
    assert(mission.On_Set_Config({RESCUE_TEAM_RED, 2U, RESCUE_MODE_COMPETITION, 2U}));
    // Heartbeat alone must NOT satisfy vision freshness!
    mission.On_Heartbeat({1000U, 1U, 0U, 0U}, 110U);
    assert(mission.Is_Link_Alive(110U));
    assert(!mission.Is_Vision_Fresh(110U));
    in.now_ms = 115U;
    out = mission.Update(in);
    assert(mission.Get_State() == MISSION_PRE_MATCH);
    assert(out.launch_rejected_cue);

    // Provide fresh vision frame seq=1, then launch.
    assert(mission.On_Vision_State(Make_Far_Vision(1U, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), 120U));
    // Duplicate frame_seq must be rejected and NOT refresh timestamp.
    assert(!mission.On_Vision_State(Make_Far_Vision(1U, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), 400U));
    assert(!mission.Is_Vision_Fresh(430U));
    // New frame_seq=2 restores freshness.
    assert(mission.On_Vision_State(Make_Far_Vision(2U, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID), 440U));
    assert(mission.Is_Vision_Fresh(450U));

    in.now_ms = 450U;
    in.button_pressed = true;
    out = mission.Update(in);
    assert(mission.Get_State() == MISSION_LAUNCH_CROSS_BUMPS);
    assert(mission.Is_Config_Locked());
    assert(out.drive_action == DRIVE_ACTION_START_DISTANCE);
    assert(out.distance_m > 0.4f);

    // Config changes and bench commands MUST be rejected after launch.
    assert(!mission.On_Set_Config({RESCUE_TEAM_BLUE, 1U, RESCUE_MODE_COMPETITION, 3U}));
    assert(!mission.On_Bench_Actuator({BENCH_SERVO_POS_B, BENCH_SERVO_POS_B, 0U}));
    assert(mission.Get_Team() == RESCUE_TEAM_RED && mission.Get_Start_Zone() == 2U);
}

static void Test_Corral_Rules_And_First_Green_Unlock()
{
    Class_Rescue_Mission mission;
    assert(mission.On_Set_Config({RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION, 3U}));
    const uint16_t safe = VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR;

    // Before first green unlock: ONLY exactly 1 green is legal!
    assert(mission.Validate_Corral(Make_Near_Vision(1U, 1U, 0U, 0U, 0U, 0U, safe)) == CORRAL_VALID_MATERIALS);
    assert(mission.Validate_Corral(Make_Near_Vision(2U, 2U, 0U, 0U, 0U, 0U, safe)) == CORRAL_ILLEGAL_HAZARD_OR_EXCESS);
    assert(mission.Validate_Corral(Make_Near_Vision(3U, 0U, 1U, 0U, 0U, 0U, safe)) == CORRAL_ILLEGAL_HAZARD_OR_EXCESS);
    assert(mission.Validate_Corral(Make_Near_Vision(4U, 0U, 0U, 1U, 0U, 0U, safe)) == CORRAL_ILLEGAL_HAZARD_OR_EXCESS);
    assert(mission.Validate_Corral(Make_Near_Vision(5U, 1U, 0U, 0U, 1U, 0U, safe)) == CORRAL_ILLEGAL_HAZARD_OR_EXCESS);
    assert(mission.Validate_Corral(Make_Near_Vision(6U, 1U, 0U, 0U, 0U, 1U, safe)) == CORRAL_ILLEGAL_HAZARD_OR_EXCESS);
    assert(mission.Validate_Corral(Make_Near_Vision(7U, 1U, 0U, 0U, 0U, 0U, VIS_FLAG_CORRAL_SAFE)) ==
           CORRAL_ILLEGAL_HAZARD_OR_EXCESS); // sweep zone not clear!
    assert(!mission.Is_Target_Eligible(TARGET_ORANGE_CASUALTY));
    assert(!mission.Is_Target_Eligible(TARGET_BLACK_CORE));
    assert(!mission.Is_Target_Eligible(TARGET_CYAN_HAZARD));
    assert(mission.Is_Target_Eligible(TARGET_GREEN_MATERIAL));

    // Walk full first-green delivery cycle end-to-end.
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

    // Finish speed bumps.
    in.button_pressed = false;
    in.chassis_motion_done = true;
    in.now_ms = (t += 100U);
    mission.Update(in);
    assert(mission.Get_State() == MISSION_SEARCH_TARGET);

    // See green block -> approach -> capture distance -> inspect corral.
    in.chassis_motion_done = false;
    in.chassis_active = false;
    assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 20, 400U), t));
    in.now_ms = (t += 50U);
    mission.Update(in);
    assert(mission.Get_State() == MISSION_APPROACH_TARGET);

    assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 10, 190U), t));
    in.now_ms = (t += 50U);
    auto out = mission.Update(in);
    assert(mission.Get_State() == MISSION_INSPECT_CORRAL);
    assert(out.camera_near && !out.gate_closed);

    // 3 consecutive NEAR frames confirm exactly 1 green -> close gate.
    for (int k = 0; k < 3; ++k)
    {
        in.now_ms = (t += 50U);
        assert(mission.On_Vision_State(Make_Near_Vision(seq++, 1U, 0U, 0U, 0U, 0U, safe), t));
        out = mission.Update(in);
    }
    assert(mission.Get_State() == MISSION_CLOSE_GATE_VERIFY);
    assert(out.gate_closed && out.camera_near);

    // 3 consecutive NEAR frames after gate settled confirm block retained -> find home.
    for (int k = 0; k < 3; ++k)
    {
        in.now_ms = (t += 50U);
        assert(mission.On_Vision_State(Make_Near_Vision(seq++, 1U, 0U, 0U, 0U, 0U, safe), t));
        out = mission.Update(in);
    }
    assert(mission.Get_State() == MISSION_FIND_HOME);
    assert(out.gate_closed && !out.camera_near);

    // Find home -> approach own material half -> refuse drop if half mismatches!
    in.now_ms = (t += 50U);
    assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_NONE, VIS_FLAG_HOME_VALID, 0, 500U), t));
    mission.Update(in);
    assert(mission.Get_State() == MISSION_APPROACH_HOME);

    // Close to ramp but WRONG half (no VIS_FLAG_HOME_HALF_MATCH): must NOT enter drop!
    in.now_ms = (t += 50U);
    assert(mission.On_Vision_State(Make_Far_Vision(seq++, TARGET_NONE, VIS_FLAG_HOME_VALID, 0, 200U), t));
    out = mission.Update(in);
    assert(mission.Get_State() == MISSION_APPROACH_HOME);
    assert(out.linear_mps == 0.0f);

    // Correct half matched -> enter drop open & push.
    in.now_ms = (t += 50U);
    assert(mission.On_Vision_State(
        Make_Far_Vision(seq++, TARGET_NONE, VIS_FLAG_HOME_VALID | VIS_FLAG_HOME_HALF_MATCH, 0, 200U), t));
    mission.Update(in);
    assert(mission.Get_State() == MISSION_DROP_OPEN_AND_PUSH);

    // 3 near frames verify legal cargo + half match -> gate opens -> gate settled -> push forward.
    for (int k = 0; k < 3; ++k)
    {
        in.now_ms = (t += 50U);
        assert(mission.On_Vision_State(
            Make_Near_Vision(seq++, 1U, 0U, 0U, 0U, 0U, safe | VIS_FLAG_HOME_HALF_MATCH), t));
        out = mission.Update(in);
    }
    assert(!out.gate_closed);
    in.now_ms = (t += 50U);
    out = mission.Update(in);
    assert(out.drive_action == DRIVE_ACTION_START_DISTANCE && out.distance_m > 0.0f);

    // Push complete -> reverse backout.
    in.chassis_motion_done = true;
    in.now_ms = (t += 100U);
    out = mission.Update(in);
    assert(mission.Get_State() == MISSION_DROP_BACKOUT_VERIFY);
    assert(out.drive_action == DRIVE_ACTION_START_DISTANCE && out.distance_m < 0.0f);

    // Backout complete -> verify full drop inside safe zone on 3 frames -> UNLOCKED!
    in.now_ms = (t += 100U);
    mission.Update(in);
    in.chassis_motion_done = false;
    assert(!mission.Is_First_Green_Unlocked());
    for (int k = 0; k < 3; ++k)
    {
        in.now_ms = (t += 50U);
        assert(mission.On_Vision_State(Make_Near_Vision(seq++, 0U, 0U, 0U, 0U, 0U, VIS_FLAG_DROP_CONFIRMED), t));
        mission.Update(in);
    }
    assert(mission.Is_First_Green_Unlocked());
    assert(mission.Get_Delivered_Green() == 1U);
    assert(mission.Get_State() == MISSION_SEARCH_TARGET);

    // Post-unlock rules: single orange OK, orange+green ILLEGAL, 3 mixed materials OK, 4 ILLEGAL.
    assert(mission.Validate_Corral(Make_Near_Vision(seq++, 0U, 0U, 1U, 0U, 0U, safe)) == CORRAL_VALID_CASUALTY);
    assert(mission.Validate_Corral(Make_Near_Vision(seq++, 1U, 0U, 1U, 0U, 0U, safe)) == CORRAL_ILLEGAL_HAZARD_OR_EXCESS);
    assert(mission.Validate_Corral(Make_Near_Vision(seq++, 2U, 1U, 0U, 0U, 0U, safe)) == CORRAL_VALID_MATERIALS);
    assert(mission.Validate_Corral(Make_Near_Vision(seq++, 2U, 2U, 0U, 0U, 0U, safe)) == CORRAL_ILLEGAL_HAZARD_OR_EXCESS);
    assert(mission.Validate_Corral(Make_Near_Vision(seq++, 1U, 0U, 0U, 1U, 0U, safe)) == CORRAL_ILLEGAL_HAZARD_OR_EXCESS);
}

static void Test_Stall_Escape_And_3Min_Timeout_And_Fault()
{
    Class_Rescue_Mission mission;
    assert(mission.On_Set_Config({RESCUE_TEAM_BLUE, 4U, RESCUE_MODE_COMPETITION, 1U}));
    assert(mission.On_Vision_State(Make_Far_Vision(1U, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID | VIS_FLAG_REAR_CLEAR), 100U));

    Struct_Mission_Inputs in{};
    in.now_ms = 100U;
    in.hardware_ready = true;
    in.can_launch = true;
    in.button_pressed = true;
    in.gate_settled = true;
    in.camera_settled = true;
    mission.Update(in);
    assert(mission.Get_State() == MISSION_LAUNCH_CROSS_BUMPS);

    // Stall during bump crossing triggers immediate bounded escape.
    in.button_pressed = false;
    in.chassis_active = true;
    in.chassis_stalled = true;
    in.now_ms = 200U;
    auto out = mission.Update(in);
    assert(mission.Get_State() == MISSION_ESCAPE_RECOVERY);
    assert(out.drive_action == DRIVE_ACTION_START_DISTANCE && out.distance_m < 0.0f);

    // Finish reverse -> starts turn -> finish turn -> returns to search.
    in.chassis_stalled = false;
    in.chassis_motion_done = true;
    in.now_ms = 400U;
    out = mission.Update(in);
    assert(out.drive_action == DRIVE_ACTION_START_TURN);
    in.now_ms = 600U;
    mission.Update(in);
    assert(mission.Get_State() == MISSION_SEARCH_TARGET);

    // 3-minute round timeout stops cleanly.
    in.now_ms = 100U + 180000U;
    out = mission.Update(in);
    assert(mission.Get_State() == MISSION_MATCH_FINISHED);
    assert(out.drive_action == DRIVE_ACTION_SOFT_STOP);
}

int main()
{
    Test_Freshness_Config_Lock_And_Launch_Gates();
    Test_Corral_Rules_And_First_Green_Unlock();
    Test_Stall_Escape_And_3Min_Timeout_And_Fault();
    std::puts("PASS: rescue mission launch gates, first-green rule, hazard/half rejection, escape, timer");
}
