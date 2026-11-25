"""Unit tests for MaixCAM protocol (including C++ interop), vision rules, and touch UI."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import unittest

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from maixcam.config import RescueVisionConfig
from maixcam.protocol import (
    CAMERA_VIEW_FAR,
    CAMERA_VIEW_NEAR,
    CAMERA_VIEW_TRANSITION,
    Heartbeat,
    MODE_BENCH_STRAIGHT,
    MODE_COMPETITION,
    MatchConfigCmd,
    RescueProtocolParser,
    TARGET_BLACK_CORE,
    TARGET_GREEN_MATERIAL,
    TARGET_ORANGE_CASUALTY,
    TEAM_BLUE,
    TEAM_RED,
    TEL_FLAG_CONFIG_LOCKED,
    Telemetry,
    VIS_FLAG_CORRAL_SAFE,
    VIS_FLAG_DROP_CONFIRMED,
    VIS_FLAG_HOME_HALF_MATCH,
    VIS_FLAG_HOME_VALID,
    VIS_FLAG_MOTION_STALLED,
    VIS_FLAG_OPPONENT_ZONE_NEAR,
    VIS_FLAG_SWEEP_CLEAR,
    VIS_FLAG_TARGET_VALID,
    VisionState,
    decode_telemetry,
    decode_vision_state,
    encode_heartbeat,
    encode_set_config,
    encode_telemetry,
    encode_vision_state,
)
from maixcam.ui import TouchUIController
from maixcam.vision import Blob, RescueVisionProcessor


class MaixcamProtocolTest(unittest.TestCase):
    def test_stream_corruption_and_split_resync(self) -> None:
        parser = RescueProtocolParser()
        tel = Telemetry(
            uptime_ms=123456,
            match_elapsed_ms=45678,
            mission_state=5,
            team=TEAM_BLUE,
            start_zone=3,
            run_mode=MODE_COMPETITION,
            status_flags=TEL_FLAG_CONFIG_LOCKED,
            fault_flags=0,
            battery_mv=24150,
            yaw_cdeg=-1234,
            vx_mm_s=-85,
            omega_mrad_s=420,
            delivered_green=2,
            delivered_black=1,
            delivered_orange=1,
            max_materials_per_trip=2,
        )
        valid = encode_telemetry(77, tel)
        bad = bytearray(valid)
        bad[-1] ^= 0x55
        stream = b"\x00\xAA\xAA\x55\x01\x81\x00\x04\x12\x34" + bytes(bad) + valid

        frames = []
        for idx in range(0, len(stream), 3):
            frames.extend(parser.feed(stream[idx : idx + 3]))

        self.assertEqual(len(frames), 1)
        self.assertGreaterEqual(parser.bad_crc_frames, 1)
        decoded = decode_telemetry(frames[0])
        self.assertEqual(decoded, tel)

    def test_cross_language_cpp_python_frame_compatibility(self) -> None:
        if shutil.which("g++") is None:
            self.skipTest("g++ not available for C++ protocol interop probe")
        out_dir = REPO_ROOT / "build/native-tests"
        out_dir.mkdir(parents=True, exist_ok=True)
        probe_cpp = out_dir / "proto_interop_probe.cpp"
        probe_exe = out_dir / ("proto_interop_probe.exe" if os.name == "nt" else "proto_interop_probe")
        probe_cpp.write_text(
            r'''
#include "1_Middleware/Protocol/rescue_protocol.h"
#include <cstdio>
#include <cstdint>

static void PrintHex(const uint8_t *buf, size_t n) {
    for (size_t i = 0; i < n; ++i) std::printf("%02x", buf[i]);
    std::printf("\n");
}

int main() {
    uint8_t buf[RESCUE_PROTO_MAX_FRAME];
    Struct_Vision_State vis{};
    vis.frame_seq = 513;
    vis.camera_view = CAMERA_VIEW_NEAR;
    vis.target_type = TARGET_ORANGE_CASUALTY;
    vis.flags = VIS_FLAG_TARGET_VALID | VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR;
    vis.target_err_x_mrad = -245;
    vis.target_dist_mm = 310;
    vis.home_err_x_mrad = 180;
    vis.home_dist_mm = 1120;
    vis.corral_green_cnt = 1;
    vis.corral_black_cnt = 2;
    vis.corral_orange_cnt = 0;
    vis.corral_cyan_cnt = 0;
    vis.corral_unknown_cnt = 0;
    vis.confidence = 94;
    PrintHex(buf, Rescue_Encode_Vision_State(19, vis, buf, sizeof(buf)));

    Struct_Rescue_Telemetry tel{};
    tel.uptime_ms = 987654;
    tel.match_elapsed_ms = 12345;
    tel.mission_state = 9;
    tel.team = RESCUE_TEAM_RED;
    tel.start_zone = 4;
    tel.run_mode = RESCUE_MODE_COMPETITION;
    tel.status_flags = TEL_FLAG_CONFIG_LOCKED | TEL_FLAG_FIRST_GREEN_UNLOCKED;
    tel.fault_flags = 0;
    tel.battery_mv = 23980;
    tel.yaw_cdeg = -9050;
    tel.vx_mm_s = 115;
    tel.omega_mrad_s = -330;
    tel.delivered_green = 3;
    tel.delivered_black = 1;
    tel.delivered_orange = 2;
    tel.max_materials_per_trip = 3;
    PrintHex(buf, Rescue_Encode_Telemetry(88, tel, buf, sizeof(buf)));
    return 0;
}
''',
            encoding="utf-8",
        )
        cmd = [
            "g++",
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(REPO_ROOT / "User_File"),
            str(probe_cpp),
            str(REPO_ROOT / "User_File/1_Middleware/Protocol/rescue_protocol.cpp"),
            "-o",
            str(probe_exe),
        ]
        if os.name == "nt":
            cmd.append("-static")
        subprocess.run(cmd, check=True)
        lines = subprocess.check_output([str(probe_exe)], text=True).strip().splitlines()
        self.assertEqual(len(lines), 2)

        cpp_vis_bytes = bytes.fromhex(lines[0])
        cpp_tel_bytes = bytes.fromhex(lines[1])

        py_vis_bytes = encode_vision_state(
            19,
            VisionState(
                frame_seq=513,
                camera_view=CAMERA_VIEW_NEAR,
                target_type=TARGET_ORANGE_CASUALTY,
                flags=VIS_FLAG_TARGET_VALID | VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR,
                target_err_x_mrad=-245,
                target_dist_mm=310,
                home_err_x_mrad=180,
                home_dist_mm=1120,
                corral_green_cnt=1,
                corral_black_cnt=2,
                corral_orange_cnt=0,
                corral_cyan_cnt=0,
                corral_unknown_cnt=0,
                confidence=94,
            ),
        )
        py_tel_bytes = encode_telemetry(
            88,
            Telemetry(
                uptime_ms=987654,
                match_elapsed_ms=12345,
                mission_state=9,
                team=TEAM_RED,
                start_zone=4,
                run_mode=MODE_COMPETITION,
                status_flags=TEL_FLAG_CONFIG_LOCKED | (1 << 5),
                fault_flags=0,
                battery_mv=23980,
                yaw_cdeg=-9050,
                vx_mm_s=115,
                omega_mrad_s=-330,
                delivered_green=3,
                delivered_black=1,
                delivered_orange=2,
                max_materials_per_trip=3,
            ),
        )
        self.assertEqual(cpp_vis_bytes, py_vis_bytes)
        self.assertEqual(cpp_tel_bytes, py_tel_bytes)

        # Also verify Python parser decodes C++ bytes cleanly.
        parser = RescueProtocolParser()
        decoded_frames = parser.feed(cpp_vis_bytes + cpp_tel_bytes)
        self.assertEqual(len(decoded_frames), 2)
        self.assertEqual(decode_vision_state(decoded_frames[0]).target_err_x_mrad, -245)
        self.assertEqual(decode_telemetry(decoded_frames[1]).yaw_cdeg, -9050)


class MaixcamVisionAndUITest(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = RescueVisionConfig()
        self.proc = RescueVisionProcessor(self.cfg)

    def test_far_view_first_green_and_post_unlock_priority_and_cyan_exclusion(self) -> None:
        blobs = [
            Blob("cyan", 145, 130, 20, 20, 350),
            Blob("orange", 130, 120, 30, 20, 500),
            Blob("black", 180, 125, 22, 22, 320, density=0.65),
            Blob("green", 80, 110, 22, 22, 420),
        ]
        # Before first green unlock: must select ONLY green (left of center -> positive bearing).
        st1 = self.proc.process(blobs, camera_near_cmd=False, camera_settled=True, team=TEAM_RED, first_green_unlocked=False)
        self.assertEqual(st1.camera_view, CAMERA_VIEW_FAR)
        self.assertTrue(st1.flags & VIS_FLAG_TARGET_VALID)
        self.assertEqual(st1.target_type, TARGET_GREEN_MATERIAL)
        self.assertGreater(st1.target_err_x_mrad, 0)

        # After unlock: prefers 15-pt orange casualty over black/green, never selects cyan.
        st2 = self.proc.process(blobs, camera_near_cmd=False, camera_settled=True, team=TEAM_RED, first_green_unlocked=True)
        self.assertEqual(st2.target_type, TARGET_ORANGE_CASUALTY)

        # Unsettled camera pitch reports TRANSITION so STM32 never trusts mid-slew frames.
        st_tr = self.proc.process(blobs, camera_near_cmd=True, camera_settled=False, team=TEAM_RED, first_green_unlocked=True)
        self.assertEqual(st_tr.camera_view, CAMERA_VIEW_TRANSITION)

    def test_near_corral_sweep_and_full_outline_drop_verification(self) -> None:
        # 1 green inside corral ROI + 1 cyan purely inside right-gate sweep ROI -> sweep_clear MUST be False!
        blobs_hazard_sweep = [
            Blob("green", 140, 150, 24, 24, 500),
            Blob("cyan", 240, 140, 24, 24, 500),
        ]
        st = self.proc.process(blobs_hazard_sweep, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False)
        self.assertEqual(st.corral_green_cnt, 1)
        self.assertEqual(st.corral_cyan_cnt, 0)
        self.assertTrue(st.flags & VIS_FLAG_CORRAL_SAFE)
        self.assertFalse(st.flags & VIS_FLAG_SWEEP_CLEAR)

        # Cyan inside corral ROI clears VIS_FLAG_CORRAL_SAFE.
        blobs_hazard_inside = [
            Blob("green", 140, 150, 24, 24, 500),
            Blob("cyan", 170, 150, 24, 24, 500),
        ]
        st_bad = self.proc.process(blobs_hazard_inside, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False)
        self.assertEqual(st_bad.corral_cyan_cnt, 1)
        self.assertFalse(st_bad.flags & VIS_FLAG_CORRAL_SAFE)

        # Block straddling purple ramp (y2 > ramp inner y=100) must NOT confirm drop!
        ramp = Blob("purple_ramp", 40, 100, 240, 30, 3000)
        straddling = [ramp, Blob("green", 140, 82, 24, 26, 520)]  # y2 = 108 > 100
        st_straddle = self.proc.process(straddling, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False)
        self.assertFalse(st_straddle.flags & VIS_FLAG_DROP_CONFIRMED)

        # Block completely past purple ramp inner edge (y2 = 90 <= 100) confirms drop!
        inside = [ramp, Blob("green", 140, 64, 24, 26, 520)]
        st_inside = self.proc.process(inside, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False)
        self.assertTrue(st_inside.flags & VIS_FLAG_DROP_CONFIRMED)

    def test_home_half_match_opponent_zone_and_visual_stall(self) -> None:
        # Red safe zone spans x=60..280 (divider at x=170). Center x=160 is within 18px margin -> NOT matched!
        # Shift zone right so center x=160 is clearly inside LEFT material half (x=80..300 -> divider=190, 190-18=172 >= 160).
        blobs = [
            Blob("red_zone", 80, 70, 220, 100, 12000),
            Blob("blue_zone", 10, 120, 60, 50, 2000),  # y2=170 >= opponent_zone_near_y (150)
        ]
        st_mat = self.proc.process(
            blobs,
            camera_near_cmd=False,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            carrying_casualty=False,
            wheels_moving=True,
            frame_motion_delta=0.4,
        )
        self.assertTrue(st_mat.flags & VIS_FLAG_HOME_VALID)
        self.assertTrue(st_mat.flags & VIS_FLAG_HOME_HALF_MATCH)
        self.assertTrue(st_mat.flags & VIS_FLAG_OPPONENT_ZONE_NEAR)
        self.assertTrue(st_mat.flags & VIS_FLAG_MOTION_STALLED)

        # Same alignment while carrying CASUALTY (needs RIGHT half) must report half_match = False!
        st_cas = self.proc.process(
            blobs,
            camera_near_cmd=False,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            carrying_casualty=True,
        )
        self.assertTrue(st_cas.flags & VIS_FLAG_HOME_VALID)
        self.assertFalse(st_cas.flags & VIS_FLAG_HOME_HALF_MATCH)

    def test_touch_ui_setup_and_lock(self) -> None:
        ui = TouchUIController(552, 368)
        tel_setup = Telemetry(status_flags=0)

        btn_blue = next(b for b in ui.buttons if b.button_id == "team_blue")
        btn_z3 = next(b for b in ui.buttons if b.button_id == "zone_3")
        btn_mode = next(b for b in ui.buttons if b.button_id == "mode_toggle")
        btn_gate = next(b for b in ui.buttons if b.button_id == "bench_gate")

        self.assertEqual(ui.handle_touch(btn_blue.x + 5, btn_blue.y + 5, True, tel_setup), "team_blue")
        ui.handle_touch(0, 0, False, tel_setup)
        self.assertEqual(ui.config.team, TEAM_BLUE)

        self.assertEqual(ui.handle_touch(btn_z3.x + 5, btn_z3.y + 5, True, tel_setup), "zone_3")
        ui.handle_touch(0, 0, False, tel_setup)
        self.assertEqual(ui.config.start_zone, 3)

        # Bench gate button ignored in COMPETITION mode, honored in BENCH mode.
        self.assertIsNone(ui.handle_touch(btn_gate.x + 5, btn_gate.y + 5, True, tel_setup))
        ui.handle_touch(0, 0, False, tel_setup)
        self.assertEqual(ui.handle_touch(btn_mode.x + 5, btn_mode.y + 5, True, tel_setup), "mode_toggle")
        ui.handle_touch(0, 0, False, tel_setup)
        self.assertEqual(ui.config.run_mode, MODE_BENCH_STRAIGHT)
        self.assertEqual(ui.handle_touch(btn_gate.x + 5, btn_gate.y + 5, True, tel_setup), "bench_gate")
        ui.handle_touch(0, 0, False, tel_setup)
        self.assertIsNotNone(ui.pop_bench_cmd())

        # Once MC02 reports CONFIG_LOCKED, taps cannot mutate config!
        tel_locked = Telemetry(status_flags=TEL_FLAG_CONFIG_LOCKED)
        self.assertEqual(ui.handle_touch(btn_mode.x + 5, btn_mode.y + 5, True, tel_locked), "locked")
        self.assertEqual(ui.config.run_mode, MODE_BENCH_STRAIGHT)


if __name__ == "__main__":
    unittest.main()
