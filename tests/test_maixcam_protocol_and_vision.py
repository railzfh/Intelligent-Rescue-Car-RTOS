"""Unit tests for MaixCAM protocol (including C++ interop), vision rules, motion estimator, and touch UI."""
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
    MODE_BENCH_STRAIGHT,
    MODE_COMPETITION,
    RescueProtocolParser,
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
    VIS_FLAG_REAR_CLEAR,
    VIS_FLAG_SWEEP_CLEAR,
    VIS_FLAG_TARGET_VALID,
    VisionState,
    decode_telemetry,
    decode_vision_state,
    encode_telemetry,
    encode_vision_state,
)
from maixcam.ui import TouchUIController
from maixcam.vision import Blob, RescueVisionProcessor, SpatialMotionEstimator, classify_unknown_foreground


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
    tel.status_flags = TEL_FLAG_CONFIG_LOCKED | TEL_FLAG_FIRST_GREEN_UNLOCKED | TEL_FLAG_CARRYING_CASUALTY;
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
            "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
            "-I", str(REPO_ROOT / "User_File"),
            str(probe_cpp),
            str(REPO_ROOT / "User_File/1_Middleware/Protocol/rescue_protocol.cpp"),
            "-o", str(probe_exe),
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
                status_flags=TEL_FLAG_CONFIG_LOCKED | (1 << 5) | (1 << 12),
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

        parser = RescueProtocolParser()
        decoded_frames = parser.feed(cpp_vis_bytes + cpp_tel_bytes)
        self.assertEqual(len(decoded_frames), 2)
        self.assertEqual(decode_vision_state(decoded_frames[0]).target_err_x_mrad, -245)
        self.assertEqual(decode_telemetry(decoded_frames[1]).yaw_cdeg, -9050)


class MaixcamVisionAndUITest(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = RescueVisionConfig()
        self.proc = RescueVisionProcessor(self.cfg)

    def test_geometry_validation_reaches_capture_distance_and_rejects_unreachable(self) -> None:
        # Default config must comfortably reach <= 210 mm capture distance.
        self.assertLess(self.cfg.min_reachable_far_distance_mm(), 210)
        self.assertLessEqual(self.proc.estimate_far_distance_mm(200), 200)

        bad_cfg = RescueVisionConfig(far_horizon_y=50, far_dist_k=42000.0)
        with self.assertRaises(ValueError):
            bad_cfg.validate(target_capture_dist_mm=210, home_drop_dist_mm=230)

    def test_far_view_first_green_priority_and_rear_clear_defaults_false(self) -> None:
        blobs = [
            Blob("cyan", 145, 130, 20, 20, 350),
            Blob("orange", 130, 120, 30, 20, 500),
            Blob("black", 180, 125, 22, 22, 320, density=0.65),
            Blob("green", 80, 110, 22, 22, 420),
        ]
        st1 = self.proc.process(blobs, camera_near_cmd=False, camera_settled=True, team=TEAM_RED, first_green_unlocked=False)
        self.assertEqual(st1.camera_view, CAMERA_VIEW_FAR)
        self.assertTrue(st1.flags & VIS_FLAG_TARGET_VALID)
        self.assertEqual(st1.target_type, TARGET_GREEN_MATERIAL)
        self.assertGreater(st1.target_err_x_mrad, 0)
        # Rear clearance must NEVER be fabricated without explicit rear sensor input.
        self.assertFalse(st1.flags & VIS_FLAG_REAR_CLEAR)

        st2 = self.proc.process(blobs, camera_near_cmd=False, camera_settled=True, team=TEAM_RED, first_green_unlocked=True)
        self.assertEqual(st2.target_type, TARGET_ORANGE_CASUALTY)

        st_tr = self.proc.process(blobs, camera_near_cmd=True, camera_settled=False, team=TEAM_RED, first_green_unlocked=True)
        self.assertEqual(st_tr.camera_view, CAMERA_VIEW_TRANSITION)

    def test_fail_closed_oversized_cyan_and_unknown_in_near_corral(self) -> None:
        # Adversarial probe: 10,000-pixel close-up cyan blob overlapping corral MUST NOT be discarded!
        big_cyan = [Blob("cyan", 120, 130, 110, 100, 10000)]
        st_cyan = self.proc.process(big_cyan, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False)
        self.assertEqual(st_cyan.corral_cyan_cnt, 1)
        self.assertFalse(st_cyan.flags & VIS_FLAG_CORRAL_SAFE)

        # Oversized merged green blob (> max_block_pixels) must fail closed as unknown, NOT be ignored!
        big_green = [Blob("green", 120, 130, 110, 100, 10500)]
        st_unk = self.proc.process(big_green, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False)
        self.assertEqual(st_unk.corral_unknown_cnt, 1)
        self.assertFalse(st_unk.flags & VIS_FLAG_CORRAL_SAFE)

        # Pure sweep-zone hazard clears SWEEP_CLEAR while leaving CORRAL_SAFE intact.
        sweep_hazard = [Blob("green", 140, 150, 24, 24, 500), Blob("cyan", 240, 140, 24, 24, 500)]
        st_sw = self.proc.process(sweep_hazard, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False)
        self.assertTrue(st_sw.flags & VIS_FLAG_CORRAL_SAFE)
        self.assertFalse(st_sw.flags & VIS_FLAG_SWEEP_CLEAR)

    def test_strict_drop_verification_requires_zone_ramp_and_full_correct_half_outline(self) -> None:
        # Red safe zone x=40..280 (cx=160); material half LEFT => x in [40, 142], casualty half RIGHT => x in [178, 280].
        home = Blob("red_zone", 40, 20, 240, 100, 15000)
        ramp = Blob("purple_ramp", 40, 100, 240, 26, 3000)

        # 1. No zone / no ramp visible -> MUST NOT confirm drop!
        no_zone = [Blob("green", 80, 50, 24, 24, 500)]
        st0 = self.proc.process(no_zone, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False)
        self.assertFalse(st0.flags & VIS_FLAG_DROP_CONFIRMED)

        # 2. Block straddling the 20 mm center divider (x=135..165 crosses 142) -> MUST NOT confirm!
        divider_cross = [home, ramp, Blob("green", 135, 50, 30, 24, 600)]
        st_div = self.proc.process(divider_cross, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False)
        self.assertFalse(st_div.flags & VIS_FLAG_DROP_CONFIRMED)

        # 3. Block straddling purple ramp (y2 = 108 > ramp.y = 100) -> MUST NOT confirm!
        straddle = [home, ramp, Blob("green", 80, 82, 24, 26, 520)]
        st_str = self.proc.process(straddle, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False)
        self.assertFalse(st_str.flags & VIS_FLAG_DROP_CONFIRMED)

        # 4. Two green blocks before first-green unlock -> MUST NOT confirm!
        two_green = [home, ramp, Blob("green", 60, 50, 24, 24, 500), Blob("green", 100, 50, 24, 24, 500)]
        st_two = self.proc.process(two_green, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False)
        self.assertFalse(st_two.flags & VIS_FLAG_DROP_CONFIRMED)

        # 5. Old green already in safe zone while current green is STILL inside corral -> MUST NOT confirm!
        still_in_corral = [home, ramp, Blob("green", 80, 50, 24, 24, 500), Blob("green", 140, 150, 24, 24, 500)]
        st_still = self.proc.process(still_in_corral, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False)
        self.assertFalse(st_still.flags & VIS_FLAG_DROP_CONFIRMED)

        # 6. Exactly 1 green block completely inside own MATERIAL half past ramp & corral empty -> CONFIRMED!
        valid_first = [home, ramp, Blob("green", 80, 50, 24, 24, 500)]
        st_ok = self.proc.process(valid_first, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False)
        self.assertTrue(st_ok.flags & VIS_FLAG_DROP_CONFIRMED)

        # 7. Orange casualty in MATERIAL (left) half rejected; in CASUALTY (right) half confirmed!
        cas_wrong = [home, ramp, Blob("orange", 80, 50, 36, 24, 700)]
        st_cw = self.proc.process(cas_wrong, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=True, carrying_casualty=True)
        self.assertFalse(st_cw.flags & VIS_FLAG_DROP_CONFIRMED)

        cas_right = [home, ramp, Blob("orange", 200, 50, 36, 24, 700)]
        st_cr = self.proc.process(cas_right, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=True, carrying_casualty=True)
        self.assertTrue(st_cr.flags & VIS_FLAG_DROP_CONFIRMED)

    def test_unknown_foreground_not_masked_by_edge_contact_or_zone(self) -> None:
        # Unknown object barely overlapping a green block (~12% area) and fully inside red_zone MUST be flagged unknown!
        known = [
            Blob("green", 100, 140, 24, 24, 500),
            Blob("red_zone", 80, 120, 160, 100, 12000),
        ]
        fg = [
            Blob("unknown", 100, 140, 24, 24, 500),  # 100% covered by green -> NOT unknown
            Blob("unknown", 121, 140, 24, 24, 500),  # only 3x24 overlap (12.5%) -> MUST stay unknown!
        ]
        unknowns = classify_unknown_foreground(fg, known, min_pixels=60)
        self.assertEqual(len(unknowns), 1)
        self.assertEqual(unknowns[0].x, 121)

    def test_home_half_match_checks_full_corral_width_and_opponent_zone(self) -> None:
        # Home x=100..320 (cx=210, left material half=[100, 192]).
        # Center=160, corral_align_half_w_px=24 => footprint [136, 184] strictly inside [100, 192]!
        blobs = [
            Blob("red_zone", 100, 70, 220, 100, 12000),
            Blob("blue_zone", 10, 120, 60, 50, 2000),
        ]
        st_mat = self.proc.process(
            blobs,
            camera_near_cmd=False,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            carrying_casualty=False,
            motion_stalled=True,
        )
        self.assertTrue(st_mat.flags & VIS_FLAG_HOME_VALID)
        self.assertTrue(st_mat.flags & VIS_FLAG_HOME_HALF_MATCH)
        self.assertTrue(st_mat.flags & VIS_FLAG_OPPONENT_ZONE_NEAR)
        self.assertTrue(st_mat.flags & VIS_FLAG_MOTION_STALLED)

        # Same alignment while carrying CASUALTY (needs RIGHT half) must report half_match = False!
        st_cas = self.proc.process(
            blobs, camera_near_cmd=False, camera_settled=True, team=TEAM_RED, first_green_unlocked=True, carrying_casualty=True
        )
        self.assertFalse(st_cas.flags & VIS_FLAG_HOME_HALF_MATCH)

    def test_spatial_motion_estimator_rejects_exposure_and_detects_constant_mean_motion(self) -> None:
        est = SpatialMotionEstimator(width=16, height=12)

        def make_stripe_grid(shift_x: int, brightness_offset: float = 0.0) -> list[float]:
            # Periodic textured pattern whose global mean is invariant under horizontal shift!
            grid = []
            for y in range(12):
                for x in range(16):
                    val = 160.0 if ((x + shift_x) % 4) < 2 else 60.0
                    if (y % 4) < 2:
                        val += 20.0
                    grid.append(val + brightness_offset)
            return grid

        # Case 1: Constant-mean scrolling texture while moving -> MUST NOT report stalled!
        g0 = make_stripe_grid(0)
        g1 = make_stripe_grid(1)  # 1 cell shift in 0.1 s = 10 cells/s (> 3.0)
        self.assertEqual(sum(g0), sum(g1))  # identical global mean brightness!
        est.update(g0, 1.0, wheels_moving=True)
        stalled_moving, speed = est.update(g1, 1.1, wheels_moving=True)
        self.assertFalse(stalled_moving)
        self.assertGreaterEqual(speed, 9.0)

        # Case 2: Stationary scene with +35 global exposure shift while wheels spin -> MUST detect stall!
        est.reset()
        est.update(make_stripe_grid(0, 0.0), 2.0, wheels_moving=True)
        stalled_exp, _ = est.update(make_stripe_grid(0, 35.0), 2.1, wheels_moving=True)
        self.assertTrue(stalled_exp)

        # Case 3: Stationary periodic checkerboard MUST report 0.0 cells/s and stalled=True (never 42.43 cells/s alias!).
        est.reset()
        checker = [180.0 if ((x + y) % 2 == 0) else 40.0 for y in range(12) for x in range(16)]
        est.update(checker, 2.5, wheels_moving=True)
        stalled_chk, speed_chk = est.update(checker, 2.6, wheels_moving=True)
        self.assertTrue(stalled_chk)
        self.assertEqual(speed_chk, 0.0)

        # Case 4: Blank textureless scene -> confidence gate returns False (no false stall).
        est.reset()
        blank = [120.0] * (16 * 12)
        est.update(blank, 3.0, wheels_moving=True)
        stalled_blank, _ = est.update(blank, 3.1, wheels_moving=True)
        self.assertFalse(stalled_blank)

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

        self.assertIsNone(ui.handle_touch(btn_gate.x + 5, btn_gate.y + 5, True, tel_setup))
        ui.handle_touch(0, 0, False, tel_setup)
        self.assertEqual(ui.handle_touch(btn_mode.x + 5, btn_mode.y + 5, True, tel_setup), "mode_toggle")
        ui.handle_touch(0, 0, False, tel_setup)
        self.assertEqual(ui.config.run_mode, MODE_BENCH_STRAIGHT)
        self.assertEqual(ui.handle_touch(btn_gate.x + 5, btn_gate.y + 5, True, tel_setup), "bench_gate")
        ui.handle_touch(0, 0, False, tel_setup)
        self.assertIsNotNone(ui.pop_bench_cmd())

        tel_locked = Telemetry(status_flags=TEL_FLAG_CONFIG_LOCKED)
        self.assertEqual(ui.handle_touch(btn_mode.x + 5, btn_mode.y + 5, True, tel_locked), "locked")
        self.assertEqual(ui.config.run_mode, MODE_BENCH_STRAIGHT)


if __name__ == "__main__":
    unittest.main()
