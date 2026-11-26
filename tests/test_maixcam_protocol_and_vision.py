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
    MISSION_DROP_BACKOUT_VERIFY,
    MISSION_DROP_OPEN_AND_PUSH,
    MISSION_SEARCH_TARGET,
    MODE_BENCH_STRAIGHT,
    MODE_COMPETITION,
    RescueProtocolParser,
    TARGET_GREEN_MATERIAL,
    TARGET_ORANGE_CASUALTY,
    TEAM_BLUE,
    TEAM_RED,
    TEL_FLAG_CONFIG_LOCKED,
    TEL_FLAG_DROP_PUSHING,
    Telemetry,
    VIS_FLAG_CORRAL_SAFE,
    VIS_FLAG_DROP_BASELINE_READY,
    VIS_FLAG_DROP_CONFIRMED,
    VIS_FLAG_DROP_PATH_CLEAR,
    VIS_FLAG_GATE_OPEN_SETTLED_ACK,
    VIS_FLAG_HOME_HALF_MATCH,
    VIS_FLAG_HOME_VALID,
    VIS_FLAG_MOTION_STALLED,
    VIS_FLAG_OPPONENT_NEAR,
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
from maixcam.vision import (
    Blob,
    RescueVisionProcessor,
    SpatialMotionEstimator,
    classify_unknown_foreground,
    extract_blobs_from_maix_image,
)


class _FakeMaixBlob:
    def __init__(self, x: int, y: int, w: int, h: int, pixels: int) -> None:
        self._x, self._y, self._w, self._h, self._px = x, y, w, h, pixels

    def x(self) -> int:
        return self._x

    def y(self) -> int:
        return self._y

    def w(self) -> int:
        return self._w

    def h(self) -> int:
        return self._h

    def pixels(self) -> int:
        return self._px


class _FakeMaixImage:
    """Exact LAB pixel-grid image implementing MaixPy find_blobs() for extraction-path tests."""

    def __init__(self, width: int, height: int, fill_lab: tuple[int, int, int]) -> None:
        self.width = width
        self.height = height
        self._grid = [[fill_lab for _ in range(width)] for _ in range(height)]

    def fill_rect(self, x: int, y: int, w: int, h: int, lab: tuple[int, int, int]) -> None:
        for yy in range(max(0, y), min(self.height, y + h)):
            row = self._grid[yy]
            for xx in range(max(0, x), min(self.width, x + w)):
                row[xx] = lab

    def get_lab_pixel(self, x: int, y: int) -> tuple[int, int, int]:
        return self._grid[y][x]

    def find_blobs(
        self,
        thresholds: list[tuple[int, int, int, int, int, int]],
        roi: tuple[int, int, int, int] | None = None,
        x_stride: int = 1,
        y_stride: int = 1,
        pixels_threshold: int = 1,
        area_threshold: int = 1,
        invert: bool = False,
        merge: bool = False,
    ) -> list[_FakeMaixBlob]:
        _ = (x_stride, y_stride)
        rx, ry, rw, rh = roi if roi is not None else (0, 0, self.width, self.height)
        x1, y1 = max(0, rx), max(0, ry)
        x2, y2 = min(self.width, rx + rw), min(self.height, ry + rh)
        t = thresholds[0]

        def matches(px: tuple[int, int, int]) -> bool:
            inside = (t[0] <= px[0] <= t[1]) and (t[2] <= px[1] <= t[3]) and (t[4] <= px[2] <= t[5])
            return (not inside) if invert else inside

        visited = [[False] * (x2 - x1) for _ in range(y2 - y1)]
        comps: list[tuple[int, int, int, int, int]] = []
        for yy in range(y1, y2):
            for xx in range(x1, x2):
                if visited[yy - y1][xx - x1] or not matches(self._grid[yy][xx]):
                    continue
                stack = [(xx, yy)]
                visited[yy - y1][xx - x1] = True
                min_x = max_x = xx
                min_y = max_y = yy
                cnt = 0
                while stack:
                    cx, cy = stack.pop()
                    cnt += 1
                    if cx < min_x:
                        min_x = cx
                    if cx > max_x:
                        max_x = cx
                    if cy < min_y:
                        min_y = cy
                    if cy > max_y:
                        max_y = cy
                    for nx, ny in ((cx - 1, cy), (cx + 1, cy), (cx, cy - 1), (cx, cy + 1)):
                        if x1 <= nx < x2 and y1 <= ny < y2 and not visited[ny - y1][nx - x1]:
                            if matches(self._grid[ny][nx]):
                                visited[ny - y1][nx - x1] = True
                                stack.append((nx, ny))
                bw = max_x - min_x + 1
                bh = max_y - min_y + 1
                comps.append((min_x, min_y, bw, bh, cnt))

        if merge and comps:
            mx1 = min(c[0] for c in comps)
            my1 = min(c[1] for c in comps)
            mx2 = max(c[0] + c[2] for c in comps)
            my2 = max(c[1] + c[3] for c in comps)
            total_px = sum(c[4] for c in comps)
            comps = [(mx1, my1, mx2 - mx1, my2 - my1, total_px)]

        out: list[_FakeMaixBlob] = []
        for bx, by, bw, bh, cnt in comps:
            if cnt >= pixels_threshold and (bw * bh) >= area_threshold:
                out.append(_FakeMaixBlob(bx, by, bw, bh, cnt))
        return out


class _FakeMaixBinaryImage:
    """Real-MaixPy-faithful Tier-2 Image mock: exposes ONLY find_blobs(), crop(), binary() (no pixel accessor!)."""

    def __init__(self, width: int, height: int, fill_lab: tuple[int, int, int]) -> None:
        self.width = width
        self.height = height
        # Deliberately NOT named _grid and NO get_lab_pixel() so extract_blobs_from_maix_image uses Tier 2!
        self._lab_rows = [[fill_lab for _ in range(width)] for _ in range(height)]

    def fill_rect(self, x: int, y: int, w: int, h: int, lab: tuple[int, int, int]) -> None:
        for yy in range(max(0, y), min(self.height, y + h)):
            row = self._lab_rows[yy]
            for xx in range(max(0, x), min(self.width, x + w)):
                row[xx] = lab

    def crop(self, x: int, y: int, w: int, h: int) -> "_FakeMaixBinaryImage":
        x1, y1 = max(0, x), max(0, y)
        x2, y2 = min(self.width, x + w), min(self.height, y + h)
        sub = _FakeMaixBinaryImage(max(1, x2 - x1), max(1, y2 - y1), (0, 0, 0))
        sub._lab_rows = [row[x1:x2] for row in self._lab_rows[y1:y2]]
        return sub

    def binary(
        self,
        thresholds: list[list[int] | tuple[int, int, int, int, int, int]],
        invert: bool = False,
        copy: bool = True,
    ) -> "_FakeMaixBinaryImage":
        white = (100, 0, 0)
        black = (0, 0, 0)
        out = _FakeMaixBinaryImage(self.width, self.height, black) if copy else self
        new_rows: list[list[tuple[int, int, int]]] = []
        for row in self._lab_rows:
            out_row: list[tuple[int, int, int]] = []
            for px in row:
                matched = any(
                    (t[0] <= px[0] <= t[1]) and (t[2] <= px[1] <= t[3]) and (t[4] <= px[2] <= t[5])
                    for t in thresholds
                )
                if invert:
                    matched = not matched
                out_row.append(white if matched else black)
            new_rows.append(out_row)
        out._lab_rows = new_rows
        return out

    def find_blobs(
        self,
        thresholds: list[tuple[int, int, int, int, int, int]],
        roi: tuple[int, int, int, int] | None = None,
        x_stride: int = 1,
        y_stride: int = 1,
        pixels_threshold: int = 1,
        area_threshold: int = 1,
        invert: bool = False,
        merge: bool = False,
    ) -> list[_FakeMaixBlob]:
        _ = (x_stride, y_stride)
        rx, ry, rw, rh = roi if roi is not None else (0, 0, self.width, self.height)
        x1, y1 = max(0, rx), max(0, ry)
        x2, y2 = min(self.width, rx + rw), min(self.height, ry + rh)
        t = thresholds[0]

        def matches(px: tuple[int, int, int]) -> bool:
            inside = (t[0] <= px[0] <= t[1]) and (t[2] <= px[1] <= t[3]) and (t[4] <= px[2] <= t[5])
            return (not inside) if invert else inside

        visited = [[False] * (x2 - x1) for _ in range(y2 - y1)]
        comps: list[tuple[int, int, int, int, int]] = []
        for yy in range(y1, y2):
            for xx in range(x1, x2):
                if visited[yy - y1][xx - x1] or not matches(self._lab_rows[yy][xx]):
                    continue
                stack = [(xx, yy)]
                visited[yy - y1][xx - x1] = True
                min_x = max_x = xx
                min_y = max_y = yy
                cnt = 0
                while stack:
                    cx, cy = stack.pop()
                    cnt += 1
                    if cx < min_x:
                        min_x = cx
                    if cx > max_x:
                        max_x = cx
                    if cy < min_y:
                        min_y = cy
                    if cy > max_y:
                        max_y = cy
                    for nx, ny in ((cx - 1, cy), (cx + 1, cy), (cx, cy - 1), (cx, cy + 1)):
                        if x1 <= nx < x2 and y1 <= ny < y2 and not visited[ny - y1][nx - x1]:
                            if matches(self._lab_rows[ny][nx]):
                                visited[ny - y1][nx - x1] = True
                                stack.append((nx, ny))
                bw = max_x - min_x + 1
                bh = max_y - min_y + 1
                if cnt >= pixels_threshold and (bw * bh) >= area_threshold:
                    comps.append((min_x, min_y, bw, bh, cnt))

        if merge and len(comps) > 1:
            # OpenMV/MaixPy merge=True (margin=0) merges overlapping bounding boxes.
            merged = list(comps)
            changed = True
            while changed:
                changed = False
                next_list: list[tuple[int, int, int, int, int]] = []
                while merged:
                    ax, ay, aw, ah, apx = merged.pop(0)
                    i = 0
                    while i < len(merged):
                        bx, by, bw, bh, bpx = merged[i]
                        if ax < bx + bw and ax + aw > bx and ay < by + bh and ay + ah > by:
                            nx1 = min(ax, bx)
                            ny1 = min(ay, by)
                            nx2 = max(ax + aw, bx + bw)
                            ny2 = max(ay + ah, by + bh)
                            ax, ay, aw, ah, apx = nx1, ny1, nx2 - nx1, ny2 - ny1, apx + bpx
                            merged.pop(i)
                            changed = True
                        else:
                            i += 1
                    next_list.append((ax, ay, aw, ah, apx))
                merged = next_list
            comps = merged

        return [_FakeMaixBlob(bx, by, bw, bh, cnt) for bx, by, bw, bh, cnt in comps]


class _FakeMaixNativeStrideImage(_FakeMaixBinaryImage):
    """Models Sipeed MaixPy / OpenMV SDK default x_stride=2, y_stride=1 alternating-column seed lattice."""

    def crop(self, *args) -> "_FakeMaixNativeStrideImage":
        res = super().crop(*args)
        res.__class__ = _FakeMaixNativeStrideImage
        return res

    def binary(self, *args, **kwargs) -> "_FakeMaixNativeStrideImage":
        res = super().binary(*args, **kwargs)
        res.__class__ = _FakeMaixNativeStrideImage
        return res

    def find_blobs(
        self,
        thresholds: list[tuple[int, int, int, int, int, int]],
        roi: tuple[int, int, int, int] | None = None,
        x_stride: int = 2,
        y_stride: int = 1,
        pixels_threshold: int = 1,
        area_threshold: int = 1,
        invert: bool = False,
        merge: bool = False,
    ) -> list[_FakeMaixBlob]:
        results = super().find_blobs(
            thresholds,
            roi=roi,
            x_stride=x_stride,
            y_stride=y_stride,
            pixels_threshold=pixels_threshold,
            area_threshold=area_threshold,
            invert=invert,
            merge=merge,
        )
        rx, ry = roi[:2] if roi is not None else (0, 0)
        t = thresholds[0]

        def matched(x: int, y: int) -> bool:
            px = self._lab_rows[y][x]
            inside = (t[0] <= px[0] <= t[1]) and (t[2] <= px[1] <= t[3]) and (t[4] <= px[2] <= t[5])
            return (not inside) if invert else inside

        return [
            b
            for b in results
            if any(
                (yy - ry) % y_stride == 0 and (xx - rx - yy) % x_stride == 0 and matched(xx, yy)
                for yy in range(b.y(), b.y() + b.h())
                for xx in range(b.x(), b.x() + b.w())
            )
        ]


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

    def test_strict_drop_verification_requires_baseline_zone_ramp_and_trip_delta(self) -> None:
        # Red safe zone x=40..280 (cx=160); material half LEFT => x in [40, 142], casualty half RIGHT => x in [178, 280].
        home = Blob("red_zone", 40, 20, 240, 100, 15000)
        ramp = Blob("purple_ramp", 40, 100, 240, 26, 3000)
        corral_green = Blob("green", 140, 150, 24, 24, 500)
        corral_black = Blob("black", 170, 150, 24, 24, 480, density=0.83)
        corral_orange = Blob("orange", 140, 150, 36, 24, 700)

        # 0. Without a pre-push baseline, drop confirmation MUST fail closed!
        st_no_base = self.proc.process(
            [home, ramp, Blob("green", 80, 50, 24, 24, 500)],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=False,
        )
        self.assertFalse(st_no_base.flags & VIS_FLAG_DROP_CONFIRMED)

        # Capture clean pre-push baseline for first-green trip (empty home, 1 green in corral).
        self.proc.process(
            [home, ramp, corral_green],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=False,
            mission_state=MISSION_DROP_OPEN_AND_PUSH,
            gate_closed=True,
        )

        # 1. No zone / no ramp visible -> MUST NOT confirm drop!
        st0 = self.proc.process(
            [Blob("green", 80, 50, 24, 24, 500)],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=False,
            mission_state=MISSION_DROP_BACKOUT_VERIFY,
        )
        self.assertFalse(st0.flags & VIS_FLAG_DROP_CONFIRMED)

        # 2. Block straddling the 20 mm center divider (x=135..165 crosses 142) -> MUST NOT confirm!
        st_div = self.proc.process(
            [home, ramp, Blob("green", 135, 50, 30, 24, 600)],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=False,
            mission_state=MISSION_DROP_BACKOUT_VERIFY,
        )
        self.assertFalse(st_div.flags & VIS_FLAG_DROP_CONFIRMED)

        # 3. Block straddling purple ramp (y2 = 108 > ramp.y = 100) -> MUST NOT confirm!
        st_str = self.proc.process(
            [home, ramp, Blob("green", 80, 82, 24, 26, 520)],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=False,
            mission_state=MISSION_DROP_BACKOUT_VERIFY,
        )
        self.assertFalse(st_str.flags & VIS_FLAG_DROP_CONFIRMED)

        # 4. Old green already in safe zone while current green is STILL inside corral -> MUST NOT confirm!
        st_still = self.proc.process(
            [home, ramp, Blob("green", 80, 50, 24, 24, 500), corral_green],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=False,
            mission_state=MISSION_DROP_BACKOUT_VERIFY,
        )
        self.assertFalse(st_still.flags & VIS_FLAG_DROP_CONFIRMED)

        # 5. Exactly +1 green inside own MATERIAL half past ramp & corral empty -> CONFIRMED!
        st_ok = self.proc.process(
            [home, ramp, Blob("green", 80, 50, 24, 24, 500)],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=False,
            mission_state=MISSION_DROP_BACKOUT_VERIFY,
        )
        self.assertTrue(st_ok.flags & VIS_FLAG_DROP_CONFIRMED)

        # Return to FAR / search clears previous trip baseline.
        self.proc.process(
            [],
            camera_near_cmd=False,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            mission_state=MISSION_SEARCH_TARGET,
        )

        # 6. Finding 6A: Prior green block already in material half + Trip 2's new green dropped OUTSIDE
        #    -> delta is (0, 0, 0) != (1, 0, 0) -> MUST NOT falsely confirm!
        prior_green = Blob("green", 60, 45, 24, 24, 500)
        self.proc.process(
            [home, ramp, prior_green, corral_green],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            carrying_casualty=False,
            max_materials_per_trip=2,
            mission_state=MISSION_DROP_OPEN_AND_PUSH,
            gate_closed=True,
        )
        st_missed = self.proc.process(
            [home, ramp, prior_green],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            carrying_casualty=False,
            max_materials_per_trip=2,
            mission_state=MISSION_DROP_BACKOUT_VERIFY,
        )
        self.assertFalse(st_missed.flags & VIS_FLAG_DROP_CONFIRMED)

        # Same trip when the new green ACTUALLY lands inside material half -> delta (+1, 0, 0) -> CONFIRMED!
        st_second_green = self.proc.process(
            [home, ramp, prior_green, Blob("green", 100, 50, 24, 24, 500)],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            carrying_casualty=False,
            max_materials_per_trip=2,
            mission_state=MISSION_DROP_BACKOUT_VERIFY,
        )
        self.assertTrue(st_second_green.flags & VIS_FLAG_DROP_CONFIRMED)

        # 7. Finding 6B: Prior green block sitting in MATERIAL (left) half MUST NOT block
        #    confirmation of a correctly placed orange casualty in CASUALTY (right) half!
        self.proc.reset_drop_baseline()
        self.proc.process(
            [home, ramp, prior_green, corral_orange],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            carrying_casualty=True,
            mission_state=MISSION_DROP_OPEN_AND_PUSH,
            gate_closed=True,
        )
        # Casualty dropped into WRONG (left) half -> REJECTED!
        st_cw = self.proc.process(
            [home, ramp, prior_green, Blob("orange", 95, 50, 36, 24, 700)],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            carrying_casualty=True,
            mission_state=MISSION_DROP_BACKOUT_VERIFY,
        )
        self.assertFalse(st_cw.flags & VIS_FLAG_DROP_CONFIRMED)
        # Casualty dropped cleanly into RIGHT half while prior_green stays in LEFT half -> CONFIRMED!
        st_cr = self.proc.process(
            [home, ramp, prior_green, Blob("orange", 200, 50, 36, 24, 700)],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            carrying_casualty=True,
            mission_state=MISSION_DROP_BACKOUT_VERIFY,
        )
        self.assertTrue(st_cr.flags & VIS_FLAG_DROP_CONFIRMED)

        # 8. Multi-block trip (1 green + 1 black): partial drop (only green) REJECTED; full (1g+1b) CONFIRMED!
        self.proc.reset_drop_baseline()
        self.proc.process(
            [home, ramp, prior_green, corral_green, corral_black],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            carrying_casualty=False,
            max_materials_per_trip=2,
            mission_state=MISSION_DROP_OPEN_AND_PUSH,
            gate_closed=True,
        )
        st_partial = self.proc.process(
            [home, ramp, prior_green, Blob("green", 100, 50, 24, 24, 500)],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            carrying_casualty=False,
            max_materials_per_trip=2,
            mission_state=MISSION_DROP_BACKOUT_VERIFY,
        )
        self.assertFalse(st_partial.flags & VIS_FLAG_DROP_CONFIRMED)
        st_both = self.proc.process(
            [home, ramp, prior_green, Blob("green", 100, 50, 24, 24, 500), Blob("black", 60, 72, 22, 22, 420, density=0.86)],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            carrying_casualty=False,
            max_materials_per_trip=2,
            mission_state=MISSION_DROP_BACKOUT_VERIFY,
        )
        self.assertTrue(st_both.flags & VIS_FLAG_DROP_CONFIRMED)

    def test_unknown_foreground_unique_pixels_and_extraction_path_surface_distinction(self) -> None:
        # 1. Edge-contacting unknown object inside red_zone MUST be flagged unknown.
        known = [
            Blob("green", 100, 140, 24, 24, 500),
            Blob("red_zone", 80, 120, 160, 100, 12000),
        ]
        fg = [
            Blob("unknown", 100, 140, 24, 24, 500),  # 100% covered by green -> NOT unknown
            Blob("unknown", 121, 140, 24, 24, 500),  # only 3x24 overlap -> MUST stay unknown!
        ]
        unknowns = classify_unknown_foreground(fg, known, min_pixels=60)
        self.assertEqual(len(unknowns), 1)
        self.assertEqual(unknowns[0].x, 121)

        # 2. Finding 5A: Two overlapping known boxes covering the SAME 40% of fg MUST NOT sum to 80%!
        overlap_known = [
            Blob("green", 100, 140, 20, 50, 1000),
            Blob("orange", 100, 140, 20, 50, 1000),
        ]
        overlap_fg = [Blob("unknown", 100, 140, 50, 50, 2500)]
        self.assertEqual(len(classify_unknown_foreground(overlap_fg, overlap_known, min_pixels=60, cfg=self.cfg)), 1)
        self.assertEqual(len(classify_unknown_foreground(overlap_fg, overlap_known, min_pixels=60)), 1)

        # 3. Finding 5B through the ACTUAL extract_blobs_from_maix_image() extraction path:
        floor_lab = (85, 0, 0)
        ramp_lab = (45, 40, -35)   # inside purple_ramp [20, 68, 18, 62, -55, -14]
        green_lab = (55, -35, 30)  # inside green [30, 82, -60, -18, 12, 55]
        unknown_lab = (55, 0, 45)  # yellow/olive foreign object: outside floor and ALL calibrated bands

        # 3a. Calibrated purple ramp ALONE intersecting near_corral_roi ([88, 128, 144, 96]) -> NOT unknown!
        img_ramp_only = _FakeMaixImage(320, 240, floor_lab)
        img_ramp_only.fill_rect(40, 105, 240, 35, ramp_lab)  # y=105..140 overlaps corral y=128..140
        blobs_ramp = extract_blobs_from_maix_image(img_ramp_only, self.cfg, scan_unknown_near=True)
        self.assertTrue(any(b.label == "purple_ramp" for b in blobs_ramp))
        self.assertEqual([b for b in blobs_ramp if b.label == "unknown"], [])
        st_ramp = self.proc.process(blobs_ramp, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False)
        self.assertEqual(st_ramp.corral_unknown_cnt, 0)
        self.assertTrue(st_ramp.flags & VIS_FLAG_CORRAL_SAFE)

        # 3b. Unknown foreign object sitting ON TOP OF the purple ramp inside near_corral_roi -> FLAGGED unknown!
        img_ramp_unk = _FakeMaixImage(320, 240, floor_lab)
        img_ramp_unk.fill_rect(40, 105, 240, 35, ramp_lab)
        img_ramp_unk.fill_rect(140, 128, 18, 12, unknown_lab)  # 216 px >= min_hazard_pixels (60)
        blobs_ramp_unk = extract_blobs_from_maix_image(img_ramp_unk, self.cfg, scan_unknown_near=True)
        self.assertTrue(any(b.label == "unknown" for b in blobs_ramp_unk))
        st_ramp_unk = self.proc.process(blobs_ramp_unk, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False)
        self.assertGreaterEqual(st_ramp_unk.corral_unknown_cnt, 1)
        self.assertFalse(st_ramp_unk.flags & VIS_FLAG_CORRAL_SAFE)

        # 3c. Green block touching unknown object inside near_corral_roi -> FLAGGED unknown!
        img_green_unk = _FakeMaixImage(320, 240, floor_lab)
        img_green_unk.fill_rect(120, 145, 20, 30, green_lab)    # 600 px valid green
        img_green_unk.fill_rect(140, 145, 30, 30, unknown_lab)  # 900 px touching unknown -> merged fg=1500 px
        blobs_gu = extract_blobs_from_maix_image(img_green_unk, self.cfg, scan_unknown_near=True)
        self.assertTrue(any(b.label == "unknown" for b in blobs_gu))
        st_gu = self.proc.process(blobs_gu, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False)
        self.assertGreaterEqual(st_gu.corral_unknown_cnt, 1)
        self.assertFalse(st_gu.flags & VIS_FLAG_CORRAL_SAFE)

        # 3d. Small unverified purple speck (< min_ramp_pixels=350, no full ramp) inside corral -> FLAGGED unknown!
        img_small_purple = _FakeMaixImage(320, 240, floor_lab)
        img_small_purple.fill_rect(140, 150, 14, 14, ramp_lab)  # 196 px >= 60, < 350
        blobs_sp = extract_blobs_from_maix_image(img_small_purple, self.cfg, scan_unknown_near=True)
        self.assertTrue(any(b.label == "unknown" for b in blobs_sp))

        # 4. Finding 2 (R4): Cyan on aligned ramp inhibits VIS_FLAG_DROP_PATH_CLEAR, AND a 324-px unknown
        #    inside near_drop_roi IS detected by extract_blobs_from_maix_image and blocks drop!
        home = Blob("red_zone", 40, 20, 240, 100, 15000)
        ramp = Blob("purple_ramp", 40, 100, 240, 26, 3000)
        corral_green = Blob("green", 140, 150, 24, 24, 500)
        cyan_on_ramp = Blob("cyan", 110, 102, 22, 22, 420)
        st_cyan_path = self.proc.process(
            [home, ramp, corral_green, cyan_on_ramp],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=False,
            mission_state=MISSION_DROP_OPEN_AND_PUSH,
            gate_closed=False,
            gate_settled=True,
        )
        self.assertTrue(st_cyan_path.flags & VIS_FLAG_CORRAL_SAFE)
        self.assertTrue(st_cyan_path.flags & VIS_FLAG_SWEEP_CLEAR)
        self.assertFalse(st_cyan_path.flags & VIS_FLAG_DROP_PATH_CLEAR)

        # 324-pixel (18x18) unknown inside near_drop_roi ([40, 20, 240, 108]) detected through extraction path!
        red_lab = (50, 45, 30)
        img_dest_unk = _FakeMaixImage(320, 240, floor_lab)
        img_dest_unk.fill_rect(40, 20, 240, 80, red_lab)
        img_dest_unk.fill_rect(40, 100, 240, 26, ramp_lab)
        img_dest_unk.fill_rect(70, 45, 24, 24, green_lab)     # delivered green in left half
        img_dest_unk.fill_rect(110, 50, 18, 18, unknown_lab)  # 324 px unknown in destination ROI!
        blobs_du = extract_blobs_from_maix_image(img_dest_unk, self.cfg, scan_unknown_near=True)
        self.assertTrue(any(b.label == "unknown" for b in blobs_du))
        self.proc.reset_drop_baseline()
        self.proc.process(
            [home, ramp, corral_green],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=False,
            mission_state=MISSION_DROP_OPEN_AND_PUSH,
            gate_closed=True,
        )
        st_du = self.proc.process(
            blobs_du,
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=False,
            mission_state=MISSION_DROP_BACKOUT_VERIFY,
        )
        self.assertFalse(st_du.flags & VIS_FLAG_DROP_PATH_CLEAR)
        self.assertFalse(st_du.flags & VIS_FLAG_DROP_CONFIRMED)

        # 5. Finding 4 (R4): Mutated cargo (1g -> 1g+1b) at gate-open-settled invalidates baseline;
        #    and preexisting ramp block rejects baseline so sliding it into target half never confirms drop!
        self.proc.reset_drop_baseline()
        self.proc.process(
            [home, ramp, corral_green],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            max_materials_per_trip=2,
            mission_state=MISSION_DROP_OPEN_AND_PUSH,
            gate_closed=True,
            gate_settled=True,
        )
        st_mut = self.proc.process(
            [home, ramp, corral_green, Blob("black", 170, 150, 24, 24, 480, density=0.83)],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            max_materials_per_trip=2,
            mission_state=MISSION_DROP_OPEN_AND_PUSH,
            gate_closed=False,
            gate_settled=True,
            wheels_moving=False,
        )
        self.assertFalse(st_mut.flags & VIS_FLAG_DROP_BASELINE_READY)
        self.assertFalse(st_mut.flags & VIS_FLAG_GATE_OPEN_SETTLED_ACK)
        st_mut_drop = self.proc.process(
            [home, ramp, Blob("green", 80, 50, 24, 24, 500)],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            max_materials_per_trip=2,
            mission_state=MISSION_DROP_BACKOUT_VERIFY,
        )
        self.assertFalse(st_mut_drop.flags & VIS_FLAG_DROP_CONFIRMED)

        # Preexisting block sitting on purple ramp before push MUST reject baseline!
        self.proc.reset_drop_baseline()
        pre_ramp_green = Blob("green", 80, 96, 24, 24, 500)  # straddles ramp y=100..126
        st_pre_ramp = self.proc.process(
            [home, ramp, pre_ramp_green, corral_green],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=False,
            mission_state=MISSION_DROP_OPEN_AND_PUSH,
            gate_closed=True,
        )
        self.assertFalse(st_pre_ramp.flags & VIS_FLAG_DROP_BASELINE_READY)
        self.assertFalse(st_pre_ramp.flags & VIS_FLAG_GATE_OPEN_SETTLED_ACK)
        st_ramp_moved = self.proc.process(
            [home, ramp, Blob("green", 80, 50, 24, 24, 500)],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=False,
            mission_state=MISSION_DROP_BACKOUT_VERIFY,
        )
        self.assertFalse(st_ramp_moved.flags & VIS_FLAG_DROP_CONFIRMED)

        # 6. Finding 6 (R4): Disjoint calibrated blue_zone + purple_ramp with NESTED bounding boxes
        #    (even with intentionally overlapping LAB ranges!) MUST produce ZERO unknown detections,
        #    while adding a real unknown object on top of them DOES produce unknown!
        overlap_cfg = RescueVisionConfig()
        overlap_cfg.lab_thresholds["blue_zone"] = [20, 62, -10, 30, -68, -22]  # intentionally overlaps purple_ramp
        blue_lab = (40, 0, -45)
        img_nested = _FakeMaixImage(320, 240, floor_lab)
        # C-shaped blue_zone wrapping around a nested purple_ramp inside near_corral_roi ([88, 128, 144, 96])!
        img_nested.fill_rect(90, 130, 120, 80, blue_lab)
        img_nested.fill_rect(120, 145, 60, 40, ramp_lab)  # nested inside blue_zone's bounding box
        blobs_nested = extract_blobs_from_maix_image(img_nested, overlap_cfg, scan_unknown_near=True)
        self.assertEqual([b for b in blobs_nested if b.label == "unknown"], [])

        img_nested.fill_rect(140, 155, 14, 14, unknown_lab)  # 196 px uncalibrated object on nested surface
        blobs_nested_unk = extract_blobs_from_maix_image(img_nested, overlap_cfg, scan_unknown_near=True)
        self.assertTrue(any(b.label == "unknown" for b in blobs_nested_unk))

        # 7. Finding 7 (R4): Plain floor at (85, 0, 0) MUST NEVER produce opponent or VIS_FLAG_OPPONENT_NEAR,
        #    both with default config AND even if custom opponent LAB overlaps floor!
        img_floor_only = _FakeMaixImage(320, 240, floor_lab)
        blobs_floor = extract_blobs_from_maix_image(img_floor_only, self.cfg, scan_unknown_near=True)
        self.assertEqual([b for b in blobs_floor if b.label == "opponent"], [])
        st_floor = self.proc.process(
            blobs_floor, camera_near_cmd=False, camera_settled=True, team=TEAM_RED, first_green_unlocked=False
        )
        self.assertFalse(st_floor.flags & VIS_FLAG_OPPONENT_NEAR)

        overlap_opp_cfg = RescueVisionConfig()
        overlap_opp_cfg.lab_thresholds["opponent"] = [35, 95, -15, 15, -15, 15]
        blobs_floor_ov = extract_blobs_from_maix_image(img_floor_only, overlap_opp_cfg, scan_unknown_near=True)
        self.assertEqual([b for b in blobs_floor_ov if b.label == "opponent"], [])

        # Real dark-grey opponent robot (50, 0, 0) on plain floor IS still detected!
        img_real_opp = _FakeMaixImage(320, 240, floor_lab)
        img_real_opp.fill_rect(100, 100, 60, 60, (50, 0, 0))  # 3600 px >= min_opponent_pixels (2200), y2=160 >= 145
        blobs_opp = extract_blobs_from_maix_image(img_real_opp, self.cfg, scan_unknown_near=False)
        self.assertEqual(len([b for b in blobs_opp if b.label == "opponent"]), 1)
        st_opp = self.proc.process(
            blobs_opp, camera_near_cmd=False, camera_settled=True, team=TEAM_RED, first_green_unlocked=False
        )
        self.assertTrue(st_opp.flags & VIS_FLAG_OPPONENT_NEAR)

    def test_round5_dynamic_roi_frozen_baseline_and_tier2_binary_paths(self) -> None:
        from maixcam.vision import _get_lab_pixel_accessor

        floor_lab = (85, 0, 0)
        red_lab = (50, 45, 30)
        blue_lab = (40, 0, -45)
        ramp_lab = (45, 40, -35)
        green_lab = (55, -35, 30)
        unknown_lab = (55, 0, 45)

        # 1. Finding 2 (R5): Unknown at (145, 4) (224 px, 16x14) inside aligned home extending to y=0
        #    (OUTSIDE default near_drop_roi y=20..128) MUST be detected in BOTH Tier 1 and Tier 2,
        #    MUST NOT falsely contaminate corral_unknown_cnt, and MUST clear DROP_PATH_CLEAR & GATE_OPEN_SETTLED_ACK!
        for img_cls in (_FakeMaixImage, _FakeMaixBinaryImage):
            img_top = img_cls(320, 240, floor_lab)
            img_top.fill_rect(100, 0, 220, 96, red_lab)        # aligned home extending above near_drop_roi.y=20!
            img_top.fill_rect(100, 96, 220, 24, ramp_lab)
            img_top.fill_rect(148, 150, 24, 24, green_lab)     # valid green block in corral
            img_top.fill_rect(145, 4, 16, 14, unknown_lab)     # 224 px unknown at y=4..18!
            if img_cls is _FakeMaixBinaryImage:
                self.assertIsNone(_get_lab_pixel_accessor(img_top))
            blobs_top = extract_blobs_from_maix_image(img_top, self.cfg, scan_unknown_near=True)
            self.assertTrue(any(b.label == "unknown" for b in blobs_top))
            self.proc.reset_drop_baseline()
            st_top = self.proc.process(
                blobs_top,
                camera_near_cmd=True,
                camera_settled=True,
                team=TEAM_RED,
                first_green_unlocked=False,
                mission_state=MISSION_DROP_OPEN_AND_PUSH,
                gate_closed=False,
                gate_settled=True,
                drop_pushing=False,
            )
            self.assertEqual(st_top.corral_unknown_cnt, 0)
            self.assertTrue(st_top.flags & VIS_FLAG_CORRAL_SAFE)
            self.assertFalse(st_top.flags & VIS_FLAG_DROP_PATH_CLEAR)
            self.assertFalse(st_top.flags & VIS_FLAG_GATE_OPEN_SETTLED_ACK)

        # 2. Finding 3 (R5): Freeze baseline on gate open, bind to MCU-authorized trip, and use explicit drop_pushing!
        #    Low-speed push (drop_pushing=False, wheels_moving=False) changing visible corral from (1,1,0) to (0,1,0)
        #    while home total changes MUST NOT rewrite baseline to (0,1,0) or confirm black alone!
        home = Blob("red_zone", 100, 20, 220, 80, 17600)
        ramp = Blob("purple_ramp", 100, 100, 220, 26, 5720)
        corral_green = Blob("green", 140, 150, 24, 24, 500)
        corral_black = Blob("black", 170, 150, 24, 24, 480, density=0.83)
        self.proc.reset_drop_baseline()
        # Gate closed pre-push snapshot: (1, 1, 0)
        st_pre = self.proc.process(
            [home, ramp, corral_green, corral_black],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            max_materials_per_trip=2,
            mission_state=MISSION_DROP_OPEN_AND_PUSH,
            gate_closed=True,
            gate_settled=True,
            drop_pushing=False,
        )
        self.assertTrue(st_pre.flags & VIS_FLAG_DROP_PATH_CLEAR)
        # Gate open + settled ACK on exact (1, 1, 0)
        st_ack = self.proc.process(
            [home, ramp, corral_green, corral_black],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            max_materials_per_trip=2,
            mission_state=MISSION_DROP_OPEN_AND_PUSH,
            gate_closed=False,
            gate_settled=True,
            drop_pushing=False,
            wheels_moving=False,
        )
        self.assertTrue(st_ack.flags & VIS_FLAG_GATE_OPEN_SETTLED_ACK)
        # Low-speed push begins before telemetry updates (wheels_moving=False, drop_pushing=False):
        # green exits corral into home (home total changes!) leaving (0, 1, 0) in corral.
        # Baseline MUST remain frozen at (1, 1, 0), NOT rewritten to (0, 1, 0)!
        st_low_speed = self.proc.process(
            [home, ramp, Blob("green", 140, 50, 24, 24, 500), corral_black],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            max_materials_per_trip=2,
            mission_state=MISSION_DROP_OPEN_AND_PUSH,
            gate_closed=False,
            gate_settled=True,
            drop_pushing=False,
            wheels_moving=False,
        )
        self.assertFalse(st_low_speed.flags & VIS_FLAG_GATE_OPEN_SETTLED_ACK)
        # Explicit push phase (TEL_FLAG_DROP_PUSHING): cargo crossing ramp keeps DROP_PATH_CLEAR while cyan blocks it.
        st_push_ok = self.proc.process(
            [home, ramp, Blob("green", 140, 96, 24, 24, 500), corral_black],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            max_materials_per_trip=2,
            mission_state=MISSION_DROP_OPEN_AND_PUSH,
            gate_closed=False,
            gate_settled=True,
            drop_pushing=True,
        )
        self.assertTrue(st_push_ok.flags & VIS_FLAG_DROP_PATH_CLEAR)
        self.assertFalse(st_push_ok.flags & VIS_FLAG_GATE_OPEN_SETTLED_ACK)
        st_push_cyan = self.proc.process(
            [home, ramp, Blob("cyan", 140, 96, 24, 24, 500), corral_black],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            max_materials_per_trip=2,
            mission_state=MISSION_DROP_OPEN_AND_PUSH,
            gate_closed=False,
            gate_settled=True,
            drop_pushing=True,
        )
        self.assertFalse(st_push_cyan.flags & VIS_FLAG_DROP_PATH_CLEAR)
        # Verify: black alone in target half (0, 1, 0) MUST NOT confirm against frozen (1, 1, 0) baseline!
        st_black_only = self.proc.process(
            [home, ramp, Blob("black", 140, 50, 24, 24, 480, density=0.83)],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            max_materials_per_trip=2,
            mission_state=MISSION_DROP_BACKOUT_VERIFY,
        )
        self.assertFalse(st_black_only.flags & VIS_FLAG_DROP_CONFIRMED)
        # Both green + black in target half (1, 1, 0) DOES confirm!
        st_both_ok = self.proc.process(
            [home, ramp, Blob("green", 120, 50, 24, 24, 500), Blob("black", 155, 50, 24, 24, 480, density=0.83)],
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            max_materials_per_trip=2,
            mission_state=MISSION_DROP_BACKOUT_VERIFY,
        )
        self.assertTrue(st_both_ok.flags & VIS_FLAG_DROP_CONFIRMED)
        # Verify TEL_FLAG_DROP_PUSHING round-trips cleanly.
        tel = Telemetry(status_flags=TEL_FLAG_DROP_PUSHING)
        parser = RescueProtocolParser()
        decoded_tel = decode_telemetry(parser.feed(encode_telemetry(1, tel))[0])
        self.assertEqual(decoded_tel.status_flags, TEL_FLAG_DROP_PUSHING)

        # 3. Finding 4 (R5): Direct Tier-2 MaixPy binary() test — verified purple_ramp at y=100..126
        #    MUST NOT mask a separate unverified 100-px (10x10) purple patch inside red_zone at y=40..50!
        img_bin_purple = _FakeMaixBinaryImage(320, 240, floor_lab)
        self.assertIsNone(_get_lab_pixel_accessor(img_bin_purple))
        img_bin_purple.fill_rect(40, 20, 240, 80, red_lab)
        img_bin_purple.fill_rect(40, 100, 240, 26, ramp_lab)
        img_bin_purple.fill_rect(100, 40, 10, 10, ramp_lab)  # 100 px unverified purple patch outside ramp!
        blobs_bp = extract_blobs_from_maix_image(img_bin_purple, self.cfg, scan_unknown_near=True)
        self.assertTrue(any(b.label == "purple_ramp" for b in blobs_bp))
        self.assertTrue(any(b.label == "unknown" for b in blobs_bp))

        # Tier-2 nested blue_zone + purple_ramp with overlapping LAB thresholds MUST still produce ZERO unknown!
        overlap_cfg = RescueVisionConfig()
        overlap_cfg.lab_thresholds["blue_zone"] = [20, 62, -10, 30, -68, -22]
        img_bin_nested = _FakeMaixBinaryImage(320, 240, floor_lab)
        img_bin_nested.fill_rect(90, 130, 120, 80, blue_lab)
        img_bin_nested.fill_rect(120, 145, 60, 40, ramp_lab)
        blobs_bn = extract_blobs_from_maix_image(img_bin_nested, overlap_cfg, scan_unknown_near=True)
        self.assertEqual([b for b in blobs_bn if b.label == "unknown"], [])

        # 4. Finding 5 (R5): Direct Tier-2 MaixPy binary() opponent floor-exclusion test —
        #    distant opponent ending at y2=67 with floor-overlapping custom opponent LAB MUST keep tight y2==67
        #    (NOT y2=240!) and MUST NOT raise VIS_FLAG_OPPONENT_NEAR — even when touching a lower green block!
        overlap_opp_cfg = RescueVisionConfig()
        overlap_opp_cfg.lab_thresholds["opponent"] = [35, 95, -15, 15, -15, 15]
        img_bin_opp = _FakeMaixBinaryImage(320, 240, floor_lab)
        self.assertIsNone(_get_lab_pixel_accessor(img_bin_opp))
        img_bin_opp.fill_rect(120, 10, 60, 57, (50, 0, 0))  # 3420 px distant opponent, y2=67 < opponent_near_y=145
        blobs_bo = extract_blobs_from_maix_image(img_bin_opp, overlap_opp_cfg, scan_unknown_near=False)
        opp_blobs = [b for b in blobs_bo if b.label == "opponent"]
        self.assertEqual(len(opp_blobs), 1)
        self.assertEqual(opp_blobs[0].y2, 67)
        st_bo = self.proc.process(
            blobs_bo, camera_near_cmd=False, camera_settled=True, team=TEAM_RED, first_green_unlocked=False
        )
        self.assertFalse(st_bo.flags & VIS_FLAG_OPPONENT_NEAR)

        # Even if a green block touches the bottom edge of the distant opponent down to y2=157,
        # Tier-2 binary search MUST STILL report exact opponent y2==67 and VIS_FLAG_OPPONENT_NEAR==0!
        img_bin_opp.fill_rect(120, 67, 25, 90, green_lab)
        blobs_bo_touch = extract_blobs_from_maix_image(img_bin_opp, overlap_opp_cfg, scan_unknown_near=False)
        opp_blobs_t = [b for b in blobs_bo_touch if b.label == "opponent"]
        self.assertEqual(len(opp_blobs_t), 1)
        self.assertEqual(opp_blobs_t[0].y2, 67)
        st_bo_t = self.proc.process(
            blobs_bo_touch, camera_near_cmd=False, camera_settled=True, team=TEAM_RED, first_green_unlocked=False
        )
        self.assertFalse(st_bo_t.flags & VIS_FLAG_OPPONENT_NEAR)

    def test_round6_component_masks_tight_unknowns_disjoint_lab_ring_and_delayed_push_bit(self) -> None:
        from maixcam.vision import _lab_ranges_overlap

        # 1. Finding 5 (R6): ALL 9 stock LAB bands (8 classes + arena_floor_lab) MUST be pairwise disjoint,
        #    and a clean red safe zone at LAB (50, 45, 30) MUST pass with zero orange/unknown false positives!
        all_bands = dict(self.cfg.lab_thresholds)
        all_bands["arena_floor_lab"] = self.cfg.arena_floor_lab
        band_names = sorted(all_bands.keys())
        for i in range(len(band_names)):
            for j in range(i + 1, len(band_names)):
                a, b = band_names[i], band_names[j]
                self.assertFalse(
                    _lab_ranges_overlap(all_bands[a], all_bands[b]),
                    f"Stock LAB bands overlap: {a}={all_bands[a]} vs {b}={all_bands[b]}",
                )

        floor_lab = (85, 0, 0)
        red_lab = (50, 45, 30)
        pure_red_lab = (36, 45, 18)
        ramp_lab = (45, 40, -35)
        green_lab = (55, -35, 30)
        cyan_lab = (68, -25, -20)
        unknown_lab = (55, 0, 45)

        for img_cls in (_FakeMaixImage, _FakeMaixBinaryImage):
            # 1a. Clean-scene positive control on BOTH tiers (stock red (50, 45, 30) + ramp + green in corral).
            img_clean = img_cls(320, 240, floor_lab)
            img_clean.fill_rect(100, 20, 220, 76, red_lab)
            img_clean.fill_rect(100, 96, 220, 24, ramp_lab)
            img_clean.fill_rect(148, 156, 24, 24, green_lab)
            blobs_clean = extract_blobs_from_maix_image(img_clean, self.cfg, scan_unknown_near=True)
            self.assertEqual([b for b in blobs_clean if b.label == "orange"], [])
            self.assertEqual([b for b in blobs_clean if b.label == "unknown"], [])
            self.proc.reset_drop_baseline()
            st_clean = self.proc.process(
                blobs_clean,
                camera_near_cmd=True,
                camera_settled=True,
                team=TEAM_RED,
                first_green_unlocked=False,
                mission_state=MISSION_DROP_OPEN_AND_PUSH,
                gate_closed=True,
            )
            self.assertTrue(st_clean.flags & VIS_FLAG_CORRAL_SAFE)
            self.assertTrue(st_clean.flags & VIS_FLAG_DROP_PATH_CLEAR)
            self.assertTrue(st_clean.flags & VIS_FLAG_DROP_BASELINE_READY)

            # 1b. Even with a custom on-site orange band overlapping red_zone, clean red home still passes,
            #     while a genuine cyan hazard on the red surface STILL blocks the drop!
            ov_cfg = RescueVisionConfig()
            ov_cfg.lab_thresholds["orange"] = [42, 85, 18, 65, 25, 72]
            blobs_clean_ov = extract_blobs_from_maix_image(img_clean, ov_cfg, scan_unknown_near=True)
            self.assertEqual([b for b in blobs_clean_ov if b.label == "orange"], [])
            self.assertEqual([b for b in blobs_clean_ov if b.label == "unknown"], [])
            img_clean.fill_rect(140, 40, 16, 16, cyan_lab)
            blobs_cyan_on_red = extract_blobs_from_maix_image(img_clean, ov_cfg, scan_unknown_near=True)
            self.assertTrue(any(b.label == "cyan" for b in blobs_cyan_on_red))
            st_haz = self.proc.process(
                blobs_cyan_on_red,
                camera_near_cmd=True,
                camera_settled=True,
                team=TEAM_RED,
                first_green_unlocked=False,
                mission_state=MISSION_DROP_OPEN_AND_PUSH,
                gate_closed=True,
            )
            self.assertFalse(st_haz.flags & VIS_FLAG_DROP_PATH_CLEAR)

            # 2. Finding 4 (R6): Destination-only unknown at y=40..54 on red_zone connected to purple_ramp
            #    extending into corral_roi (y=96..136) MUST emit exact tight bounds (145, 40, 16, 14),
            #    keep corral_unknown_cnt == 0 (CORRAL_SAFE=1), and clear DROP_PATH_CLEAR!
            img_dest = img_cls(320, 240, floor_lab)
            img_dest.fill_rect(100, 20, 220, 76, pure_red_lab)
            img_dest.fill_rect(100, 96, 220, 40, ramp_lab)
            img_dest.fill_rect(145, 40, 16, 14, unknown_lab)
            img_dest.fill_rect(148, 156, 24, 24, green_lab)
            blobs_dest = extract_blobs_from_maix_image(img_dest, self.cfg, scan_unknown_near=True)
            unk_rects = [(b.x, b.y, b.w, b.h) for b in blobs_dest if b.label == "unknown"]
            self.assertEqual(unk_rects, [(145, 40, 16, 14)])
            self.proc.reset_drop_baseline()
            st_dest = self.proc.process(
                blobs_dest,
                camera_near_cmd=True,
                camera_settled=True,
                team=TEAM_RED,
                first_green_unlocked=False,
                mission_state=MISSION_DROP_OPEN_AND_PUSH,
                gate_closed=True,
            )
            self.assertEqual(st_dest.corral_unknown_cnt, 0)
            self.assertTrue(st_dest.flags & VIS_FLAG_CORRAL_SAFE)
            self.assertFalse(st_dest.flags & VIS_FLAG_DROP_PATH_CLEAR)

            # 3. Finding 3 (R6): Detached 81-px (9x9) purple patch inside a C-shaped purple_ramp's AABB
            #    MUST be flagged unknown in BOTH Tier 1 and Tier 2 and clear DROP_PATH_CLEAR!
            img_c = img_cls(320, 240, floor_lab)
            img_c.fill_rect(100, 20, 220, 70, pure_red_lab)
            img_c.fill_rect(100, 90, 220, 5, ramp_lab)
            img_c.fill_rect(100, 95, 5, 40, ramp_lab)
            img_c.fill_rect(100, 135, 220, 5, ramp_lab)
            img_c.fill_rect(190, 113, 9, 9, ramp_lab)  # 81 px detached patch inside C-ramp AABB!
            img_c.fill_rect(148, 156, 24, 24, green_lab)
            blobs_c = extract_blobs_from_maix_image(img_c, self.cfg, scan_unknown_near=True)
            unk_c = [(b.x, b.y, b.w, b.h) for b in blobs_c if b.label == "unknown"]
            self.assertEqual(unk_c, [(190, 113, 9, 9)])
            self.proc.reset_drop_baseline()
            st_c = self.proc.process(
                blobs_c,
                camera_near_cmd=True,
                camera_settled=True,
                team=TEAM_RED,
                first_green_unlocked=False,
                mission_state=MISSION_DROP_OPEN_AND_PUSH,
                gate_closed=True,
            )
            self.assertEqual(st_c.corral_unknown_cnt, 0)
            self.assertFalse(st_c.flags & VIS_FLAG_DROP_PATH_CLEAR)
            self.assertFalse(st_c.flags & VIS_FLAG_DROP_BASELINE_READY)

            # 4. Finding 6 (R6): 1280 opponent pixels inside an unrelated non-floor green ring with
            #    floor-overlapping opponent LAB MUST NOT double-count or trigger VIS_FLAG_OPPONENT_NEAR!
            ring_cfg = RescueVisionConfig()
            ring_cfg.lab_thresholds["opponent"] = [35, 95, -15, 15, -15, 15]
            img_ring = img_cls(320, 240, floor_lab)
            for rect in ((96, 115, 80, 5), (96, 189, 80, 5), (96, 120, 5, 69), (171, 120, 5, 69)):
                img_ring.fill_rect(*rect, green_lab)
            img_ring.fill_rect(119, 145, 32, 40, (50, 0, 0))  # 1280 px < min_opponent_pixels (2200)
            blobs_ring = extract_blobs_from_maix_image(img_ring, ring_cfg)
            self.assertEqual([b for b in blobs_ring if b.label == "opponent"], [])
            st_ring = self.proc.process(
                blobs_ring, camera_near_cmd=False, camera_settled=True, team=TEAM_RED, first_green_unlocked=False
            )
            self.assertFalse(st_ring.flags & VIS_FLAG_OPPONENT_NEAR)

        # 5. Finding 7 (R6): Moving 1 of 2 cargo blocks 1 px past corral boundary BEFORE TEL_FLAG_DROP_PUSHING
        #    arrives MUST keep VIS_FLAG_DROP_PATH_CLEAR == 1 (cannot abort clean push!), while clearing
        #    VIS_FLAG_DROP_BASELINE_READY and VIS_FLAG_GATE_OPEN_SETTLED_ACK and keeping baseline frozen!
        proc = RescueVisionProcessor(self.cfg)
        drop_kw = dict(
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            max_materials_per_trip=2,
            mission_state=MISSION_DROP_OPEN_AND_PUSH,
            gate_settled=True,
        )
        home = Blob("red_zone", 100, 20, 220, 60, 13200)
        ramp = Blob("purple_ramp", 100, 80, 220, 22, 4840)
        green = Blob("green", 140, 105, 24, 24, 500)
        black = Blob("black", 170, 150, 24, 24, 480, density=0.83)
        pre = [home, ramp, green, black]
        st_closed = proc.process(pre, gate_closed=True, **drop_kw)
        self.assertTrue(st_closed.flags & VIS_FLAG_DROP_PATH_CLEAR)
        self.assertTrue(st_closed.flags & VIS_FLAG_DROP_BASELINE_READY)
        st_open = proc.process(pre, gate_closed=False, **drop_kw)
        self.assertTrue(st_open.flags & VIS_FLAG_GATE_OPEN_SETTLED_ACK)
        self.assertTrue(st_open.flags & VIS_FLAG_DROP_BASELINE_READY)

        post_1px = [home, ramp, Blob("green", 140, 104, 24, 24, 500), black]
        st_before_bit = proc.process(post_1px, gate_closed=False, drop_pushing=False, **drop_kw)
        st_after_bit = proc.process(post_1px, gate_closed=False, drop_pushing=True, **drop_kw)
        self.assertTrue(st_before_bit.flags & VIS_FLAG_DROP_PATH_CLEAR)
        self.assertTrue(st_after_bit.flags & VIS_FLAG_DROP_PATH_CLEAR)
        self.assertFalse(st_before_bit.flags & VIS_FLAG_DROP_BASELINE_READY)
        self.assertFalse(st_before_bit.flags & VIS_FLAG_GATE_OPEN_SETTLED_ACK)
        # Adding a cyan hazard immediately clears VIS_FLAG_DROP_PATH_CLEAR regardless of drop_pushing!
        st_new_haz = proc.process(
            post_1px + [Blob("cyan", 140, 82, 20, 20, 400)], gate_closed=False, drop_pushing=True, **drop_kw
        )
        self.assertFalse(st_new_haz.flags & VIS_FLAG_DROP_PATH_CLEAR)

    def test_round7_unit_stride_actual_clipped_pixels_and_backout_flags(self) -> None:
        floor_lab = (85, 0, 0)
        red_lab = (50, 45, 30)
        ramp_lab = (45, 40, -35)
        green_lab = (55, -35, 30)
        unknown_lab = (55, 0, 45)
        kw = dict(camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False)

        def make_scene(cls):
            img = cls(320, 240, floor_lab)
            img.fill_rect(100, 20, 220, 69, red_lab)
            img.fill_rect(100, 89, 220, 23, ramp_lab)
            img.fill_rect(193, 175, 24, 24, green_lab)
            return img

        # 1. Finding 3 (R7): 64-px 1-px-wide diagonal unknown (misses default x_stride=2 seed lattice!)
        #    MUST be caught in corral AND in home across Tier 1, Tier 2, AND _FakeMaixNativeStrideImage!
        for cls in (_FakeMaixImage, _FakeMaixBinaryImage, _FakeMaixNativeStrideImage):
            img_diag_corral = make_scene(cls)
            for i in range(64):
                img_diag_corral.fill_rect(106 + i, 143 + i, 1, 1, unknown_lab)
            blobs_dc = extract_blobs_from_maix_image(img_diag_corral, self.cfg, scan_unknown_near=True)
            st_dc = RescueVisionProcessor(self.cfg).process(
                blobs_dc, mission_state=MISSION_DROP_OPEN_AND_PUSH, gate_closed=True, **kw
            )
            self.assertGreaterEqual(st_dc.corral_unknown_cnt, 1, cls.__name__)
            self.assertFalse(st_dc.flags & VIS_FLAG_DROP_PATH_CLEAR, cls.__name__)
            self.assertFalse(st_dc.flags & VIS_FLAG_DROP_BASELINE_READY, cls.__name__)

            # Also verify 64-px diagonal unknown inside home (merge_small=False ROI) is caught!
            img_diag_home = make_scene(cls)
            for i in range(64):
                img_diag_home.fill_rect(120 + i, 22 + i, 1, 1, unknown_lab)
            blobs_dh = extract_blobs_from_maix_image(img_diag_home, self.cfg, scan_unknown_near=True)
            st_dh = RescueVisionProcessor(self.cfg).process(
                blobs_dh, mission_state=MISSION_DROP_OPEN_AND_PUSH, gate_closed=True, **kw
            )
            self.assertEqual(st_dh.corral_unknown_cnt, 0, cls.__name__)
            self.assertFalse(st_dh.flags & VIS_FLAG_DROP_PATH_CLEAR, cls.__name__)
            self.assertFalse(st_dh.flags & VIS_FLAG_DROP_BASELINE_READY, cls.__name__)

        # 2. Finding 4 (R7): 150-px L-shaped unverified purple patch completely OUTSIDE near_corral_roi
        #    (zero actual pixels inside corral, only concave AABB corner intersects corral) MUST produce
        #    corral_unknown_cnt == 0 and exact actual-pixel clipped bounds in ALL tiers!
        for cls in (_FakeMaixImage, _FakeMaixBinaryImage, _FakeMaixNativeStrideImage):
            img_l = make_scene(cls)
            img_l.fill_rect(75, 119, 27, 3, ramp_lab)
            img_l.fill_rect(75, 122, 3, 23, ramp_lab)
            blobs_l = extract_blobs_from_maix_image(img_l, self.cfg, scan_unknown_near=True)
            unk_bounds = [(b.x, b.y, b.w, b.h, b.pixels) for b in blobs_l if b.label == "unknown"]
            self.assertEqual(unk_bounds, [(75, 119, 27, 9, 99)], cls.__name__)
            st_l = RescueVisionProcessor(self.cfg).process(blobs_l, **kw)
            self.assertEqual(st_l.corral_unknown_cnt, 0, cls.__name__)
            self.assertTrue(st_l.flags & VIS_FLAG_CORRAL_SAFE, cls.__name__)

        # 3. Finding 5 (R7): Retain _drop_baseline during MISSION_DROP_BACKOUT_VERIFY for delivery confirmation,
        #    but NEVER emit VIS_FLAG_DROP_BASELINE_READY or VIS_FLAG_GATE_OPEN_SETTLED_ACK outside pre-push!
        proc = RescueVisionProcessor(self.cfg)
        home = Blob("red_zone", 100, 20, 220, 60, 13200)
        ramp = Blob("purple_ramp", 100, 80, 220, 22, 4840)
        green_corral = Blob("green", 147, 156, 24, 24, 576)
        st_pre = proc.process(
            [home, ramp, green_corral], mission_state=MISSION_DROP_OPEN_AND_PUSH, gate_closed=True, **kw
        )
        self.assertTrue(st_pre.flags & VIS_FLAG_DROP_BASELINE_READY)
        st_backout_empty = proc.process(
            [home, ramp],
            mission_state=MISSION_DROP_BACKOUT_VERIFY,
            gate_closed=False,
            gate_settled=True,
            **kw,
        )
        self.assertFalse(st_backout_empty.flags & VIS_FLAG_DROP_BASELINE_READY)
        self.assertFalse(st_backout_empty.flags & VIS_FLAG_GATE_OPEN_SETTLED_ACK)
        self.assertFalse(st_backout_empty.flags & VIS_FLAG_DROP_CONFIRMED)
        # Baseline is still retained so actual delivery inside target half confirms cleanly while keeping pre-push bits 0!
        st_backout_done = proc.process(
            [home, ramp, Blob("green", 130, 40, 24, 24, 576)],
            mission_state=MISSION_DROP_BACKOUT_VERIFY,
            gate_closed=False,
            gate_settled=True,
            **kw,
        )
        self.assertTrue(st_backout_done.flags & VIS_FLAG_DROP_CONFIRMED)
        self.assertFalse(st_backout_done.flags & VIS_FLAG_DROP_BASELINE_READY)
        self.assertFalse(st_backout_done.flags & VIS_FLAG_GATE_OPEN_SETTLED_ACK)

        # 4. Finding 3 (R8): Same-label component-identity preservation during Tier-2 clipping!
        #    An unverified purple L has ZERO pixels inside corral_roi, while a disconnected VERIFIED
        #    450-px purple_ramp (91, 130, 30, 15) sits inside the L's bounding box AND inside corral_roi.
        #    ALL tiers MUST keep (91, 130, 30, 15, 450) strictly as purple_ramp (NOT unknown!) and
        #    report corral_unknown_cnt == 0, VIS_FLAG_CORRAL_SAFE == 1!
        for cls in (_FakeMaixImage, _FakeMaixBinaryImage, _FakeMaixNativeStrideImage):
            img_same = make_scene(cls)
            img_same.fill_rect(75, 119, 50, 2, ramp_lab)   # unverified purple L (164 px, outside corral)
            img_same.fill_rect(75, 121, 2, 32, ramp_lab)
            img_same.fill_rect(91, 130, 30, 15, ramp_lab)  # disconnected VERIFIED 450-px purple_ramp inside L's AABB!
            blobs_same = extract_blobs_from_maix_image(img_same, self.cfg, scan_unknown_near=True)
            ramp_box = (91, 130, 30, 15, 450)
            labels_at_box = [b.label for b in blobs_same if (b.x, b.y, b.w, b.h, b.pixels) == ramp_box]
            self.assertEqual(labels_at_box, ["purple_ramp"], cls.__name__)
            st_same = RescueVisionProcessor(self.cfg).process(
                blobs_same, mission_state=MISSION_DROP_OPEN_AND_PUSH, gate_closed=True, **kw
            )
            self.assertEqual(st_same.corral_unknown_cnt, 0, cls.__name__)
            self.assertTrue(st_same.flags & VIS_FLAG_CORRAL_SAFE, cls.__name__)

            # Also test partially clipped verified purple_ramp extending right past the L's bbox (x=91..141)!
            img_partial_v = make_scene(cls)
            img_partial_v.fill_rect(75, 119, 50, 2, ramp_lab)
            img_partial_v.fill_rect(75, 121, 2, 32, ramp_lab)
            img_partial_v.fill_rect(91, 130, 50, 15, ramp_lab)  # 750-px verified ramp clipped by L's x2=125!
            blobs_pv = extract_blobs_from_maix_image(img_partial_v, self.cfg, scan_unknown_near=True)
            st_pv = RescueVisionProcessor(self.cfg).process(
                blobs_pv, mission_state=MISSION_DROP_OPEN_AND_PUSH, gate_closed=True, **kw
            )
            self.assertEqual(st_pv.corral_unknown_cnt, 0, cls.__name__)
            self.assertTrue(st_pv.flags & VIS_FLAG_CORRAL_SAFE, cls.__name__)

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
