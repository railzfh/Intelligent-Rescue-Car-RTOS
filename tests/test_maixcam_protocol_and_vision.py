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
    VIS_FLAG_SETTLE_GEN_MASK,
    VIS_FLAG_SETTLE_GEN_SHIFT,
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


class _FakeMaix8ConnImage(_FakeMaixImage):
    """Tier-1 pixel-accessor image whose find_blobs() uses 8-connected components."""

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
        nbrs8 = ((-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1))
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
                    for dx, dy in nbrs8:
                        nx, ny = cx + dx, cy + dy
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

        # 5. Finding 3 (R9): Nested disconnected same-label components where an UNVERIFIED 262-px
        #    green C outside corral encloses the bounding box of a CLIPPED, disconnected VERIFIED
        #    green E arm inside corral (125 px <= 262 px). ALL tiers MUST preserve exact parent
        #    component identity -> unknown_count == 0, VIS_FLAG_CORRAL_SAFE == 1!
        #    And when an actual unverified green blob is ALSO added inside corral, ALL tiers MUST
        #    report ONLY that unverified blob and mark corral unsafe!
        nested_cfg = RescueVisionConfig()
        nested_cfg.near_corral_roi = [100, 126, 30, 24]
        nested_cfg.near_drop_roi = [40, 20, 240, 85]
        nested_cfg.min_block_pixels = 300
        for cls in (_FakeMaixImage, _FakeMaixBinaryImage, _FakeMaixNativeStrideImage):
            img_ce = cls(320, 240, floor_lab)
            # Sub-minimum green C (262 px), completely outside corral [100..130, 126..150].
            for rect in ((90, 120, 51, 2), (90, 122, 2, 29), (90, 151, 51, 2)):
                img_ce.fill_rect(*rect, green_lab)
            # Disconnected verified green E (776 px), with middle arm clipped into corral at (105, 136, 25, 5).
            for rect in ((85, 115, 65, 3), (147, 118, 3, 37), (105, 136, 42, 5), (85, 155, 65, 4)):
                img_ce.fill_rect(*rect, green_lab)
            blobs_ce = extract_blobs_from_maix_image(img_ce, nested_cfg, scan_unknown_near=True)
            self.assertEqual([b for b in blobs_ce if b.label == "unknown"], [], cls.__name__)
            st_ce = RescueVisionProcessor(nested_cfg).process(blobs_ce, **kw)
            self.assertEqual(st_ce.corral_unknown_cnt, 0, cls.__name__)
            self.assertTrue(st_ce.flags & VIS_FLAG_CORRAL_SAFE, cls.__name__)

            # Positive hazard control: extend unverified green C with a real 80-px stub INSIDE corral
            # (still < 300 px total? Use 18-px? Total 262 + 30 = 292 < 300, >= 25? Actually min_hazard_pixels=64 ->
            # shrink C outer bars by 50 px and add 72-px stub inside corral so total C = 284 < 300!)
            img_ce_haz = cls(320, 240, floor_lab)
            for rect in ((90, 120, 41, 2), (90, 122, 2, 29), (90, 151, 41, 2)):
                img_ce_haz.fill_rect(*rect, green_lab)
            # Connect a 72-px (9x8) unverified stub of the C into corral at (100, 127, 9, 8) (y=127..134,
            # strictly separated from verified E's middle arm at y=136..140) via bridge at x=92..100, y=128..130!
            img_ce_haz.fill_rect(92, 128, 8, 2, green_lab)
            img_ce_haz.fill_rect(100, 127, 9, 8, green_lab)
            # Trim 20 px from bottom bar -> total C = 290 < 300 (strictly unverified!), 72 px inside corral >= 64!
            img_ce_haz.fill_rect(121, 151, 10, 2, floor_lab)
            for rect in ((85, 115, 65, 3), (147, 118, 3, 37), (105, 136, 42, 5), (85, 155, 65, 4)):
                img_ce_haz.fill_rect(*rect, green_lab)
            blobs_ce_haz = extract_blobs_from_maix_image(img_ce_haz, nested_cfg, scan_unknown_near=True)
            unks = [(b.x, b.y, b.w, b.h, b.pixels) for b in blobs_ce_haz if b.label == "unknown"]
            self.assertEqual(unks, [(100, 127, 9, 8, 72)], cls.__name__)
            st_ce_haz = RescueVisionProcessor(nested_cfg).process(blobs_ce_haz, **kw)
            self.assertEqual(st_ce_haz.corral_unknown_cnt, 1, cls.__name__)
            self.assertFalse(st_ce_haz.flags & VIS_FLAG_CORRAL_SAFE, cls.__name__)

        # 6. Finding 2 (R10c): Overlapping custom LAB bands matching the SAME 40-px unverified patch
        #    inside corral MUST count UNIQUE pixels (40 < min_hazard_pixels=60), NEVER double-counting
        #    to 80 px! Also check partial overlap: 55 unique px stays safe, 60 unique px flags unsafe!
        ov_band_cfg = RescueVisionConfig()
        ov_band_cfg.lab_thresholds["orange"] = [35, 85, 18, 65, 20, 75]
        ov_band_cfg.lab_thresholds["red_zone"] = [25, 75, 25, 78, 5, 45]  # overlaps orange on (50, 45, 30)!
        shared_lab = (50, 45, 30)  # matches BOTH orange and red_zone!
        orange_only_lab = (65, 45, 60)  # matches ONLY orange!
        red_only_lab = (35, 45, 15)     # matches ONLY red_zone!
        for cls in (_FakeMaixImage, _FakeMaixBinaryImage, _FakeMaixNativeStrideImage):
            # 40-px (8x5) shared patch -> 40 unique px < 60 -> MUST NOT become 80-px unknown!
            img_ov40 = cls(320, 240, floor_lab)
            img_ov40.fill_rect(120, 150, 8, 5, shared_lab)
            blobs_ov40 = extract_blobs_from_maix_image(img_ov40, ov_band_cfg, scan_unknown_near=True)
            self.assertEqual([b for b in blobs_ov40 if b.label == "unknown"], [], cls.__name__)
            st_ov40 = RescueVisionProcessor(ov_band_cfg).process(blobs_ov40, **kw)
            self.assertEqual(st_ov40.corral_unknown_cnt, 0, cls.__name__)
            self.assertTrue(st_ov40.flags & VIS_FLAG_CORRAL_SAFE, cls.__name__)

            # Partial overlap: 20 orange-only + 15 shared + 20 red-only = 55 unique px (< 60 -> SAFE!),
            # even though orange sees 35 px and red_zone sees 35 px (sum=70 >= 60)!
            img_ov55 = cls(320, 240, floor_lab)
            img_ov55.fill_rect(120, 150, 4, 5, orange_only_lab)  # 20 px
            img_ov55.fill_rect(124, 150, 3, 5, shared_lab)       # 15 px
            img_ov55.fill_rect(127, 150, 4, 5, red_only_lab)     # 20 px -> total unique = 55 px!
            blobs_ov55 = extract_blobs_from_maix_image(img_ov55, ov_band_cfg, scan_unknown_near=True)
            self.assertEqual([b for b in blobs_ov55 if b.label == "unknown"], [], cls.__name__)
            st_ov55 = RescueVisionProcessor(ov_band_cfg).process(blobs_ov55, **kw)
            self.assertEqual(st_ov55.corral_unknown_cnt, 0, cls.__name__)
            self.assertTrue(st_ov55.flags & VIS_FLAG_CORRAL_SAFE, cls.__name__)

            # Add 5 more red-only pixels -> 60 unique px (>= 60 -> UNSAFE across ALL tiers!).
            img_ov55.fill_rect(131, 150, 1, 5, red_only_lab)
            blobs_ov60 = extract_blobs_from_maix_image(img_ov55, ov_band_cfg, scan_unknown_near=True)
            self.assertEqual(len([b for b in blobs_ov60 if b.label == "unknown"]), 1, cls.__name__)
            st_ov60 = RescueVisionProcessor(ov_band_cfg).process(blobs_ov60, **kw)
            self.assertEqual(st_ov60.corral_unknown_cnt, 1, cls.__name__)
            self.assertFalse(st_ov60.flags & VIS_FLAG_CORRAL_SAFE, cls.__name__)

        # 7. Finding 3 (R10c): With default ROIs (near_drop_roi=[40,20,240,108], near_sweep_roi=[232,118,64,106]),
        #    an 84-px L-shaped unknown patch inside near_drop_roi with ZERO actual pixels inside near_sweep_roi
        #    (even though its concave AABB overlaps near_sweep_roi!) MUST keep SWEEP_CLEAR=True and
        #    CORRAL_SAFE=True while STILL blocking DROP_PATH_CLEAR=False across ALL three image tiers!
        #    Same for a concave cyan hazard outside near_sweep_roi; and protruding into near_sweep_roi clears SWEEP_CLEAR!
        cyan_lab = (68, -25, -20)
        for cls in (_FakeMaixImage, _FakeMaixBinaryImage, _FakeMaixNativeStrideImage):
            img_sw = cls(320, 240, floor_lab)
            # L-shape strictly inside near_drop_roi (x: 40..280, y: 20..128) and strictly outside near_sweep_roi (x>=232, y>=118):
            # top bar: (228, 115, 22, 2) -> 44 px (y=115..116 < 118)
            # left bar: (228, 117, 4, 10) -> 40 px (x=228..231 < 232, y=117..126 < 128)
            # Total = 84 px >= 60, AABB = (228, 115, 22, 12) overlaps sweep [232..296, 118..224] with 0 actual pixels!
            img_sw.fill_rect(228, 115, 22, 2, unknown_lab)
            img_sw.fill_rect(228, 117, 4, 10, unknown_lab)
            blobs_sw = extract_blobs_from_maix_image(img_sw, self.cfg, scan_unknown_near=True)
            st_sw = RescueVisionProcessor(self.cfg).process(blobs_sw, **kw)
            self.assertTrue(st_sw.flags & VIS_FLAG_SWEEP_CLEAR, cls.__name__)
            self.assertTrue(st_sw.flags & VIS_FLAG_CORRAL_SAFE, cls.__name__)
            self.assertFalse(st_sw.flags & VIS_FLAG_DROP_PATH_CLEAR, cls.__name__)

            # Same geometry with cyan hazard -> SWEEP_CLEAR=True, DROP_PATH_CLEAR=False!
            img_sw_c = cls(320, 240, floor_lab)
            img_sw_c.fill_rect(228, 115, 22, 2, cyan_lab)
            img_sw_c.fill_rect(228, 117, 4, 10, cyan_lab)
            blobs_sw_c = extract_blobs_from_maix_image(img_sw_c, self.cfg, scan_unknown_near=True)
            st_sw_c = RescueVisionProcessor(self.cfg).process(blobs_sw_c, **kw)
            self.assertTrue(st_sw_c.flags & VIS_FLAG_SWEEP_CLEAR, cls.__name__)
            self.assertFalse(st_sw_c.flags & VIS_FLAG_DROP_PATH_CLEAR, cls.__name__)

            # Positive sweep control: extend L-shape 4 pixels INTO near_sweep_roi at (232, 118, 4, 4) -> SWEEP_CLEAR=False!
            img_sw.fill_rect(232, 118, 4, 4, unknown_lab)
            blobs_sw_hit = extract_blobs_from_maix_image(img_sw, self.cfg, scan_unknown_near=True)
            st_sw_hit = RescueVisionProcessor(self.cfg).process(blobs_sw_hit, **kw)
            self.assertFalse(st_sw_hit.flags & VIS_FLAG_SWEEP_CLEAR, cls.__name__)

    def test_round11_cross_label_disambiguation_and_tier1_8conn_occupancy(self) -> None:
        floor_lab = (85, 0, 0)
        green_only_lab = (55, -35, 30)
        orange_only_lab = (65, 45, 60)
        shared_go_lab = (60, 10, 45)  # matches overlapping custom green & orange bands!
        cyan_only_lab = (68, -25, -20)
        shared_cg_lab = (60, -25, 10)  # matches overlapping custom cyan & green bands!
        kw = dict(camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False)

        ov_go_cfg = RescueVisionConfig()
        ov_go_cfg.lab_thresholds["green"] = [30, 82, -60, 20, 12, 55]
        ov_go_cfg.lab_thresholds["orange"] = [48, 88, 0, 58, 36, 78]

        # 1. Finding 2 (R11): Overlapping custom object LAB bands MUST disambiguate BEFORE emitting
        #    verified or rejected objects into blobs across ALL tiers!
        for cls in (_FakeMaixImage, _FakeMaixBinaryImage, _FakeMaixNativeStrideImage, _FakeMaix8ConnImage):
            # 1a. One 120-px (12x10) foreground patch matching both green & orange -> EXACTLY ONE unknown
            #     (NEVER 1 green + 1 orange!).
            img_120 = cls(320, 240, floor_lab)
            img_120.fill_rect(120, 150, 12, 10, shared_go_lab)
            blobs_120 = extract_blobs_from_maix_image(img_120, ov_go_cfg, scan_unknown_near=True)
            self.assertEqual([b.label for b in blobs_120 if b.label in ("green", "orange", "unknown")], ["unknown"], cls.__name__)
            st_120 = RescueVisionProcessor(ov_go_cfg).process(blobs_120, **kw)
            self.assertEqual(st_120.corral_green_cnt, 0, cls.__name__)
            self.assertEqual(st_120.corral_orange_cnt, 0, cls.__name__)
            self.assertEqual(st_120.corral_unknown_cnt, 1, cls.__name__)
            self.assertFalse(st_120.flags & VIS_FLAG_CORRAL_SAFE, cls.__name__)

            # 1b. Partially overlapping 150-px foreground patch (50 green-only + 50 shared + 50 orange-only):
            #     green sees 100 px, orange sees 100 px sharing 50 px -> STILL EXACTLY ONE unknown!
            img_part = cls(320, 240, floor_lab)
            img_part.fill_rect(120, 150, 5, 10, green_only_lab)
            img_part.fill_rect(125, 150, 5, 10, shared_go_lab)
            img_part.fill_rect(130, 150, 5, 10, orange_only_lab)
            blobs_part = extract_blobs_from_maix_image(img_part, ov_go_cfg, scan_unknown_near=True)
            self.assertEqual([b.label for b in blobs_part if b.label in ("green", "orange", "unknown")], ["unknown"], cls.__name__)
            st_part = RescueVisionProcessor(ov_go_cfg).process(blobs_part, **kw)
            self.assertEqual(st_part.corral_green_cnt, 0, cls.__name__)
            self.assertEqual(st_part.corral_orange_cnt, 0, cls.__name__)
            self.assertEqual(st_part.corral_unknown_cnt, 1, cls.__name__)
            self.assertFalse(st_part.flags & VIS_FLAG_CORRAL_SAFE, cls.__name__)

            # 1c. One 9600-px (120x80 > max_block_pixels=9500) oversized patch matching overlapping green & orange
            #     (both inside corral AND straddling corral boundary!) -> EXACTLY ONE rejected unknown (NEVER 2!).
            for ox, oy in ((100, 136), (80, 120)):
                img_big = cls(320, 240, floor_lab)
                img_big.fill_rect(ox, oy, 120, 80, shared_go_lab)
                blobs_big = extract_blobs_from_maix_image(img_big, ov_go_cfg, scan_unknown_near=True)
                self.assertEqual(len([b for b in blobs_big if b.label in ("green", "orange", "unknown")]), 1, cls.__name__)
                st_big = RescueVisionProcessor(ov_go_cfg).process(blobs_big, **kw)
                self.assertEqual(st_big.corral_unknown_cnt, 1, cls.__name__)
                self.assertFalse(st_big.flags & VIS_FLAG_CORRAL_SAFE, cls.__name__)

            # 1d. Disjoint pure green (144 px) + pure orange (144 px) under overlapping bands -> NO false conflict!
            img_disj = cls(320, 240, floor_lab)
            img_disj.fill_rect(110, 150, 12, 12, green_only_lab)
            img_disj.fill_rect(150, 150, 12, 12, orange_only_lab)
            blobs_disj = extract_blobs_from_maix_image(img_disj, ov_go_cfg, scan_unknown_near=True)
            st_disj = RescueVisionProcessor(ov_go_cfg).process(blobs_disj, **kw)
            self.assertEqual(st_disj.corral_green_cnt, 1, cls.__name__)
            self.assertEqual(st_disj.corral_orange_cnt, 1, cls.__name__)
            self.assertEqual(st_disj.corral_unknown_cnt, 0, cls.__name__)

            # 1e. Overlapping cyan & green bands on one 120-px patch -> hazard priority: 1 cyan, 0 green, 0 unknown!
            ov_cg_cfg = RescueVisionConfig()
            ov_cg_cfg.lab_thresholds["cyan"] = [48, 90, -48, -16, -42, 20]
            ov_cg_cfg.lab_thresholds["green"] = [30, 82, -60, -18, 0, 55]
            img_cg = cls(320, 240, floor_lab)
            img_cg.fill_rect(120, 150, 12, 10, shared_cg_lab)
            blobs_cg = extract_blobs_from_maix_image(img_cg, ov_cg_cfg, scan_unknown_near=True)
            st_cg = RescueVisionProcessor(ov_cg_cfg).process(blobs_cg, **kw)
            self.assertEqual(st_cg.corral_cyan_cnt, 1, cls.__name__)
            self.assertEqual(st_cg.corral_green_cnt, 0, cls.__name__)
            self.assertEqual(st_cg.corral_unknown_cnt, 0, cls.__name__)
            self.assertFalse(st_cg.flags & VIS_FLAG_CORRAL_SAFE, cls.__name__)

        # 2. Finding 3 (R11): Tier-1 pixel-clipped sweep occupancy with an 8-connected host image adapter
        #    (_FakeMaix8ConnImage exposes get_lab_pixel -> Tier 1!). An 84-px 8-connected cyan component
        #    strictly outside near_sweep_roi ([232, 118, 64, 106]) whose AABB overlaps near_sweep_roi
        #    MUST preserve SWEEP_CLEAR=True while clearing DROP_PATH_CLEAR=False!
        img_8c = _FakeMaix8ConnImage(320, 240, floor_lab)
        # Two 4-disconnected blocks joined diagonally at (231,117)-(232,116):
        # left vertical block: (228, 117, 4, 10) -> 40 px (x=228..231 < 232, y=117..126)
        # top horizontal block: (232, 115, 22, 2) -> 44 px (x=232..253, y=115..116 < 118)
        # 8-connected total = 84 px, AABB = (228, 115, 26, 12) overlaps sweep [232..296, 118..224] with 0 actual pixels!
        img_8c.fill_rect(228, 117, 4, 10, cyan_only_lab)
        img_8c.fill_rect(232, 115, 22, 2, cyan_only_lab)
        blobs_8c = extract_blobs_from_maix_image(img_8c, self.cfg, scan_unknown_near=True)
        cyan_8c = [b for b in blobs_8c if b.label == "cyan"]
        self.assertEqual(len(cyan_8c), 1)
        self.assertEqual(cyan_8c[0].pixels, 84)
        self.assertIs(cyan_8c[0].in_sweep, False)
        st_8c = RescueVisionProcessor(self.cfg).process(blobs_8c, **kw)
        self.assertTrue(st_8c.flags & VIS_FLAG_SWEEP_CLEAR)
        self.assertFalse(st_8c.flags & VIS_FLAG_DROP_PATH_CLEAR)

        # Positive control: extend 1 diagonal pixel INTO near_sweep_roi at (232, 118) -> SWEEP_CLEAR=False!
        img_8c.fill_rect(232, 118, 1, 1, cyan_only_lab)
        blobs_8c_hit = extract_blobs_from_maix_image(img_8c, self.cfg, scan_unknown_near=True)
        st_8c_hit = RescueVisionProcessor(self.cfg).process(blobs_8c_hit, **kw)
        self.assertFalse(st_8c_hit.flags & VIS_FLAG_SWEEP_CLEAR)

    def test_round12_cyan_footprint_low_thresh_conflict_tier3_and_ramp_adjacency(self) -> None:
        from types import SimpleNamespace

        floor = (85, 0, 0)
        shared_cg = (60, -25, 10)
        green_only = (55, -35, 30)
        orange_only = (65, 45, 60)
        shared_go = (60, 10, 45)
        red = (52, 56, 22)
        purple = (45, 35, -30)
        unknown_lab = (55, 0, 45)
        kw = dict(camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False)

        # 1. P1-5: Cyan conflict MUST retain cyan priority AND use the FULL connected conflict footprint
        #    across ALL safety regions (drop ROI, sweep ROI, corral) with NO second min-size test on clipped tail!
        cg_cfg = RescueVisionConfig()
        cg_cfg.lab_thresholds["cyan"] = [48, 90, -48, -16, -42, 20]
        cg_cfg.lab_thresholds["green"] = [30, 82, -60, -18, 0, 55]
        for cls in (_FakeMaixImage, _FakeMaixBinaryImage, _FakeMaixNativeStrideImage):
            # 100-px cyan-connected tail inside near_drop_roi (x=40..50) -> MUST clear DROP_PATH_CLEAR!
            img_drop = cls(320, 240, floor)
            img_drop.fill_rect(97, 25, 200, 50, red)
            img_drop.fill_rect(97, 75, 200, 8, purple)
            img_drop.fill_rect(30, 100, 10, 10, shared_cg)
            img_drop.fill_rect(40, 100, 10, 10, green_only)
            blobs_drop = extract_blobs_from_maix_image(img_drop, cg_cfg, scan_unknown_near=True)
            self.assertEqual([b.label for b in blobs_drop if b.label in ("cyan", "green", "unknown")], ["cyan"], cls.__name__)
            st_drop = RescueVisionProcessor(cg_cfg).process(blobs_drop, **kw)
            self.assertFalse(st_drop.flags & VIS_FLAG_DROP_PATH_CLEAR, cls.__name__)

            # 48-px (< min_hazard_pixels=60) cyan-connected tail inside near_sweep_roi -> MUST clear SWEEP_CLEAR!
            img_sw = cls(320, 240, floor)
            img_sw.fill_rect(232, 108, 12, 10, shared_cg)
            img_sw.fill_rect(232, 118, 12, 4, green_only)
            blobs_sw = extract_blobs_from_maix_image(img_sw, cg_cfg, scan_unknown_near=True)
            self.assertEqual([b.label for b in blobs_sw if b.label in ("cyan", "green", "unknown")], ["cyan"], cls.__name__)
            st_sw = RescueVisionProcessor(cg_cfg).process(blobs_sw, **kw)
            self.assertFalse(st_sw.flags & VIS_FLAG_SWEEP_CLEAR, cls.__name__)

        # 2. P2-2: Competing alternate-class components down to min_hazard_pixels (60) participate in
        #    cross-label disambiguation, while standalone < min_block_pixels (90) is still NOT accepted as legal!
        go_cfg = RescueVisionConfig()
        go_cfg.lab_thresholds["green"] = [30, 82, -60, 20, 12, 55]
        go_cfg.lab_thresholds["orange"] = [48, 88, 0, 58, 36, 78]
        for cls in (_FakeMaixImage, _FakeMaixBinaryImage, _FakeMaixNativeStrideImage):
            img_70 = cls(320, 240, floor)
            img_70.fill_rect(120, 150, 5, 10, green_only)  # 50 green-only
            img_70.fill_rect(125, 150, 7, 10, shared_go)   # 70 shared -> green=120, orange=70 (>= 60, < 90)!
            blobs_70 = extract_blobs_from_maix_image(img_70, go_cfg, scan_unknown_near=True)
            st_70 = RescueVisionProcessor(go_cfg).process(blobs_70, **kw)
            self.assertEqual(st_70.corral_green_cnt, 0, cls.__name__)
            self.assertEqual(st_70.corral_orange_cnt, 0, cls.__name__)
            self.assertEqual(st_70.corral_unknown_cnt, 1, cls.__name__)
            self.assertFalse(st_70.flags & VIS_FLAG_CORRAL_SAFE, cls.__name__)

            # Standalone 70-px orange-only patch MUST NOT be accepted as a legal orange block!
            img_only70 = cls(320, 240, floor)
            img_only70.fill_rect(120, 150, 7, 10, orange_only)
            blobs_only70 = extract_blobs_from_maix_image(img_only70, go_cfg, scan_unknown_near=True)
            st_only70 = RescueVisionProcessor(go_cfg).process(blobs_only70, **kw)
            self.assertEqual(st_only70.corral_orange_cnt, 0, cls.__name__)
            self.assertEqual(st_only70.corral_unknown_cnt, 1, cls.__name__)

        # 3. P2-3: Tier 3 (find_blobs-only adapter) subtracts accounted conflict/rejected pixels instead of
        #    re-emitting ROI slices -> straddling 9600-px patch produces EXACTLY ONE unknown!
        img_t3 = _FakeMaixImage(320, 240, floor)
        img_t3.fill_rect(80, 120, 120, 80, shared_go)
        t3_adapter = SimpleNamespace(find_blobs=img_t3.find_blobs)
        blobs_t3 = extract_blobs_from_maix_image(t3_adapter, go_cfg, scan_unknown_near=True)
        self.assertEqual([(b.label, b.pixels) for b in blobs_t3], [("unknown", 9600)])
        st_t3 = RescueVisionProcessor(go_cfg).process(blobs_t3, **kw)
        self.assertEqual(st_t3.corral_unknown_cnt, 1)
        # Adding a separate 100-px uncalibrated object in corral is STILL detected!
        img_t3.fill_rect(210, 150, 10, 10, unknown_lab)
        blobs_t3_extra = extract_blobs_from_maix_image(SimpleNamespace(find_blobs=img_t3.find_blobs), go_cfg, scan_unknown_near=True)
        st_t3_extra = RescueVisionProcessor(go_cfg).process(blobs_t3_extra, **kw)
        self.assertEqual(st_t3_extra.corral_unknown_cnt, 2)

        # 4. P2-4: Detached ramp MUST NOT override adjacent ramp or allow cargo past home.y2 to confirm!
        proc = RescueVisionProcessor(self.cfg)
        home = Blob("red_zone", 97, 25, 200, 50, 10000)         # y = 25..75
        correct_ramp = Blob("purple_ramp", 97, 75, 190, 4, 760) # adjacent at y = 75..79
        detached_ramp = Blob("purple_ramp", 97, 110, 200, 8, 1600)  # detached 35 px below!
        surfaces = [home, correct_ramp, detached_ramp]
        st_base = proc.process(
            surfaces + [Blob("green", 140, 150, 12, 10, 120)],
            True, True, TEAM_RED, False,
            mission_state=MISSION_DROP_OPEN_AND_PUSH, gate_closed=True,
        )
        self.assertTrue(st_base.flags & VIS_FLAG_DROP_BASELINE_READY)
        # Cargo ending at y=87 (> home.y2=75) MUST NOT confirm!
        st_partial = proc.process(
            surfaces + [Blob("green", 140, 69, 12, 18, 216)],
            True, True, TEAM_RED, False,
            mission_state=MISSION_DROP_BACKOUT_VERIFY, gate_closed=False,
        )
        self.assertFalse(st_partial.flags & VIS_FLAG_DROP_CONFIRMED)
        # Cargo fully inside own material half AND behind home.y2=75 & correct_ramp.y=75 DOES confirm!
        st_full = proc.process(
            surfaces + [Blob("green", 140, 40, 12, 18, 216)],
            True, True, TEAM_RED, False,
            mission_state=MISSION_DROP_BACKOUT_VERIFY, gate_closed=False,
        )
        self.assertTrue(st_full.flags & VIS_FLAG_DROP_CONFIRMED)
        # Ambiguous competing adjacent ramps -> fail closed!
        amb_ramp_b = Blob("purple_ramp", 120, 79, 150, 4, 600)
        st_amb = proc.process(
            [home, correct_ramp, amb_ramp_b, Blob("green", 140, 40, 12, 18, 216)],
            True, True, TEAM_RED, False,
            mission_state=MISSION_DROP_BACKOUT_VERIFY, gate_closed=False,
        )
        self.assertFalse(st_amb.flags & VIS_FLAG_DROP_PATH_CLEAR)
        self.assertFalse(st_amb.flags & VIS_FLAG_DROP_CONFIRMED)
        # Only detached ramp (no adjacent ramp) -> fail closed!
        st_det = proc.process(
            [home, detached_ramp, Blob("green", 140, 40, 12, 18, 216)],
            True, True, TEAM_RED, False,
            mission_state=MISSION_DROP_BACKOUT_VERIFY, gate_closed=False,
        )
        self.assertFalse(st_det.flags & VIS_FLAG_DROP_PATH_CLEAR)
        self.assertFalse(st_det.flags & VIS_FLAG_DROP_CONFIRMED)

        # 5. Round 13 P2-4: Protruding ramp wings MUST be included in misplaced occupancy so a stray block
        #    on a protruding ramp wing invalidates baseline and can NEVER substitute for trip cargo!
        proc_wing = RescueVisionProcessor(self.cfg)
        wing_ramp = Blob("purple_ramp", 70, 75, 250, 12, 3000)  # protrudes left/right of home [97, 297)
        stray_wing = Blob("green", 71, 75, 12, 10, 120)
        st_wing_base = proc_wing.process(
            [home, wing_ramp, stray_wing, Blob("green", 140, 150, 12, 10, 120)],
            True, True, TEAM_RED, False,
            mission_state=MISSION_DROP_OPEN_AND_PUSH, gate_closed=True,
        )
        self.assertFalse(st_wing_base.flags & VIS_FLAG_DROP_BASELINE_READY)
        st_wing_post = proc_wing.process(
            [home, wing_ramp, Blob("green", 140, 40, 12, 10, 120)],
            True, True, TEAM_RED, False,
            mission_state=MISSION_DROP_BACKOUT_VERIFY, gate_closed=False,
        )
        self.assertFalse(st_wing_post.flags & VIS_FLAG_DROP_CONFIRMED)

        # 6. Round 13 P2-3: Tier 3 MUST subtract component occupancy BEFORE deduplicating residuals so an
        #    adjacent 210-px unknown enclosed inside a concave L-conflict's AABB is NOT suppressed!
        img_l_t3 = _FakeMaixImage(320, 240, floor)
        img_l_t3.fill_rect(100, 130, 53, 4, shared_go)
        img_l_t3.fill_rect(100, 134, 4, 41, shared_go)
        img_l_t3.fill_rect(104, 144, 15, 14, (55, 0, 85))  # 210-px uncalibrated object inside L's AABB!
        exact_l = extract_blobs_from_maix_image(img_l_t3, go_cfg, scan_unknown_near=True)
        tier3_l = extract_blobs_from_maix_image(
            SimpleNamespace(find_blobs=img_l_t3.find_blobs), go_cfg, scan_unknown_near=True
        )
        self.assertEqual(sum(b.label == "unknown" for b in exact_l), 2)
        self.assertEqual(sum(b.label == "unknown" for b in tier3_l), 2)

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

    def test_round14_tier3_concave_unknown_around_disconnected_known_block(self) -> None:
        """R14 P2-5 & R15 P2-4/P1-2: paired Tier-3 concave unknown vs concave known ramp, plus pre-match settle gen ack."""
        from types import SimpleNamespace
        from maixcam.protocol import VIS_FLAG_PRE_MATCH_GEN_MASK, VIS_FLAG_PRE_MATCH_GEN_SHIFT

        cfg = RescueVisionConfig()
        # 1. R14 P2-5: 128-pixel C-shaped unknown surrounding a disconnected 144-pixel green block -> MUST detect unknown!
        img = _FakeMaixImage(320, 240, (85, 0, 0))
        unknown_lab = (55, 0, 85)
        img.fill_rect(120, 40, 45, 1, unknown_lab)
        img.fill_rect(120, 41, 1, 38, unknown_lab)
        img.fill_rect(120, 79, 45, 1, unknown_lab)
        img.fill_rect(135, 55, 12, 12, (55, -35, 30))

        exact = extract_blobs_from_maix_image(img, cfg, scan_unknown_near=True)
        fallback = extract_blobs_from_maix_image(
            SimpleNamespace(find_blobs=img.find_blobs), cfg, scan_unknown_near=True
        )
        self.assertTrue(any(b.label == "unknown" and b.pixels == 128 for b in exact))
        self.assertTrue(any(b.label == "unknown" and b.pixels == 128 for b in fallback))
        self.assertTrue(any(b.label == "green" and b.pixels == 144 for b in fallback))
        self.assertFalse(
            any(b.label == "unknown" and (b.x, b.y, b.w, b.h) == (135, 55, 12, 12) for b in fallback)
        )

        # 2. R15 P2-4 paired regression: 1,312-pixel C-shaped calibrated purple_ramp surrounding a disconnected
        #    144-pixel green block -> MUST emit ZERO unknowns in BOTH Tier 1 and Tier 3!
        ramp_lab = (45, 25, -25)
        img_ramp = _FakeMaixImage(320, 240, (85, 0, 0))
        img_ramp.fill_rect(100, 30, 80, 10, ramp_lab)  # top bar: 800 px
        img_ramp.fill_rect(100, 40, 10, 40, ramp_lab)  # left spine: 400 px
        img_ramp.fill_rect(100, 80, 80, 10, ramp_lab)  # bottom bar: 800 px (total 2000 px C-shape; trim to ~1312)
        img_ramp = _FakeMaixImage(320, 240, (85, 0, 0))
        img_ramp.fill_rect(100, 30, 68, 8, ramp_lab)   # 544 px
        img_ramp.fill_rect(100, 38, 8, 28, ramp_lab)   # 224 px
        img_ramp.fill_rect(100, 66, 68, 8, ramp_lab)   # 544 px -> total = 1312 px C-shaped purple_ramp!
        img_ramp.fill_rect(128, 46, 12, 12, (55, -35, 30))  # disconnected 144 px green inside the C hollow!
        exact_ramp = extract_blobs_from_maix_image(img_ramp, cfg, scan_unknown_near=True)
        fallback_ramp = extract_blobs_from_maix_image(
            SimpleNamespace(find_blobs=img_ramp.find_blobs), cfg, scan_unknown_near=True
        )
        self.assertEqual([b for b in exact_ramp if b.label == "unknown"], [])
        self.assertEqual([b for b in fallback_ramp if b.label == "unknown"], [])
        self.assertTrue(any(b.label == "purple_ramp" and b.pixels == 1312 for b in fallback_ramp))
        self.assertTrue(any(b.label == "green" and b.pixels == 144 for b in fallback_ramp))

        # 2b. R16 P2-4 interlocking geometry: 1,584-px purple_ramp with a tongue protruding INSIDE
        #     the bounding box of a disconnected 528-px reversed-C green block -> ZERO unknowns in Tier 1 & Tier 3!
        img_lock = _FakeMaixImage(320, 240, (85, 0, 0))
        for rx, ry, rw, rh in ((100, 30, 68, 8), (100, 38, 8, 28), (100, 66, 68, 8), (108, 49, 34, 8)):
            img_lock.fill_rect(rx, ry, rw, rh, ramp_lab)
        for gx, gy, gw, gh in ((120, 41, 38, 6), (152, 47, 6, 12), (120, 59, 38, 6)):
            img_lock.fill_rect(gx, gy, gw, gh, (55, -35, 30))
        exact_lock = extract_blobs_from_maix_image(img_lock, cfg, scan_unknown_near=True)
        fallback_lock = extract_blobs_from_maix_image(
            SimpleNamespace(find_blobs=img_lock.find_blobs), cfg, scan_unknown_near=True
        )
        self.assertEqual([b for b in exact_lock if b.label == "unknown"], [])
        self.assertEqual([b for b in fallback_lock if b.label == "unknown"], [])
        self.assertTrue(any(b.label == "purple_ramp" and b.pixels == 1584 for b in fallback_lock))
        self.assertTrue(any(b.label == "green" and b.pixels == 528 for b in fallback_lock))

        # 2c. Adding a 120-px unknown wart attached to the interlocking purple_ramp MUST still be caught in Tier 3!
        img_lock.fill_rect(100, 74, 15, 8, unknown_lab)  # 120 px unknown attached to ramp bottom bar
        fallback_wart = extract_blobs_from_maix_image(
            SimpleNamespace(find_blobs=img_lock.find_blobs), cfg, scan_unknown_near=True
        )
        self.assertTrue(any(b.label == "unknown" for b in fallback_wart))

        # 2d. R17 P2-2: floor-overlapping calibration inside an unknown ring MUST NOT be credited as foreground in Tier 3!
        cfg_floor_overlap = RescueVisionConfig()
        cfg_floor_overlap.lab_thresholds["purple_ramp"] = [20, 75, 12, 60, -70, -12]
        cfg_floor_overlap.validate()
        img_ring = _FakeMaixImage(320, 240, (85, 0, 0))
        img_ring.fill_rect(100, 140, 42, 42, (55, 5, 30))      # 164-px unknown ring border
        img_ring.fill_rect(101, 141, 40, 40, (75, 14, -14))    # 1,600-px floor-matching patch overlapping purple_ramp
        for src in (img_ring, SimpleNamespace(find_blobs=img_ring.find_blobs)):
            ring_blobs = extract_blobs_from_maix_image(src, cfg_floor_overlap, scan_unknown_near=True)
            ring_state = RescueVisionProcessor(cfg_floor_overlap).process(
                ring_blobs,
                camera_near_cmd=True,
                camera_settled=True,
                team=TEAM_RED,
                first_green_unlocked=False,
                carrying_casualty=False,
                max_materials_per_trip=1,
            )
            self.assertEqual(ring_state.corral_unknown_cnt, 1)
            self.assertFalse(bool(ring_state.flags & VIS_FLAG_CORRAL_SAFE))
            self.assertTrue(any(b.label == "unknown" and b.pixels == 164 for b in ring_blobs))

        # 3. R15 P1-2, R16 P1-2 & R17 P1-1: MaixCAM echoes pre_match_settle_gen ONLY when FAR settled and gate is open & settled,
        #    AND run_maixcam() drains queued + in-flight frames (buff_num + 1 reads) without relying on no-op clear_buff()!
        proc = RescueVisionProcessor(cfg)
        st_closed = proc.process(
            [], camera_near_cmd=False, camera_settled=True, team=TEAM_RED,
            first_green_unlocked=False, gate_closed=True, gate_settled=True, pre_match_settle_gen=2,
        )
        self.assertEqual((st_closed.flags & VIS_FLAG_PRE_MATCH_GEN_MASK) >> VIS_FLAG_PRE_MATCH_GEN_SHIFT, 0)
        st_unsettled = proc.process(
            [], camera_near_cmd=False, camera_settled=True, team=TEAM_RED,
            first_green_unlocked=False, gate_closed=False, gate_settled=False, pre_match_settle_gen=2,
        )
        self.assertEqual((st_unsettled.flags & VIS_FLAG_PRE_MATCH_GEN_MASK) >> VIS_FLAG_PRE_MATCH_GEN_SHIFT, 0)
        st_open_settled = proc.process(
            [], camera_near_cmd=False, camera_settled=True, team=TEAM_RED,
            first_green_unlocked=False, gate_closed=False, gate_settled=True, pre_match_settle_gen=2,
        )
        self.assertEqual((st_open_settled.flags & VIS_FLAG_PRE_MATCH_GEN_MASK) >> VIS_FLAG_PRE_MATCH_GEN_SHIFT, 2)

        # End-to-end run_maixcam() drain check:
        #   - callable no-op clear_buff() (matching MaixCAM MMF maix_camera_mmf.cpp) with default buff_num=3 -> drains 4 reads
        #   - larger backend buff_num=5 with no-op clear_buff() -> drains 6 reads
        #   - no clear_buff() attribute at all -> drains 4 reads
        #   - multi-frame UART burst toggling settled True -> False -> True in one serial.read() -> re-arms full drain
        from unittest.mock import patch
        from maixcam import main as maix_main
        from maixcam.protocol import (
            CAMERA_VIEW_FAR,
            CAMERA_VIEW_TRANSITION,
            TEL_FLAG_CAMERA_SETTLED,
            TEL_FLAG_GATE_SETTLED,
            TEL_FLAG_PRE_MATCH_GEN_SHIFT,
            decode_vision_state,
            encode_telemetry,
        )

        payload_gen1 = encode_telemetry(
            1,
            Telemetry(
                status_flags=TEL_FLAG_CAMERA_SETTLED
                | TEL_FLAG_GATE_SETTLED
                | (1 << TEL_FLAG_PRE_MATCH_GEN_SHIFT),
            ),
        )
        payload_unsettled = encode_telemetry(2, Telemetry(status_flags=0))
        payload_gen1_again = encode_telemetry(
            3,
            Telemetry(
                status_flags=TEL_FLAG_CAMERA_SETTLED
                | TEL_FLAG_GATE_SETTLED
                | (1 << TEL_FLAG_PRE_MATCH_GEN_SHIFT),
            ),
        )

        for mode, extra_buff_num, expected_drain in (
            ("noop_clear_default", None, 4),
            ("noop_clear_buff5", 5, 6),
            ("no_clear_attr", None, 4),
        ):
            stale_frames = [SimpleNamespace(resize=lambda *_: None, tag=f"stale_{i}") for i in range(expected_drain)]
            fresh = SimpleNamespace(resize=lambda *_: None, tag="fresh")
            queue = list(stale_frames)
            written: list[bytes] = []

            def _read_frame(q=queue, fr=fresh):
                return q.pop(0) if q else fr

            cam_kwargs = {"read": _read_frame}
            if mode != "no_clear_attr":
                cam_kwargs["clear_buff"] = lambda: None  # real MaixCAM MMF logs warning and returns without flushing
            if extra_buff_num is not None:
                cam_kwargs["buff_num"] = extra_buff_num
            cam_obj = SimpleNamespace(**cam_kwargs)

            n_iters = expected_drain + 1
            exit_seq = iter([False] * n_iters + [True]).__next__
            mono_seq = iter([0.0] + [1.0 + 0.06 * i for i in range(n_iters)]).__next__
            fake_maix = SimpleNamespace(
                app=SimpleNamespace(need_exit=exit_seq),
                camera=SimpleNamespace(Camera=lambda *_a, **_k: cam_obj),
                display=SimpleNamespace(Display=lambda: SimpleNamespace(width=lambda: 552, height=lambda: 368, show=lambda _: None)),
                pinmap=SimpleNamespace(set_pin_function=lambda *_: None),
                touchscreen=SimpleNamespace(TouchScreen=lambda: SimpleNamespace(read=lambda: (0, 0, False))),
                uart=SimpleNamespace(UART=lambda *_: SimpleNamespace(read=lambda: payload_gen1, write=written.append)),
            )

            def _fake_blobs(frame, *_a, **_k):
                return [Blob("green", 145, 150, 30, 25, 750, 1.0)] if getattr(frame, "tag", "") != "fresh" else []

            with (
                patch.dict(sys.modules, {"maix": fake_maix}),
                patch.object(maix_main, "time", SimpleNamespace(monotonic=mono_seq)),
                patch.object(maix_main, "extract_blobs_from_maix_image", side_effect=_fake_blobs),
                patch.object(maix_main, "_extract_motion_grid", return_value=[0] * (16 * 12)),
                patch.object(maix_main, "render_touch_overlay"),
            ):
                maix_main.run_maixcam()

            p_check = RescueProtocolParser()
            states = [
                vs
                for raw_tx in written
                for fr in p_check.feed(raw_tx)
                if (vs := decode_vision_state(fr)) is not None
            ]
            self.assertEqual(len(states), n_iters, msg=mode)
            for idx in range(expected_drain):
                echo_gen = (states[idx].flags & VIS_FLAG_PRE_MATCH_GEN_MASK) >> VIS_FLAG_PRE_MATCH_GEN_SHIFT
                self.assertEqual(echo_gen, 0, msg=f"{mode} drain iter {idx}")
                self.assertEqual(states[idx].camera_view, CAMERA_VIEW_TRANSITION, msg=f"{mode} drain iter {idx}")
                self.assertEqual(states[idx].target_type, 0, msg=f"{mode} drain iter {idx}")
            final_gen = (states[-1].flags & VIS_FLAG_PRE_MATCH_GEN_MASK) >> VIS_FLAG_PRE_MATCH_GEN_SHIFT
            self.assertEqual(final_gen, 1, msg=mode)
            self.assertEqual(states[-1].camera_view, CAMERA_VIEW_FAR, msg=mode)
            self.assertEqual(states[-1].target_type, 0, msg=mode)

        # Multi-frame burst re-arm: once drained (iter 4), a single serial.read() containing
        # [unsettled, settled(gen=1)] MUST re-arm a full 4-frame drain before echoing gen=1 again!
        burst_frames = (
            [SimpleNamespace(resize=lambda *_: None, tag=f"pre0_{i}") for i in range(4)]
            + [SimpleNamespace(resize=lambda *_: None, tag="fresh0")]
            + [SimpleNamespace(resize=lambda *_: None, tag=f"pre1_{i}") for i in range(4)]
            + [SimpleNamespace(resize=lambda *_: None, tag="fresh1")]
        )
        burst_rx = iter(
            [payload_gen1, b"", b"", b"", b"", payload_unsettled + payload_gen1_again, b"", b"", b"", b""]
        ).__next__
        burst_written: list[bytes] = []
        burst_cam = SimpleNamespace(read=iter(burst_frames).__next__, clear_buff=lambda: None)
        burst_maix = SimpleNamespace(
            app=SimpleNamespace(need_exit=iter([False] * 10 + [True]).__next__),
            camera=SimpleNamespace(Camera=lambda *_a, **_k: burst_cam),
            display=SimpleNamespace(Display=lambda: SimpleNamespace(width=lambda: 552, height=lambda: 368, show=lambda _: None)),
            pinmap=SimpleNamespace(set_pin_function=lambda *_: None),
            touchscreen=SimpleNamespace(TouchScreen=lambda: SimpleNamespace(read=lambda: (0, 0, False))),
            uart=SimpleNamespace(UART=lambda *_: SimpleNamespace(read=burst_rx, write=burst_written.append)),
        )
        with (
            patch.dict(sys.modules, {"maix": burst_maix}),
            patch.object(maix_main, "time", SimpleNamespace(monotonic=iter([0.0] + [1.0 + 0.06 * i for i in range(10)]).__next__)),
            patch.object(maix_main, "extract_blobs_from_maix_image", return_value=[]),
            patch.object(maix_main, "_extract_motion_grid", return_value=[0] * (16 * 12)),
            patch.object(maix_main, "render_touch_overlay"),
        ):
            maix_main.run_maixcam()
        p_burst = RescueProtocolParser()
        burst_states = [
            vs
            for raw_tx in burst_written
            for fr in p_burst.feed(raw_tx)
            if (vs := decode_vision_state(fr)) is not None
        ]
        self.assertEqual(len(burst_states), 10)
        burst_gens = [(s.flags & VIS_FLAG_PRE_MATCH_GEN_MASK) >> VIS_FLAG_PRE_MATCH_GEN_SHIFT for s in burst_states]
        self.assertEqual(burst_gens, [0, 0, 0, 0, 1, 0, 0, 0, 0, 1])

    def test_round18_gate_closed_drain_and_floor_overlap_cargo_exclusion(self) -> None:
        """R18 & R19: cycle-bound close drain (including missed open/reclose telemetry), cyan floor fail-closed, and Tier-3 cached L-green."""
        from types import SimpleNamespace
        from unittest.mock import patch
        from maixcam import main as maix_main
        from maixcam.protocol import (
            TEL_FLAG_CAMERA_NEAR,
            TEL_FLAG_CAMERA_SETTLED,
            TEL_FLAG_GATE_CLOSED,
            TEL_FLAG_GATE_SETTLED,
            TEL_FLAG_SETTLE_GEN_SHIFT,
        )

        # 1. R18 P1 & R19 P1: Camera pitch stays settled in NEAR while gate closes open -> closing -> closed_settled(gen=1),
        #    then MaixCAM MISSES ALL opening/reclosing telemetry during an unload retry and receives closed_settled(gen=2) directly!
        #    - All 4 buffered pre-close frames for gen=1 emit CAMERA_VIEW_NEAR with LIVE hazard detection and settle_gen=0;
        #      frame 5 emits settle_gen=1 (empty corral).
        #    - Receiving gen=2 directly WITHOUT seeing an open/unsettled edge MUST re-arm the 4-frame drain (emitting settle_gen=0
        #      for all 4 queued pre-reclose frames!) and echo settle_gen=2 ONLY on frame 5!
        tel_near_open = encode_telemetry(
            1, Telemetry(status_flags=TEL_FLAG_CAMERA_NEAR | TEL_FLAG_CAMERA_SETTLED | TEL_FLAG_GATE_SETTLED)
        )
        tel_near_closing = encode_telemetry(
            2, Telemetry(status_flags=TEL_FLAG_CAMERA_NEAR | TEL_FLAG_CAMERA_SETTLED | TEL_FLAG_GATE_CLOSED)
        )
        tel_near_closed_gen1 = encode_telemetry(
            3,
            Telemetry(
                status_flags=TEL_FLAG_CAMERA_NEAR
                | TEL_FLAG_CAMERA_SETTLED
                | TEL_FLAG_GATE_CLOSED
                | TEL_FLAG_GATE_SETTLED
                | (1 << TEL_FLAG_SETTLE_GEN_SHIFT)
            ),
        )
        tel_near_closed_gen2 = encode_telemetry(
            4,
            Telemetry(
                status_flags=TEL_FLAG_CAMERA_NEAR
                | TEL_FLAG_CAMERA_SETTLED
                | TEL_FLAG_GATE_CLOSED
                | TEL_FLAG_GATE_SETTLED
                | (2 << TEL_FLAG_SETTLE_GEN_SHIFT)
            ),
        )
        tags = (
            [f"open_drain_{i}" for i in range(4)]
            + ["open_live"]
            + ["closing_live"]
            + ["pre_close_0", "pre_close_cyan", "pre_close_2", "pre_close_3"]
            + ["post_close_empty"]
            + ["reclose_pre_0", "reclose_pre_1", "reclose_pre_2", "reclose_pre_3"]
            + ["reclose_post_fresh"]
        )
        rx_seq = (
            [tel_near_open, b"", b"", b"", b""]
            + [tel_near_closing]
            + [tel_near_closed_gen1, b"", b"", b""]
            + [b""]
            + [tel_near_closed_gen2, b"", b"", b""]  # missed open/reclose telemetry entirely!
            + [b""]
        )
        self.assertEqual(len(tags), 16)
        self.assertEqual(len(rx_seq), 16)

        frames = [SimpleNamespace(resize=lambda *_: None, tag=t) for t in tags]
        written: list[bytes] = []
        cam_obj = SimpleNamespace(read=iter(frames).__next__, clear_buff=lambda: None)
        fake_maix = SimpleNamespace(
            app=SimpleNamespace(need_exit=iter([False] * len(tags) + [True]).__next__),
            camera=SimpleNamespace(Camera=lambda *_a, **_k: cam_obj),
            display=SimpleNamespace(Display=lambda: SimpleNamespace(width=lambda: 552, height=lambda: 368, show=lambda _: None)),
            pinmap=SimpleNamespace(set_pin_function=lambda *_: None),
            touchscreen=SimpleNamespace(TouchScreen=lambda: SimpleNamespace(read=lambda: (0, 0, False))),
            uart=SimpleNamespace(UART=lambda *_: SimpleNamespace(read=iter(rx_seq).__next__, write=written.append)),
        )

        def _blobs_by_tag(frame, *_a, **_k):
            tag = frame.tag
            if tag == "pre_close_cyan":
                return [
                    Blob("green", 140, 150, 16, 16, 256, 1.0),
                    Blob("cyan", 120, 150, 10, 10, 100, 1.0),
                ]
            if tag == "post_close_empty":
                return []
            return [Blob("green", 140, 150, 16, 16, 256, 1.0)]

        with (
            patch.dict(sys.modules, {"maix": fake_maix}),
            patch.object(maix_main, "time", SimpleNamespace(monotonic=iter([0.0] + [1.0 + 0.06 * i for i in range(len(tags))]).__next__)),
            patch.object(maix_main, "extract_blobs_from_maix_image", side_effect=_blobs_by_tag),
            patch.object(maix_main, "_extract_motion_grid", return_value=[0] * (16 * 12)),
            patch.object(maix_main, "render_touch_overlay"),
        ):
            maix_main.run_maixcam()

        p_close = RescueProtocolParser()
        close_states = [
            vs
            for raw_tx in written
            for fr in p_close.feed(raw_tx)
            if (vs := decode_vision_state(fr)) is not None
        ]
        self.assertEqual(len(close_states), len(tags))
        # Indices 6..9 are the 4 buffered pre-close frames after closed_settled(gen=1) arrived:
        for idx in range(6, 10):
            self.assertEqual(close_states[idx].camera_view, CAMERA_VIEW_NEAR, msg=tags[idx])
            self.assertEqual(
                (close_states[idx].flags & VIS_FLAG_SETTLE_GEN_MASK) >> VIS_FLAG_SETTLE_GEN_SHIFT, 0, msg=tags[idx]
            )
        # Hazard monitoring MUST remain live during the drain (index 7 has cyan hazard)!
        self.assertEqual(close_states[7].corral_cyan_cnt, 1)
        self.assertFalse(bool(close_states[7].flags & VIS_FLAG_CORRAL_SAFE))
        # Index 10 is the first genuine post-close image (empty corral) -> settle_gen == 1, green == 0!
        self.assertEqual(close_states[10].camera_view, CAMERA_VIEW_NEAR)
        self.assertEqual((close_states[10].flags & VIS_FLAG_SETTLE_GEN_MASK) >> VIS_FLAG_SETTLE_GEN_SHIFT, 1)
        self.assertEqual(close_states[10].corral_green_cnt, 0)
        # Indices 11..14 after direct gen=1 -> gen=2 jump (missed open/reclose telemetry!) MUST drain with gen == 0,
        # and index 15 echoes settle_gen == 2!
        for idx in range(11, 15):
            self.assertEqual(close_states[idx].camera_view, CAMERA_VIEW_NEAR, msg=tags[idx])
            self.assertEqual(
                (close_states[idx].flags & VIS_FLAG_SETTLE_GEN_MASK) >> VIS_FLAG_SETTLE_GEN_SHIFT, 0, msg=tags[idx]
            )
        self.assertEqual((close_states[15].flags & VIS_FLAG_SETTLE_GEN_MASK) >> VIS_FLAG_SETTLE_GEN_SHIFT, 2)

        # 2. R18 P2, R19 P1-2, R19 P2:
        #    - Floor-only patch under floor-overlapping green calibration -> NEVER accepted as green!
        #    - Cyan calibration overlapping floor band -> MUST retain cyan hazard (fail closed) across all 3 tiers!
        #    - 110-px L-shaped non-floor green component under floor-overlapping calibration -> 1 green, 0 unknown in ALL 3 tiers!
        cfg_overlap = RescueVisionConfig()
        cfg_overlap.lab_thresholds["green"] = [30, 82, -60, 14, 12, 55]  # overlaps arena_floor_lab at (75, -14, 14)
        cfg_overlap.lab_thresholds["cyan"] = [48, 90, -48, -6, -42, -6]  # overlaps arena_floor_lab at (75, -10, -10), not (85,0,0)
        cfg_overlap.validate()
        floor_base = (85, 0, 0)
        floor_green_overlap = (75, -14, 14)  # matches BOTH arena_floor_lab AND green!
        floor_cyan_overlap = (75, -10, -10)  # matches BOTH arena_floor_lab AND cyan!
        real_green = (55, -35, 30)           # non-floor green
        unknown_lab = (55, 5, 85)            # non-floor unknown (outside green [30..82,-60..14,12..55] and all bands)

        def _to_tier2(im: _FakeMaixImage) -> _FakeMaixBinaryImage:
            b = _FakeMaixBinaryImage(im.width, im.height, floor_base)
            b._lab_rows = [list(row) for row in im._grid]
            return b

        for tier_name, make_src in (
            ("Tier1", lambda im: im),
            ("Tier2", _to_tier2),
            ("Tier3", lambda im: SimpleNamespace(find_blobs=im.find_blobs)),
        ):
            # 2a. Pure 600-px floor-only patch inside corral -> green=0, unknown=0, CORRAL_SAFE=True, SWEEP_CLEAR=True!
            img_fl = _FakeMaixImage(320, 240, floor_base)
            img_fl.fill_rect(120, 150, 30, 20, floor_green_overlap)  # 600 px pure floor
            fl_blobs = extract_blobs_from_maix_image(make_src(img_fl), cfg_overlap, scan_unknown_near=True)
            fl_st = RescueVisionProcessor(cfg_overlap).process(
                fl_blobs, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False
            )
            self.assertEqual(fl_st.corral_green_cnt, 0, msg=tier_name)
            self.assertEqual(fl_st.corral_unknown_cnt, 0, msg=tier_name)
            self.assertTrue(bool(fl_st.flags & VIS_FLAG_CORRAL_SAFE), msg=tier_name)
            self.assertTrue(bool(fl_st.flags & VIS_FLAG_SWEEP_CLEAR), msg=tier_name)

            # 2b. C-shaped floor-only green patch enclosing a 100-px unknown block -> green=0, unknown=1, CORRAL_SAFE=False!
            img_c = _FakeMaixImage(320, 240, floor_base)
            img_c.fill_rect(110, 140, 40, 8, floor_green_overlap)
            img_c.fill_rect(110, 148, 8, 24, floor_green_overlap)
            img_c.fill_rect(110, 172, 40, 8, floor_green_overlap)
            img_c.fill_rect(126, 154, 10, 10, unknown_lab)  # 100 px unknown in hollow
            c_blobs = extract_blobs_from_maix_image(make_src(img_c), cfg_overlap, scan_unknown_near=True)
            c_st = RescueVisionProcessor(cfg_overlap).process(
                c_blobs, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False
            )
            self.assertEqual(c_st.corral_green_cnt, 0, msg=tier_name)
            self.assertEqual(c_st.corral_unknown_cnt, 1, msg=tier_name)
            self.assertFalse(bool(c_st.flags & VIS_FLAG_CORRAL_SAFE), msg=tier_name)

            # 2c. 600-px floor patch touching a sub-threshold 50-px real green speck -> floor stripped, green=0!
            img_sp = _FakeMaixImage(320, 240, floor_base)
            img_sp.fill_rect(120, 150, 30, 20, floor_green_overlap)
            img_sp.fill_rect(150, 150, 5, 10, real_green)  # 50 px real green (< min_block_pixels=90)
            sp_blobs = extract_blobs_from_maix_image(make_src(img_sp), cfg_overlap, scan_unknown_near=True)
            sp_st = RescueVisionProcessor(cfg_overlap).process(
                sp_blobs, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False
            )
            self.assertEqual(sp_st.corral_green_cnt, 0, msg=tier_name)

            # 2d. 600-px floor patch touching a valid 120-px real green block -> floor stripped, green=1 (120 px)!
            img_ok = _FakeMaixImage(320, 240, floor_base)
            img_ok.fill_rect(120, 150, 30, 20, floor_green_overlap)
            img_ok.fill_rect(150, 150, 12, 10, real_green)  # 120 px real green (>= 90)
            ok_blobs = extract_blobs_from_maix_image(make_src(img_ok), cfg_overlap, scan_unknown_near=True)
            ok_st = RescueVisionProcessor(cfg_overlap).process(
                ok_blobs, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False
            )
            self.assertEqual(ok_st.corral_green_cnt, 1, msg=tier_name)
            self.assertEqual(ok_st.corral_unknown_cnt, 0, msg=tier_name)
            self.assertTrue(bool(ok_st.flags & VIS_FLAG_CORRAL_SAFE), msg=tier_name)
            self.assertTrue(any(b.label == "green" and b.pixels == 120 for b in ok_blobs), msg=tier_name)

            # 2e. R19 P1-2: 120-px cyan patch overlapping floor band MUST be retained as cyan hazard (fail closed)!
            img_cy = _FakeMaixImage(320, 240, floor_base)
            img_cy.fill_rect(120, 150, 12, 10, floor_cyan_overlap)  # 120 px cyan in corral
            img_cy.fill_rect(240, 150, 12, 10, floor_cyan_overlap)  # 120 px cyan in sweep
            cy_blobs = extract_blobs_from_maix_image(make_src(img_cy), cfg_overlap, scan_unknown_near=True)
            cy_st = RescueVisionProcessor(cfg_overlap).process(
                cy_blobs, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False
            )
            self.assertEqual(cy_st.corral_cyan_cnt, 1, msg=tier_name)
            self.assertFalse(bool(cy_st.flags & VIS_FLAG_CORRAL_SAFE), msg=tier_name)
            self.assertFalse(bool(cy_st.flags & VIS_FLAG_SWEEP_CLEAR), msg=tier_name)

            # 2f. R19 P2: 110-px L-shaped non-floor green component under floor-overlapping calibration
            #     MUST produce green=1, unknown=0, CORRAL_SAFE=True across Tier 1, Tier 2, AND Tier 3!
            img_l = _FakeMaixImage(320, 240, floor_base)
            img_l.fill_rect(120, 150, 15, 5, real_green)  # 75 px horizontal bar
            img_l.fill_rect(120, 155, 5, 7, real_green)   # 35 px vertical leg -> 110 px L-shape!
            l_blobs = extract_blobs_from_maix_image(make_src(img_l), cfg_overlap, scan_unknown_near=True)
            l_st = RescueVisionProcessor(cfg_overlap).process(
                l_blobs, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False
            )
            self.assertEqual(l_st.corral_green_cnt, 1, msg=tier_name)
            self.assertEqual(l_st.corral_unknown_cnt, 0, msg=tier_name)
            self.assertTrue(bool(l_st.flags & VIS_FLAG_CORRAL_SAFE), msg=tier_name)
            # Attaching a 100-px unknown wart to that L-shaped green component MUST still emit unknown!
            img_l.fill_rect(125, 155, 10, 10, unknown_lab)
            lw_blobs = extract_blobs_from_maix_image(make_src(img_l), cfg_overlap, scan_unknown_near=True)
            lw_st = RescueVisionProcessor(cfg_overlap).process(
                lw_blobs, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False
            )
            self.assertEqual(lw_st.corral_unknown_cnt, 1, msg=tier_name)
            self.assertFalse(bool(lw_st.flags & VIS_FLAG_CORRAL_SAFE), msg=tier_name)


if __name__ == "__main__":
    unittest.main()
