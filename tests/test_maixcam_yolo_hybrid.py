"""Unit tests for MaixCAM YOLO11/YOLOv8 + lightweight LAB hybrid vision pipeline."""
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

REPO_ROOT = Path(__file__).resolve().parents[1]
TESTS_DIR = Path(__file__).resolve().parent
for _p in (str(REPO_ROOT), str(TESTS_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import maixcam.main as maix_main
from maixcam.config import RescueVisionConfig
from maixcam.main import _camera_drain_frames, _effective_drain_frames
from maixcam.protocol import (
    CAMERA_VIEW_FAR,
    CAMERA_VIEW_TRANSITION,
    MISSION_DROP_BACKOUT_VERIFY,
    MISSION_DROP_OPEN_AND_PUSH,
    RescueProtocolParser,
    TARGET_GREEN_MATERIAL,
    TEAM_RED,
    TEL_FLAG_CAMERA_SETTLED,
    TEL_FLAG_CONFIG_LOCKED,
    TEL_FLAG_GATE_SETTLED,
    TEL_FLAG_PRE_MATCH_GEN_SHIFT,
    Telemetry,
    VIS_FLAG_CORRAL_SAFE,
    VIS_FLAG_DROP_BASELINE_READY,
    VIS_FLAG_DROP_CONFIRMED,
    VIS_FLAG_DROP_PATH_CLEAR,
    VIS_FLAG_HOME_HALF_MATCH,
    VIS_FLAG_HOME_VALID,
    VIS_FLAG_OPPONENT_NEAR,
    VIS_FLAG_PRE_MATCH_GEN_MASK,
    VIS_FLAG_PRE_MATCH_GEN_SHIFT,
    VIS_FLAG_SWEEP_CLEAR,
    VIS_FLAG_TARGET_VALID,
    decode_vision_state,
    encode_telemetry,
)
from maixcam.ui import TouchUIController
from maixcam.vision import RescueVisionProcessor
from maixcam.yolo import (
    HybridVisionExtractor,
    MaixYoloAdapter,
    YoloDetection,
    build_hybrid_extractor,
    detections_to_blobs,
    normalize_yolo_label,
    resolve_cross_class_conflicts,
)
from test_maixcam_protocol_and_vision import _FakeMaixImage


class _FakeRawYoloObj:
    def __init__(self, x: int, y: int, w: int, h: int, class_id: int, score: float) -> None:
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.class_id = class_id
        self.score = score


class _FakeMaixYoloDetector:
    """Simulates maix.nn.YOLO11 with optional 1-frame dual_buff pipeline latency."""

    def __init__(
        self,
        labels: list[str],
        per_image_outputs: dict[int, list[_FakeRawYoloObj]],
        dual_buff: bool = False,
        input_w: int = 320,
        input_h: int = 224,
    ) -> None:
        self.labels = labels
        self._outputs = per_image_outputs
        self.dual_buff = dual_buff
        self._input_w = input_w
        self._input_h = input_h
        self._pending: list[_FakeRawYoloObj] = []
        self.detect_calls: list[tuple[int, float, float]] = []

    def input_width(self) -> int:
        return self._input_w

    def input_height(self) -> int:
        return self._input_h

    def input_format(self) -> str:
        return "FMT_RGB888"

    def detect(self, img, conf_th: float = 0.5, iou_th: float = 0.45):
        self.detect_calls.append((id(img), conf_th, iou_th))
        raw = [o for o in self._outputs.get(id(img), []) if o.score >= conf_th]
        if self.dual_buff:
            out = self._pending
            self._pending = raw
            return out
        return raw


class TestMaixcamYoloHybrid(unittest.TestCase):
    def test_config_validation_and_roundtrip_320x224(self) -> None:
        cfg = RescueVisionConfig(frame_width=320, frame_height=224, vision_backend="hybrid", yolo_arch="yolo11")
        cfg.validate()
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "config.json"
            cfg.save(p)
            loaded = RescueVisionConfig.load(p)
            loaded.validate()
            self.assertEqual(loaded.frame_height, 224)
            self.assertEqual(loaded.vision_backend, "hybrid")
            self.assertEqual(loaded.yolo_arch, "yolo11")
            self.assertAlmostEqual(loaded.yolo_hazard_conf_th, 0.25)

        bad = RescueVisionConfig(vision_backend="invalid")
        with self.assertRaises(ValueError):
            bad.validate()
        bad_arch = RescueVisionConfig(yolo_arch="yolov3")
        with self.assertRaises(ValueError):
            bad_arch.validate()
        bad_conf = RescueVisionConfig(yolo_hazard_conf_th=0.0)
        with self.assertRaises(ValueError):
            bad_conf.validate()

    def test_label_normalization_and_unmapped_fail_closed(self) -> None:
        cfg = RescueVisionConfig()
        self.assertEqual(normalize_yolo_label("Green-Block", cfg), "green")
        self.assertEqual(normalize_yolo_label("casualty", cfg), "orange")
        self.assertEqual(normalize_yolo_label("HAZARD", cfg), "cyan")
        self.assertEqual(normalize_yolo_label("mystery_debris", cfg), "unknown")

        cfg.yolo_unknown_unmapped = False
        self.assertIsNone(normalize_yolo_label("mystery_debris", cfg))

    def test_cross_class_conflict_cyan_overrides_green_and_ambiguous_becomes_unknown(self) -> None:
        cfg = RescueVisionConfig()
        # 1. Low-confidence cyan (0.30 >= 0.25) MUST suppress overlapping high-confidence green (0.85).
        dets = [
            YoloDetection(120, 150, 30, 30, "green", 0.85),
            YoloDetection(122, 151, 29, 29, "cyan", 0.30),
        ]
        resolved = resolve_cross_class_conflicts(dets, cfg)
        self.assertEqual([d.label for d in resolved], ["cyan"])

        # 2. Ambiguous green (0.62) vs black (0.58) within margin (< 0.10) -> fail-closed "unknown".
        amb = [
            YoloDetection(120, 150, 28, 28, "green", 0.62),
            YoloDetection(121, 150, 28, 28, "black", 0.58),
        ]
        resolved_amb = resolve_cross_class_conflicts(amb, cfg)
        self.assertEqual([d.label for d in resolved_amb], ["unknown"])

        # 3. Clear margin green (0.85) vs black (0.50) -> keeps green.
        clear = [
            YoloDetection(120, 150, 28, 28, "green", 0.85),
            YoloDetection(121, 150, 28, 28, "black", 0.50),
        ]
        resolved_clear = resolve_cross_class_conflicts(clear, cfg)
        self.assertEqual([d.label for d in resolved_clear], ["green"])

        # 4. Block sitting on red_zone / purple_ramp is never suppressed by surface boxes.
        on_surface = [
            YoloDetection(40, 20, 240, 75, "red_zone", 0.90),
            YoloDetection(60, 40, 26, 26, "green", 0.80),
        ]
        resolved_surf = resolve_cross_class_conflicts(on_surface, cfg)
        self.assertEqual(sorted(d.label for d in resolved_surf), ["green", "red_zone"])

    def test_touching_same_color_blocks_stay_separate_and_large_closeup_is_valid(self) -> None:
        cfg = RescueVisionConfig(frame_width=320, frame_height=224)
        # Two touching green blocks in near_corral_roi [88, 128, 144, 96]: LAB merges them, YOLO counts 2!
        dets = [
            YoloDetection(100, 145, 36, 36, "green", 0.88),
            YoloDetection(136, 145, 36, 36, "green", 0.84),
        ]
        blobs = detections_to_blobs(resolve_cross_class_conflicts(dets, cfg), cfg)
        state = RescueVisionProcessor(cfg).process(
            blobs,
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=True,
            max_materials_per_trip=3,
        )
        self.assertEqual(state.corral_green_cnt, 2)
        self.assertTrue(bool(state.flags & VIS_FLAG_CORRAL_SAFE))
        self.assertTrue(bool(state.flags & VIS_FLAG_SWEEP_CLEAR))

        # Close-up YOLO block with area 110x100 = 11000 px (> LAB max_block_pixels=9500) remains valid!
        big_det = [YoloDetection(105, 124, 110, 100, "green", 0.91)]
        big_blobs = detections_to_blobs(big_det, cfg)
        st_big = RescueVisionProcessor(cfg).process(
            big_blobs,
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=False,
        )
        self.assertEqual(st_big.corral_green_cnt, 1)
        self.assertEqual(st_big.corral_unknown_cnt, 0)
        self.assertTrue(bool(st_big.flags & VIS_FLAG_CORRAL_SAFE))

    def test_far_view_small_yolo_target_confidence_and_opponent_alert(self) -> None:
        cfg = RescueVisionConfig(frame_width=320, frame_height=224)
        dets = [
            YoloDetection(155, 80, 8, 7, "green", 0.77),  # 56 px (< LAB min 90, >= YOLO min 36)
            YoloDetection(20, 110, 42, 42, "opponent", 0.68),  # 1764 px (< LAB opp 2200), y2=152 >= 145
        ]
        blobs = detections_to_blobs(resolve_cross_class_conflicts(dets, cfg), cfg)
        st = RescueVisionProcessor(cfg).process(
            blobs,
            camera_near_cmd=False,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=False,
        )
        self.assertTrue(bool(st.flags & VIS_FLAG_TARGET_VALID))
        self.assertEqual(st.target_type, TARGET_GREEN_MATERIAL)
        self.assertEqual(st.confidence, 77)
        self.assertTrue(bool(st.flags & VIS_FLAG_OPPONENT_NEAR))

    def test_corral_graze_filtering_vs_fail_closed_hazard_graze(self) -> None:
        cfg = RescueVisionConfig(frame_width=320, frame_height=224)
        # near_corral_roi = [88, 128, 144, 96] -> x in [88, 232).
        # Green box grazes 1 px inside x=88 (area overlap = 1*20 = 20 < 25, center outside) -> NOT counted.
        # Cyan box grazes 1 px inside x=88 -> MUST still fail closed (corral_cyan_cnt = 1)!
        dets = [
            YoloDetection(69, 150, 20, 20, "green", 0.80),  # x2=89 -> 1x20=20 px overlap (< 25 and < 15% of 400=60)
            YoloDetection(69, 180, 20, 20, "cyan", 0.30),   # x2=89 -> 1 px graze into corral
        ]
        blobs = detections_to_blobs(resolve_cross_class_conflicts(dets, cfg), cfg)
        st = RescueVisionProcessor(cfg).process(
            blobs,
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=False,
        )
        self.assertEqual(st.corral_green_cnt, 0)
        self.assertEqual(st.corral_cyan_cnt, 1)
        self.assertFalse(bool(st.flags & VIS_FLAG_CORRAL_SAFE))

    def test_hybrid_near_view_lab_cyan_and_unknown_safety_net_and_drop_confirm(self) -> None:
        cfg = RescueVisionConfig(frame_width=320, frame_height=224)
        floor = (85, 0, 0)
        red_zone = (45, 55, 20)
        purple_ramp = (45, 40, -35)
        green_lab = (55, -35, 30)
        cyan_lab = (65, -32, -24)
        unknown_lab = (55, 5, 85)
        labels = ["green", "black", "orange", "cyan", "opponent"]

        # Pre-push frame: red safe zone positioned so frame center (x=160) aligns inside its LEFT
        # (material) half [100, 192], plus associated purple ramp and 1 green block in corral.
        pre_img = _FakeMaixImage(320, 224, floor)
        pre_img.fill_rect(100, 20, 220, 75, red_zone)
        pre_img.fill_rect(100, 95, 220, 15, purple_ramp)
        pre_img.fill_rect(140, 150, 24, 24, green_lab)

        # Post-push frame: green block moved into left half (material half) of red zone; corral empty.
        post_img = _FakeMaixImage(320, 224, floor)
        post_img.fill_rect(100, 20, 220, 75, red_zone)
        post_img.fill_rect(100, 95, 220, 15, purple_ramp)
        post_img.fill_rect(140, 40, 24, 24, green_lab)

        # Hazard-miss frame: YOLO misses a cyan corner peeking into near_sweep_roi [232, 118, 64, 106].
        cyan_miss_img = _FakeMaixImage(320, 224, floor)
        cyan_miss_img.fill_rect(140, 150, 24, 24, green_lab)
        cyan_miss_img.fill_rect(240, 150, 12, 10, cyan_lab)  # 120 px cyan in sweep ROI

        # Unknown-obstacle frame: YOLO detects green block, misses an unknown object in corral.
        unk_img = _FakeMaixImage(320, 224, floor)
        unk_img.fill_rect(140, 150, 24, 24, green_lab)
        unk_img.fill_rect(100, 150, 12, 10, unknown_lab)  # 120 px unknown in corral

        # Both YOLO and LAB see the same cyan block in corral -> MUST NOT double-count (count == 1).
        cyan_both_img = _FakeMaixImage(320, 224, floor)
        cyan_both_img.fill_rect(140, 150, 20, 20, cyan_lab)

        fake_det = _FakeMaixYoloDetector(
            labels=labels,
            per_image_outputs={
                id(pre_img): [_FakeRawYoloObj(140, 150, 24, 24, 0, 0.90)],
                id(post_img): [_FakeRawYoloObj(140, 40, 24, 24, 0, 0.88)],
                id(cyan_miss_img): [_FakeRawYoloObj(140, 150, 24, 24, 0, 0.90)],
                id(unk_img): [_FakeRawYoloObj(140, 150, 24, 24, 0, 0.90)],
                id(cyan_both_img): [_FakeRawYoloObj(140, 150, 20, 20, 3, 0.65)],
            },
            dual_buff=False,
        )
        adapter = MaixYoloAdapter("stub.mud", dual_buff=False, detector=fake_det)
        extractor = HybridVisionExtractor(cfg, detector=adapter)
        processor = RescueVisionProcessor(cfg)

        # 1. Pre-push baseline ready + zero false unknown from YOLO green block.
        pre_blobs, aligned_pre, ready = extractor.extract(pre_img, scan_unknown_near=True)
        self.assertTrue(ready)
        self.assertIs(aligned_pre, pre_img)
        st_pre = processor.process(
            pre_blobs,
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=False,
            mission_state=MISSION_DROP_OPEN_AND_PUSH,
            gate_closed=True,
            gate_settled=True,
        )
        self.assertEqual(st_pre.corral_green_cnt, 1)
        self.assertEqual(st_pre.corral_unknown_cnt, 0)
        self.assertTrue(bool(st_pre.flags & VIS_FLAG_HOME_VALID))
        self.assertTrue(bool(st_pre.flags & VIS_FLAG_HOME_HALF_MATCH))
        self.assertTrue(bool(st_pre.flags & VIS_FLAG_DROP_PATH_CLEAR))
        self.assertTrue(bool(st_pre.flags & VIS_FLAG_DROP_BASELINE_READY))

        # 2. Post-push drop confirmation succeeds cleanly.
        post_blobs, _, _ = extractor.extract(post_img, scan_unknown_near=True)
        st_post = processor.process(
            post_blobs,
            camera_near_cmd=True,
            camera_settled=True,
            team=TEAM_RED,
            first_green_unlocked=False,
            mission_state=MISSION_DROP_BACKOUT_VERIFY,
            gate_closed=False,
            gate_settled=True,
        )
        self.assertEqual(st_post.corral_green_cnt, 0)
        self.assertTrue(bool(st_post.flags & VIS_FLAG_DROP_CONFIRMED))

        # 3. LAB cyan safety net catches YOLO miss in sweep ROI.
        cm_blobs, _, _ = extractor.extract(cyan_miss_img, scan_unknown_near=True)
        st_cm = RescueVisionProcessor(cfg).process(
            cm_blobs, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False
        )
        self.assertFalse(bool(st_cm.flags & VIS_FLAG_SWEEP_CLEAR))

        # 4. LAB unknown safety net catches unmodeled obstacle in corral.
        uk_blobs, _, _ = extractor.extract(unk_img, scan_unknown_near=True)
        st_uk = RescueVisionProcessor(cfg).process(
            uk_blobs, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False
        )
        self.assertEqual(st_uk.corral_green_cnt, 1)
        self.assertEqual(st_uk.corral_unknown_cnt, 1)
        self.assertFalse(bool(st_uk.flags & VIS_FLAG_CORRAL_SAFE))

        # 5. YOLO + LAB agreeing on cyan does not double-count.
        cb_blobs, _, _ = extractor.extract(cyan_both_img, scan_unknown_near=True)
        st_cb = RescueVisionProcessor(cfg).process(
            cb_blobs, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False
        )
        self.assertEqual(st_cb.corral_cyan_cnt, 1)
        self.assertFalse(bool(st_cb.flags & VIS_FLAG_CORRAL_SAFE))

    def test_dual_buff_prev_frame_alignment_and_drain_depth(self) -> None:
        cfg = RescueVisionConfig(frame_width=320, frame_height=224, yolo_dual_buff=True)
        floor = (85, 0, 0)
        cyan_lab = (65, -32, -24)
        green_lab = (55, -35, 30)
        labels = ["green", "black", "orange", "cyan", "opponent"]

        # Frame 1 (pre-settle): contains cyan hazard.
        f1 = _FakeMaixImage(320, 224, floor)
        f1.fill_rect(140, 150, 20, 20, cyan_lab)
        # Frame 2 (post-settle): contains only 1 clean green block.
        f2 = _FakeMaixImage(320, 224, floor)
        f2.fill_rect(140, 150, 20, 20, green_lab)
        # Frame 3: empty floor.
        f3 = _FakeMaixImage(320, 224, floor)

        fake_det = _FakeMaixYoloDetector(
            labels=labels,
            per_image_outputs={
                id(f1): [_FakeRawYoloObj(140, 150, 20, 20, 3, 0.80)],
                id(f2): [_FakeRawYoloObj(140, 150, 20, 20, 0, 0.90)],
                id(f3): [],
            },
            dual_buff=True,
        )
        adapter = MaixYoloAdapter("stub.mud", dual_buff=True, detector=fake_det)
        extractor = HybridVisionExtractor(cfg, detector=adapter)

        fake_cam = SimpleNamespace(buff_num=3)
        self.assertEqual(_camera_drain_frames(fake_cam), 4)
        self.assertEqual(extractor.pipeline_delay_frames, 1)
        self.assertEqual(_effective_drain_frames(fake_cam, extractor), 5)

        # Call 1 (cold start): primes NPU with f1, not ready yet.
        b1, _, ready1 = extractor.extract(f1, scan_unknown_near=True)
        self.assertFalse(ready1)
        self.assertEqual(b1, [])

        # Call 2 (pass f2): returns detections for f1 AND aligns LAB against f1 (cyan only, no green!).
        b2, aligned2, ready2 = extractor.extract(f2, scan_unknown_near=True)
        self.assertTrue(ready2)
        self.assertIs(aligned2, f1)
        st2 = RescueVisionProcessor(cfg).process(
            b2, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False
        )
        self.assertEqual(st2.corral_cyan_cnt, 1)
        self.assertEqual(st2.corral_green_cnt, 0)

        # Call 3 (pass f3): returns detections for f2 AND aligns LAB against f2 (green=1, cyan=0, safe!).
        b3, aligned3, ready3 = extractor.extract(f3, scan_unknown_near=True)
        self.assertTrue(ready3)
        self.assertIs(aligned3, f2)
        st3 = RescueVisionProcessor(cfg).process(
            b3, camera_near_cmd=True, camera_settled=True, team=TEAM_RED, first_green_unlocked=False
        )
        self.assertEqual(st3.corral_green_cnt, 1)
        self.assertEqual(st3.corral_cyan_cnt, 0)
        self.assertEqual(st3.corral_unknown_cnt, 0)
        self.assertTrue(bool(st3.flags & VIS_FLAG_CORRAL_SAFE))

    def test_missing_model_graceful_fallback_and_strict_error(self) -> None:
        cfg_fb = RescueVisionConfig(
            vision_backend="hybrid",
            yolo_model_path="/nonexistent/path/rescue_yolo11n.mud",
            yolo_fallback_to_lab=True,
        )
        ext = build_hybrid_extractor(cfg_fb)
        self.assertEqual(ext.active_backend, "lab")
        self.assertTrue(ext.fallback_active)
        self.assertEqual(ext.status_tag, "LAB(FB)")
        self.assertEqual(ext.pipeline_delay_frames, 0)

        cfg_strict = RescueVisionConfig(
            vision_backend="hybrid",
            yolo_model_path="/nonexistent/path/rescue_yolo11n.mud",
            yolo_fallback_to_lab=False,
        )
        with self.assertRaises(Exception):
            build_hybrid_extractor(cfg_strict)

    def test_touch_ui_vision_mode_toggle_and_lock(self) -> None:
        ui = TouchUIController(width=552, height=368, vision_mode="hybrid")
        btn = next(b for b in ui.buttons if b.button_id == "vision_mode")
        self.assertLessEqual(btn.y + btn.h, ui.height)
        cx, cy = btn.x + btn.w // 2, btn.y + btn.h // 2

        tel_open = Telemetry(status_flags=0)
        self.assertEqual(ui.handle_touch(cx, cy, True, tel_open), "vision_mode")
        self.assertEqual(ui.vision_mode, "yolo")
        self.assertTrue(ui.vision_mode_dirty)
        ui.handle_touch(cx, cy, False, tel_open)

        self.assertEqual(ui.handle_touch(cx, cy, True, tel_open), "vision_mode")
        self.assertEqual(ui.vision_mode, "lab")
        ui.handle_touch(cx, cy, False, tel_open)

        self.assertEqual(ui.handle_touch(cx, cy, True, tel_open), "vision_mode")
        self.assertEqual(ui.vision_mode, "hybrid")
        ui.handle_touch(cx, cy, False, tel_open)

        # Locked after launch -> touch ignored.
        tel_locked = Telemetry(status_flags=TEL_FLAG_CONFIG_LOCKED)
        self.assertEqual(ui.handle_touch(cx, cy, True, tel_locked), "locked")
        self.assertEqual(ui.vision_mode, "hybrid")

    def test_run_maixcam_end_to_end_with_yolo11_dual_buff_drain(self) -> None:
        """End-to-end run_maixcam() with YOLO11 dual_buff=True MUST drain buff_num+2 frames before acking gen=1."""
        floor = (85, 0, 0)
        # 4 pre-settle frames contain a green block; 2 post-settle fresh frames are empty.
        pre_frames = [_FakeMaixImage(320, 224, floor) for _ in range(4)]
        fresh_frames = [_FakeMaixImage(320, 224, floor) for _ in range(2)]
        for fr in pre_frames + fresh_frames:
            fr.resize = lambda *_: fr  # type: ignore[attr-defined]
        all_frames = pre_frames + fresh_frames

        outputs = {id(fr): [_FakeRawYoloObj(145, 130, 24, 24, 0, 0.90)] for fr in pre_frames}
        outputs.update({id(fr): [] for fr in fresh_frames})
        fake_det = _FakeMaixYoloDetector(
            labels=["green", "black", "orange", "cyan", "opponent"],
            per_image_outputs=outputs,
            dual_buff=True,
        )

        payload_gen1 = encode_telemetry(
            1,
            Telemetry(
                team=TEAM_RED,
                start_zone=1,
                status_flags=(
                    TEL_FLAG_CAMERA_SETTLED
                    | TEL_FLAG_GATE_SETTLED
                    | (1 << TEL_FLAG_PRE_MATCH_GEN_SHIFT)
                ),
            ),
        )
        written: list[bytes] = []
        cam_obj = SimpleNamespace(read=iter(all_frames).__next__, buff_num=3)
        n_iters = len(all_frames)  # 6 reads: 1 priming + 4 drained emissions + 1 fresh post-settle emission
        fake_maix = SimpleNamespace(
            app=SimpleNamespace(need_exit=iter([False] * n_iters + [True]).__next__),
            camera=SimpleNamespace(Camera=lambda *_a, **_k: cam_obj),
            display=SimpleNamespace(
                Display=lambda: SimpleNamespace(width=lambda: 552, height=lambda: 368, show=lambda _: None)
            ),
            pinmap=SimpleNamespace(set_pin_function=lambda *_: None),
            touchscreen=SimpleNamespace(TouchScreen=lambda: SimpleNamespace(read=lambda: (0, 0, False))),
            uart=SimpleNamespace(UART=lambda *_: SimpleNamespace(read=lambda: payload_gen1, write=written.append)),
            nn=SimpleNamespace(YOLO11=lambda model, dual_buff=False: fake_det),
        )

        with tempfile.TemporaryDirectory() as tmp:
            mud_path = Path(tmp) / "rescue_yolo11n.mud"
            mud_path.write_text("[basic]\ntype=cvimodel\n", encoding="utf-8")
            cfg_path = Path(tmp) / "config.json"
            cfg = RescueVisionConfig(
                frame_width=320,
                frame_height=224,
                vision_backend="hybrid",
                yolo_arch="yolo11",
                yolo_model_path=str(mud_path),
                yolo_dual_buff=True,
            )
            cfg.save(cfg_path)

            mono_seq = iter([0.0] + [1.0 + 0.06 * i for i in range(n_iters)]).__next__
            with (
                patch.dict(sys.modules, {"maix": fake_maix}),
                patch.object(maix_main, "time", SimpleNamespace(monotonic=mono_seq)),
                patch.object(maix_main, "_extract_motion_grid", return_value=[0] * (16 * 12)),
                patch.object(maix_main, "render_touch_overlay"),
            ):
                maix_main.run_maixcam(config_path=cfg_path)

        parser = RescueProtocolParser()
        states = [
            vs
            for raw_tx in written
            for fr in parser.feed(raw_tx)
            if (vs := decode_vision_state(fr)) is not None
        ]
        # Priming frame skipped -> 5 emitted VisionStates: 4 drained (gen=0, TRANSITION) + 1 fresh (gen=1, FAR, empty).
        self.assertEqual(len(states), 5)
        for idx in range(4):
            echo_gen = (states[idx].flags & VIS_FLAG_PRE_MATCH_GEN_MASK) >> VIS_FLAG_PRE_MATCH_GEN_SHIFT
            self.assertEqual(echo_gen, 0, msg=f"drain emission {idx}")
            self.assertEqual(states[idx].camera_view, CAMERA_VIEW_TRANSITION, msg=f"drain emission {idx}")
            self.assertEqual(states[idx].target_type, 0, msg=f"drain emission {idx}")
        final_gen = (states[-1].flags & VIS_FLAG_PRE_MATCH_GEN_MASK) >> VIS_FLAG_PRE_MATCH_GEN_SHIFT
        self.assertEqual(final_gen, 1)
        self.assertEqual(states[-1].camera_view, CAMERA_VIEW_FAR)
        self.assertEqual(states[-1].target_type, 0)


if __name__ == "__main__":
    unittest.main()
