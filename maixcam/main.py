"""MaixCAM touchscreen main runtime loop for Intelligent Rescue Car.

Wiring:
  MaixCAM UART1 A19 (TX) -> DM-MC02 PA10 (USART1_RX)
  MaixCAM UART1 A18 (RX) <- DM-MC02 PA9  (USART1_TX)
  MaixCAM GND            -- DM-MC02 GND  (signals + GND only; power from dedicated 5V BEC)
"""
from __future__ import annotations

from pathlib import Path
import time

from .config import RescueVisionConfig
from .protocol import (
    Heartbeat,
    RescueProtocolParser,
    TEL_FLAG_CAMERA_NEAR,
    TEL_FLAG_CAMERA_SETTLED,
    TEL_FLAG_CARRYING_CASUALTY,
    TEL_FLAG_CONFIG_LOCKED,
    TEL_FLAG_DROP_PUSHING,
    TEL_FLAG_FIRST_GREEN_UNLOCKED,
    TEL_FLAG_GATE_CLOSED,
    TEL_FLAG_GATE_SETTLED,
    TEL_FLAG_PRE_MATCH_GEN_MASK,
    TEL_FLAG_PRE_MATCH_GEN_SHIFT,
    TEL_FLAG_SETTLE_GEN_MASK,
    TEL_FLAG_SETTLE_GEN_SHIFT,
    Telemetry,
    decode_telemetry,
    encode_bench_actuator,
    encode_heartbeat,
    encode_set_config,
    encode_vision_state,
)
from .ui import TouchUIController, render_touch_overlay
from .vision import RescueVisionProcessor, SpatialMotionEstimator, extract_blobs_from_maix_image
from .yolo import HybridVisionExtractor, build_hybrid_extractor


def _extract_motion_grid(img, width: int, height: int) -> list[int]:
    from maix import image

    small = img.resize(width, height).to_format(image.Format.FMT_GRAYSCALE)
    return list(small.to_bytes())


_CAMERA_BUFF_NUM = 3  # MaixCDK MMF default camera buffer queue depth


def _camera_drain_frames(cam, configured_buff_num: int = _CAMERA_BUFF_NUM) -> int:
    """Return number of reads to discard (queued buffers + 1 in-flight exposure).

    Sipeed's MaixCAM MMF Camera.clear_buff() is a logged no-op, so settle transitions
    must drain queued and in-flight frames via real cam.read() calls and withhold settle
    acknowledgments until a demonstrably fresh capture is obtained.
    """
    raw_depth = getattr(cam, "buff_num", None)
    if callable(raw_depth):
        try:
            raw_depth = raw_depth()
        except Exception:
            raw_depth = None
    depth = configured_buff_num
    if isinstance(raw_depth, int) and not isinstance(raw_depth, bool) and raw_depth > 0:
        depth = max(depth, raw_depth)
    return depth + 1


def _effective_drain_frames(
    cam,
    extractor: HybridVisionExtractor,
    configured_buff_num: int = _CAMERA_BUFF_NUM,
) -> int:
    """Include the extra NPU dual_buff pipeline stage when active."""
    return _camera_drain_frames(cam, configured_buff_num) + extractor.pipeline_delay_frames


def _open_camera(camera_mod, cfg: RescueVisionConfig, extractor: HybridVisionExtractor):
    fmt = extractor.detector.input_format() if extractor.detector is not None else None
    if fmt is not None:
        try:
            return camera_mod.Camera(cfg.frame_width, cfg.frame_height, fmt, buff_num=_CAMERA_BUFF_NUM)
        except TypeError:
            try:
                return camera_mod.Camera(cfg.frame_width, cfg.frame_height, fmt)
            except TypeError:
                pass
    try:
        return camera_mod.Camera(cfg.frame_width, cfg.frame_height, buff_num=_CAMERA_BUFF_NUM)
    except TypeError:
        return camera_mod.Camera(cfg.frame_width, cfg.frame_height)


def run_maixcam(config_path: Path | None = None) -> None:
    from maix import app, camera, display, pinmap, touchscreen, uart

    cfg_file = config_path or (Path(__file__).resolve().parent / "config.json")
    cfg = RescueVisionConfig.load(cfg_file)
    cfg.validate()

    extractor = build_hybrid_extractor(cfg, lab_extractor=extract_blobs_from_maix_image)
    if extractor.fallback_active:
        print(
            f"[maixcam] YOLO unavailable ({extractor.load_error}); falling back to LAB mode.",
            flush=True,
        )
    else:
        print(
            f"[maixcam] vision backend={extractor.active_backend} "
            f"dual_buff={extractor.uses_dual_buff} model={cfg.yolo_model_path}",
            flush=True,
        )

    pinmap.set_pin_function(cfg.uart_tx_pin, "UART1_TX")
    pinmap.set_pin_function(cfg.uart_rx_pin, "UART1_RX")
    serial = uart.UART(cfg.uart_device, cfg.uart_baud)

    cam = _open_camera(camera, cfg, extractor)
    drain_depth = _effective_drain_frames(cam, extractor)
    disp = display.Display()
    ts = touchscreen.TouchScreen()

    ui = TouchUIController(width=disp.width(), height=disp.height(), vision_mode=extractor.active_backend)
    processor = RescueVisionProcessor(cfg)
    motion_estimator = SpatialMotionEstimator(
        width=cfg.motion_grid_w,
        height=cfg.motion_grid_h,
        min_texture_std=cfg.motion_min_texture_std,
        static_sad_max=cfg.motion_static_sad_max,
        min_cells_per_s=cfg.motion_min_cells_per_s,
    )
    parser = RescueProtocolParser()
    telemetry = Telemetry()

    tx_seq = 0
    start_s = time.monotonic()
    last_loop_s = start_s
    fps_ema = 0.0
    last_hb_s = 0.0
    last_cfg_s = 0.0
    last_vis_s = 0.0
    prev_camera_near = False
    prev_camera_settled = False
    prev_gate_open_settled = False
    prev_gate_closed_settled = False
    fenced_pre_match_gen = 0
    fenced_gate_close_gen = 0
    fenced_gate_open_gen = 0
    pre_match_drain_remaining = 0
    gate_close_drain_remaining = 0
    gate_open_drain_remaining = 0
    cam_settle_drain_remaining = drain_depth
    gate_settle_drain_remaining = 0

    def _observe_telemetry(tel: Telemetry) -> None:
        nonlocal prev_camera_near, prev_camera_settled
        nonlocal prev_gate_open_settled, prev_gate_closed_settled
        nonlocal fenced_pre_match_gen, fenced_gate_close_gen, fenced_gate_open_gen
        nonlocal pre_match_drain_remaining, gate_close_drain_remaining, gate_open_drain_remaining
        nonlocal cam_settle_drain_remaining, gate_settle_drain_remaining

        cam_near = bool(tel.status_flags & TEL_FLAG_CAMERA_NEAR)
        cam_settled = bool(tel.status_flags & TEL_FLAG_CAMERA_SETTLED)
        g_closed = bool(tel.status_flags & TEL_FLAG_GATE_CLOSED)
        g_settled = bool(tel.status_flags & TEL_FLAG_GATE_SETTLED)
        g_open_settled = (not g_closed) and g_settled
        g_closed_settled = g_closed and g_settled
        tel_gen = (tel.status_flags & TEL_FLAG_SETTLE_GEN_MASK) >> TEL_FLAG_SETTLE_GEN_SHIFT

        if cam_near != prev_camera_near or not cam_settled:
            motion_estimator.reset()
            cam_settle_drain_remaining = drain_depth
            prev_camera_near = cam_near
        elif cam_settled and not prev_camera_settled:
            cam_settle_drain_remaining = drain_depth
        prev_camera_settled = cam_settled

        if not g_settled:
            gate_settle_drain_remaining = 0
        elif (g_open_settled and not prev_gate_open_settled) or (g_closed_settled and not prev_gate_closed_settled):
            gate_settle_drain_remaining = drain_depth
        prev_gate_open_settled = g_open_settled
        prev_gate_closed_settled = g_closed_settled

        pm_ready = (not cam_near) and cam_settled and g_open_settled and (1 <= tel_gen <= 3)
        if not pm_ready:
            fenced_pre_match_gen = 0
            pre_match_drain_remaining = 0
        elif tel_gen != fenced_pre_match_gen:
            fenced_pre_match_gen = tel_gen
            pre_match_drain_remaining = drain_depth

        gc_ready = cam_near and cam_settled and g_closed_settled and (1 <= tel_gen <= 3)
        if not gc_ready:
            fenced_gate_close_gen = 0
            gate_close_drain_remaining = 0
        elif tel_gen != fenced_gate_close_gen:
            fenced_gate_close_gen = tel_gen
            gate_close_drain_remaining = drain_depth

        go_ready = cam_near and cam_settled and g_open_settled and (1 <= tel_gen <= 3)
        if not go_ready:
            fenced_gate_open_gen = 0
            gate_open_drain_remaining = 0
        elif tel_gen != fenced_gate_open_gen:
            fenced_gate_open_gen = tel_gen
            gate_open_drain_remaining = drain_depth

    while not app.need_exit():
        now_s = time.monotonic()
        uptime_ms = int((now_s - start_s) * 1000.0)

        # 1. Read incoming telemetry frames from DM-MC02, observing every frame in burst order
        #    so a transient settle/pitch/gate transition within one serial.read() always re-arms drain.
        raw = serial.read()
        if raw:
            for frame in parser.feed(raw):
                tel = decode_telemetry(frame)
                if tel is not None:
                    telemetry = tel
                    _observe_telemetry(tel)

        camera_near = bool(telemetry.status_flags & TEL_FLAG_CAMERA_NEAR)
        camera_settled = bool(telemetry.status_flags & TEL_FLAG_CAMERA_SETTLED)
        first_green_unlocked = bool(telemetry.status_flags & TEL_FLAG_FIRST_GREEN_UNLOCKED)
        carrying_casualty = bool(telemetry.status_flags & TEL_FLAG_CARRYING_CASUALTY)
        gate_closed = bool(telemetry.status_flags & TEL_FLAG_GATE_CLOSED)
        gate_settled = bool(telemetry.status_flags & TEL_FLAG_GATE_SETTLED)
        gate_open_settled = (not gate_closed) and gate_settled
        gate_closed_settled = gate_closed and gate_settled
        drop_pushing = bool(telemetry.status_flags & TEL_FLAG_DROP_PUSHING)
        tel_settle_gen = (telemetry.status_flags & TEL_FLAG_SETTLE_GEN_MASK) >> TEL_FLAG_SETTLE_GEN_SHIFT
        wheels_moving = abs(telemetry.vx_mm_s) > 35 or abs(telemetry.omega_mrad_s) > 250
        pre_match_ready_state = (
            (not camera_near)
            and camera_settled
            and gate_open_settled
            and (1 <= tel_settle_gen <= 3)
        )
        gate_close_ready_state = (
            camera_near
            and camera_settled
            and gate_closed_settled
            and (1 <= tel_settle_gen <= 3)
        )
        gate_open_ready_state = (
            camera_near
            and camera_settled
            and gate_open_settled
            and (1 <= tel_settle_gen <= 3)
        )

        # 2. Capture frame; while draining buffered/in-flight pre-settle frames, withhold settled
        #    camera/gate/cycle-gen acknowledgments so a pre-settle frame can NEVER authorize launch, close, or push.
        img = cam.read()
        if img is None:
            continue

        draining_cam = cam_settle_drain_remaining > 0
        if draining_cam:
            cam_settle_drain_remaining -= 1

        draining_gate = gate_settle_drain_remaining > 0
        if draining_gate:
            gate_settle_drain_remaining -= 1

        draining_pre_match = pre_match_drain_remaining > 0
        if draining_pre_match:
            pre_match_drain_remaining -= 1

        draining_gate_close = gate_close_drain_remaining > 0
        if draining_gate_close:
            gate_close_drain_remaining -= 1

        draining_gate_open = gate_open_drain_remaining > 0
        if draining_gate_open:
            gate_open_drain_remaining -= 1

        effective_camera_settled = camera_settled and (not draining_cam) and (not draining_pre_match)
        effective_gate_settled = (
            gate_settled
            and (not draining_gate)
            and (not draining_pre_match)
            and (not draining_gate_close)
            and (not draining_gate_open)
        )
        if (
            pre_match_ready_state
            and effective_camera_settled
            and effective_gate_settled
            and fenced_pre_match_gen == tel_settle_gen
        ):
            effective_pre_match_gen = tel_settle_gen
        else:
            effective_pre_match_gen = 0

        if (
            gate_close_ready_state
            and effective_camera_settled
            and effective_gate_settled
            and fenced_gate_close_gen == tel_settle_gen
        ):
            effective_gate_settle_gen = tel_settle_gen
        elif (
            gate_open_ready_state
            and effective_camera_settled
            and effective_gate_settled
            and fenced_gate_open_gen == tel_settle_gen
        ):
            effective_gate_settle_gen = tel_settle_gen
        else:
            effective_gate_settle_gen = 0

        blobs, aligned_img, frame_ready = extractor.extract(
            img,
            scan_unknown_near=(camera_near and effective_camera_settled),
        )
        if not frame_ready:
            # Cold-start priming frame under YOLO dual_buff=True; wait for aligned frame.
            continue

        loop_dt = max(1e-3, now_s - last_loop_s)
        last_loop_s = now_s
        inst_fps = 1.0 / loop_dt
        fps_ema = inst_fps if fps_ema <= 0.0 else (0.85 * fps_ema + 0.15 * inst_fps)

        if effective_camera_settled:
            grid = _extract_motion_grid(aligned_img, cfg.motion_grid_w, cfg.motion_grid_h)
            motion_stalled, _ = motion_estimator.update(grid, now_s, wheels_moving=wheels_moving)
        else:
            motion_estimator.reset()
            motion_stalled = False

        vision_state = processor.process(
            blobs=blobs,
            camera_near_cmd=camera_near,
            camera_settled=effective_camera_settled,
            team=telemetry.team if telemetry.team else ui.config.team,
            first_green_unlocked=first_green_unlocked,
            carrying_casualty=carrying_casualty,
            max_materials_per_trip=telemetry.max_materials_per_trip or ui.config.max_materials_per_trip,
            motion_stalled=motion_stalled,
            rear_clear=False,
            mission_state=telemetry.mission_state,
            gate_closed=gate_closed,
            gate_settled=effective_gate_settled,
            drop_pushing=drop_pushing,
            pre_match_settle_gen=effective_pre_match_gen,
            gate_settle_gen=effective_gate_settle_gen,
        )

        # 3. Poll touchscreen (locked automatically once MC02 confirms launch).
        tx, ty, pressed = ts.read()
        ui.handle_touch(tx, ty, bool(pressed), telemetry)
        if ui.vision_mode_dirty and not (telemetry.status_flags & TEL_FLAG_CONFIG_LOCKED):
            ui.vision_mode_dirty = False
            try:
                extractor.set_backend(ui.vision_mode)
            except RuntimeError:
                ui.vision_mode = extractor.active_backend
            drain_depth = _effective_drain_frames(cam, extractor)
            motion_estimator.reset()
            cam_settle_drain_remaining = drain_depth

        # 4. Transmit protocol frames at controlled rates (20 Hz vision, 5 Hz heartbeat/config).
        if now_s - last_hb_s >= 0.20:
            last_hb_s = now_s
            ui_state = 1 if (telemetry.status_flags & TEL_FLAG_CONFIG_LOCKED) else 0
            serial.write(encode_heartbeat(tx_seq, Heartbeat(uptime_ms, 1, ui_state, 0)))
            tx_seq = (tx_seq + 1) & 0xFF

        if not (telemetry.status_flags & TEL_FLAG_CONFIG_LOCKED):
            if ui.config_dirty or (now_s - last_cfg_s >= 0.25):
                last_cfg_s = now_s
                ui.config_dirty = False
                serial.write(encode_set_config(tx_seq, ui.config))
                tx_seq = (tx_seq + 1) & 0xFF
            bench_cmd = ui.pop_bench_cmd()
            if bench_cmd is not None:
                serial.write(encode_bench_actuator(tx_seq, bench_cmd))
                tx_seq = (tx_seq + 1) & 0xFF

        if now_s - last_vis_s >= 0.05:
            last_vis_s = now_s
            serial.write(encode_vision_state(tx_seq, vision_state))
            tx_seq = (tx_seq + 1) & 0xFF

        # 5. Render camera feed + detections + calibrated ROIs + touchscreen UI.
        disp_img = aligned_img.resize(disp.width(), disp.height())
        backend_tag = f"{extractor.status_tag} {fps_ema:.0f}fps"
        render_touch_overlay(disp_img, ui, telemetry, blobs=blobs, cfg=cfg, backend_tag=backend_tag)
        disp.show(disp_img)


if __name__ == "__main__":
    run_maixcam()
