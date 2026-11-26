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
    TEL_FLAG_FIRST_GREEN_UNLOCKED,
    TEL_FLAG_GATE_CLOSED,
    Telemetry,
    decode_telemetry,
    encode_bench_actuator,
    encode_heartbeat,
    encode_set_config,
    encode_vision_state,
)
from .ui import TouchUIController, render_touch_overlay
from .vision import RescueVisionProcessor, SpatialMotionEstimator, extract_blobs_from_maix_image


def _extract_motion_grid(img, width: int, height: int) -> list[int]:
    from maix import image

    small = img.resize(width, height).to_format(image.Format.FMT_GRAYSCALE)
    return list(small.to_bytes())


def run_maixcam(config_path: Path | None = None) -> None:
    from maix import app, camera, display, pinmap, touchscreen, uart

    cfg_file = config_path or (Path(__file__).resolve().parent / "config.json")
    cfg = RescueVisionConfig.load(cfg_file)
    cfg.validate()

    pinmap.set_pin_function(cfg.uart_tx_pin, "UART1_TX")
    pinmap.set_pin_function(cfg.uart_rx_pin, "UART1_RX")
    serial = uart.UART(cfg.uart_device, cfg.uart_baud)

    cam = camera.Camera(cfg.frame_width, cfg.frame_height)
    disp = display.Display()
    ts = touchscreen.TouchScreen()

    ui = TouchUIController(width=disp.width(), height=disp.height())
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
    last_hb_s = 0.0
    last_cfg_s = 0.0
    last_vis_s = 0.0
    prev_camera_near = False

    while not app.need_exit():
        now_s = time.monotonic()
        uptime_ms = int((now_s - start_s) * 1000.0)

        # 1. Read incoming telemetry frames from DM-MC02.
        raw = serial.read()
        if raw:
            for frame in parser.feed(raw):
                tel = decode_telemetry(frame)
                if tel is not None:
                    telemetry = tel

        camera_near = bool(telemetry.status_flags & TEL_FLAG_CAMERA_NEAR)
        camera_settled = bool(telemetry.status_flags & TEL_FLAG_CAMERA_SETTLED)
        first_green_unlocked = bool(telemetry.status_flags & TEL_FLAG_FIRST_GREEN_UNLOCKED)
        carrying_casualty = bool(telemetry.status_flags & TEL_FLAG_CARRYING_CASUALTY)
        gate_closed = bool(telemetry.status_flags & TEL_FLAG_GATE_CLOSED)
        wheels_moving = abs(telemetry.vx_mm_s) > 35 or abs(telemetry.omega_mrad_s) > 250

        if camera_near != prev_camera_near or not camera_settled:
            motion_estimator.reset()
            prev_camera_near = camera_near

        # 2. Capture frame, run LAB blob pipeline (with near unknown-foreground scan) & spatial motion estimator.
        img = cam.read()
        blobs = extract_blobs_from_maix_image(img, cfg, scan_unknown_near=(camera_near and camera_settled))
        grid = _extract_motion_grid(img, cfg.motion_grid_w, cfg.motion_grid_h)
        motion_stalled, _ = motion_estimator.update(grid, now_s, wheels_moving=wheels_moving)

        vision_state = processor.process(
            blobs=blobs,
            camera_near_cmd=camera_near,
            camera_settled=camera_settled,
            team=telemetry.team if telemetry.team else ui.config.team,
            first_green_unlocked=first_green_unlocked,
            carrying_casualty=carrying_casualty,
            max_materials_per_trip=telemetry.max_materials_per_trip or ui.config.max_materials_per_trip,
            motion_stalled=motion_stalled,
            rear_clear=False,
            mission_state=telemetry.mission_state,
            gate_closed=gate_closed,
        )

        # 3. Poll touchscreen (locked automatically once MC02 confirms launch).
        tx, ty, pressed = ts.read()
        ui.handle_touch(tx, ty, bool(pressed), telemetry)

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

        # 5. Render camera feed + calibrated ROIs + touchscreen UI.
        disp_img = img.resize(disp.width(), disp.height())
        render_touch_overlay(disp_img, ui, telemetry)
        disp.show(disp_img)


if __name__ == "__main__":
    run_maixcam()
