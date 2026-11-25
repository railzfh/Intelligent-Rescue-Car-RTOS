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
    Telemetry,
    decode_telemetry,
    encode_bench_actuator,
    encode_heartbeat,
    encode_set_config,
    encode_vision_state,
)
from .ui import TouchUIController, render_touch_overlay
from .vision import RescueVisionProcessor, extract_blobs_from_maix_image


def run_maixcam(config_path: Path | None = None) -> None:
    from maix import app, camera, display, pinmap, touchscreen, uart

    cfg_file = config_path or (Path(__file__).resolve().parent / "config.json")
    cfg = RescueVisionConfig.load(cfg_file)

    pinmap.set_pin_function(cfg.uart_tx_pin, "UART1_TX")
    pinmap.set_pin_function(cfg.uart_rx_pin, "UART1_RX")
    serial = uart.UART(cfg.uart_device, cfg.uart_baud)

    cam = camera.Camera(cfg.frame_width, cfg.frame_height)
    disp = display.Display()
    ts = touchscreen.TouchScreen()

    ui = TouchUIController(width=disp.width(), height=disp.height())
    processor = RescueVisionProcessor(cfg)
    parser = RescueProtocolParser()
    telemetry = Telemetry()

    tx_seq = 0
    start_s = time.monotonic()
    last_hb_s = 0.0
    last_cfg_s = 0.0
    last_vis_s = 0.0
    prev_luma = None

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

        # 2. Capture frame and run LAB blob pipeline.
        img = cam.read()
        blobs = extract_blobs_from_maix_image(img, cfg)

        # Coarse mean-intensity delta as lightweight visual slip indicator.
        stats = img.get_statistics()
        cur_luma = float(stats.l_mean())
        motion_delta = 10.0 if prev_luma is None else abs(cur_luma - prev_luma)
        prev_luma = cur_luma

        camera_near = bool(telemetry.status_flags & TEL_FLAG_CAMERA_NEAR)
        camera_settled = bool(telemetry.status_flags & TEL_FLAG_CAMERA_SETTLED)
        first_green_unlocked = bool(telemetry.status_flags & TEL_FLAG_FIRST_GREEN_UNLOCKED)
        wheels_moving = abs(telemetry.vx_mm_s) > 35 or abs(telemetry.omega_mrad_s) > 250
        carrying_casualty = bool(telemetry.status_flags & TEL_FLAG_CARRYING_CASUALTY)

        vision_state = processor.process(
            blobs=blobs,
            camera_near_cmd=camera_near,
            camera_settled=camera_settled,
            team=telemetry.team if telemetry.team else ui.config.team,
            first_green_unlocked=first_green_unlocked,
            carrying_casualty=carrying_casualty,
            wheels_moving=wheels_moving,
            frame_motion_delta=motion_delta,
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
