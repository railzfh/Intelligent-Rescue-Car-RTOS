"""Touchscreen dataset capture tool for collecting YOLO training images on MaixCAM.

Run directly on MaixCAM before/during arena practice:
  python3 -m maixcam.capture_dataset

Controls (right-hand touchscreen panel):
  VIEW:FAR / VIEW:NEAR  - tag saved filenames by camera pitch angle
  SHOT                  - capture one raw 320x224 frame immediately
  AUTO:OFF / AUTO:2HZ   - toggle continuous 2 Hz burst capture while driving/moving blocks
"""
from __future__ import annotations

from pathlib import Path
import time

from .config import RescueVisionConfig


def run_capture(
    output_dir: Path | None = None,
    width: int = 320,
    height: int = 224,
    auto_interval_s: float = 0.5,
) -> None:
    from maix import app, camera, display, image, touchscreen

    cfg_file = Path(__file__).resolve().parent / "config.json"
    if cfg_file.exists():
        cfg = RescueVisionConfig.load(cfg_file)
        width = cfg.frame_width
        height = cfg.frame_height

    save_root = output_dir or Path("/root/rescue_dataset/images")
    save_root.mkdir(parents=True, exist_ok=True)

    cam = camera.Camera(width, height)
    disp = display.Display()
    ts = touchscreen.TouchScreen()

    dw, dh = disp.width(), disp.height()
    panel_w = max(140, int(dw * 0.28))
    px = dw - panel_w
    pad = 8
    btn_h = max(48, (dh - 60) // 4)
    buttons = (
        ("view", px + pad, 44, panel_w - 2 * pad, btn_h),
        ("shot", px + pad, 44 + btn_h + pad, panel_w - 2 * pad, btn_h),
        ("auto", px + pad, 44 + 2 * (btn_h + pad), panel_w - 2 * pad, btn_h),
    )

    c_white = image.Color.from_rgb(245, 245, 245)
    c_bg = image.Color.from_rgb(24, 28, 36)
    c_green = image.Color.from_rgb(40, 175, 85)
    c_blue = image.Color.from_rgb(35, 105, 235)
    c_amber = image.Color.from_rgb(220, 155, 25)
    c_dim = image.Color.from_rgb(65, 72, 86)

    view_tag = "far"
    auto_mode = False
    last_pressed = False
    last_auto_s = 0.0
    saved_count = len(list(save_root.glob("*.jpg")))

    def _save(raw_img) -> None:
        nonlocal saved_count
        saved_count += 1
        ts_ms = int(time.time() * 1000)
        fname = save_root / f"{view_tag}_{ts_ms}_{saved_count:04d}.jpg"
        raw_img.save(str(fname))

    while not app.need_exit():
        now_s = time.monotonic()
        raw = cam.read()
        if raw is None:
            continue

        tx, ty, pressed = ts.read()
        rising = bool(pressed) and not last_pressed
        last_pressed = bool(pressed)

        if rising:
            for bid, bx, by, bw, bh in buttons:
                if bx <= tx <= bx + bw and by <= ty <= by + bh:
                    if bid == "view":
                        view_tag = "near" if view_tag == "far" else "far"
                    elif bid == "shot":
                        _save(raw)
                    elif bid == "auto":
                        auto_mode = not auto_mode
                        last_auto_s = now_s
                    break

        if auto_mode and (now_s - last_auto_s >= auto_interval_s):
            last_auto_s = now_s
            _save(raw)

        disp_img = raw.resize(dw, dh)
        disp_img.draw_rect(0, 0, dw, 36, color=c_bg, thickness=-1)
        disp_img.draw_string(
            8,
            10,
            f"CAP {width}x{height} | {view_tag.upper()} | SAVED:{saved_count}",
            color=c_white,
            scale=1.0,
        )
        for bid, bx, by, bw, bh in buttons:
            if bid == "view":
                label = f"VIEW:{view_tag.upper()}"
                col = c_blue
            elif bid == "shot":
                label = "SHOT"
                col = c_green
            else:
                label = "AUTO:2HZ" if auto_mode else "AUTO:OFF"
                col = c_amber if auto_mode else c_dim
            disp_img.draw_rect(bx, by, bw, bh, color=col, thickness=-1)
            disp_img.draw_rect(bx, by, bw, bh, color=c_white, thickness=1)
            disp_img.draw_string(bx + 8, by + 16, label, color=c_white, scale=1.0)
        disp.show(disp_img)


if __name__ == "__main__":
    run_capture()
