"""Touchscreen pre-match setup and live telemetry HUD for MaixCAM."""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

from .protocol import (
    BENCH_SERVO_KEEP,
    BENCH_SERVO_POS_A,
    BENCH_SERVO_POS_B,
    BenchActuatorCmd,
    MODE_BENCH_STRAIGHT,
    MODE_COMPETITION,
    MatchConfigCmd,
    TEAM_BLUE,
    TEAM_RED,
    TEAM_UNSET,
    TEL_FLAG_BATTERY_OK,
    TEL_FLAG_CAMERA_NEAR,
    TEL_FLAG_CONFIG_LOCKED,
    TEL_FLAG_FIRST_GREEN_UNLOCKED,
    TEL_FLAG_GATE_CLOSED,
    TEL_FLAG_IMU_OK,
    TEL_FLAG_MOTORS_OK,
    TEL_FLAG_VISION_FRESH,
    Telemetry,
)

MISSION_STATE_NAMES = {
    0: "PRE_MATCH",
    1: "BENCH_1M",
    2: "CROSS_BUMPS",
    3: "SEARCH",
    4: "APPROACH",
    5: "INSPECT",
    6: "SEPARATE",
    7: "CLOSE_VER",
    8: "FIND_HOME",
    9: "GO_HOME",
    10: "DROP_PUSH",
    11: "DROP_VER",
    12: "ESCAPE",
    13: "FINISHED",
    14: "FAULT",
}


@dataclass(frozen=True)
class TouchButton:
    button_id: str
    label: str
    x: int
    y: int
    w: int
    h: int

    def contains(self, px: int, py: int) -> bool:
        return self.x <= px <= self.x + self.w and self.y <= py <= self.y + self.h


class TouchUIController:
    """Handles MaixCAM touchscreen team/zone/mode selection and config locking."""

    def __init__(self, width: int = 552, height: int = 368) -> None:
        self.width = width
        self.height = height
        self.config = MatchConfigCmd(
            team=TEAM_UNSET,
            start_zone=1,
            run_mode=MODE_COMPETITION,
            max_materials_per_trip=1,
        )
        self.bench_gate_closed = False
        self.bench_camera_near = False
        self.pending_bench_cmd: Optional[BenchActuatorCmd] = None
        self.config_dirty = True
        self._last_pressed = False
        self.buttons = self._build_layout()

    def _build_layout(self) -> List[TouchButton]:
        # Right-side control panel leaves camera preview unobstructed on the left.
        panel_w = max(168, int(self.width * 0.34))
        px = self.width - panel_w
        pad = 6
        btn_h = max(36, (self.height - 64) // 6)
        half_w = (panel_w - 3 * pad) // 2
        qtr_w = (panel_w - 5 * pad) // 4

        y0 = 44
        y1 = y0 + btn_h + pad
        y2 = y1 + btn_h + pad
        y3 = y2 + btn_h + pad
        y4 = y3 + btn_h + pad
        return [
            TouchButton("team_red", "RED", px + pad, y0, half_w, btn_h),
            TouchButton("team_blue", "BLUE", px + 2 * pad + half_w, y0, half_w, btn_h),
            TouchButton("zone_1", "Z1", px + pad, y1, qtr_w, btn_h),
            TouchButton("zone_2", "Z2", px + 2 * pad + qtr_w, y1, qtr_w, btn_h),
            TouchButton("zone_3", "Z3", px + 3 * pad + 2 * qtr_w, y1, qtr_w, btn_h),
            TouchButton("zone_4", "Z4", px + 4 * pad + 3 * qtr_w, y1, qtr_w, btn_h),
            TouchButton("mode_toggle", "MODE", px + pad, y2, panel_w - 2 * pad, btn_h),
            TouchButton("max_mat", "LOAD:1", px + pad, y3, panel_w - 2 * pad, btn_h),
            TouchButton("bench_gate", "GATE", px + pad, y4, half_w, btn_h),
            TouchButton("bench_cam", "PITCH", px + 2 * pad + half_w, y4, half_w, btn_h),
        ]

    def handle_touch(self, x: int, y: int, pressed: bool, telemetry: Telemetry) -> Optional[str]:
        rising = pressed and not self._last_pressed
        self._last_pressed = pressed
        if not rising:
            return None

        # Never allow changing team/zone/servos once MC02 has locked match config.
        if telemetry.status_flags & TEL_FLAG_CONFIG_LOCKED:
            return "locked"

        for btn in self.buttons:
            if not btn.contains(x, y):
                continue
            if btn.button_id == "team_red":
                self.config.team = TEAM_RED
                self.config.run_mode = MODE_COMPETITION
                self.config_dirty = True
                return btn.button_id
            if btn.button_id == "team_blue":
                self.config.team = TEAM_BLUE
                self.config.run_mode = MODE_COMPETITION
                self.config_dirty = True
                return btn.button_id
            if btn.button_id.startswith("zone_"):
                self.config.start_zone = int(btn.button_id[-1])
                self.config_dirty = True
                return btn.button_id
            if btn.button_id == "mode_toggle":
                self.config.run_mode = (
                    MODE_BENCH_STRAIGHT
                    if self.config.run_mode == MODE_COMPETITION
                    else MODE_COMPETITION
                )
                self.config_dirty = True
                return btn.button_id
            if btn.button_id == "max_mat":
                self.config.max_materials_per_trip = (self.config.max_materials_per_trip % 3) + 1
                self.config_dirty = True
                return btn.button_id
            if btn.button_id == "bench_gate" and self.config.run_mode == MODE_BENCH_STRAIGHT:
                self.bench_gate_closed = not self.bench_gate_closed
                self.pending_bench_cmd = BenchActuatorCmd(
                    gate_cmd=BENCH_SERVO_POS_B if self.bench_gate_closed else BENCH_SERVO_POS_A,
                    pitch_cmd=BENCH_SERVO_KEEP,
                )
                return btn.button_id
            if btn.button_id == "bench_cam" and self.config.run_mode == MODE_BENCH_STRAIGHT:
                self.bench_camera_near = not self.bench_camera_near
                self.pending_bench_cmd = BenchActuatorCmd(
                    gate_cmd=BENCH_SERVO_KEEP,
                    pitch_cmd=BENCH_SERVO_POS_B if self.bench_camera_near else BENCH_SERVO_POS_A,
                )
                return btn.button_id
        return None

    def pop_bench_cmd(self) -> Optional[BenchActuatorCmd]:
        cmd = self.pending_bench_cmd
        self.pending_bench_cmd = None
        return cmd

    def hud_summary_lines(self, telemetry: Telemetry) -> Tuple[str, str]:
        state_name = MISSION_STATE_NAMES.get(telemetry.mission_state, f"S{telemetry.mission_state}")
        batt_v = telemetry.battery_mv / 1000.0
        team_str = {TEAM_RED: "RED", TEAM_BLUE: "BLUE"}.get(telemetry.team, "UNSET")
        lock_str = "LOCKED" if (telemetry.status_flags & TEL_FLAG_CONFIG_LOCKED) else "SETUP"
        unlock_str = "UNLOCKED" if (telemetry.status_flags & TEL_FLAG_FIRST_GREEN_UNLOCKED) else "1ST_GRN"
        line1 = f"{team_str} Z{telemetry.start_zone} | {state_name} | {lock_str} | {batt_v:.1f}V"
        line2 = (
            f"{unlock_str} G:{telemetry.delivered_green} B:{telemetry.delivered_black} "
            f"O:{telemetry.delivered_orange} F:{telemetry.fault_flags:#x}"
        )
        return line1, line2


def render_touch_overlay(img, ui: TouchUIController, telemetry: Telemetry) -> None:
    """Draw HUD and touch controls onto a MaixPy display image."""
    from maix import image  # Imported lazily so desktop tests require no MaixPy SDK.

    c_white = image.Color.from_rgb(245, 245, 245)
    c_bg = image.Color.from_rgb(24, 28, 36)
    c_red = image.Color.from_rgb(215, 45, 45)
    c_blue = image.Color.from_rgb(35, 105, 235)
    c_green = image.Color.from_rgb(40, 175, 85)
    c_dim = image.Color.from_rgb(65, 72, 86)
    c_amber = image.Color.from_rgb(220, 155, 25)

    line1, line2 = ui.hud_summary_lines(telemetry)
    img.draw_rect(0, 0, ui.width, 38, color=c_bg, thickness=-1)
    img.draw_string(6, 4, line1, color=c_white, scale=1.0)
    img.draw_string(6, 20, line2, color=c_amber if telemetry.fault_flags else c_green, scale=1.0)

    locked = bool(telemetry.status_flags & TEL_FLAG_CONFIG_LOCKED)
    for btn in ui.buttons:
        color = c_dim
        label = btn.label
        if btn.button_id == "team_red":
            color = c_red if ui.config.team == TEAM_RED else c_dim
        elif btn.button_id == "team_blue":
            color = c_blue if ui.config.team == TEAM_BLUE else c_dim
        elif btn.button_id.startswith("zone_"):
            z = int(btn.button_id[-1])
            color = c_green if ui.config.start_zone == z else c_dim
        elif btn.button_id == "mode_toggle":
            label = "BENCH 1M" if ui.config.run_mode == MODE_BENCH_STRAIGHT else "MATCH"
            color = c_amber if ui.config.run_mode == MODE_BENCH_STRAIGHT else c_green
        elif btn.button_id == "max_mat":
            label = f"MAX MAT: {ui.config.max_materials_per_trip}"
        elif btn.button_id == "bench_gate":
            label = "GATE:CLS" if (telemetry.status_flags & TEL_FLAG_GATE_CLOSED) else "GATE:OPN"
            color = c_amber if ui.config.run_mode == MODE_BENCH_STRAIGHT else c_bg
        elif btn.button_id == "bench_cam":
            label = "CAM:NEAR" if (telemetry.status_flags & TEL_FLAG_CAMERA_NEAR) else "CAM:FAR"
            color = c_amber if ui.config.run_mode == MODE_BENCH_STRAIGHT else c_bg

        if locked:
            color = c_bg
        img.draw_rect(btn.x, btn.y, btn.w, btn.h, color=color, thickness=-1)
        img.draw_rect(btn.x, btn.y, btn.w, btn.h, color=c_white, thickness=1)
        img.draw_string(btn.x + 6, btn.y + 10, label, color=c_white, scale=1.0)
