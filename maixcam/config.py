"""Calibration and runtime configuration for MaixCAM touchscreen vision."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
from typing import Dict, List, Tuple

LabThreshold = Tuple[int, int, int, int, int, int]
Rect = Tuple[int, int, int, int]  # (x, y, w, h) in camera frame pixels


@dataclass
class RescueVisionConfig:
    frame_width: int = 320
    frame_height: int = 240
    camera_hfov_deg: float = 68.0

    uart_device: str = "/dev/ttyS1"
    uart_baud: int = 115200
    uart_tx_pin: str = "A19"
    uart_rx_pin: str = "A18"

    # On-site adjustable LAB thresholds: (L_min, L_max, A_min, A_max, B_min, B_max).
    # Defaults are starting points for 3D-printed PLA blocks under indoor lighting;
    # always verify on the competition arena during the 3-minute debugging window.
    lab_thresholds: Dict[str, List[int]] = field(
        default_factory=lambda: {
            "green": [30, 82, -60, -18, 12, 55],
            "black": [0, 26, -12, 12, -12, 12],
            "orange": [42, 85, 18, 65, 25, 72],
            "cyan": [48, 90, -45, -8, -42, -6],
            "red_zone": [28, 72, 26, 75, 10, 58],
            "blue_zone": [20, 62, -10, 30, -68, -22],
            "purple_ramp": [20, 68, 18, 62, -55, -14],
            "opponent": [35, 95, -15, 15, -15, 15],
        }
    )

    # Pixel-area filters to separate real targets from noise, shadows, and safe zones.
    min_block_pixels: int = 90
    max_block_pixels: int = 9500
    min_black_density: float = 0.42
    min_zone_pixels: int = 1600
    min_ramp_pixels: int = 350
    min_opponent_pixels: int = 2200

    # Simple pinhole ground-distance calibration: dist_mm = far_dist_k / max(1, bottom_y - horizon_y).
    far_horizon_y: int = 50
    far_dist_k: float = 42000.0
    opponent_near_y: int = 145
    opponent_zone_near_y: int = 150

    # Half-zone side convention when facing own safe zone from the arena center:
    # True means material half is on the LEFT of the 20 mm center divider, casualty on the RIGHT.
    material_half_on_left_red: bool = True
    material_half_on_left_blue: bool = True
    half_match_margin_px: int = 18

    # NEAR-view calibrated ROIs (camera pitched down onto corral mouth & ramp drop zone).
    near_corral_roi: List[int] = field(default_factory=lambda: [88, 128, 144, 96])
    near_sweep_roi: List[int] = field(default_factory=lambda: [232, 118, 64, 106])
    near_drop_roi: List[int] = field(default_factory=lambda: [60, 26, 200, 96])
    near_ramp_band_y: List[int] = field(default_factory=lambda: [95, 145])

    # Visual motion-stall threshold (mean pixel delta while wheels are turning).
    visual_stall_delta_min: float = 1.8

    def save(self, path: Path) -> None:
        path.write_text(json.dumps(asdict(self), indent=2, ensure_ascii=False), encoding="utf-8")

    @classmethod
    def load(cls, path: Path) -> "RescueVisionConfig":
        if not path.exists():
            return cls()
        raw = json.loads(path.read_text(encoding="utf-8"))
        cfg = cls()
        for key, value in raw.items():
            if hasattr(cfg, key):
                setattr(cfg, key, value)
        return cfg
