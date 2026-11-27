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
            "orange": [48, 88, 14, 58, 36, 78],
            "cyan": [48, 90, -48, -16, -42, -6],
            "red_zone": [25, 68, 30, 78, 5, 34],
            "blue_zone": [20, 56, -10, 17, -68, -22],
            "purple_ramp": [20, 68, 18, 62, -55, -14],
            "opponent": [28, 66, -15, 15, -15, 15],
        }
    )
    # Arena floor LAB band used with invert=True in near ROIs to flag unrecognized foreground objects.
    arena_floor_lab: List[int] = field(default_factory=lambda: [68, 100, -14, 14, -14, 14])

    # Pixel-area filters to separate real targets from noise, shadows, and safe zones.
    min_block_pixels: int = 90
    max_block_pixels: int = 9500
    min_hazard_pixels: int = 60
    min_black_density: float = 0.42
    min_zone_pixels: int = 1600
    min_ramp_pixels: int = 350
    min_opponent_pixels: int = 2200

    # Pinhole ground-distance calibration: dist_mm = far_dist_k / max(8, bottom_y - far_horizon_y).
    # At bottom_y=240, dy=200 -> 150 mm, comfortably reaching target_capture_dist_mm=210 and home_drop_dist_mm=230.
    far_horizon_y: int = 40
    far_dist_k: float = 30000.0
    opponent_near_y: int = 145
    opponent_zone_near_y: int = 150

    # Half-zone side convention when facing own safe zone from the arena center:
    # True means material half is on the LEFT of the 20 mm center divider, casualty on the RIGHT.
    material_half_on_left_red: bool = True
    material_half_on_left_blue: bool = True
    half_match_margin_px: int = 18
    corral_align_half_w_px: int = 24
    ramp_home_max_gap_px: int = 12

    # NEAR-view calibrated ROIs (camera pitched down onto corral mouth & ramp drop zone).
    near_corral_roi: List[int] = field(default_factory=lambda: [88, 128, 144, 96])
    near_sweep_roi: List[int] = field(default_factory=lambda: [232, 118, 64, 106])
    near_drop_roi: List[int] = field(default_factory=lambda: [40, 20, 240, 108])
    near_ramp_band_y: List[int] = field(default_factory=lambda: [95, 145])

    # Spatial motion estimator thresholds (16x12 zero-mean grid, exposure-invariant).
    motion_grid_w: int = 16
    motion_grid_h: int = 12
    motion_min_texture_std: float = 6.0
    motion_static_sad_max: float = 4.5
    motion_min_cells_per_s: float = 3.0

    # YOLO + lightweight LAB hybrid vision settings (MaixCAM SG2002 NPU, 320x224 INT8 recommended).
    # Backends:
    #   "hybrid" - YOLO objects + LAB zones/ramp + near-view LAB cyan/unknown safety net (recommended)
    #   "yolo"   - YOLO primary + LAB zones/ramp (unless YOLO model outputs surfaces)
    #   "lab"    - original pure LAB pipeline (instant match-day fallback)
    vision_backend: str = "hybrid"
    yolo_arch: str = "yolo11"  # "yolo11", "yolov8", or "yolo5"
    yolo_model_path: str = "/root/models/rescue_yolo11n.mud"
    yolo_dual_buff: bool = False
    yolo_fallback_to_lab: bool = True
    yolo_conf_th: float = 0.45
    yolo_hazard_conf_th: float = 0.25
    yolo_opponent_conf_th: float = 0.40
    yolo_surface_conf_th: float = 0.40
    yolo_iou_th: float = 0.45
    yolo_cross_class_iou_th: float = 0.50
    yolo_ambiguous_score_margin: float = 0.10
    yolo_min_box_pixels: int = 36
    yolo_max_box_pixels: int = 24000
    yolo_unknown_unmapped: bool = True
    yolo_use_lab_surfaces: bool = True
    hybrid_lab_cyan_backup: bool = True
    hybrid_lab_unknown_backup: bool = True
    yolo_label_map: Dict[str, str] = field(
        default_factory=lambda: {
            "green": "green",
            "green_block": "green",
            "material_green": "green",
            "black": "black",
            "black_block": "black",
            "core": "black",
            "material_black": "black",
            "orange": "orange",
            "orange_block": "orange",
            "casualty": "orange",
            "cyan": "cyan",
            "cyan_block": "cyan",
            "hazard": "cyan",
            "danger": "cyan",
            "opponent": "opponent",
            "robot": "opponent",
            "car": "opponent",
            "red_zone": "red_zone",
            "blue_zone": "blue_zone",
            "purple_ramp": "purple_ramp",
            "ramp": "purple_ramp",
        }
    )

    def min_reachable_far_distance_mm(self) -> int:
        dy = max(8, self.frame_height - self.far_horizon_y)
        return max(80, min(3000, int(round(self.far_dist_k / dy))))

    def validate(self, target_capture_dist_mm: int = 210, home_drop_dist_mm: int = 230) -> None:
        if self.frame_width <= 0 or self.frame_height <= 0:
            raise ValueError("frame dimensions must be positive")
        if self.far_horizon_y < 0 or self.far_horizon_y >= self.frame_height - 16:
            raise ValueError("far_horizon_y must leave at least 16 vertical pixels below horizon")
        if self.far_dist_k <= 0.0:
            raise ValueError("far_dist_k must be positive")
        min_reach = self.min_reachable_far_distance_mm()
        required = min(target_capture_dist_mm, home_drop_dist_mm)
        if min_reach >= required:
            raise ValueError(
                f"uncalibrated geometry: min reachable far distance {min_reach} mm >= threshold {required} mm"
            )
        for name, roi in (
            ("near_corral_roi", self.near_corral_roi),
            ("near_sweep_roi", self.near_sweep_roi),
            ("near_drop_roi", self.near_drop_roi),
        ):
            if len(roi) != 4 or roi[2] <= 0 or roi[3] <= 0:
                raise ValueError(f"{name} must be [x, y, w, h] with positive size")
            if roi[0] < 0 or roi[1] < 0 or roi[0] + roi[2] > self.frame_width or roi[1] + roi[3] > self.frame_height:
                raise ValueError(f"{name}={roi} exceeds frame {self.frame_width}x{self.frame_height}")
        if self.vision_backend not in ("hybrid", "yolo", "lab"):
            raise ValueError(f"unsupported vision_backend: {self.vision_backend!r}")
        if self.yolo_arch not in ("yolo11", "yolov8", "yolo5"):
            raise ValueError(f"unsupported yolo_arch: {self.yolo_arch!r}")
        for name, value in (
            ("yolo_conf_th", self.yolo_conf_th),
            ("yolo_hazard_conf_th", self.yolo_hazard_conf_th),
            ("yolo_opponent_conf_th", self.yolo_opponent_conf_th),
            ("yolo_surface_conf_th", self.yolo_surface_conf_th),
            ("yolo_iou_th", self.yolo_iou_th),
            ("yolo_cross_class_iou_th", self.yolo_cross_class_iou_th),
        ):
            if not (0.0 < float(value) <= 1.0):
                raise ValueError(f"{name} must be in (0, 1], got {value}")
        if self.yolo_ambiguous_score_margin < 0.0:
            raise ValueError("yolo_ambiguous_score_margin must be non-negative")
        if self.yolo_min_box_pixels <= 0 or self.yolo_max_box_pixels < self.yolo_min_box_pixels:
            raise ValueError("yolo box pixel bounds must satisfy 0 < min <= max")

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
