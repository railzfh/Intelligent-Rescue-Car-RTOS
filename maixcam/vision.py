"""Vision decision pipeline for single-axis pitch MaixCAM on Intelligent Rescue Car.

Separates pure geometric/rule processing (fully testable on desktop Python) from
the thin MaixPy image blob extraction wrapper.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Iterable, List, Optional, Tuple

from .config import RescueVisionConfig
from .protocol import (
    CAMERA_VIEW_FAR,
    CAMERA_VIEW_NEAR,
    CAMERA_VIEW_TRANSITION,
    TARGET_BLACK_CORE,
    TARGET_CYAN_HAZARD,
    TARGET_GREEN_MATERIAL,
    TARGET_NONE,
    TARGET_ORANGE_CASUALTY,
    TEAM_BLUE,
    TEAM_RED,
    VIS_FLAG_CORRAL_SAFE,
    VIS_FLAG_DROP_CONFIRMED,
    VIS_FLAG_HOME_HALF_MATCH,
    VIS_FLAG_HOME_VALID,
    VIS_FLAG_MOTION_STALLED,
    VIS_FLAG_OPPONENT_NEAR,
    VIS_FLAG_OPPONENT_ZONE_NEAR,
    VIS_FLAG_RAMP_VISIBLE,
    VIS_FLAG_REAR_CLEAR,
    VIS_FLAG_SWEEP_CLEAR,
    VIS_FLAG_TARGET_VALID,
    VisionState,
)


@dataclass(frozen=True)
class Blob:
    label: str  # green, black, orange, cyan, unknown, red_zone, blue_zone, purple_ramp, opponent
    x: int
    y: int
    w: int
    h: int
    pixels: int
    density: float = 1.0  # pixels / (w * h)

    @property
    def cx(self) -> float:
        return self.x + 0.5 * self.w

    @property
    def cy(self) -> float:
        return self.y + 0.5 * self.h

    @property
    def x2(self) -> int:
        return self.x + self.w

    @property
    def y2(self) -> int:
        return self.y + self.h


def _rects_intersect(ax: int, ay: int, aw: int, ah: int, bx: int, by: int, bw: int, bh: int) -> bool:
    return ax < bx + bw and ax + aw > bx and ay < by + bh and ay + ah > by


def _blob_in_roi(blob: Blob, roi: List[int]) -> bool:
    return _rects_intersect(blob.x, blob.y, blob.w, blob.h, roi[0], roi[1], roi[2], roi[3])


def _label_to_target_type(label: str) -> int:
    mapping = {
        "green": TARGET_GREEN_MATERIAL,
        "black": TARGET_BLACK_CORE,
        "orange": TARGET_ORANGE_CASUALTY,
        "cyan": TARGET_CYAN_HAZARD,
    }
    return mapping.get(label, TARGET_NONE)


class RescueVisionProcessor:
    """Converts raw frame blobs + MC02 context into compact VisionState frames."""

    def __init__(self, config: RescueVisionConfig) -> None:
        self.cfg = config
        self._frame_seq = 0

    def bearing_mrad(self, cx: float) -> int:
        # Positive error means target is to the LEFT of image center (matches +omega CCW).
        half_w = 0.5 * self.cfg.frame_width
        norm = (half_w - cx) / max(1.0, half_w)
        hfov_rad = math.radians(self.cfg.camera_hfov_deg)
        return int(round(norm * 0.5 * hfov_rad * 1000.0))

    def estimate_far_distance_mm(self, bottom_y: int) -> int:
        dy = max(8, bottom_y - self.cfg.far_horizon_y)
        dist = int(round(self.cfg.far_dist_k / dy))
        return max(120, min(3000, dist))

    def _is_valid_block(self, blob: Blob) -> bool:
        if blob.pixels < self.cfg.min_block_pixels or blob.pixels > self.cfg.max_block_pixels:
            return False
        if blob.label == "black":
            # Reject elongated dark floor shadows by requiring compact pyramid solidity.
            aspect = max(blob.w, blob.h) / max(1, min(blob.w, blob.h))
            if blob.density < self.cfg.min_black_density or aspect > 2.2:
                return False
        return True

    def _select_best_target(
        self,
        blocks: List[Blob],
        first_green_unlocked: bool,
        hazards: List[Blob],
    ) -> Optional[Blob]:
        # Rule priority: before unlock ONLY green; after unlock orange (15) > black (10) > green (5).
        priority = {"orange": 3, "black": 2, "green": 1}
        candidates: List[Tuple[int, int, float, Blob]] = []
        for b in blocks:
            if b.label == "cyan" or b.label not in priority:
                continue
            if not first_green_unlocked and b.label != "green":
                continue
            # Avoid selecting a block tightly adjacent to a cyan hazard when other options exist.
            hazard_penalty = 0
            for hz in hazards:
                if abs(b.cx - hz.cx) < 42 and abs(b.cy - hz.cy) < 36:
                    hazard_penalty = 1
                    break
            dist_mm = self.estimate_far_distance_mm(b.y2)
            # Sort key: higher priority, no adjacent cyan hazard, closer distance, smaller bearing.
            candidates.append(
                (
                    -priority[b.label],
                    hazard_penalty,
                    dist_mm + 0.4 * abs(self.bearing_mrad(b.cx)),
                    b,
                )
            )
        if not candidates:
            return None
        candidates.sort(key=lambda item: (item[0], item[1], item[2]))
        return candidates[0][3]

    def _evaluate_home_zone(
        self,
        zones: List[Blob],
        ramps: List[Blob],
        team: int,
        carrying_casualty: bool,
    ) -> Tuple[bool, bool, bool, int, int]:
        own_label = "red_zone" if team == TEAM_RED else ("blue_zone" if team == TEAM_BLUE else "")
        opp_label = "blue_zone" if team == TEAM_RED else ("red_zone" if team == TEAM_BLUE else "")

        own_zones = [z for z in zones if z.label == own_label and z.pixels >= self.cfg.min_zone_pixels]
        opp_zones = [z for z in zones if z.label == opp_label and z.pixels >= self.cfg.min_zone_pixels]
        opp_near = any(z.y2 >= self.cfg.opponent_zone_near_y for z in opp_zones)

        if not own_zones:
            return False, False, opp_near, 0, 0

        home = max(own_zones, key=lambda z: z.pixels)
        material_left = (
            self.cfg.material_half_on_left_red if team == TEAM_RED else self.cfg.material_half_on_left_blue
        )
        want_left = (not carrying_casualty) if material_left else carrying_casualty
        # Aim for the center of the required half (left quarter or right quarter of the safe zone).
        aim_x = home.x + (0.25 * home.w if want_left else 0.75 * home.w)
        home_err_mrad = self.bearing_mrad(aim_x)

        ramp_y = max((r.y2 for r in ramps if r.pixels >= self.cfg.min_ramp_pixels), default=home.y2)
        home_dist_mm = self.estimate_far_distance_mm(ramp_y)

        # Verify image center falls cleanly on the desired side of the 20 mm center divider.
        center_x = 0.5 * self.cfg.frame_width
        divider_x = home.cx
        margin = self.cfg.half_match_margin_px
        if want_left:
            half_matches = home.x <= center_x <= (divider_x - margin)
        else:
            half_matches = (divider_x + margin) <= center_x <= home.x2

        return True, half_matches, opp_near, home_err_mrad, home_dist_mm

    def process(
        self,
        blobs: Iterable[Blob],
        camera_near_cmd: bool,
        camera_settled: bool,
        team: int,
        first_green_unlocked: bool,
        carrying_casualty: bool = False,
        wheels_moving: bool = False,
        frame_motion_delta: float = 10.0,
        rear_clear: bool = True,
    ) -> VisionState:
        self._frame_seq = (self._frame_seq + 1) & 0xFFFF
        view = CAMERA_VIEW_TRANSITION
        if camera_settled:
            view = CAMERA_VIEW_NEAR if camera_near_cmd else CAMERA_VIEW_FAR

        all_blobs = list(blobs)
        blocks = [b for b in all_blobs if b.label in ("green", "black", "orange", "cyan") and self._is_valid_block(b)]
        unknowns = [b for b in all_blobs if b.label == "unknown" and b.pixels >= self.cfg.min_block_pixels]
        hazards = [b for b in blocks if b.label == "cyan"]
        zones = [b for b in all_blobs if b.label in ("red_zone", "blue_zone")]
        ramps = [b for b in all_blobs if b.label == "purple_ramp" and b.pixels >= self.cfg.min_ramp_pixels]
        opponents = [b for b in all_blobs if b.label == "opponent" and b.pixels >= self.cfg.min_opponent_pixels]

        flags = 0
        if rear_clear:
            flags |= VIS_FLAG_REAR_CLEAR
        if ramps:
            flags |= VIS_FLAG_RAMP_VISIBLE
        if any(op.y2 >= self.cfg.opponent_near_y for op in opponents):
            flags |= VIS_FLAG_OPPONENT_NEAR
        if wheels_moving and frame_motion_delta < self.cfg.visual_stall_delta_min:
            flags |= VIS_FLAG_MOTION_STALLED

        state = VisionState(frame_seq=self._frame_seq, camera_view=view, confidence=0)

        home_valid, half_match, opp_zone_near, home_err, home_dist = self._evaluate_home_zone(
            zones, ramps, team, carrying_casualty
        )
        if opp_zone_near:
            flags |= VIS_FLAG_OPPONENT_ZONE_NEAR
        if home_valid:
            flags |= VIS_FLAG_HOME_VALID
            state.home_err_x_mrad = home_err
            state.home_dist_mm = home_dist
            if half_match:
                flags |= VIS_FLAG_HOME_HALF_MATCH

        if view == CAMERA_VIEW_FAR:
            best = self._select_best_target(blocks, first_green_unlocked, hazards)
            if best is not None:
                flags |= VIS_FLAG_TARGET_VALID
                state.target_type = _label_to_target_type(best.label)
                state.target_err_x_mrad = self.bearing_mrad(best.cx)
                state.target_dist_mm = self.estimate_far_distance_mm(best.y2)
                state.confidence = 90
            elif home_valid:
                state.confidence = 85
            else:
                state.confidence = 70

        elif view == CAMERA_VIEW_NEAR:
            corral_roi = self.cfg.near_corral_roi
            sweep_roi = self.cfg.near_sweep_roi
            drop_roi = self.cfg.near_drop_roi
            ramp_top_y, ramp_bot_y = self.cfg.near_ramp_band_y

            green_cnt = sum(1 for b in blocks if b.label == "green" and _blob_in_roi(b, corral_roi))
            black_cnt = sum(1 for b in blocks if b.label == "black" and _blob_in_roi(b, corral_roi))
            orange_cnt = sum(1 for b in blocks if b.label == "orange" and _blob_in_roi(b, corral_roi))
            cyan_cnt = sum(1 for b in blocks if b.label == "cyan" and _blob_in_roi(b, corral_roi))
            unknown_cnt = sum(1 for b in unknowns if _blob_in_roi(b, corral_roi))

            state.corral_green_cnt = min(255, green_cnt)
            state.corral_black_cnt = min(255, black_cnt)
            state.corral_orange_cnt = min(255, orange_cnt)
            state.corral_cyan_cnt = min(255, cyan_cnt)
            state.corral_unknown_cnt = min(255, unknown_cnt)

            if cyan_cnt == 0 and unknown_cnt == 0:
                flags |= VIS_FLAG_CORRAL_SAFE

            # Sweep zone must be clear of ANY block (hazard or extra target) before lowering the inverted-L rod.
            sweep_occupied = any(_blob_in_roi(b, sweep_roi) for b in blocks + unknowns)
            if not sweep_occupied:
                flags |= VIS_FLAG_SWEEP_CLEAR

            # Drop verification: require entire block outline strictly past the purple ramp inner edge
            # (smaller y = farther inside safe zone) and NO cargo sitting on the purple ramp band.
            ramp_inner_y = min((r.y for r in ramps), default=ramp_top_y)
            cargo_blobs = [b for b in blocks if b.label in ("green", "black", "orange")]
            dropped_fully_inside = [
                b
                for b in cargo_blobs
                if _blob_in_roi(b, drop_roi) and b.y2 <= ramp_inner_y
            ]
            straddling_ramp = any(
                b.y < ramp_bot_y and b.y2 > ramp_inner_y for b in cargo_blobs
            )
            cyan_in_drop_or_ramp = any(
                _blob_in_roi(b, drop_roi) or (b.y < ramp_bot_y and b.y2 > ramp_inner_y)
                for b in hazards
            )
            if dropped_fully_inside and not straddling_ramp and not cyan_in_drop_or_ramp:
                flags |= VIS_FLAG_DROP_CONFIRMED

            state.confidence = 92

        state.flags = flags
        return state


def extract_blobs_from_maix_image(img, cfg: RescueVisionConfig) -> List[Blob]:
    """Run LAB blob detection on a MaixPy image object."""
    blobs: List[Blob] = []
    for label, thresh in cfg.lab_thresholds.items():
        min_area = cfg.min_zone_pixels if label.endswith("_zone") else cfg.min_block_pixels
        if label == "purple_ramp":
            min_area = cfg.min_ramp_pixels
        elif label == "opponent":
            min_area = cfg.min_opponent_pixels
        found = img.find_blobs([tuple(thresh)], pixels_threshold=min_area, area_threshold=min_area, merge=True)
        for item in found:
            w = max(1, int(item.w()))
            h = max(1, int(item.h()))
            px = int(item.pixels())
            blobs.append(
                Blob(
                    label=label,
                    x=int(item.x()),
                    y=int(item.y()),
                    w=w,
                    h=h,
                    pixels=px,
                    density=px / float(w * h),
                )
            )
    return blobs
