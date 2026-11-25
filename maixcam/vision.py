"""Vision decision pipeline for single-axis pitch MaixCAM on Intelligent Rescue Car.

Separates pure geometric/rule processing and spatial motion estimation (fully testable
on desktop Python) from the thin MaixPy image blob extraction wrapper.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, List, Optional, Sequence, Tuple

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


def _rect_intersection_area(ax: int, ay: int, aw: int, ah: int, bx: int, by: int, bw: int, bh: int) -> int:
    ix1 = max(ax, bx)
    iy1 = max(ay, by)
    ix2 = min(ax + aw, bx + bw)
    iy2 = min(ay + ah, by + bh)
    if ix2 <= ix1 or iy2 <= iy1:
        return 0
    return (ix2 - ix1) * (iy2 - iy1)


def classify_unknown_foreground(
    fg_blobs: Iterable[Blob],
    known_blobs: Iterable[Blob],
    min_pixels: int,
    coverage_threshold: float = 0.70,
) -> List[Blob]:
    """Flag foreground blobs not substantially covered by known block/hazard classes.

    Zones/ramps/opponents never mask unknown objects inside the corral or sweep zone,
    and minor bounding-box edge contact (< coverage_threshold) still fails closed.
    """
    known_blocks = [b for b in known_blobs if b.label in ("green", "black", "orange", "cyan")]
    unknowns: List[Blob] = []
    for fg in fg_blobs:
        if fg.pixels < min_pixels:
            continue
        fg_area = max(1, fg.w * fg.h)
        covered_area = 0
        for kb in known_blocks:
            covered_area += _rect_intersection_area(fg.x, fg.y, fg.w, fg.h, kb.x, kb.y, kb.w, kb.h)
        if (covered_area / float(fg_area)) < coverage_threshold:
            unknowns.append(Blob("unknown", fg.x, fg.y, fg.w, fg.h, fg.pixels, fg.density))
    return unknowns


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


class SpatialMotionEstimator:
    """Exposure-invariant 2D block-matching motion estimator on a small grayscale grid.

    Subtracts per-frame mean brightness so exposure/auto-gain flicker cannot hide stalls,
    and matches spatial shifts so constant-mean scrolling textures do not false-trigger stalls.
    """

    def __init__(
        self,
        width: int = 16,
        height: int = 12,
        search_radius: int = 3,
        min_texture_std: float = 6.0,
        static_sad_max: float = 4.5,
        min_cells_per_s: float = 3.0,
    ) -> None:
        self.width = width
        self.height = height
        self.search_radius = search_radius
        self.min_texture_std = min_texture_std
        self.static_sad_max = static_sad_max
        self.min_cells_per_s = min_cells_per_s
        self._prev_norm: Optional[List[float]] = None
        self._prev_time_s: Optional[float] = None

    def reset(self) -> None:
        self._prev_norm = None
        self._prev_time_s = None

    def update(self, grid: Sequence[float], now_s: float, wheels_moving: bool) -> Tuple[bool, float]:
        """Return (is_visually_stalled, estimated_speed_cells_per_s)."""
        expected = self.width * self.height
        if len(grid) != expected:
            self.reset()
            return False, 0.0

        mean = sum(grid) / float(expected)
        cur_norm = [float(v) - mean for v in grid]
        variance = sum(v * v for v in cur_norm) / float(expected)
        texture_std = math.sqrt(variance)

        if self._prev_norm is None or self._prev_time_s is None:
            self._prev_norm = cur_norm
            self._prev_time_s = now_s
            return False, 0.0

        dt_s = now_s - self._prev_time_s
        prev_norm = self._prev_norm
        self._prev_norm = cur_norm
        self._prev_time_s = now_s

        # Require valid elapsed time and sufficient texture confidence before judging motion.
        if dt_s < 0.01 or dt_s > 0.5 or texture_std < self.min_texture_std:
            return False, 0.0

        r = self.search_radius
        w = self.width
        h = self.height
        best_dx = 0
        best_dy = 0
        best_sad = float("inf")
        best_dist_sq = 0
        sad_zero = float("inf")

        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                total = 0.0
                count = 0
                for y in range(r, h - r):
                    row_prev = y * w
                    row_cur = (y + dy) * w
                    for x in range(r, w - r):
                        total += abs(cur_norm[row_cur + x + dx] - prev_norm[row_prev + x])
                        count += 1
                sad = total / float(max(1, count))
                dist_sq = dx * dx + dy * dy
                if dx == 0 and dy == 0:
                    sad_zero = sad
                # Break periodic/alias SAD ties in favor of minimal spatial displacement.
                if (sad < best_sad - 1e-6) or (abs(sad - best_sad) <= 1e-6 and dist_sq < best_dist_sq):
                    best_sad = sad
                    best_dx = dx
                    best_dy = dy
                    best_dist_sq = dist_sq

        # If zero shift explains the scene within static tolerance and matches the best periodic shift,
        # do not misreport a stationary periodic texture (e.g. checkerboard) as rapid motion.
        if sad_zero <= self.static_sad_max and sad_zero <= best_sad + 0.5:
            best_dx = 0
            best_dy = 0

        disp_cells = math.hypot(best_dx, best_dy)
        speed_cells_per_s = disp_cells / dt_s

        # Stalled only when wheels are spinning, spatial shift is near zero, AND
        # zero-shift normalized residual confirms the scene genuinely did not move.
        stalled = (
            wheels_moving
            and speed_cells_per_s < self.min_cells_per_s
            and sad_zero <= self.static_sad_max
        )
        return stalled, speed_cells_per_s


class RescueVisionProcessor:
    """Converts raw frame blobs + MC02 context into compact VisionState frames."""

    def __init__(self, config: RescueVisionConfig) -> None:
        config.validate()
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
        return max(80, min(3000, dist))

    def _is_valid_target_block(self, blob: Blob) -> bool:
        if blob.pixels < self.cfg.min_block_pixels or blob.pixels > self.cfg.max_block_pixels:
            return False
        if blob.label == "black":
            # Reject elongated dark floor shadows from target selection.
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
            if b.label not in priority:
                continue
            if not first_green_unlocked and b.label != "green":
                continue
            hazard_penalty = 0
            for hz in hazards:
                if abs(b.cx - hz.cx) < 42 and abs(b.cy - hz.cy) < 36:
                    hazard_penalty = 1
                    break
            dist_mm = self.estimate_far_distance_mm(b.y2)
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

    def _target_half_bounds(self, home: Blob, team: int, carrying_casualty: bool) -> Tuple[float, float, float]:
        """Return (half_x1, half_x2, aim_x) strictly inside own target half past the 20 mm divider."""
        material_left = (
            self.cfg.material_half_on_left_red if team == TEAM_RED else self.cfg.material_half_on_left_blue
        )
        want_left = (not carrying_casualty) if material_left else carrying_casualty
        divider_x = home.cx
        margin = float(self.cfg.half_match_margin_px)
        if want_left:
            return float(home.x), divider_x - margin, home.x + 0.25 * home.w
        return divider_x + margin, float(home.x2), home.x + 0.75 * home.w

    def _evaluate_home_zone(
        self,
        zones: List[Blob],
        ramps: List[Blob],
        team: int,
        carrying_casualty: bool,
    ) -> Tuple[Optional[Blob], Optional[Blob], bool, bool, int, int]:
        own_label = "red_zone" if team == TEAM_RED else ("blue_zone" if team == TEAM_BLUE else "")
        opp_label = "blue_zone" if team == TEAM_RED else ("red_zone" if team == TEAM_BLUE else "")

        own_zones = [z for z in zones if z.label == own_label and z.pixels >= self.cfg.min_zone_pixels]
        opp_zones = [z for z in zones if z.label == opp_label and z.pixels >= self.cfg.min_zone_pixels]
        opp_near = any(z.y2 >= self.cfg.opponent_zone_near_y for z in opp_zones)

        if not own_zones:
            return None, None, False, opp_near, 0, 0

        home = max(own_zones, key=lambda z: z.pixels)
        half_x1, half_x2, aim_x = self._target_half_bounds(home, team, carrying_casualty)
        home_err_mrad = self.bearing_mrad(aim_x)

        # Associate a purple ramp horizontally overlapping own safe zone.
        assoc_ramps = [r for r in ramps if r.x < home.x2 and r.x2 > home.x]
        assoc_ramp = max(assoc_ramps, key=lambda r: r.pixels) if assoc_ramps else None
        ramp_y = assoc_ramp.y2 if assoc_ramp is not None else home.y2
        home_dist_mm = self.estimate_far_distance_mm(ramp_y)

        # Verify full projected corral width fits strictly inside target half past divider margin.
        center_x = 0.5 * self.cfg.frame_width
        half_corral = float(self.cfg.corral_align_half_w_px)
        half_matches = (
            half_x2 > half_x1
            and (center_x - half_corral) >= half_x1
            and (center_x + half_corral) <= half_x2
        )

        return home, assoc_ramp, half_matches, opp_near, home_err_mrad, home_dist_mm

    def _verify_drop_inside_correct_half(
        self,
        home: Optional[Blob],
        assoc_ramp: Optional[Blob],
        blocks: List[Blob],
        hazards: List[Blob],
        unknowns: List[Blob],
        corral_empty: bool,
        team: int,
        first_green_unlocked: bool,
        carrying_casualty: bool,
        max_materials_per_trip: int,
    ) -> bool:
        # Must observe BOTH own team safe zone and its associated purple ramp (never fallback line),
        # AND the corral itself must be empty (an old block already in the safe zone cannot confirm
        # a drop while the current block is still sitting inside our corral!).
        if not corral_empty or home is None or assoc_ramp is None:
            return False

        half_x1, half_x2, _ = self._target_half_bounds(home, team, carrying_casualty)
        if half_x2 <= half_x1:
            return False

        ramp_inner_y = assoc_ramp.y
        ramp_bot_y = assoc_ramp.y2
        drop_roi = self.cfg.near_drop_roi

        # Fail closed if ANY cyan hazard or unknown object is inside home, drop ROI, or ramp band.
        for bad in hazards + unknowns:
            in_home = _rects_intersect(bad.x, bad.y, bad.w, bad.h, home.x, home.y, home.w, home.h)
            on_ramp = bad.y < ramp_bot_y and bad.y2 > ramp_inner_y
            if in_home or on_ramp or _blob_in_roi(bad, drop_roi):
                return False

        # Every cargo blob in/near home or ramp must be strictly inside target half past ramp inner edge.
        dropped_green = 0
        dropped_black = 0
        dropped_orange = 0
        for b in blocks:
            in_home = _rects_intersect(b.x, b.y, b.w, b.h, home.x, home.y, home.w, home.h)
            on_ramp = b.y < ramp_bot_y and b.y2 > ramp_inner_y and b.x < home.x2 and b.x2 > home.x
            if not (in_home or on_ramp):
                continue

            # Reject if straddling the purple ramp or crossing outside the target half (e.g. divider/wrong half).
            if on_ramp or b.y2 > ramp_inner_y or b.y < home.y:
                return False
            if b.x < half_x1 or b.x2 > half_x2:
                return False

            if b.label == "green":
                dropped_green += 1
            elif b.label == "black":
                dropped_black += 1
            elif b.label == "orange":
                dropped_orange += 1

        # Match competition cargo rules strictly.
        if not first_green_unlocked:
            return dropped_green == 1 and dropped_black == 0 and dropped_orange == 0
        if carrying_casualty:
            return dropped_orange == 1 and dropped_green == 0 and dropped_black == 0
        mat_total = dropped_green + dropped_black
        limit = max(1, min(3, max_materials_per_trip))
        return 1 <= mat_total <= limit and dropped_orange == 0

    def process(
        self,
        blobs: Iterable[Blob],
        camera_near_cmd: bool,
        camera_settled: bool,
        team: int,
        first_green_unlocked: bool,
        carrying_casualty: bool = False,
        max_materials_per_trip: int = 1,
        motion_stalled: bool = False,
        rear_clear: bool = False,
    ) -> VisionState:
        self._frame_seq = (self._frame_seq + 1) & 0xFFFF
        view = CAMERA_VIEW_TRANSITION
        if camera_settled:
            view = CAMERA_VIEW_NEAR if camera_near_cmd else CAMERA_VIEW_FAR

        all_blobs = list(blobs)

        # Fail-closed hazard and unknown handling:
        # - Cyan hazards have NO upper pixel cap so close-up/large cyan blocks are never dropped.
        # - Rejected green/black/orange blobs (oversized merged clusters or ambiguous shapes)
        #   are preserved as unknown obstacles rather than silently discarded.
        hazards: List[Blob] = []
        blocks: List[Blob] = []
        unknowns: List[Blob] = []
        for b in all_blobs:
            if b.label == "cyan":
                if b.pixels >= self.cfg.min_hazard_pixels:
                    hazards.append(b)
            elif b.label in ("green", "black", "orange"):
                if self._is_valid_target_block(b):
                    blocks.append(b)
                elif b.pixels >= self.cfg.min_hazard_pixels:
                    unknowns.append(b)
            elif b.label == "unknown":
                if b.pixels >= self.cfg.min_hazard_pixels:
                    unknowns.append(b)

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
        if motion_stalled:
            flags |= VIS_FLAG_MOTION_STALLED

        state = VisionState(frame_seq=self._frame_seq, camera_view=view, confidence=0)

        home, assoc_ramp, half_match, opp_zone_near, home_err, home_dist = self._evaluate_home_zone(
            zones, ramps, team, carrying_casualty
        )
        if opp_zone_near:
            flags |= VIS_FLAG_OPPONENT_ZONE_NEAR
        if home is not None:
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
            elif home is not None:
                state.confidence = 85
            else:
                state.confidence = 70

        elif view == CAMERA_VIEW_NEAR:
            corral_roi = self.cfg.near_corral_roi
            sweep_roi = self.cfg.near_sweep_roi

            green_cnt = sum(1 for b in blocks if b.label == "green" and _blob_in_roi(b, corral_roi))
            black_cnt = sum(1 for b in blocks if b.label == "black" and _blob_in_roi(b, corral_roi))
            orange_cnt = sum(1 for b in blocks if b.label == "orange" and _blob_in_roi(b, corral_roi))
            cyan_cnt = sum(1 for b in hazards if _blob_in_roi(b, corral_roi))
            unknown_cnt = sum(1 for b in unknowns if _blob_in_roi(b, corral_roi))

            state.corral_green_cnt = min(255, green_cnt)
            state.corral_black_cnt = min(255, black_cnt)
            state.corral_orange_cnt = min(255, orange_cnt)
            state.corral_cyan_cnt = min(255, cyan_cnt)
            state.corral_unknown_cnt = min(255, unknown_cnt)

            if cyan_cnt == 0 and unknown_cnt == 0:
                flags |= VIS_FLAG_CORRAL_SAFE

            # Sweep zone must be clear of ANY block, cyan hazard, or unknown foreground.
            sweep_occupied = any(_blob_in_roi(b, sweep_roi) for b in blocks + hazards + unknowns)
            if not sweep_occupied:
                flags |= VIS_FLAG_SWEEP_CLEAR

            corral_empty = (
                green_cnt == 0 and black_cnt == 0 and orange_cnt == 0 and cyan_cnt == 0 and unknown_cnt == 0
            )
            if self._verify_drop_inside_correct_half(
                home=home,
                assoc_ramp=assoc_ramp,
                blocks=blocks,
                hazards=hazards,
                unknowns=unknowns,
                corral_empty=corral_empty,
                team=team,
                first_green_unlocked=first_green_unlocked,
                carrying_casualty=carrying_casualty,
                max_materials_per_trip=max_materials_per_trip,
            ):
                flags |= VIS_FLAG_DROP_CONFIRMED

            state.confidence = 92

        state.flags = flags
        return state


def extract_blobs_from_maix_image(img, cfg: RescueVisionConfig, scan_unknown_near: bool = False) -> List[Blob]:
    """Run LAB blob detection on a MaixPy image object, plus fail-closed unknown foreground scan."""
    blobs: List[Blob] = []
    for label, thresh in cfg.lab_thresholds.items():
        min_area = cfg.min_zone_pixels if label.endswith("_zone") else cfg.min_block_pixels
        if label == "cyan":
            min_area = cfg.min_hazard_pixels
        elif label == "purple_ramp":
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

    # In NEAR view, detect non-floor foreground inside corral/sweep ROIs not covered by known blocks.
    if scan_unknown_near:
        known = list(blobs)
        raw_fg: List[Blob] = []
        for roi in (cfg.near_corral_roi, cfg.near_sweep_roi):
            fg_blobs = img.find_blobs(
                [tuple(cfg.arena_floor_lab)],
                roi=tuple(roi),
                pixels_threshold=cfg.min_hazard_pixels,
                area_threshold=cfg.min_hazard_pixels,
                invert=True,
                merge=True,
            )
            for item in fg_blobs:
                fx, fy, fw, fh = int(item.x()), int(item.y()), max(1, int(item.w())), max(1, int(item.h()))
                px = int(item.pixels())
                raw_fg.append(Blob("unknown", fx, fy, fw, fh, px, px / float(fw * fh)))
        blobs.extend(classify_unknown_foreground(raw_fg, known, cfg.min_hazard_pixels))

    return blobs
