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
    MISSION_DROP_BACKOUT_VERIFY,
    MISSION_DROP_OPEN_AND_PUSH,
    TARGET_BLACK_CORE,
    TARGET_CYAN_HAZARD,
    TARGET_GREEN_MATERIAL,
    TARGET_NONE,
    TARGET_ORANGE_CASUALTY,
    TEAM_BLUE,
    TEAM_RED,
    VIS_FLAG_CORRAL_SAFE,
    VIS_FLAG_DROP_BASELINE_READY,
    VIS_FLAG_DROP_CONFIRMED,
    VIS_FLAG_DROP_PATH_CLEAR,
    VIS_FLAG_GATE_OPEN_SETTLED_ACK,
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


_SURFACE_LABELS = ("purple_ramp", "red_zone", "blue_zone")
_OBJECT_LABELS = ("green", "black", "orange", "cyan")


@dataclass(frozen=True)
class DropBaseline:
    team: int
    first_green_unlocked: bool
    carrying_casualty: bool
    expected_cargo: Tuple[int, int, int]
    target_half_counts: Tuple[int, int, int]
    other_half_counts: Tuple[int, int, int]
    misplaced_counts: Tuple[int, int, int]


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


def _rect_intersection(
    ax: int, ay: int, aw: int, ah: int, bx: int, by: int, bw: int, bh: int
) -> Optional[Tuple[int, int, int, int]]:
    ix1 = max(ax, bx)
    iy1 = max(ay, by)
    ix2 = min(ax + aw, bx + bw)
    iy2 = min(ay + ah, by + bh)
    if ix2 <= ix1 or iy2 <= iy1:
        return None
    return (ix1, iy1, ix2, iy2)


def _rect_union_area(rects: Sequence[Tuple[int, int, int, int]]) -> int:
    """Exact area of the union of axis-aligned rectangles (x1, y1, x2, y2)."""
    if not rects:
        return 0
    xs = sorted({x for r in rects for x in (r[0], r[2])})
    total = 0
    for i in range(len(xs) - 1):
        x1, x2 = xs[i], xs[i + 1]
        if x2 <= x1:
            continue
        intervals = sorted((r[1], r[3]) for r in rects if r[0] < x2 and r[2] > x1)
        if not intervals:
            continue
        cur_y1, cur_y2 = intervals[0]
        y_len = 0
        for y1, y2 in intervals[1:]:
            if y1 <= cur_y2:
                if y2 > cur_y2:
                    cur_y2 = y2
            else:
                y_len += cur_y2 - cur_y1
                cur_y1, cur_y2 = y1, y2
        y_len += cur_y2 - cur_y1
        total += (x2 - x1) * y_len
    return total


def _lab_ranges_overlap(a: Sequence[int], b: Sequence[int]) -> bool:
    return (
        a[0] <= b[1]
        and b[0] <= a[1]
        and a[2] <= b[3]
        and b[2] <= a[3]
        and a[4] <= b[5]
        and b[4] <= a[5]
    )


def _is_verified_object_blob(blob: Blob, min_pixels: int, cfg: Optional[RescueVisionConfig] = None) -> bool:
    if blob.label == "cyan":
        req = cfg.min_hazard_pixels if cfg is not None else min_pixels
        return blob.pixels >= req
    if blob.label not in ("green", "black", "orange"):
        return False
    min_px = cfg.min_block_pixels if cfg is not None else min_pixels
    max_px = cfg.max_block_pixels if cfg is not None else 9500
    if blob.pixels < min_px or blob.pixels > max_px:
        return False
    if blob.label == "black":
        min_density = cfg.min_black_density if cfg is not None else 0.42
        aspect = max(blob.w, blob.h) / max(1, min(blob.w, blob.h))
        if blob.density < min_density or aspect > 2.2:
            return False
    return True


def _group_unique_pixels(items: Sequence[Tuple[Tuple[int, int, int, int], float, int]]) -> int:
    """Upper bound on unique pixels for possibly-overlapping blobs (no double counting)."""
    if not items:
        return 0
    xs = sorted({x for r, _, _ in items for x in (r[0], r[2])})
    ys = sorted({y for r, _, _ in items for y in (r[1], r[3])})
    cell_sum = 0.0
    for i in range(len(xs) - 1):
        x1, x2 = xs[i], xs[i + 1]
        if x2 <= x1:
            continue
        dx = x2 - x1
        for j in range(len(ys) - 1):
            y1, y2 = ys[j], ys[j + 1]
            if y2 <= y1:
                continue
            max_d = 0.0
            for r, d, _ in items:
                if r[0] <= x1 and r[2] >= x2 and r[1] <= y1 and r[3] >= y2:
                    if d > max_d:
                        max_d = d
            if max_d > 0.0:
                cell_sum += dx * (y2 - y1) * max_d
    cap_sum = sum(px for _, _, px in items)
    return min(int(round(cell_sum)), cap_sum)


def _unique_explained_pixels(
    fg: Blob,
    explaining_blobs: Sequence[Blob],
    cfg: Optional[RescueVisionConfig] = None,
) -> Tuple[int, int]:
    """Return (unique_explained_pixels, unique_union_area) clipped to fg without double counting."""
    clipped: List[Tuple[str, Tuple[int, int, int, int], float, int]] = []
    for eb in explaining_blobs:
        inter = _rect_intersection(fg.x, fg.y, fg.w, fg.h, eb.x, eb.y, eb.w, eb.h)
        if inter is None:
            continue
        ia = (inter[2] - inter[0]) * (inter[3] - inter[1])
        eff_d = min(1.0, max(0.0, eb.pixels / float(max(1, eb.w * eb.h))))
        est_px = min(eb.pixels, ia, int(round(ia * eff_d)))
        if est_px <= 0:
            continue
        clipped.append((eb.label, inter, eff_d, est_px))

    if not clipped:
        return 0, 0

    union_area = _rect_union_area([r for _, r, _, _ in clipped])

    if cfg is None:
        # Without LAB definitions, conservatively deduplicate across ALL overlapping blobs.
        unique_px = _group_unique_pixels([(r, d, px) for _, r, d, px in clipped])
        return min(unique_px, union_area), union_area

    # Group labels whose LAB boxes overlap (including same label) so shared pixels are never double-counted,
    # while genuinely disjoint LAB classes (e.g. green block sitting on purple ramp) can each claim their own pixels.
    n = len(clipped)
    visited = [False] * n
    total_px = 0
    for i in range(n):
        if visited[i]:
            continue
        stack = [i]
        visited[i] = True
        group_items: List[Tuple[Tuple[int, int, int, int], float, int]] = []
        while stack:
            idx = stack.pop()
            lbl_a, rect_a, d_a, px_a = clipped[idx]
            group_items.append((rect_a, d_a, px_a))
            lab_a = cfg.lab_thresholds.get(lbl_a)
            for j in range(n):
                if visited[j]:
                    continue
                lbl_b = clipped[j][0]
                lab_b = cfg.lab_thresholds.get(lbl_b)
                if lbl_a == lbl_b or lab_a is None or lab_b is None or _lab_ranges_overlap(lab_a, lab_b):
                    visited[j] = True
                    stack.append(j)
        total_px += _group_unique_pixels(group_items)

    return min(total_px, union_area), union_area


def classify_unknown_foreground(
    fg_blobs: Iterable[Blob],
    known_blobs: Iterable[Blob],
    min_pixels: int,
    coverage_threshold: float = 0.70,
    cfg: Optional[RescueVisionConfig] = None,
    surface_blobs: Iterable[Blob] = (),
) -> List[Blob]:
    """Flag foreground blobs not accounted for by unique verified pixels.

    - Overlapping detections are deduplicated via exact 2D union/cell coverage so two boxes
      covering the same 40% never sum to 80%.
    - Only verified block/hazard blobs from `known_blobs` count (full-frame zones/ramps in
      `known_blobs` never mask unknown objects).
    - ROI-measured calibrated surface blobs (`purple_ramp`, `red_zone`, `blue_zone`) may be
      supplied separately via `surface_blobs` so a calibrated ramp alone does not trigger a
      false obstruction while an unknown object sitting on top of it still leaves unexplained pixels.
    """
    verified_objects = [
        b for b in known_blobs if b.label in _OBJECT_LABELS and _is_verified_object_blob(b, min_pixels, cfg)
    ]
    verified_surfaces = [b for b in surface_blobs if b.label in _SURFACE_LABELS and b.pixels > 0]
    explaining = verified_objects + verified_surfaces

    unknowns: List[Blob] = []
    for fg in fg_blobs:
        if fg.pixels < min_pixels:
            continue
        fg_area = max(1, fg.w * fg.h)
        explained_px, union_area = _unique_explained_pixels(fg, explaining, cfg=cfg)
        unexplained_px = max(0, fg.pixels - explained_px)
        pixel_coverage = explained_px / float(max(1, fg.pixels))
        area_coverage = union_area / float(fg_area)
        if (
            unexplained_px >= min_pixels
            or pixel_coverage < coverage_threshold
            or (area_coverage < coverage_threshold and explained_px < fg.pixels)
        ):
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
        self._drop_baseline: Optional[DropBaseline] = None

    def reset_drop_baseline(self) -> None:
        self._drop_baseline = None

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
        return blob.label in ("green", "black", "orange") and _is_verified_object_blob(
            blob, self.cfg.min_block_pixels, self.cfg
        )

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

    def _zone_half_bounds(
        self, home: Blob, team: int, carrying_casualty: bool
    ) -> Tuple[Tuple[float, float], Tuple[float, float], float]:
        """Return ((target_x1, target_x2), (other_x1, other_x2), aim_x) past the 20 mm divider."""
        material_left = (
            self.cfg.material_half_on_left_red if team == TEAM_RED else self.cfg.material_half_on_left_blue
        )
        want_left = (not carrying_casualty) if material_left else carrying_casualty
        divider_x = home.cx
        margin = float(self.cfg.half_match_margin_px)
        left_bounds = (float(home.x), divider_x - margin)
        right_bounds = (divider_x + margin, float(home.x2))
        if want_left:
            return left_bounds, right_bounds, home.x + 0.25 * home.w
        return right_bounds, left_bounds, home.x + 0.75 * home.w

    def _target_half_bounds(self, home: Blob, team: int, carrying_casualty: bool) -> Tuple[float, float, float]:
        """Return (half_x1, half_x2, aim_x) strictly inside own target half past the 20 mm divider."""
        target_bounds, _, aim_x = self._zone_half_bounds(home, team, carrying_casualty)
        return target_bounds[0], target_bounds[1], aim_x

    def _is_valid_trip_cargo(
        self,
        cargo: Tuple[int, int, int],
        first_green_unlocked: bool,
        carrying_casualty: bool,
        max_materials_per_trip: int,
    ) -> bool:
        g, b, o = cargo
        if not first_green_unlocked:
            return g == 1 and b == 0 and o == 0
        if carrying_casualty:
            return o == 1 and g == 0 and b == 0
        limit = max(1, min(3, max_materials_per_trip))
        return 1 <= (g + b) <= limit and o == 0

    def _partition_home_blocks(
        self,
        home: Blob,
        assoc_ramp: Blob,
        blocks: List[Blob],
        target_bounds: Tuple[float, float],
        other_bounds: Tuple[float, float],
    ) -> Tuple[Tuple[int, int, int], Tuple[int, int, int], Tuple[int, int, int]]:
        """Partition blocks in/on home or ramp into (target_half, other_half, misplaced) counts."""
        ramp_inner_y = assoc_ramp.y
        ramp_bot_y = assoc_ramp.y2
        tgt = [0, 0, 0]
        oth = [0, 0, 0]
        mis = [0, 0, 0]
        idx_map = {"green": 0, "black": 1, "orange": 2}
        for b in blocks:
            idx = idx_map.get(b.label)
            if idx is None:
                continue
            in_home = _rects_intersect(b.x, b.y, b.w, b.h, home.x, home.y, home.w, home.h)
            on_ramp = b.y < ramp_bot_y and b.y2 > ramp_inner_y and b.x < home.x2 and b.x2 > home.x
            if not (in_home or on_ramp):
                continue
            if not on_ramp and b.y >= home.y and b.y2 <= ramp_inner_y:
                if target_bounds[1] > target_bounds[0] and b.x >= target_bounds[0] and b.x2 <= target_bounds[1]:
                    tgt[idx] += 1
                    continue
                if other_bounds[1] > other_bounds[0] and b.x >= other_bounds[0] and b.x2 <= other_bounds[1]:
                    oth[idx] += 1
                    continue
            mis[idx] += 1
        return (tgt[0], tgt[1], tgt[2]), (oth[0], oth[1], oth[2]), (mis[0], mis[1], mis[2])

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

    def _is_drop_path_hazards_clear(
        self,
        home: Optional[Blob],
        assoc_ramp: Optional[Blob],
        hazards: List[Blob],
        unknowns: List[Blob],
    ) -> bool:
        if home is None or assoc_ramp is None:
            return False
        corral_roi = self.cfg.near_corral_roi
        drop_roi = self.cfg.near_drop_roi
        top_y = min(home.y, corral_roi[1])
        bot_y = max(home.y2, corral_roi[1] + corral_roi[3])
        corridor_roi = [corral_roi[0], top_y, corral_roi[2], max(1, bot_y - top_y)]
        ramp_inner_y = assoc_ramp.y
        ramp_bot_y = assoc_ramp.y2
        for bad in hazards + unknowns:
            in_home = _rects_intersect(bad.x, bad.y, bad.w, bad.h, home.x, home.y, home.w, home.h)
            on_ramp = bad.y < ramp_bot_y and bad.y2 > ramp_inner_y
            if in_home or on_ramp or _blob_in_roi(bad, drop_roi) or _blob_in_roi(bad, corridor_roi):
                return False
        return True

    def _update_drop_baseline(
        self,
        home: Optional[Blob],
        assoc_ramp: Optional[Blob],
        blocks: List[Blob],
        corral_cargo: Tuple[int, int, int],
        corral_safe: bool,
        drop_path_hazards_clear: bool,
        team: int,
        first_green_unlocked: bool,
        carrying_casualty: bool,
        max_materials_per_trip: int,
        mission_state: Optional[int],
        gate_closed: Optional[bool],
        gate_settled: Optional[bool],
        drop_pushing: bool,
    ) -> Tuple[bool, Tuple[int, int, int]]:
        # Retain frozen baseline for delivery verification during push/backout, but NEVER report
        # pre-push baseline authorization as ready once pushing or outside MISSION_DROP_OPEN_AND_PUSH!
        if drop_pushing or (mission_state is not None and mission_state != MISSION_DROP_OPEN_AND_PUSH):
            return False, (0, 0, 0)
        if not corral_safe or not drop_path_hazards_clear or home is None or assoc_ramp is None:
            if gate_closed is True:
                self._drop_baseline = None
            return False, (0, 0, 0)

        target_bounds, other_bounds, _ = self._zone_half_bounds(home, team, carrying_casualty)
        if target_bounds[1] <= target_bounds[0]:
            return False, (0, 0, 0)
        tgt_cnt, oth_cnt, mis_cnt = self._partition_home_blocks(home, assoc_ramp, blocks, target_bounds, other_bounds)

        if not self._is_valid_trip_cargo(corral_cargo, first_green_unlocked, carrying_casualty, max_materials_per_trip):
            return False, mis_cnt

        # Reject any pre-push baseline containing preexisting misplaced blocks on ramp/divider
        # so a stray ramp block moving into the target half can never masquerade as trip cargo.
        if mis_cnt != (0, 0, 0):
            if gate_closed is True or self._drop_baseline is None:
                self._drop_baseline = None
            return False, mis_cnt

        if gate_closed is True:
            # Gate still closed before push: keep refreshing to the latest clean pre-open snapshot.
            self._drop_baseline = DropBaseline(
                team=team,
                first_green_unlocked=first_green_unlocked,
                carrying_casualty=carrying_casualty,
                expected_cargo=corral_cargo,
                target_half_counts=tgt_cnt,
                other_half_counts=oth_cnt,
                misplaced_counts=mis_cnt,
            )
            return True, mis_cnt

        if gate_closed is False:
            # Once the gate opens, the pre-push baseline is STRICTLY FROZEN and bound to the
            # MCU-authorized trip. Never capture a new baseline or rewrite an existing one here!
            if self._drop_baseline is None:
                return False, mis_cnt

            prev = self._drop_baseline
            exact_match = (
                corral_cargo == prev.expected_cargo
                and tgt_cnt == prev.target_half_counts
                and oth_cnt == prev.other_half_counts
                and mis_cnt == (0, 0, 0)
            )
            if exact_match:
                return True, mis_cnt
            # If extra cargo entered the corral before push, invalidate so forced verification fails closed.
            # If cargo decreased (e.g. low-speed push beginning), KEEP prev frozen — never downgrade expected_cargo!
            if sum(corral_cargo) > sum(prev.expected_cargo):
                self._drop_baseline = None
            return False, mis_cnt

        # Standalone mode (gate_closed is None): freeze upon initial capture; never downgrade expected_cargo.
        if self._drop_baseline is None:
            self._drop_baseline = DropBaseline(
                team=team,
                first_green_unlocked=first_green_unlocked,
                carrying_casualty=carrying_casualty,
                expected_cargo=corral_cargo,
                target_half_counts=tgt_cnt,
                other_half_counts=oth_cnt,
                misplaced_counts=mis_cnt,
            )
            return True, mis_cnt

        prev = self._drop_baseline
        if corral_cargo == prev.expected_cargo and tgt_cnt == prev.target_half_counts and oth_cnt == prev.other_half_counts:
            return True, mis_cnt
        if sum(corral_cargo) > sum(prev.expected_cargo):
            self._drop_baseline = None
        return False, mis_cnt

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
        mission_state: Optional[int] = None,
    ) -> bool:
        # Must have a clean pre-push baseline for THIS trip (with zero misplaced ramp blocks),
        # observe BOTH own team safe zone and its associated purple ramp, AND confirm corral is empty.
        base = self._drop_baseline
        if base is None or base.misplaced_counts != (0, 0, 0) or not corral_empty or home is None or assoc_ramp is None:
            return False
        if mission_state is not None and mission_state != MISSION_DROP_BACKOUT_VERIFY:
            return False
        if (
            base.team != team
            or base.first_green_unlocked != first_green_unlocked
            or base.carrying_casualty != carrying_casualty
        ):
            return False
        if not self._is_valid_trip_cargo(
            base.expected_cargo, first_green_unlocked, carrying_casualty, max_materials_per_trip
        ):
            return False

        target_bounds, other_bounds, _ = self._zone_half_bounds(home, team, carrying_casualty)
        if target_bounds[1] <= target_bounds[0]:
            return False

        # Fail closed if ANY cyan hazard or unknown object is inside home, push path, drop ROI, or ramp band.
        if not self._is_drop_path_hazards_clear(home, assoc_ramp, hazards, unknowns):
            return False

        tgt_now, oth_now, mis_now = self._partition_home_blocks(home, assoc_ramp, blocks, target_bounds, other_bounds)

        # Zero tolerance for blocks straddling the purple ramp or 20 mm center divider.
        if mis_now != (0, 0, 0):
            return False

        # Verify post-push delta in the target half matches the current trip's exact cargo count & classes.
        delta_target = (
            tgt_now[0] - base.target_half_counts[0],
            tgt_now[1] - base.target_half_counts[1],
            tgt_now[2] - base.target_half_counts[2],
        )
        if delta_target != base.expected_cargo:
            return False

        # Opposite half may retain prior valid deliveries (e.g. first green during casualty drop),
        # but must NOT gain any new block or lose a same-class block into the target half.
        for i in range(3):
            if oth_now[i] > base.other_half_counts[i]:
                return False
            if base.expected_cargo[i] > 0 and oth_now[i] != base.other_half_counts[i]:
                return False

        return True

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
        mission_state: Optional[int] = None,
        gate_closed: Optional[bool] = None,
        gate_settled: Optional[bool] = None,
        drop_pushing: bool = False,
        wheels_moving: bool = False,
    ) -> VisionState:
        self._frame_seq = (self._frame_seq + 1) & 0xFFFF
        view = CAMERA_VIEW_TRANSITION
        if camera_settled:
            view = CAMERA_VIEW_NEAR if camera_near_cmd else CAMERA_VIEW_FAR

        if view == CAMERA_VIEW_FAR or (
            mission_state is not None
            and mission_state not in (MISSION_DROP_OPEN_AND_PUSH, MISSION_DROP_BACKOUT_VERIFY)
        ):
            self._drop_baseline = None
        elif self._drop_baseline is not None and (
            self._drop_baseline.team != team
            or self._drop_baseline.first_green_unlocked != first_green_unlocked
            or self._drop_baseline.carrying_casualty != carrying_casualty
        ):
            self._drop_baseline = None

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

            corral_safe = cyan_cnt == 0 and unknown_cnt == 0
            if corral_safe:
                flags |= VIS_FLAG_CORRAL_SAFE

            # Sweep zone must be clear of ANY block, cyan hazard, or unknown foreground.
            sweep_occupied = any(_blob_in_roi(b, sweep_roi) for b in blocks + hazards + unknowns)
            if not sweep_occupied:
                flags |= VIS_FLAG_SWEEP_CLEAR

            _ = wheels_moving  # retained for backwards-compatible signature only
            drop_path_hazards_clear = self._is_drop_path_hazards_clear(home, assoc_ramp, hazards, unknowns)

            baseline_ok, mis_cnt = self._update_drop_baseline(
                home=home,
                assoc_ramp=assoc_ramp,
                blocks=blocks,
                corral_cargo=(green_cnt, black_cnt, orange_cnt),
                corral_safe=corral_safe,
                drop_path_hazards_clear=drop_path_hazards_clear,
                team=team,
                first_green_unlocked=first_green_unlocked,
                carrying_casualty=carrying_casualty,
                max_materials_per_trip=max_materials_per_trip,
                mission_state=mission_state,
                gate_closed=gate_closed,
                gate_settled=gate_settled,
                drop_pushing=drop_pushing,
            )

            corral_empty = (
                green_cnt == 0 and black_cnt == 0 and orange_cnt == 0 and cyan_cnt == 0 and unknown_cnt == 0
            )
            # Separate continuous push-path hazard clearance (VIS_FLAG_DROP_PATH_CLEAR) from pre-push
            # baseline authorization (VIS_FLAG_DROP_BASELINE_READY / VIS_FLAG_GATE_OPEN_SETTLED_ACK):
            # - Delayed TEL_FLAG_DROP_PUSHING telemetry can NEVER abort a clean push as own cargo exits
            #   the corral onto the ramp.
            # - Any cyan hazard or unknown obstruction STILL clears VIS_FLAG_DROP_PATH_CLEAR immediately.
            if drop_path_hazards_clear:
                flags |= VIS_FLAG_DROP_PATH_CLEAR

            in_pre_push_phase = (mission_state is None or mission_state == MISSION_DROP_OPEN_AND_PUSH) and not drop_pushing
            baseline_ready = (
                in_pre_push_phase
                and baseline_ok
                and corral_safe
                and drop_path_hazards_clear
                and mis_cnt == (0, 0, 0)
            )
            if baseline_ready:
                flags |= VIS_FLAG_DROP_BASELINE_READY

            if gate_closed is False and gate_settled is True and baseline_ready:
                flags |= VIS_FLAG_GATE_OPEN_SETTLED_ACK

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
                mission_state=mission_state,
            ):
                flags |= VIS_FLAG_DROP_CONFIRMED

            state.confidence = 92

        state.flags = flags
        return state


def _lab_in_thresh(px: Sequence[int], thresh: Sequence[int]) -> bool:
    return (
        thresh[0] <= px[0] <= thresh[1]
        and thresh[2] <= px[1] <= thresh[3]
        and thresh[4] <= px[2] <= thresh[5]
    )


def _get_lab_pixel_accessor(img):
    if hasattr(img, "get_lab_pixel"):
        return img.get_lab_pixel
    if hasattr(img, "_grid"):
        grid = img._grid
        return lambda x, y: grid[y][x]
    return None


def _find_blobs_unit_stride(img, thresholds, **kwargs):
    """Call MaixPy/OpenMV find_blobs with explicit x_stride=1, y_stride=1 so native default x_stride=2 never skips thin/diagonal blobs."""
    try:
        return img.find_blobs(thresholds, x_stride=1, y_stride=1, **kwargs)
    except TypeError:
        return img.find_blobs(thresholds, **kwargs)


def _extract_roi_blobs_for_label(img, label: str, thresh: Sequence[int], roi: Tuple[int, int, int, int]) -> List[Blob]:
    found = _find_blobs_unit_stride(img, [tuple(thresh)], roi=roi, pixels_threshold=1, area_threshold=1, merge=False)
    out: List[Blob] = []
    for item in found:
        w = max(1, int(item.w()))
        h = max(1, int(item.h()))
        px = int(item.pixels())
        out.append(Blob(label=label, x=int(item.x()), y=int(item.y()), w=w, h=h, pixels=px, density=px / float(w * h)))
    return out


def _filter_opponent_excluding_floor(
    img,
    item,
    opp_thresh: Sequence[int],
    cfg: RescueVisionConfig,
) -> Optional[Blob]:
    bx, by, bw, bh = int(item.x()), int(item.y()), max(1, int(item.w())), max(1, int(item.h()))
    px = int(item.pixels())
    floor_thresh = cfg.arena_floor_lab

    get_lab = _get_lab_pixel_accessor(img)
    if get_lab is not None:
        min_x, min_y = bx + bw, by + bh
        max_x, max_y = bx - 1, by - 1
        non_floor_opp = 0
        for yy in range(by, by + bh):
            for xx in range(bx, bx + bw):
                lab = get_lab(xx, yy)
                if _lab_in_thresh(lab, opp_thresh) and not _lab_in_thresh(lab, floor_thresh):
                    non_floor_opp += 1
                    if xx < min_x:
                        min_x = xx
                    if xx > max_x:
                        max_x = xx
                    if yy < min_y:
                        min_y = yy
                    if yy > max_y:
                        max_y = yy
        if non_floor_opp < cfg.min_opponent_pixels:
            return None
        ow = max_x - min_x + 1
        oh = max_y - min_y + 1
        return Blob("opponent", min_x, min_y, ow, oh, non_floor_opp, non_floor_opp / float(ow * oh))

    # Tier 2: Real MaixPy Image single-crop unique (opponent \ floor) count + exact binary-search bounds!
    # Evaluating floor_mask and both_mask ONCE on the candidate crop guarantees every pixel is counted
    # at most once (even when an opponent lies inside an unrelated non-floor ring) and derives exact
    # tight bounds strictly from (opponent \ floor).
    if hasattr(img, "crop") and hasattr(img, "binary"):
        white = [(50, 100, -128, 127, -128, 127)]
        sub = img.crop(bx, by, bw, bh)
        floor_mask = sub.binary([list(floor_thresh)], invert=False, copy=True)
        both_mask = sub.binary([list(floor_thresh), list(opp_thresh)], invert=False, copy=True)

        def _opp_minus_floor_px(rx: int, ry: int, rw: int, rh: int) -> int:
            if rw <= 0 or rh <= 0:
                return 0
            r = (rx, ry, rw, rh)
            f_px = sum(int(b.pixels()) for b in _find_blobs_unit_stride(floor_mask, white, roi=r, pixels_threshold=1, area_threshold=1, merge=True))
            b_px = sum(int(b.pixels()) for b in _find_blobs_unit_stride(both_mask, white, roi=r, pixels_threshold=1, area_threshold=1, merge=True))
            return max(0, b_px - f_px)

        floor_px = sum(int(b.pixels()) for b in _find_blobs_unit_stride(floor_mask, white, pixels_threshold=1, area_threshold=1, merge=True))
        both_px = sum(int(b.pixels()) for b in _find_blobs_unit_stride(both_mask, white, pixels_threshold=1, area_threshold=1, merge=True))
        total_opp_px = max(0, both_px - floor_px)
        if total_opp_px < cfg.min_opponent_pixels:
            return None
        if floor_px == 0 and both_px == bw * bh:
            ox1, oy1, ox2, oy2 = bx, by, bx + bw, by + bh
        else:
            lo, hi = 0, bh - 1
            while lo < hi:
                mid = (lo + hi) // 2
                if _opp_minus_floor_px(0, 0, bw, mid + 1) > 0:
                    hi = mid
                else:
                    lo = mid + 1
            ty1 = lo
            lo, hi = ty1 + 1, bh
            while lo < hi:
                mid = (lo + hi) // 2
                if _opp_minus_floor_px(0, ty1, bw, mid - ty1) >= total_opp_px:
                    hi = mid
                else:
                    lo = mid + 1
            ty2 = lo
            th = max(1, ty2 - ty1)
            lo, hi = 0, bw - 1
            while lo < hi:
                mid = (lo + hi) // 2
                if _opp_minus_floor_px(0, ty1, mid + 1, th) > 0:
                    hi = mid
                else:
                    lo = mid + 1
            tx1 = lo
            lo, hi = tx1 + 1, bw
            while lo < hi:
                mid = (lo + hi) // 2
                if _opp_minus_floor_px(tx1, ty1, mid - tx1, th) >= total_opp_px:
                    hi = mid
                else:
                    lo = mid + 1
            tx2 = lo
            ox1, oy1, ox2, oy2 = bx + tx1, by + ty1, bx + tx2, by + ty2
        ow = max(1, ox2 - ox1)
        oh = max(1, oy2 - oy1)
        return Blob("opponent", ox1, oy1, ow, oh, total_opp_px, min(1.0, total_opp_px / float(ow * oh)))

    # Fallback using find_blobs: intersect opponent ROI with non-floor pixels so plain floor never triggers.
    non_floor = _find_blobs_unit_stride(
        img,
        [tuple(floor_thresh)],
        roi=(bx, by, bw, bh),
        pixels_threshold=1,
        area_threshold=1,
        invert=True,
        merge=True,
    )
    if not non_floor:
        return None
    non_floor_px = sum(int(nf.pixels()) for nf in non_floor)
    kept_px = min(px, non_floor_px)
    if kept_px < cfg.min_opponent_pixels:
        return None
    nx1 = min(int(nf.x()) for nf in non_floor)
    ny1 = min(int(nf.y()) for nf in non_floor)
    nx2 = max(int(nf.x()) + max(1, int(nf.w())) for nf in non_floor)
    ny2 = max(int(nf.y()) + max(1, int(nf.h())) for nf in non_floor)
    ow = max(1, nx2 - nx1)
    oh = max(1, ny2 - ny1)
    return Blob("opponent", nx1, ny1, ow, oh, kept_px, min(1.0, kept_px / float(ow * oh)))


def _min_area_for_label(label: str, cfg: RescueVisionConfig) -> int:
    if label.endswith("_zone"):
        return cfg.min_zone_pixels
    if label == "cyan":
        return cfg.min_hazard_pixels
    if label == "purple_ramp":
        return cfg.min_ramp_pixels
    if label == "opponent":
        return cfg.min_opponent_pixels
    return cfg.min_block_pixels


def _is_verified_blob_for_label(b: Blob, cfg: RescueVisionConfig) -> bool:
    if b.label in _SURFACE_LABELS:
        return b.pixels >= _min_area_for_label(b.label, cfg)
    if b.label in _OBJECT_LABELS:
        return _is_verified_object_blob(b, cfg.min_hazard_pixels, cfg)
    return False


def _merge_adjacent_unknowns(cands: Sequence[Blob]) -> List[Blob]:
    """Merge touching/1-px-adjacent unexplained blobs without merging distant regions."""
    clusters = [[b.x, b.y, b.x2, b.y2, b.pixels] for b in cands if b.pixels > 0]
    changed = True
    while changed and len(clusters) > 1:
        changed = False
        next_clusters: List[List[int]] = []
        while clusters:
            ax1, ay1, ax2, ay2, apx = clusters.pop(0)
            i = 0
            while i < len(clusters):
                bx1, by1, bx2, by2, bpx = clusters[i]
                # 1-pixel adjacency: touching edges merge; separated regions stay strictly separate.
                if ax1 <= bx2 and ax2 >= bx1 and ay1 <= by2 and ay2 >= by1:
                    nx1 = min(ax1, bx1)
                    ny1 = min(ay1, by1)
                    nx2 = max(ax2, bx2)
                    ny2 = max(ay2, by2)
                    area = max(1, (nx2 - nx1) * (ny2 - ny1))
                    ax1, ay1, ax2, ay2, apx = nx1, ny1, nx2, ny2, min(area, apx + bpx)
                    clusters.pop(i)
                    changed = True
                else:
                    i += 1
            next_clusters.append([ax1, ay1, ax2, ay2, apx])
        clusters = next_clusters
    out: List[Blob] = []
    for x1, y1, x2, y2, px in clusters:
        w = max(1, x2 - x1)
        h = max(1, y2 - y1)
        out.append(Blob("unknown", x1, y1, w, h, px, min(1.0, px / float(w * h))))
    return out


def extract_blobs_from_maix_image(img, cfg: RescueVisionConfig, scan_unknown_near: bool = False) -> List[Blob]:
    """Run LAB blob detection on a MaixPy image object, plus component-exact unknown foreground scan."""
    blobs: List[Blob] = []
    verified_by_label: dict[str, List[Blob]] = {lbl: [] for lbl in (_SURFACE_LABELS + _OBJECT_LABELS)}
    unverified_by_label: dict[str, List[Blob]] = {lbl: [] for lbl in (_SURFACE_LABELS + _OBJECT_LABELS)}

    # Extract surfaces first so custom overlapping LAB bands (e.g. red_zone vs orange) can resolve
    # exact-component surface/object ambiguity without ever suppressing genuine cyan hazards.
    ordered_labels = [lbl for lbl in (_SURFACE_LABELS + _OBJECT_LABELS) if lbl in cfg.lab_thresholds]
    for lbl in cfg.lab_thresholds:
        if lbl not in ordered_labels:
            ordered_labels.append(lbl)

    verified_surface_geoms = set()
    emitted_rejected_object_rects = set()

    for label in ordered_labels:
        thresh = cfg.lab_thresholds[label]
        min_area = _min_area_for_label(label, cfg)
        if label == "opponent":
            found = _find_blobs_unit_stride(img, [tuple(thresh)], pixels_threshold=min_area, area_threshold=min_area, merge=True)
            for item in found:
                filtered = _filter_opponent_excluding_floor(img, item, thresh, cfg)
                if filtered is not None:
                    blobs.append(filtered)
            continue

        # In NEAR mode, capture all connected components (merge=False, x_stride=1, y_stride=1) so detached
        # unverified patches inside concave verified components (e.g. a C-shaped ramp) are separated by component!
        px_thr = 1 if (scan_unknown_near and label in (_SURFACE_LABELS + _OBJECT_LABELS)) else min_area
        found = _find_blobs_unit_stride(img, [tuple(thresh)], pixels_threshold=px_thr, area_threshold=px_thr, merge=False)
        for item in found:
            w = max(1, int(item.w()))
            h = max(1, int(item.h()))
            px = int(item.pixels())
            b = Blob(label=label, x=int(item.x()), y=int(item.y()), w=w, h=h, pixels=px, density=px / float(w * h))

            # If a custom overlapping band classifies an entire verified surface identically as an
            # oversized non-cyan object, suppress the duplicate surface shadow (never suppress cyan!).
            if (
                label in ("green", "black", "orange")
                and px > cfg.max_block_pixels
                and (b.x, b.y, b.w, b.h, b.pixels) in verified_surface_geoms
            ):
                continue

            if label in _SURFACE_LABELS:
                if px >= min_area:
                    blobs.append(b)
                    verified_by_label[label].append(b)
                    verified_surface_geoms.add((b.x, b.y, b.w, b.h, b.pixels))
                elif scan_unknown_near:
                    unverified_by_label[label].append(b)
            elif label in _OBJECT_LABELS:
                if px >= min_area:
                    blobs.append(b)
                    if _is_verified_object_blob(b, cfg.min_hazard_pixels, cfg):
                        verified_by_label[label].append(b)
                    else:
                        emitted_rejected_object_rects.add((b.x, b.y, b.w, b.h))
                        if scan_unknown_near:
                            unverified_by_label[label].append(b)
                elif scan_unknown_near:
                    unverified_by_label[label].append(b)
            elif px >= min_area:
                blobs.append(b)

    # In NEAR view, detect unexplained non-floor foreground across ALL fixed near ROIs PLUS the dynamic
    # home zone, ramp band, and projected push-corridor geometry, masking ONLY verified connected
    # components (never full AABBs) and emitting tight unexplained bounds (never whole connected surfaces).
    if scan_unknown_near:
        verified_full_objects = [b for lbl in _OBJECT_LABELS for b in verified_by_label[lbl]]
        verified_full_surfaces = [b for lbl in _SURFACE_LABELS for b in verified_by_label[lbl]]
        get_lab = _get_lab_pixel_accessor(img)
        use_maix_binary = get_lab is None and hasattr(img, "crop") and hasattr(img, "binary")

        def _clip_roi(r: Sequence[int]) -> Optional[Tuple[int, int, int, int]]:
            x1 = max(0, int(r[0]))
            y1 = max(0, int(r[1]))
            x2 = min(cfg.frame_width, int(r[0]) + int(r[2]))
            y2 = min(cfg.frame_height, int(r[1]) + int(r[3]))
            if x2 <= x1 or y2 <= y1:
                return None
            return (x1, y1, x2 - x1, y2 - y1)

        raw_scan_rois: List[Tuple[Sequence[int], bool]] = [
            (cfg.near_corral_roi, True),
            (cfg.near_sweep_roi, True),
            (cfg.near_drop_roi, False),
        ]
        corral_roi = cfg.near_corral_roi
        for surf in verified_full_surfaces:
            raw_scan_rois.append(((surf.x, surf.y, surf.w, surf.h), False))
            if surf.label in ("red_zone", "blue_zone"):
                top_y = min(surf.y, corral_roi[1])
                bot_y = max(surf.y2, corral_roi[1] + corral_roi[3])
                raw_scan_rois.append(((corral_roi[0], top_y, corral_roi[2], max(1, bot_y - top_y)), False))
            elif surf.label == "purple_ramp":
                raw_scan_rois.append(((0, surf.y, cfg.frame_width, surf.h), False))

        scan_rois: List[Tuple[Tuple[int, int, int, int], bool]] = []
        seen_scan_rois = set()
        for cand, merge_fg in raw_scan_rois:
            clipped = _clip_roi(cand)
            key = (clipped, merge_fg)
            if clipped is not None and key not in seen_scan_rois:
                seen_scan_rois.add(key)
                scan_rois.append((clipped, merge_fg))

        emitted_unknown_rects = set(emitted_rejected_object_rects)

        # Tier 1: Exact per-pixel verified connected-component mask + tight unexplained connected components!
        if get_lab is not None:
            fw, fh = cfg.frame_width, cfg.frame_height
            explained = [[False] * fw for _ in range(fh)]
            for label in (_SURFACE_LABELS + _OBJECT_LABELS):
                vlist = verified_by_label[label]
                if not vlist or label not in cfg.lab_thresholds:
                    continue
                thresh = cfg.lab_thresholds[label]
                ux1 = max(0, min(b.x for b in vlist))
                uy1 = max(0, min(b.y for b in vlist))
                ux2 = min(fw, max(b.x2 for b in vlist))
                uy2 = min(fh, max(b.y2 for b in vlist))
                vis_c = [[False] * (ux2 - ux1) for _ in range(uy2 - uy1)]
                for yy in range(uy1, uy2):
                    for xx in range(ux1, ux2):
                        if vis_c[yy - uy1][xx - ux1] or not _lab_in_thresh(get_lab(xx, yy), thresh):
                            continue
                        stack = [(xx, yy)]
                        vis_c[yy - uy1][xx - ux1] = True
                        comp_pts: List[Tuple[int, int]] = []
                        min_x = max_x = xx
                        min_y = max_y = yy
                        while stack:
                            cx, cy = stack.pop()
                            comp_pts.append((cx, cy))
                            if cx < min_x:
                                min_x = cx
                            if cx > max_x:
                                max_x = cx
                            if cy < min_y:
                                min_y = cy
                            if cy > max_y:
                                max_y = cy
                            for nx, ny in ((cx - 1, cy), (cx + 1, cy), (cx, cy - 1), (cx, cy + 1)):
                                if ux1 <= nx < ux2 and uy1 <= ny < uy2 and not vis_c[ny - uy1][nx - ux1]:
                                    if _lab_in_thresh(get_lab(nx, ny), thresh):
                                        vis_c[ny - uy1][nx - ux1] = True
                                        stack.append((nx, ny))
                        cw = max_x - min_x + 1
                        ch = max_y - min_y + 1
                        cnt = len(comp_pts)
                        cand_blob = Blob(label, min_x, min_y, cw, ch, cnt, cnt / float(cw * ch))
                        if _is_verified_blob_for_label(cand_blob, cfg):
                            for px_x, px_y in comp_pts:
                                explained[px_y][px_x] = True

            for roi, merge_small in scan_rois:
                rx, ry, rw, rh = roi
                x1, y1, x2, y2 = rx, ry, rx + rw, ry + rh
                vis_u = [[False] * rw for _ in range(rh)]
                small_comps: List[Tuple[int, int, int, int, int]] = []
                for yy in range(y1, y2):
                    for xx in range(x1, x2):
                        if vis_u[yy - y1][xx - x1] or explained[yy][xx]:
                            continue
                        if _lab_in_thresh(get_lab(xx, yy), cfg.arena_floor_lab):
                            continue
                        stack = [(xx, yy)]
                        vis_u[yy - y1][xx - x1] = True
                        min_x = max_x = xx
                        min_y = max_y = yy
                        cnt = 0
                        while stack:
                            cx, cy = stack.pop()
                            cnt += 1
                            if cx < min_x:
                                min_x = cx
                            if cx > max_x:
                                max_x = cx
                            if cy < min_y:
                                min_y = cy
                            if cy > max_y:
                                max_y = cy
                            for nx, ny in (
                                (cx - 1, cy - 1), (cx, cy - 1), (cx + 1, cy - 1),
                                (cx - 1, cy), (cx + 1, cy),
                                (cx - 1, cy + 1), (cx, cy + 1), (cx + 1, cy + 1),
                            ):
                                if x1 <= nx < x2 and y1 <= ny < y2 and not vis_u[ny - y1][nx - x1]:
                                    if not explained[ny][nx] and not _lab_in_thresh(get_lab(nx, ny), cfg.arena_floor_lab):
                                        vis_u[ny - y1][nx - x1] = True
                                        stack.append((nx, ny))
                        cw = max_x - min_x + 1
                        ch = max_y - min_y + 1
                        if cnt >= cfg.min_hazard_pixels:
                            rect = (min_x, min_y, cw, ch)
                            if rect not in emitted_unknown_rects:
                                emitted_unknown_rects.add(rect)
                                blobs.append(Blob("unknown", min_x, min_y, cw, ch, cnt, cnt / float(cw * ch)))
                        elif merge_small:
                            small_comps.append((min_x, min_y, max_x + 1, max_y + 1, cnt))
                if merge_small and small_comps:
                    tot = sum(c[4] for c in small_comps)
                    if tot >= cfg.min_hazard_pixels:
                        mx1 = min(c[0] for c in small_comps)
                        my1 = min(c[1] for c in small_comps)
                        mx2 = max(c[2] for c in small_comps)
                        my2 = max(c[3] for c in small_comps)
                        mw, mh = max(1, mx2 - mx1), max(1, my2 - my1)
                        rect = (mx1, my1, mw, mh)
                        if rect not in emitted_unknown_rects:
                            emitted_unknown_rects.add(rect)
                            blobs.append(Blob("unknown", mx1, my1, mw, mh, tot, min(1.0, tot / float(mw * mh))))
            return blobs

        # Tier 2: Real MaixPy Image exact component-level unknown detection using ONLY crop/binary/find_blobs!
        # Set A: pixels matching NEITHER floor NOR any calibrated class (exact tight connected components).
        # Set B: connected components of calibrated classes that FAILED verification (e.g. 81-px detached
        #        purple patch inside a C-shaped ramp), checked against overlapping verified classes.
        if use_maix_binary:
            all_known_thresholds = [list(cfg.arena_floor_lab)] + [
                list(cfg.lab_thresholds[lbl])
                for lbl in (_SURFACE_LABELS + _OBJECT_LABELS)
                if lbl in cfg.lab_thresholds
            ]
            black = [(0, 40, -128, 127, -128, 127)]
            white = [(50, 100, -128, 127, -128, 127)]

            # Filter Set B against verified components of OTHER classes whose LAB bands overlap.
            unverified_set_b: List[Blob] = []
            for label in (_SURFACE_LABELS + _OBJECT_LABELS):
                if label not in cfg.lab_thresholds:
                    continue
                thresh = cfg.lab_thresholds[label]
                overlapping_other = [
                    other
                    for other in (_SURFACE_LABELS + _OBJECT_LABELS)
                    if other != label
                    and other in cfg.lab_thresholds
                    and verified_by_label[other]
                    and _lab_ranges_overlap(thresh, cfg.lab_thresholds[other])
                ]
                for c in unverified_by_label[label]:
                    covered_by_other = False
                    for other in overlapping_other:
                        oth_thresh = cfg.lab_thresholds[other]
                        if any(
                            _rects_intersect(c.x, c.y, c.w, c.h, u.x, u.y, u.w, u.h)
                            for u in unverified_by_label[other]
                        ):
                            continue
                        for v in verified_by_label[other]:
                            if not _rects_intersect(c.x, c.y, c.w, c.h, v.x, v.y, v.w, v.h):
                                continue
                            sub_c = img.crop(c.x, c.y, c.w, c.h)
                            oth_m = sub_c.binary([list(oth_thresh)], invert=False, copy=True)
                            both_m = sub_c.binary([list(oth_thresh), list(thresh)], invert=False, copy=True)
                            oth_px = sum(int(b.pixels()) for b in _find_blobs_unit_stride(oth_m, white, pixels_threshold=1, area_threshold=1, merge=True))
                            both_px = sum(int(b.pixels()) for b in _find_blobs_unit_stride(both_m, white, pixels_threshold=1, area_threshold=1, merge=True))
                            if max(0, both_px - oth_px) == 0:
                                covered_by_other = True
                                break
                        if covered_by_other:
                            break
                    if not covered_by_other:
                        unverified_set_b.append(c)

            corral_clipped = _clip_roi(cfg.near_corral_roi)

            def _actual_unverified_subblobs_in_rect(
                comp: Blob, rect_x: int, rect_y: int, rect_w: int, rect_h: int
            ) -> List[Blob]:
                """Clip an unverified component to a rectangle using ACTUAL component pixels (never AABB area!)."""
                if rect_w <= 0 or rect_h <= 0:
                    return []
                c_thresh = cfg.lab_thresholds.get(comp.label)
                if c_thresh is None:
                    return []
                sub_raw = _find_blobs_unit_stride(
                    img,
                    [tuple(c_thresh)],
                    roi=(rect_x, rect_y, rect_w, rect_h),
                    pixels_threshold=1,
                    area_threshold=1,
                    merge=False,
                )
                out_subs: List[Blob] = []
                for item in sub_raw:
                    sx, sy = int(item.x()), int(item.y())
                    sw, sh = max(1, int(item.w())), max(1, int(item.h()))
                    spx = int(item.pixels())
                    out_subs.append(Blob("unknown", sx, sy, sw, sh, spx, min(1.0, spx / float(sw * sh))))
                return out_subs

            for roi, merge_small in scan_rois:
                rx, ry, rw, rh = roi
                roi_img = img.crop(rx, ry, rw, rh)
                matched_mask = roi_img.binary(all_known_thresholds, invert=False, copy=True)
                uncal_raw = _find_blobs_unit_stride(matched_mask, black, pixels_threshold=1, area_threshold=1, merge=False)
                roi_cands: List[Blob] = []
                for item in uncal_raw:
                    w = max(1, int(item.w()))
                    h = max(1, int(item.h()))
                    px = int(item.pixels())
                    roi_cands.append(Blob("unknown", rx + int(item.x()), ry + int(item.y()), w, h, px, px / float(w * h)))
                for c in unverified_set_b:
                    inter = _rect_intersection(rx, ry, rw, rh, c.x, c.y, c.w, c.h)
                    if inter is None:
                        continue
                    ix1, iy1, ix2, iy2 = inter
                    iw, ih = max(1, ix2 - ix1), max(1, iy2 - iy1)
                    if ix1 == c.x and iy1 == c.y and iw == c.w and ih == c.h:
                        # If a concave unverified component fits inside a wide ROI while its AABB overlaps
                        # corral_roi with ZERO actual pixels inside corral_roi, split outside corral_roi so
                        # it never falsely inflates corral_unknown_cnt!
                        if (
                            corral_clipped is not None
                            and roi != corral_clipped
                            and c.pixels < c.w * c.h
                            and _rects_intersect(c.x, c.y, c.w, c.h, *corral_clipped)
                        ):
                            ci = _rect_intersection(c.x, c.y, c.w, c.h, *corral_clipped)
                            in_corral = (
                                _actual_unverified_subblobs_in_rect(c, ci[0], ci[1], ci[2] - ci[0], ci[3] - ci[1])
                                if ci is not None
                                else []
                            )
                            if not in_corral and ci is not None:
                                cx1, cy1, cx2, cy2 = ci
                                strips = (
                                    (c.x, c.y, c.w, cy1 - c.y),
                                    (c.x, cy2, c.w, c.y2 - cy2),
                                    (c.x, cy1, cx1 - c.x, cy2 - cy1),
                                    (cx2, cy1, c.x2 - cx2, cy2 - cy1),
                                )
                                for sx, sy, sw, sh in strips:
                                    roi_cands.extend(_actual_unverified_subblobs_in_rect(c, sx, sy, sw, sh))
                                continue
                        roi_cands.append(Blob("unknown", c.x, c.y, c.w, c.h, c.pixels, c.density))
                    else:
                        roi_cands.extend(_actual_unverified_subblobs_in_rect(c, ix1, iy1, iw, ih))

                merged_cands = _merge_adjacent_unknowns(roi_cands)
                small_cands: List[Blob] = []
                for unk in merged_cands:
                    if unk.pixels >= cfg.min_hazard_pixels:
                        rect = (unk.x, unk.y, unk.w, unk.h)
                        if rect not in emitted_unknown_rects:
                            emitted_unknown_rects.add(rect)
                            blobs.append(unk)
                    elif merge_small:
                        small_cands.append(unk)
                if merge_small and small_cands:
                    tot = sum(b.pixels for b in small_cands)
                    if tot >= cfg.min_hazard_pixels:
                        mx1 = min(b.x for b in small_cands)
                        my1 = min(b.y for b in small_cands)
                        mx2 = max(b.x2 for b in small_cands)
                        my2 = max(b.y2 for b in small_cands)
                        mw, mh = max(1, mx2 - mx1), max(1, my2 - my1)
                        rect = (mx1, my1, mw, mh)
                        if rect not in emitted_unknown_rects:
                            emitted_unknown_rects.add(rect)
                            blobs.append(Blob("unknown", mx1, my1, mw, mh, tot, min(1.0, tot / float(mw * mh))))
            return blobs

        # Tier 3: Fallback for find_blobs-only mocks without pixel access or binary().
        for roi, merge_fg in scan_rois:
            fg_blobs = _find_blobs_unit_stride(
                img,
                [tuple(cfg.arena_floor_lab)],
                roi=roi,
                pixels_threshold=cfg.min_hazard_pixels,
                area_threshold=cfg.min_hazard_pixels,
                invert=True,
                merge=merge_fg,
            )
            for item in fg_blobs:
                fx, fy, fw, fh = int(item.x()), int(item.y()), max(1, int(item.w())), max(1, int(item.h()))
                px = int(item.pixels())
                if px < cfg.min_hazard_pixels:
                    continue
                fg = Blob("unknown", fx, fy, fw, fh, px, px / float(fw * fh))
                fg_roi = (fx, fy, fw, fh)
                if fg_roi in emitted_unknown_rects:
                    continue
                roi_surfaces: List[Blob] = []
                for surface_label in _SURFACE_LABELS:
                    thresh = cfg.lab_thresholds.get(surface_label)
                    if thresh is None:
                        continue
                    parent_surfaces = verified_by_label[surface_label]
                    if not parent_surfaces:
                        continue
                    for sub in _extract_roi_blobs_for_label(img, surface_label, thresh, fg_roi):
                        if any(
                            _rects_intersect(sub.x, sub.y, sub.w, sub.h, p.x, p.y, p.w, p.h)
                            for p in parent_surfaces
                        ):
                            roi_surfaces.append(sub)

                roi_objects: List[Blob] = []
                for obj_label in _OBJECT_LABELS:
                    thresh = cfg.lab_thresholds.get(obj_label)
                    if thresh is None:
                        continue
                    parent_objs = verified_by_label[obj_label]
                    if not parent_objs:
                        continue
                    for sub in _extract_roi_blobs_for_label(img, obj_label, thresh, fg_roi):
                        if any(
                            _rects_intersect(sub.x, sub.y, sub.w, sub.h, p.x, p.y, p.w, p.h) for p in parent_objs
                        ):
                            roi_objects.append(sub)

                explained_px, union_area = _unique_explained_pixels(fg, roi_objects + roi_surfaces, cfg=cfg)
                unexplained_px = max(0, fg.pixels - explained_px)
                pixel_coverage = explained_px / float(max(1, fg.pixels))
                area_coverage = union_area / float(max(1, fg.w * fg.h))
                if (
                    unexplained_px >= cfg.min_hazard_pixels
                    or pixel_coverage < 0.70
                    or (area_coverage < 0.70 and explained_px < fg.pixels)
                ):
                    emitted_unknown_rects.add(fg_roi)
                    blobs.append(fg)

    return blobs
