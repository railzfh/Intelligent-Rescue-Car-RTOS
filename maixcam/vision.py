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
        # Never capture, modify, or rewrite baseline once the explicit MCU push phase or backout verify starts.
        if drop_pushing or (mission_state is not None and mission_state != MISSION_DROP_OPEN_AND_PUSH):
            return self._drop_baseline is not None, (0, 0, 0)
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
            # Before pushing (not in explicit MCU push phase), require zero preexisting ramp/divider
            # blocks and a valid frozen baseline; during explicit push phase, allow own cargo to cross
            # the ramp while still immediately inhibiting on any cyan hazard or unknown obstruction.
            if drop_path_hazards_clear and (drop_pushing or corral_empty or (baseline_ok and mis_cnt == (0, 0, 0))):
                flags |= VIS_FLAG_DROP_PATH_CLEAR

            if (
                gate_closed is False
                and gate_settled is True
                and not drop_pushing
                and baseline_ok
                and corral_safe
                and drop_path_hazards_clear
                and mis_cnt == (0, 0, 0)
            ):
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


def _extract_roi_blobs_for_label(img, label: str, thresh: Sequence[int], roi: Tuple[int, int, int, int]) -> List[Blob]:
    found = img.find_blobs([tuple(thresh)], roi=roi, pixels_threshold=1, area_threshold=1, merge=False)
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

    # Tier 2: Real MaixPy Image binary() union with tight opponent-minus-floor bounds!
    # Recompute position strictly from non-floor components that contain opponent pixels so
    # overlapping floor LAB bands never inflate opponent bounds (e.g. y2=240 on a distant robot).
    if hasattr(img, "crop") and hasattr(img, "binary"):
        non_floor_comps = img.find_blobs(
            [tuple(floor_thresh)],
            roi=(bx, by, bw, bh),
            pixels_threshold=1,
            area_threshold=1,
            invert=True,
            merge=False,
        )
        if not non_floor_comps:
            return None
        white = [(50, 100, -128, 127, -128, 127)]
        total_opp_px = 0
        min_x, min_y = bx + bw, by + bh
        max_x, max_y = bx, by
        for nf in non_floor_comps:
            nx, ny, nw, nh = int(nf.x()), int(nf.y()), max(1, int(nf.w())), max(1, int(nf.h()))
            sub = img.crop(nx, ny, nw, nh)
            floor_mask = sub.binary([list(floor_thresh)], invert=False, copy=True)
            both_mask = sub.binary([list(floor_thresh), list(opp_thresh)], invert=False, copy=True)

            def _opp_minus_floor_px(rx: int, ry: int, rw: int, rh: int) -> int:
                if rw <= 0 or rh <= 0:
                    return 0
                r = (rx, ry, rw, rh)
                f_px = sum(int(b.pixels()) for b in floor_mask.find_blobs(white, roi=r, pixels_threshold=1, area_threshold=1, merge=True))
                b_px = sum(int(b.pixels()) for b in both_mask.find_blobs(white, roi=r, pixels_threshold=1, area_threshold=1, merge=True))
                return max(0, b_px - f_px)

            floor_px = sum(int(b.pixels()) for b in floor_mask.find_blobs(white, pixels_threshold=1, area_threshold=1, merge=True))
            both_px = sum(int(b.pixels()) for b in both_mask.find_blobs(white, pixels_threshold=1, area_threshold=1, merge=True))
            nf_opp_px = max(0, both_px - floor_px)
            if nf_opp_px <= 0:
                continue
            total_opp_px += nf_opp_px
            if nf_opp_px == max(0, nw * nh - floor_px):
                # Fast path: every non-floor pixel in sub belongs to opponent \ floor.
                ox1, oy1, ox2, oy2 = nx, ny, nx + nw, ny + nh
            else:
                # Exact binary search for tight bounds of (opponent \ floor) inside sub even when
                # another non-floor object touches the opponent component!
                lo, hi = 0, nh - 1
                while lo < hi:
                    mid = (lo + hi) // 2
                    if _opp_minus_floor_px(0, 0, nw, mid + 1) > 0:
                        hi = mid
                    else:
                        lo = mid + 1
                ty1 = lo
                lo, hi = ty1 + 1, nh
                while lo < hi:
                    mid = (lo + hi) // 2
                    if _opp_minus_floor_px(0, ty1, nw, mid - ty1) >= nf_opp_px:
                        hi = mid
                    else:
                        lo = mid + 1
                ty2 = lo
                th = max(1, ty2 - ty1)
                lo, hi = 0, nw - 1
                while lo < hi:
                    mid = (lo + hi) // 2
                    if _opp_minus_floor_px(0, ty1, mid + 1, th) > 0:
                        hi = mid
                    else:
                        lo = mid + 1
                tx1 = lo
                lo, hi = tx1 + 1, nw
                while lo < hi:
                    mid = (lo + hi) // 2
                    if _opp_minus_floor_px(tx1, ty1, mid - tx1, th) >= nf_opp_px:
                        hi = mid
                    else:
                        lo = mid + 1
                tx2 = lo
                ox1, oy1, ox2, oy2 = nx + tx1, ny + ty1, nx + tx2, ny + ty2
            if ox1 < min_x:
                min_x = ox1
            if oy1 < min_y:
                min_y = oy1
            if ox2 > max_x:
                max_x = ox2
            if oy2 > max_y:
                max_y = oy2
        if total_opp_px < cfg.min_opponent_pixels or max_x <= min_x or max_y <= min_y:
            return None
        ow = max_x - min_x
        oh = max_y - min_y
        return Blob("opponent", min_x, min_y, ow, oh, total_opp_px, min(1.0, total_opp_px / float(ow * oh)))

    # Fallback using find_blobs: intersect opponent ROI with non-floor pixels so plain floor never triggers.
    non_floor = img.find_blobs(
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
        merge_label = label in ("red_zone", "blue_zone", "opponent")
        found = img.find_blobs([tuple(thresh)], pixels_threshold=min_area, area_threshold=min_area, merge=merge_label)
        for item in found:
            if label == "opponent":
                filtered = _filter_opponent_excluding_floor(img, item, thresh, cfg)
                if filtered is not None:
                    blobs.append(filtered)
                continue
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

    # In NEAR view, detect non-floor foreground across ALL fixed near ROIs PLUS the dynamic
    # home zone, ramp band, and projected push-corridor geometry used to authorize drops.
    if scan_unknown_near:
        verified_full_objects = [
            b
            for b in blobs
            if b.label in _OBJECT_LABELS and _is_verified_object_blob(b, cfg.min_hazard_pixels, cfg)
        ]
        verified_full_surfaces = [b for b in blobs if b.label in _SURFACE_LABELS]
        verified_all = verified_full_objects + verified_full_surfaces
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

        emitted_unknown_rects = set()
        for roi, merge_fg in scan_rois:
            fg_blobs = img.find_blobs(
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

                # Tier 1: Exact per-pixel union when LAB pixel access is available (zero density/box error).
                if get_lab is not None:
                    active_verified = [
                        (vb, cfg.lab_thresholds[vb.label])
                        for vb in verified_all
                        if vb.label in cfg.lab_thresholds
                        and _rects_intersect(fx, fy, fw, fh, vb.x, vb.y, vb.w, vb.h)
                    ]
                    fg_exact_px = 0
                    explained_exact_px = 0
                    for yy in range(fy, fy + fh):
                        for xx in range(fx, fx + fw):
                            lab = get_lab(xx, yy)
                            if _lab_in_thresh(lab, cfg.arena_floor_lab):
                                continue
                            fg_exact_px += 1
                            for vb, vthresh in active_verified:
                                if (
                                    vb.x <= xx < vb.x2
                                    and vb.y <= yy < vb.y2
                                    and _lab_in_thresh(lab, vthresh)
                                ):
                                    explained_exact_px += 1
                                    break
                    unexplained_px = max(0, fg_exact_px - explained_exact_px)
                    if fg_exact_px >= cfg.min_hazard_pixels and (
                        unexplained_px >= cfg.min_hazard_pixels
                        or (explained_exact_px / float(max(1, fg_exact_px))) < 0.70
                    ):
                        emitted_unknown_rects.add(fg_roi)
                        blobs.append(fg)
                    continue

                # Tier 2: Exact C-accelerated per-verified-box elementary-cell binary union on real MaixPy
                # Image objects: only enables a class threshold inside cells covered by verified blobs of
                # that class so separate unverified patches of the same color outside verified boxes are caught!
                if use_maix_binary:
                    active_verified = [
                        vb
                        for vb in verified_all
                        if vb.label in cfg.lab_thresholds
                        and _rects_intersect(fx, fy, fw, fh, vb.x, vb.y, vb.w, vb.h)
                    ]
                    if not active_verified:
                        emitted_unknown_rects.add(fg_roi)
                        blobs.append(fg)
                        continue
                    xs = {fx, fx + fw}
                    ys = {fy, fy + fh}
                    for vb in active_verified:
                        xs.add(max(fx, vb.x))
                        xs.add(min(fx + fw, vb.x2))
                        ys.add(max(fy, vb.y))
                        ys.add(min(fy + fh, vb.y2))
                    sorted_xs = sorted(xs)
                    sorted_ys = sorted(ys)

                    black = [(0, 40, -128, 127, -128, 127)]
                    total_unexplained_px = 0
                    for xi in range(len(sorted_xs) - 1):
                        cx1, cx2 = sorted_xs[xi], sorted_xs[xi + 1]
                        if cx2 <= cx1:
                            continue
                        for yi in range(len(sorted_ys) - 1):
                            cy1, cy2 = sorted_ys[yi], sorted_ys[yi + 1]
                            if cy2 <= cy1:
                                continue
                            cell_thresholds = [list(cfg.arena_floor_lab)]
                            seen_labels = set()
                            for vb in active_verified:
                                if (
                                    vb.label not in seen_labels
                                    and vb.x <= cx1
                                    and vb.x2 >= cx2
                                    and vb.y <= cy1
                                    and vb.y2 >= cy2
                                ):
                                    seen_labels.add(vb.label)
                                    cell_thresholds.append(list(cfg.lab_thresholds[vb.label]))
                            cell_img = img.crop(cx1, cy1, cx2 - cx1, cy2 - cy1)
                            matched_mask = cell_img.binary(cell_thresholds, invert=False, copy=True)
                            unmatched = matched_mask.find_blobs(
                                black, pixels_threshold=1, area_threshold=1, merge=True
                            )
                            total_unexplained_px += sum(int(u.pixels()) for u in unmatched)
                    explained_px = max(0, px - total_unexplained_px)
                    if total_unexplained_px >= cfg.min_hazard_pixels or (explained_px / float(max(1, px))) < 0.70:
                        emitted_unknown_rects.add(fg_roi)
                        blobs.append(fg)
                    continue

                # Tier 3: Fallback for find_blobs-only mocks without pixel access or binary().
                roi_surfaces: List[Blob] = []
                for surface_label in _SURFACE_LABELS:
                    thresh = cfg.lab_thresholds.get(surface_label)
                    if thresh is None:
                        continue
                    parent_surfaces = [b for b in verified_full_surfaces if b.label == surface_label]
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
                    parent_objs = [b for b in verified_full_objects if b.label == obj_label]
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
