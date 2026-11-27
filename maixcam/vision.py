"""Vision decision pipeline for single-axis pitch MaixCAM on Intelligent Rescue Car.

Separates pure geometric/rule processing and spatial motion estimation (fully testable
on desktop Python) from the thin MaixPy image blob extraction wrapper.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

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
    VIS_FLAG_PRE_MATCH_GEN_SHIFT,
    VIS_FLAG_RAMP_VISIBLE,
    VIS_FLAG_REAR_CLEAR,
    VIS_FLAG_SETTLE_GEN_SHIFT,
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
    in_corral: Optional[bool] = None
    in_sweep: Optional[bool] = None

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


@dataclass(frozen=True)
class _ProvenRegion:
    blob: Blob
    parent: Optional[Blob] = None  # Resolve intact sparse parents only when an exact union is needed.
    points: Optional[List[Tuple[int, int]]] = None


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
    """Estimate unique pixels using uniform density within each bounding box."""
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
    """Return estimated explained pixels and exact bounding-box union area clipped to fg."""
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
        inner_limit_y = min(home.y2, assoc_ramp.y)
        ramp_bot_y = assoc_ramp.y2
        seam_x1 = min(home.x, assoc_ramp.x)
        seam_x2 = max(home.x2, assoc_ramp.x2)
        tgt = [0, 0, 0]
        oth = [0, 0, 0]
        mis = [0, 0, 0]
        idx_map = {"green": 0, "black": 1, "orange": 2}
        for b in blocks:
            idx = idx_map.get(b.label)
            if idx is None:
                continue
            in_home = _rects_intersect(b.x, b.y, b.w, b.h, home.x, home.y, home.w, home.h)
            on_full_ramp = _rects_intersect(b.x, b.y, b.w, b.h, assoc_ramp.x, assoc_ramp.y, assoc_ramp.w, assoc_ramp.h)
            in_home_ramp_seam = b.y < ramp_bot_y and b.y2 > inner_limit_y and b.x < seam_x2 and b.x2 > seam_x1
            on_ramp = on_full_ramp or in_home_ramp_seam
            if not (in_home or on_ramp):
                continue
            if not on_ramp and b.y >= home.y and b.y2 <= inner_limit_y:
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

        # Associate a purple ramp horizontally overlapping own safe zone AND vertically adjacent
        # to home's lower approach edge; fail closed if none or multiple ambiguous ramps match.
        max_gap = max(0, int(self.cfg.ramp_home_max_gap_px))
        assoc_ramps = [
            r
            for r in ramps
            if r.x < home.x2
            and r.x2 > home.x
            and r.y > home.y
            and r.y >= home.y2 - max(r.h, max_gap)
            and r.y <= home.y2 + max_gap
            and r.y2 >= home.y2 - max_gap
        ]
        assoc_ramp = assoc_ramps[0] if len(assoc_ramps) == 1 else None
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
        ramp_inner_y = min(home.y2, assoc_ramp.y)
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
        pre_match_settle_gen: int = 0,
        gate_settle_gen: int = 0,
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
            if (
                not camera_near_cmd
                and camera_settled
                and gate_closed is False
                and gate_settled is True
                and 1 <= pre_match_settle_gen <= 3
            ):
                flags |= (pre_match_settle_gen & 0x3) << VIS_FLAG_PRE_MATCH_GEN_SHIFT
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

            def _in_corral(b: Blob) -> bool:
                return b.in_corral if b.in_corral is not None else _blob_in_roi(b, corral_roi)

            def _in_sweep(b: Blob) -> bool:
                return b.in_sweep if b.in_sweep is not None else _blob_in_roi(b, sweep_roi)

            green_cnt = sum(1 for b in blocks if b.label == "green" and _in_corral(b))
            black_cnt = sum(1 for b in blocks if b.label == "black" and _in_corral(b))
            orange_cnt = sum(1 for b in blocks if b.label == "orange" and _in_corral(b))
            cyan_cnt = sum(1 for b in hazards if _in_corral(b))
            unknown_cnt = sum(1 for b in unknowns if _in_corral(b))

            state.corral_green_cnt = min(255, green_cnt)
            state.corral_black_cnt = min(255, black_cnt)
            state.corral_orange_cnt = min(255, orange_cnt)
            state.corral_cyan_cnt = min(255, cyan_cnt)
            state.corral_unknown_cnt = min(255, unknown_cnt)

            corral_safe = cyan_cnt == 0 and unknown_cnt == 0
            if corral_safe:
                flags |= VIS_FLAG_CORRAL_SAFE

            # Sweep zone must be clear of ANY block, cyan hazard, or unknown foreground with actual pixels in sweep_roi.
            sweep_occupied = any(_in_sweep(b) for b in blocks + hazards + unknowns)
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
                if 1 <= gate_settle_gen <= 3:
                    flags |= (gate_settle_gen & 0x3) << VIS_FLAG_SETTLE_GEN_SHIFT
            elif gate_closed is True and gate_settled is True and 1 <= gate_settle_gen <= 3:
                flags |= (gate_settle_gen & 0x3) << VIS_FLAG_SETTLE_GEN_SHIFT

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


def _tier1_find_components(
    get_lab: Callable[[int, int], Tuple[int, int, int]],
    thresh: Sequence[int],
    x1: int,
    y1: int,
    x2: int,
    y2: int,
    eight_conn: bool = False,
) -> List[Tuple[int, int, int, int, int, List[Tuple[int, int]]]]:
    """Find connected components matching thresh in [x1,x2) x [y1,y2) using 4- or 8-connectivity."""
    w = x2 - x1
    h = y2 - y1
    if w <= 0 or h <= 0:
        return []
    nbrs = (
        ((-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1))
        if eight_conn
        else ((-1, 0), (1, 0), (0, -1), (0, 1))
    )
    vis = [[False] * w for _ in range(h)]
    comps: List[Tuple[int, int, int, int, int, List[Tuple[int, int]]]] = []
    for yy in range(y1, y2):
        for xx in range(x1, x2):
            if vis[yy - y1][xx - x1] or not _lab_in_thresh(get_lab(xx, yy), thresh):
                continue
            stack = [(xx, yy)]
            vis[yy - y1][xx - x1] = True
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
                for dx, dy in nbrs:
                    nx, ny = cx + dx, cy + dy
                    if x1 <= nx < x2 and y1 <= ny < y2 and not vis[ny - y1][nx - x1]:
                        if _lab_in_thresh(get_lab(nx, ny), thresh):
                            vis[ny - y1][nx - x1] = True
                            stack.append((nx, ny))
            cw = max_x - min_x + 1
            ch = max_y - min_y + 1
            comps.append((min_x, min_y, cw, ch, len(comp_pts), comp_pts))
    return comps


def extract_blobs_from_maix_image(img, cfg: RescueVisionConfig, scan_unknown_near: bool = False) -> List[Blob]:
    """Run LAB blob detection on a MaixPy image object, plus component-exact unknown foreground scan."""
    blobs: List[Blob] = []
    verified_by_label: dict[str, List[Blob]] = {lbl: [] for lbl in (_SURFACE_LABELS + _OBJECT_LABELS)}
    unverified_by_label: dict[str, List[Blob]] = {lbl: [] for lbl in (_SURFACE_LABELS + _OBJECT_LABELS)}
    all_raw_by_label: dict[str, List[Blob]] = {lbl: [] for lbl in (_SURFACE_LABELS + _OBJECT_LABELS)}

    get_lab = _get_lab_pixel_accessor(img)
    use_maix_binary = get_lab is None and hasattr(img, "crop") and hasattr(img, "binary")
    fw, fh = cfg.frame_width, cfg.frame_height

    def _clip_roi(r: Sequence[int]) -> Optional[Tuple[int, int, int, int]]:
        x1 = max(0, int(r[0]))
        y1 = max(0, int(r[1]))
        x2 = min(fw, int(r[0]) + int(r[2]))
        y2 = min(fh, int(r[1]) + int(r[3]))
        if x2 <= x1 or y2 <= y1:
            return None
        return (x1, y1, x2 - x1, y2 - y1)

    corral_clipped = _clip_roi(cfg.near_corral_roi)
    sweep_clipped = _clip_roi(cfg.near_sweep_roi)

    def _pts_hit_roi(pts: Iterable[Tuple[int, int]], target: Optional[Tuple[int, int, int, int]]) -> bool:
        if target is None:
            return False
        tx1, ty1, tw, th = target
        tx2, ty2 = tx1 + tw, ty1 + th
        return any(tx1 <= px < tx2 and ty1 <= py < ty2 for px, py in pts)

    rle_cache: Dict[
        Tuple[str, Tuple[int, int, int, int]],
        Tuple[List[List[int]], List[Tuple[int, int, int, int, int]], bool],
    ] = {}

    def _rle_fill_mask(
        src_img,
        thresh_list,
        box: Tuple[int, int, int, int],
        grid: List[List[bool]],
        ox: int,
        oy: int,
        val: bool = True,
    ) -> None:
        bx1, by1, bx2, by2 = box
        bw, bh = bx2 - bx1, by2 - by1
        if bw <= 0 or bh <= 0:
            return
        gh = len(grid)
        gw = len(grid[0]) if gh else 0
        if bh <= bw:
            for yy in range(by1, by2):
                gy = yy - oy
                if not (0 <= gy < gh):
                    continue
                row_g = grid[gy]
                for run in _find_blobs_unit_stride(
                    src_img, thresh_list, roi=(bx1, yy, bw, 1), pixels_threshold=1, area_threshold=1, merge=False
                ):
                    rx1 = max(bx1, int(run.x()))
                    rx2 = min(bx2, int(run.x()) + max(1, int(run.w())))
                    for xx in range(rx1, rx2):
                        gx = xx - ox
                        if 0 <= gx < gw:
                            row_g[gx] = val
        else:
            for xx in range(bx1, bx2):
                gx = xx - ox
                if not (0 <= gx < gw):
                    continue
                for run in _find_blobs_unit_stride(
                    src_img, thresh_list, roi=(xx, by1, 1, bh), pixels_threshold=1, area_threshold=1, merge=False
                ):
                    ry1 = max(by1, int(run.y()))
                    ry2 = min(by2, int(run.y()) + max(1, int(run.h())))
                    for yy in range(ry1, ry2):
                        gy = yy - oy
                        if 0 <= gy < gh:
                            grid[gy][gx] = val

    def _resolve_exact_label_components(
        label: str,
        c_thresh: Sequence[int],
        box: Tuple[int, int, int, int],
        expected_geoms: set,
    ) -> Tuple[List[List[int]], List[Tuple[int, int, int, int, int]], bool]:
        key = (label, box)
        cached = rle_cache.get(key)
        if cached is not None:
            return cached
        ux1, uy1, ux2, uy2 = box
        uw, uh = ux2 - ux1, uy2 - uy1
        grid = [[False] * uw for _ in range(uh)]
        t_arg = [tuple(c_thresh)]
        # Reconstruct the exact binary pixel grid using 1-px strip find_blobs() calls
        # (exact run-length encoding along the shorter axis, zero pixel-accessor requirement).
        if uh <= uw:
            for yy in range(uy1, uy2):
                row_mask = grid[yy - uy1]
                for run in _find_blobs_unit_stride(
                    img, t_arg, roi=(ux1, yy, uw, 1), pixels_threshold=1, area_threshold=1, merge=False
                ):
                    rx1 = max(ux1, int(run.x()))
                    rx2 = min(ux2, int(run.x()) + max(1, int(run.w())))
                    for xx in range(rx1, rx2):
                        row_mask[xx - ux1] = True
        else:
            for xx in range(ux1, ux2):
                for run in _find_blobs_unit_stride(
                    img, t_arg, roi=(xx, uy1, 1, uh), pixels_threshold=1, area_threshold=1, merge=False
                ):
                    ry1 = max(uy1, int(run.y()))
                    ry2 = min(uy2, int(run.y()) + max(1, int(run.h())))
                    for yy in range(ry1, ry2):
                        grid[yy - uy1][xx - ux1] = True

        def _label_grid(eight_conn: bool) -> Tuple[List[List[int]], List[Tuple[int, int, int, int, int]]]:
            cid_grid = [[-1] * uw for _ in range(uh)]
            geoms: List[Tuple[int, int, int, int, int]] = []
            nbrs = (
                ((-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1))
                if eight_conn
                else ((-1, 0), (1, 0), (0, -1), (0, 1))
            )
            for ly in range(uh):
                row = grid[ly]
                cid_row = cid_grid[ly]
                for lx in range(uw):
                    if not row[lx] or cid_row[lx] != -1:
                        continue
                    cid = len(geoms)
                    cid_row[lx] = cid
                    stack = [(lx, ly)]
                    mn_x = mx_x = lx
                    mn_y = mx_y = ly
                    cnt = 0
                    while stack:
                        cx, cy = stack.pop()
                        cnt += 1
                        if cx < mn_x:
                            mn_x = cx
                        if cx > mx_x:
                            mx_x = cx
                        if cy < mn_y:
                            mn_y = cy
                        if cy > mx_y:
                            mx_y = cy
                        for dx, dy in nbrs:
                            nx, ny = cx + dx, cy + dy
                            if 0 <= nx < uw and 0 <= ny < uh and grid[ny][nx] and cid_grid[ny][nx] == -1:
                                cid_grid[ny][nx] = cid
                                stack.append((nx, ny))
                    geoms.append((ux1 + mn_x, uy1 + mn_y, mx_x - mn_x + 1, mx_y - mn_y + 1, cnt))
            return cid_grid, geoms

        used_eight = False
        cid_grid, geoms = _label_grid(eight_conn=False)
        if expected_geoms and not expected_geoms.issubset(set(geoms)):
            cid_grid8, geoms8 = _label_grid(eight_conn=True)
            if expected_geoms.issubset(set(geoms8)):
                cid_grid, geoms = cid_grid8, geoms8
                used_eight = True
        result = (cid_grid, geoms, used_eight)
        rle_cache[key] = result
        return result

    def _tier1_find_matching_components(
        thresh: Sequence[int],
        x1: int,
        y1: int,
        x2: int,
        y2: int,
        expected_geoms: set,
    ) -> List[Tuple[int, int, int, int, int, List[Tuple[int, int]]]]:
        assert get_lab is not None
        comps = _tier1_find_components(get_lab, thresh, x1, y1, x2, y2, eight_conn=False)
        found_g = {(c[0], c[1], c[2], c[3], c[4]) for c in comps}
        if expected_geoms and not expected_geoms.issubset(found_g):
            comps8 = _tier1_find_components(get_lab, thresh, x1, y1, x2, y2, eight_conn=True)
            found_g8 = {(c[0], c[1], c[2], c[3], c[4]) for c in comps8}
            if expected_geoms.issubset(found_g8) or len(expected_geoms & found_g8) > len(expected_geoms & found_g):
                return comps8
        return comps

    # Extract surfaces first so custom overlapping LAB bands (e.g. red_zone vs orange) can resolve
    # exact-component surface/object ambiguity without ever suppressing genuine cyan hazards.
    ordered_labels = [lbl for lbl in (_SURFACE_LABELS + _OBJECT_LABELS) if lbl in cfg.lab_thresholds]
    for lbl in cfg.lab_thresholds:
        if lbl not in ordered_labels:
            ordered_labels.append(lbl)

    verified_surface_geoms = set()
    emitted_rejected_object_rects = set()
    precomputed_pts_by_key: Dict[Tuple[str, int, int, int, int, int], List[Tuple[int, int]]] = {}
    proven_non_floor_keys: set[Tuple[str, int, int, int, int, int]] = set()
    # (blob, meets_label_min_area, is_verified_object)
    raw_obj_cands: List[Tuple[Blob, bool, bool]] = []

    def _filter_blobs_excluding_floor(
        raw_blobs: List[Blob], label: str, thresh: Sequence[int], min_px: int
    ) -> List[Blob]:
        """Exclude arena-floor pixels whenever a custom label threshold overlaps arena_floor_lab.

        Prevents floor-only patches from ever being accepted as legal cargo/zones/ramps (R18 P2)
        and strips floor bridges attached to real blocks identically across Tiers 1, 2, and 3.
        """
        if not raw_blobs:
            return []
        floor_thresh = cfg.arena_floor_lab
        pure_non_floor: List[Blob] = []
        mixed: List[Blob] = []
        white = [(50, 100, -128, 127, -128, 127)]

        for b in raw_blobs:
            bx1, by1, bx2, by2 = max(0, b.x), max(0, b.y), min(fw, b.x2), min(fh, b.y2)
            bw_c, bh_c = bx2 - bx1, by2 - by1
            if bw_c <= 0 or bh_c <= 0:
                continue
            if get_lab is not None:
                has_floor = False
                has_nf_match = False
                for yy in range(by1, by2):
                    for xx in range(bx1, bx2):
                        lab = get_lab(xx, yy)
                        if _lab_in_thresh(lab, floor_thresh):
                            has_floor = True
                        elif _lab_in_thresh(lab, thresh):
                            has_nf_match = True
                        if has_floor and has_nf_match:
                            break
                    if has_floor and has_nf_match:
                        break
                if not has_nf_match:
                    continue
                if not has_floor:
                    pure_non_floor.append(b)
                else:
                    mixed.append(b)
            elif use_maix_binary:
                sub = img.crop(bx1, by1, bw_c, bh_c)
                fl_m = sub.binary([list(floor_thresh)], invert=False, copy=True)
                bo_m = sub.binary([list(floor_thresh), list(thresh)], invert=False, copy=True)
                fl_px = sum(
                    int(x.pixels())
                    for x in _find_blobs_unit_stride(fl_m, white, pixels_threshold=1, area_threshold=1, merge=True)
                )
                bo_px = sum(
                    int(x.pixels())
                    for x in _find_blobs_unit_stride(bo_m, white, pixels_threshold=1, area_threshold=1, merge=True)
                )
                if bo_px <= fl_px:
                    continue
                if fl_px == 0:
                    pure_non_floor.append(b)
                else:
                    mixed.append(b)
            else:
                nf = _find_blobs_unit_stride(
                    img,
                    [tuple(floor_thresh)],
                    roi=(bx1, by1, bw_c, bh_c),
                    pixels_threshold=1,
                    area_threshold=1,
                    invert=True,
                    merge=True,
                )
                if not nf:
                    continue
                nf_px = sum(int(x.pixels()) for x in nf)
                if nf_px >= bw_c * bh_c:
                    pure_non_floor.append(b)
                else:
                    mixed.append(b)

        if not mixed:
            for p in pure_non_floor:
                proven_non_floor_keys.add((label, p.x, p.y, p.w, p.h, p.pixels))
            return pure_non_floor

        # Extraction scans whole union boxes, including the space between input components.
        # Grow each box to a fixed point against prior scan boxes and unclaimed pure blobs.
        remaining_pure = pure_non_floor
        scan_boxes: List[Tuple[int, int, int, int]] = []
        for m in mixed:
            ux1, uy1, ux2, uy2 = m.x, m.y, m.x2, m.y2
            while True:
                before = (ux1, uy1, ux2, uy2)
                unclaimed_boxes = []
                for x1, y1, x2, y2 in scan_boxes:
                    if x1 < ux2 and ux1 < x2 and y1 < uy2 and uy1 < y2:
                        ux1, uy1 = min(ux1, x1), min(uy1, y1)
                        ux2, uy2 = max(ux2, x2), max(uy2, y2)
                    else:
                        unclaimed_boxes.append((x1, y1, x2, y2))
                scan_boxes = unclaimed_boxes

                unclaimed_pure = []
                for p in remaining_pure:
                    if _rects_intersect(ux1, uy1, ux2 - ux1, uy2 - uy1, p.x, p.y, p.w, p.h):
                        ux1, uy1 = min(ux1, p.x), min(uy1, p.y)
                        ux2, uy2 = max(ux2, p.x2), max(uy2, p.y2)
                    else:
                        unclaimed_pure.append(p)
                remaining_pure = unclaimed_pure
                if before == (ux1, uy1, ux2, uy2):
                    break
            scan_boxes.append((ux1, uy1, ux2, uy2))

        extracted: List[Blob] = []
        for x1, y1, x2, y2 in scan_boxes:
            ux1, uy1, ux2, uy2 = max(0, x1), max(0, y1), min(fw, x2), min(fh, y2)
            uw, uh = ux2 - ux1, uy2 - uy1
            if uw <= 0 or uh <= 0:
                continue
            if get_lab is not None:
                grid = [
                    [
                        _lab_in_thresh(get_lab(ux1 + xx, uy1 + yy), thresh)
                        and not _lab_in_thresh(get_lab(ux1 + xx, uy1 + yy), floor_thresh)
                        for xx in range(uw)
                    ]
                    for yy in range(uh)
                ]
            else:
                cand_grid = [[False] * uw for _ in range(uh)]
                floor_grid = [[False] * uw for _ in range(uh)]
                _rle_fill_mask(img, [tuple(thresh)], (ux1, uy1, ux2, uy2), cand_grid, ux1, uy1, True)
                _rle_fill_mask(img, [tuple(floor_thresh)], (ux1, uy1, ux2, uy2), floor_grid, ux1, uy1, True)
                grid = [[cand_grid[yy][xx] and not floor_grid[yy][xx] for xx in range(uw)] for yy in range(uh)]

            vis_nf = [[False] * uw for _ in range(uh)]
            for gy in range(uh):
                row_g = grid[gy]
                for gx in range(uw):
                    if vis_nf[gy][gx] or not row_g[gx]:
                        continue
                    vis_nf[gy][gx] = True
                    st = [(gx, gy)]
                    pts: List[Tuple[int, int]] = []
                    mn_x = mx_x = gx
                    mn_y = mx_y = gy
                    while st:
                        cx, cy = st.pop()
                        pts.append((ux1 + cx, uy1 + cy))
                        if cx < mn_x:
                            mn_x = cx
                        if cx > mx_x:
                            mx_x = cx
                        if cy < mn_y:
                            mn_y = cy
                        if cy > mx_y:
                            mx_y = cy
                        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                            nx, ny = cx + dx, cy + dy
                            if 0 <= nx < uw and 0 <= ny < uh and not vis_nf[ny][nx] and grid[ny][nx]:
                                vis_nf[ny][nx] = True
                                st.append((nx, ny))
                    cnt = len(pts)
                    if cnt >= min_px:
                        cw, ch = mx_x - mn_x + 1, mx_y - mn_y + 1
                        nb = Blob(label=label, x=ux1 + mn_x, y=uy1 + mn_y, w=cw, h=ch, pixels=cnt, density=cnt / float(cw * ch))
                        precomputed_pts_by_key[(label, nb.x, nb.y, nb.w, nb.h, nb.pixels)] = pts
                        extracted.append(nb)

        for p in remaining_pure:
            proven_non_floor_keys.add((label, p.x, p.y, p.w, p.h, p.pixels))
        for nb in extracted:
            proven_non_floor_keys.add((label, nb.x, nb.y, nb.w, nb.h, nb.pixels))
        return remaining_pure + extracted

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
        # For object labels in FAR mode, capture down to min(min_area, min_hazard_pixels) so competing
        # overlapping alternate-class evidence participates in cross-label disambiguation too.
        if scan_unknown_near and label in (_SURFACE_LABELS + _OBJECT_LABELS):
            px_thr = 1
        elif label in _OBJECT_LABELS:
            px_thr = min(min_area, cfg.min_hazard_pixels)
        else:
            px_thr = min_area
        found = _find_blobs_unit_stride(img, [tuple(thresh)], pixels_threshold=px_thr, area_threshold=px_thr, merge=False)
        found_blobs: List[Blob] = []
        for item in found:
            w = max(1, int(item.w()))
            h = max(1, int(item.h()))
            px = int(item.pixels())
            found_blobs.append(
                Blob(label=label, x=int(item.x()), y=int(item.y()), w=w, h=h, pixels=px, density=px / float(w * h))
            )
        # Strip floor pixels from overlapping legal-cargo and surface calibrations, but NEVER
        # subtract floor from `cyan` hazards so ambiguous cyan/floor overlap always fails closed (R19 P1).
        if label != "cyan" and _lab_ranges_overlap(thresh, cfg.arena_floor_lab):
            found_blobs = _filter_blobs_excluding_floor(found_blobs, label, thresh, px_thr)
        for b in found_blobs:
            px = b.pixels

            if label in all_raw_by_label:
                all_raw_by_label[label].append(b)

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
                cand_min = min(min_area, cfg.min_hazard_pixels)
                if px >= cand_min:
                    meets_min = px >= min_area
                    is_ver = meets_min and _is_verified_object_blob(b, cfg.min_hazard_pixels, cfg)
                    raw_obj_cands.append((b, meets_min, is_ver))
                elif scan_unknown_near:
                    unverified_by_label[label].append(b)
            elif px >= min_area:
                blobs.append(b)

    # Cross-label object disambiguation BEFORE emitting verified or rejected objects into blobs:
    # Include competing components down to min_hazard_pixels while retaining each label's min_area
    # to accept a standalone legal block.
    accounted_obj_pts: set[Tuple[int, int]] = set()
    accounted_obj_boxes: List[Tuple[int, int, int, int]] = []
    accounted_clusters: List[Tuple[set[Tuple[int, int]], Tuple[int, int, int, int], bool]] = []
    accounted_rejected_blobs: List[Blob] = []

    def _emit_singleton_obj(b: Blob, meets_min: bool, is_ver: bool) -> None:
        if not meets_min:
            if scan_unknown_near:
                unverified_by_label[b.label].append(b)
            return
        blobs.append(b)
        if is_ver:
            verified_by_label[b.label].append(b)
        else:
            emitted_rejected_object_rects.add((b.x, b.y, b.w, b.h))
            if b.pixels >= cfg.min_hazard_pixels:
                accounted_rejected_blobs.append(b)
            elif scan_unknown_near:
                unverified_by_label[b.label].append(b)

    pot_obj_pairs: List[Tuple[int, int]] = []
    for i_c in range(len(raw_obj_cands)):
        bi = raw_obj_cands[i_c][0]
        ti = cfg.lab_thresholds.get(bi.label)
        if ti is None:
            continue
        for j_c in range(i_c + 1, len(raw_obj_cands)):
            bj = raw_obj_cands[j_c][0]
            if bi.label == bj.label:
                continue
            tj = cfg.lab_thresholds.get(bj.label)
            if tj is None or not _lab_ranges_overlap(ti, tj):
                continue
            if _rects_intersect(bi.x, bi.y, bi.w, bi.h, bj.x, bj.y, bj.w, bj.h):
                pot_obj_pairs.append((i_c, j_c))

    if not pot_obj_pairs:
        for b, meets_min, is_ver in raw_obj_cands:
            _emit_singleton_obj(b, meets_min, is_ver)
    else:
        cand_pts_cache: Dict[int, set[Tuple[int, int]]] = {}
        inferred_cand_footprints: set[int] = set()

        def _get_cand_pixels(idx: int) -> set[Tuple[int, int]]:
            cached = cand_pts_cache.get(idx)
            if cached is not None:
                return cached
            b = raw_obj_cands[idx][0]
            pre_pts = precomputed_pts_by_key.get((b.label, b.x, b.y, b.w, b.h, b.pixels))
            if pre_pts is not None:
                pts = set(pre_pts)
                cand_pts_cache[idx] = pts
                return pts
            bx1, by1, bx2, by2 = max(0, b.x), max(0, b.y), min(fw, b.x2), min(fh, b.y2)
            if b.pixels >= b.w * b.h:
                pts = {(xx, yy) for yy in range(by1, by2) for xx in range(bx1, bx2)}
                cand_pts_cache[idx] = pts
                return pts
            thresh_b = cfg.lab_thresholds[b.label]
            siblings = [
                o
                for o in all_raw_by_label.get(b.label, [])
                if o is not b and _rects_intersect(bx1, by1, bx2 - bx1, by2 - by1, o.x, o.y, o.w, o.h)
            ]
            parents = [b] + siblings
            ux1 = max(0, min(p.x for p in parents))
            uy1 = max(0, min(p.y for p in parents))
            ux2 = min(fw, max(p.x2 for p in parents))
            uy2 = min(fh, max(p.y2 for p in parents))
            exp_g = {(p.x, p.y, p.w, p.h, p.pixels) for p in parents}
            b_geom = (b.x, b.y, b.w, b.h, b.pixels)
            pts: set[Tuple[int, int]] = set()
            if get_lab is not None:
                comps = _tier1_find_matching_components(thresh_b, ux1, uy1, ux2, uy2, exp_g)
                for cx, cy, cw, ch, ccnt, cpts in comps:
                    if (cx, cy, cw, ch, ccnt) == b_geom:
                        pts = set(cpts)
                        break
                if not pts:
                    inferred_cand_footprints.add(idx)
                    pts = {
                        (xx, yy)
                        for yy in range(by1, by2)
                        for xx in range(bx1, bx2)
                        if _lab_in_thresh(get_lab(xx, yy), thresh_b)
                    }
            else:
                cid_grid, geoms, _ = _resolve_exact_label_components(b.label, thresh_b, (ux1, uy1, ux2, uy2), exp_g)
                b_cids = {cid for cid, g in enumerate(geoms) if g == b_geom}
                if len(b_cids) != 1:
                    inferred_cand_footprints.add(idx)
                if b_cids:
                    for ly, row_c in enumerate(cid_grid):
                        ay = uy1 + ly
                        for lx, cid in enumerate(row_c):
                            if cid in b_cids:
                                pts.add((ux1 + lx, ay))
                if not pts:
                    for ly, row_c in enumerate(cid_grid):
                        ay = uy1 + ly
                        if not (by1 <= ay < by2):
                            continue
                        for lx, cid in enumerate(row_c):
                            ax = ux1 + lx
                            if cid != -1 and bx1 <= ax < bx2:
                                pts.add((ax, ay))
            if not pts:
                pts = {(xx, yy) for yy in range(by1, by2) for xx in range(bx1, bx2)}
            cand_pts_cache[idx] = pts
            return pts

        dsu_parent = list(range(len(raw_obj_cands)))

        def _dsu_find(x: int) -> int:
            while dsu_parent[x] != x:
                dsu_parent[x] = dsu_parent[dsu_parent[x]]
                x = dsu_parent[x]
            return x

        for i_c, j_c in pot_obj_pairs:
            ri, rj = _dsu_find(i_c), _dsu_find(j_c)
            if ri == rj:
                continue
            if _get_cand_pixels(i_c) & _get_cand_pixels(j_c):
                dsu_parent[rj] = ri

        clusters_by_root: Dict[int, List[int]] = {}
        for idx_c in range(len(raw_obj_cands)):
            root = _dsu_find(idx_c)
            clusters_by_root.setdefault(root, []).append(idx_c)

        for members in clusters_by_root.values():
            if len(members) == 1:
                b, meets_min, is_ver = raw_obj_cands[members[0]]
                _emit_singleton_obj(b, meets_min, is_ver)
                continue

            union_pts: set[Tuple[int, int]] = set()
            for m in members:
                union_pts.update(_get_cand_pixels(m))
            if not union_pts:
                continue
            ux1 = min(p[0] for p in union_pts)
            uy1 = min(p[1] for p in union_pts)
            ux2 = max(p[0] for p in union_pts) + 1
            uy2 = max(p[1] for p in union_pts) + 1
            uw, uh = max(1, ux2 - ux1), max(1, uy2 - uy1)
            ucnt = len(union_pts)
            u_ic = _pts_hit_roi(union_pts, corral_clipped) if scan_unknown_near else None
            u_isw = _pts_hit_roi(union_pts, sweep_clipped) if scan_unknown_near else None

            # Retain cyan hazard priority while always using the FULL connected conflict footprint
            # across every safety region (corral, sweep, drop ROI, home, ramp) with no second
            # minimum-size test on clipped non-cyan tails.
            has_cyan = any(raw_obj_cands[m][0].label == "cyan" for m in members)
            emit_label = "cyan" if has_cyan else "unknown"
            merged_blob = Blob(emit_label, ux1, uy1, uw, uh, ucnt, min(1.0, ucnt / float(uw * uh)), u_ic, u_isw)
            blobs.append(merged_blob)
            emitted_rejected_object_rects.add((ux1, uy1, uw, uh))
            accounted_obj_pts.update(union_pts)
            accounted_obj_boxes.append((ux1, uy1, ux2, uy2))
            cluster_floor_disjoint = all(
                (t_m := cfg.lab_thresholds.get(raw_obj_cands[m][0].label)) is not None
                and not _lab_ranges_overlap(t_m, cfg.arena_floor_lab)
                for m in members
            )
            # An approximate footprint can warn of a conflict, but must not explain away Tier-3 foreground.
            if inferred_cand_footprints.isdisjoint(members):
                accounted_clusters.append((union_pts, (ux1, uy1, ux2, uy2), cluster_floor_disjoint))

    # In NEAR view, detect unexplained non-floor foreground across ALL fixed near ROIs PLUS the dynamic
    # home zone, ramp band, and projected push-corridor geometry, masking ONLY verified connected
    # components (never full AABBs) and emitting tight unexplained bounds (never whole connected surfaces).
    if scan_unknown_near:
        verified_full_objects = [b for lbl in _OBJECT_LABELS for b in verified_by_label[lbl]]
        verified_full_surfaces = [b for lbl in _SURFACE_LABELS for b in verified_by_label[lbl]]

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

        emitted_unknown_idx: Dict[Tuple[int, int, int, int], int] = {}

        def _emit_unknown(unk: Blob) -> None:
            rect = (unk.x, unk.y, unk.w, unk.h)
            if rect in emitted_rejected_object_rects:
                return
            prev_idx = emitted_unknown_idx.get(rect)
            if prev_idx is None:
                emitted_unknown_idx[rect] = len(blobs)
                blobs.append(unk)
            else:
                prev = blobs[prev_idx]
                ic = (
                    (bool(prev.in_corral) or bool(unk.in_corral))
                    if (prev.in_corral is not None or unk.in_corral is not None)
                    else None
                )
                isw = (
                    (bool(prev.in_sweep) or bool(unk.in_sweep))
                    if (prev.in_sweep is not None or unk.in_sweep is not None)
                    else None
                )
                if ic != prev.in_corral or isw != prev.in_sweep or unk.pixels > prev.pixels:
                    px = max(prev.pixels, unk.pixels)
                    blobs[prev_idx] = Blob(
                        "unknown", prev.x, prev.y, prev.w, prev.h, px, min(1.0, px / float(prev.w * prev.h)), ic, isw
                    )

        # Tier 1: Exact per-pixel verified connected-component mask + tight unexplained connected components!
        if get_lab is not None:
            explained = [[False] * fw for _ in range(fh)]
            for ax, ay in accounted_obj_pts:
                if 0 <= ax < fw and 0 <= ay < fh:
                    explained[ay][ax] = True
            comp_pts_by_key: Dict[Tuple[str, int, int, int, int, int], List[Tuple[int, int]]] = dict(
                precomputed_pts_by_key
            )
            for label in (_SURFACE_LABELS + _OBJECT_LABELS):
                vlist = verified_by_label[label]
                if not vlist or label not in cfg.lab_thresholds:
                    continue
                rem_vlist: List[Blob] = []
                for vb in vlist:
                    v_key = (label, vb.x, vb.y, vb.w, vb.h, vb.pixels)
                    pre = comp_pts_by_key.get(v_key)
                    if pre is not None:
                        for px_x, px_y in pre:
                            explained[px_y][px_x] = True
                    else:
                        rem_vlist.append(vb)
                if not rem_vlist:
                    continue
                thresh = cfg.lab_thresholds[label]
                ux1 = max(0, min(b.x for b in rem_vlist))
                uy1 = max(0, min(b.y for b in rem_vlist))
                ux2 = min(fw, max(b.x2 for b in rem_vlist))
                uy2 = min(fh, max(b.y2 for b in rem_vlist))
                expected_v = {(b.x, b.y, b.w, b.h, b.pixels) for b in rem_vlist}
                for min_x, min_y, cw, ch, cnt, comp_pts in _tier1_find_matching_components(
                    thresh, ux1, uy1, ux2, uy2, expected_v
                ):
                    if (min_x, min_y, cw, ch, cnt) in expected_v:
                        comp_pts_by_key[(label, min_x, min_y, cw, ch, cnt)] = comp_pts
                        for px_x, px_y in comp_pts:
                            explained[px_y][px_x] = True

            # Annotate all object/hazard blobs with exact pixel-clipped corral/sweep occupancy,
            # and mark already-emitted rejected object pixels as explained so scan_rois never double-emits them.
            accounted_rejected_keys = {
                (rb.label, rb.x, rb.y, rb.w, rb.h, rb.pixels) for rb in accounted_rejected_blobs
            }
            for idx_b, b in enumerate(blobs):
                if b.label not in _OBJECT_LABELS or (b.in_corral is not None and b.in_sweep is not None):
                    continue
                b_key = (b.label, b.x, b.y, b.w, b.h, b.pixels)
                is_accounted_rej = b_key in accounted_rejected_keys
                if b.pixels >= b.w * b.h:
                    ic = corral_clipped is not None and _rects_intersect(b.x, b.y, b.w, b.h, *corral_clipped)
                    isw = sweep_clipped is not None and _rects_intersect(b.x, b.y, b.w, b.h, *sweep_clipped)
                    if is_accounted_rej:
                        for py in range(max(0, b.y), min(fh, b.y2)):
                            row_e = explained[py]
                            for px_x in range(max(0, b.x), min(fw, b.x2)):
                                row_e[px_x] = True
                else:
                    pts = comp_pts_by_key.get(b_key)
                    if pts is None and b.label in cfg.lab_thresholds:
                        thresh_b = cfg.lab_thresholds[b.label]
                        bx1, by1, bx2, by2 = max(0, b.x), max(0, b.y), min(fw, b.x2), min(fh, b.y2)
                        siblings = [
                            o
                            for o in all_raw_by_label.get(b.label, [])
                            if o is not b and _rects_intersect(bx1, by1, bx2 - bx1, by2 - by1, o.x, o.y, o.w, o.h)
                        ]
                        parents = [b] + siblings
                        ux1 = max(0, min(p.x for p in parents))
                        uy1 = max(0, min(p.y for p in parents))
                        ux2 = min(fw, max(p.x2 for p in parents))
                        uy2 = min(fh, max(p.y2 for p in parents))
                        exp_b = {(p.x, p.y, p.w, p.h, p.pixels) for p in parents}
                        for mn_x, mn_y, cw, ch, cnt, c_pts in _tier1_find_matching_components(
                            thresh_b, ux1, uy1, ux2, uy2, exp_b
                        ):
                            if (mn_x, mn_y, cw, ch, cnt) == (b.x, b.y, b.w, b.h, b.pixels):
                                pts = c_pts
                                comp_pts_by_key[b_key] = c_pts
                                break
                    if pts is not None:
                        ic = _pts_hit_roi(pts, corral_clipped)
                        isw = _pts_hit_roi(pts, sweep_clipped)
                        if is_accounted_rej:
                            for px_x, px_y in pts:
                                explained[px_y][px_x] = True
                    else:
                        ic = corral_clipped is not None and _rects_intersect(b.x, b.y, b.w, b.h, *corral_clipped)
                        isw = sweep_clipped is not None and _rects_intersect(b.x, b.y, b.w, b.h, *sweep_clipped)
                blobs[idx_b] = Blob(b.label, b.x, b.y, b.w, b.h, b.pixels, b.density, ic, isw)

            for roi, merge_small in scan_rois:
                rx, ry, rw, rh = roi
                x1, y1, x2, y2 = rx, ry, rx + rw, ry + rh
                vis_u = [[False] * rw for _ in range(rh)]
                small_comps: List[Tuple[int, int, int, int, int, bool, bool]] = []
                for yy in range(y1, y2):
                    for xx in range(x1, x2):
                        if vis_u[yy - y1][xx - x1] or explained[yy][xx]:
                            continue
                        if _lab_in_thresh(get_lab(xx, yy), cfg.arena_floor_lab):
                            continue
                        stack = [(xx, yy)]
                        vis_u[yy - y1][xx - x1] = True
                        u_pts: List[Tuple[int, int]] = []
                        min_x = max_x = xx
                        min_y = max_y = yy
                        cnt = 0
                        while stack:
                            cx, cy = stack.pop()
                            u_pts.append((cx, cy))
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
                        u_ic = _pts_hit_roi(u_pts, corral_clipped)
                        u_isw = _pts_hit_roi(u_pts, sweep_clipped)
                        if cnt >= cfg.min_hazard_pixels:
                            _emit_unknown(Blob("unknown", min_x, min_y, cw, ch, cnt, cnt / float(cw * ch), u_ic, u_isw))
                        elif merge_small:
                            small_comps.append((min_x, min_y, max_x + 1, max_y + 1, cnt, u_ic, u_isw))
                if merge_small and small_comps:
                    tot = sum(c[4] for c in small_comps)
                    if tot >= cfg.min_hazard_pixels:
                        mx1 = min(c[0] for c in small_comps)
                        my1 = min(c[1] for c in small_comps)
                        mx2 = max(c[2] for c in small_comps)
                        my2 = max(c[3] for c in small_comps)
                        mw, mh = max(1, mx2 - mx1), max(1, my2 - my1)
                        m_ic = any(c[5] for c in small_comps)
                        m_isw = any(c[6] for c in small_comps)
                        _emit_unknown(Blob("unknown", mx1, my1, mw, mh, tot, min(1.0, tot / float(mw * mh)), m_ic, m_isw))
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
            verified_geoms_by_label = {
                lbl: {(v.x, v.y, v.w, v.h, v.pixels) for v in verified_by_label[lbl]}
                for lbl in (_SURFACE_LABELS + _OBJECT_LABELS)
            }
            non_verified_raw_by_label = {
                lbl: [
                    c
                    for c in all_raw_by_label[lbl]
                    if (c.x, c.y, c.w, c.h, c.pixels) not in verified_geoms_by_label[lbl]
                ]
                for lbl in (_SURFACE_LABELS + _OBJECT_LABELS)
            }
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
                            for u in non_verified_raw_by_label[other]
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

            def _actual_unverified_subblobs_in_rect(
                comp: Blob, rect_x: int, rect_y: int, rect_w: int, rect_h: int
            ) -> List[Blob]:
                """Clip an unverified component to a rectangle using ACTUAL component pixels, preserving
                exact parent component identity so disconnected verified (or other) same-label components
                enclosed inside comp's bounding box (e.g. a verified green E arm inside an unverified
                green C's AABB) are never misclassified as unknown."""
                if rect_w <= 0 or rect_h <= 0:
                    return []
                pre_pts = precomputed_pts_by_key.get((comp.label, comp.x, comp.y, comp.w, comp.h, comp.pixels))
                if pre_pts is not None:
                    rx2, ry2 = rect_x + rect_w, rect_y + rect_h
                    clipped = [(x, y) for x, y in pre_pts if rect_x <= x < rx2 and rect_y <= y < ry2]
                    if not clipped:
                        return []
                    mn_x = min(p[0] for p in clipped)
                    mn_y = min(p[1] for p in clipped)
                    mx_x = max(p[0] for p in clipped)
                    mx_y = max(p[1] for p in clipped)
                    bw, bh = mx_x - mn_x + 1, mx_y - mn_y + 1
                    cnt = len(clipped)
                    return [Blob("unknown", mn_x, mn_y, bw, bh, cnt, min(1.0, cnt / float(bw * bh)))]
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
                if not sub_raw:
                    return []
                all_verified_geoms = verified_geoms_by_label.get(comp.label, set())
                all_same_label = all_raw_by_label.get(comp.label, [])
                comp_geom = (comp.x, comp.y, comp.w, comp.h, comp.pixels)

                out_subs: List[Blob] = []
                ambiguous_parents: List[Blob] = []
                needs_exact = False
                for item in sub_raw:
                    sx, sy = int(item.x()), int(item.y())
                    sw, sh = max(1, int(item.w())), max(1, int(item.h()))
                    spx = int(item.pixels())
                    # Impossible for comp to contain this sub-blob -> skip immediately.
                    if not (
                        comp.x <= sx
                        and comp.y <= sy
                        and comp.x2 >= sx + sw
                        and comp.y2 >= sy + sh
                        and comp.pixels >= spx
                    ):
                        continue
                    # Fast path 1: exact match to a full-frame verified component -> exclude!
                    if (sx, sy, sw, sh, spx) in all_verified_geoms:
                        continue
                    # Fast path 2: check if ANY other same-label component (verified or unverified)
                    # could also contain this sub-blob.
                    other_cands = [
                        b
                        for b in all_same_label
                        if b is not comp
                        and b.x <= sx
                        and b.y <= sy
                        and b.x2 >= sx + sw
                        and b.y2 >= sy + sh
                        and b.pixels >= spx
                    ]
                    if not other_cands:
                        out_subs.append(Blob("unknown", sx, sy, sw, sh, spx, min(1.0, spx / float(sw * sh))))
                    else:
                        needs_exact = True
                        ambiguous_parents.extend(other_cands)

                if not needs_exact:
                    return out_subs

                # Exact pixel-level component resolution over union(comp, ambiguous_parents) so
                # nested disconnected same-label components (e.g. unverified C enclosing clipped
                # verified E arm) preserve true parent identity with 100% exactness.
                parents = [comp] + ambiguous_parents
                ux1 = min(rect_x, *(b.x for b in parents))
                uy1 = min(rect_y, *(b.y for b in parents))
                ux2 = max(rect_x + rect_w, *(b.x2 for b in parents))
                uy2 = max(rect_y + rect_h, *(b.y2 for b in parents))
                expected_geoms = {(b.x, b.y, b.w, b.h, b.pixels) for b in parents}
                cid_grid, geoms, used_eight = _resolve_exact_label_components(
                    comp.label, c_thresh, (ux1, uy1, ux2, uy2), expected_geoms
                )
                target_cids = {
                    cid for cid, g in enumerate(geoms) if g == comp_geom and g not in all_verified_geoms
                }
                if not target_cids:
                    return []
                nbrs = (
                    ((-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1))
                    if used_eight
                    else ((-1, 0), (1, 0), (0, -1), (0, 1))
                )
                vis_r = [[False] * rect_w for _ in range(rect_h)]
                out_exact: List[Blob] = []
                for ry in range(rect_h):
                    gy = rect_y + ry - uy1
                    for rx in range(rect_w):
                        if vis_r[ry][rx]:
                            continue
                        gx = rect_x + rx - ux1
                        if cid_grid[gy][gx] not in target_cids:
                            continue
                        vis_r[ry][rx] = True
                        stack = [(rx, ry)]
                        mn_x = mx_x = rx
                        mn_y = mx_y = ry
                        cnt = 0
                        while stack:
                            cx, cy = stack.pop()
                            cnt += 1
                            if cx < mn_x:
                                mn_x = cx
                            if cx > mx_x:
                                mx_x = cx
                            if cy < mn_y:
                                mn_y = cy
                            if cy > mx_y:
                                mx_y = cy
                            for dx, dy in nbrs:
                                nx, ny = cx + dx, cy + dy
                                if 0 <= nx < rect_w and 0 <= ny < rect_h and not vis_r[ny][nx]:
                                    if cid_grid[rect_y + ny - uy1][rect_x + nx - ux1] in target_cids:
                                        vis_r[ny][nx] = True
                                        stack.append((nx, ny))
                        bw = mx_x - mn_x + 1
                        bh = mx_y - mn_y + 1
                        out_exact.append(
                            Blob("unknown", rect_x + mn_x, rect_y + mn_y, bw, bh, cnt, min(1.0, cnt / float(bw * bh)))
                        )
                return out_exact

            def _obj_blob_hits_roi_tier2(b: Blob, target: Optional[Tuple[int, int, int, int]]) -> bool:
                if target is None:
                    return False
                pre_pts = precomputed_pts_by_key.get((b.label, b.x, b.y, b.w, b.h, b.pixels))
                if pre_pts is not None:
                    return _pts_hit_roi(pre_pts, target)
                inter = _rect_intersection(b.x, b.y, b.w, b.h, *target)
                if inter is None:
                    return False
                if b.pixels >= b.w * b.h:
                    return True
                t_b = cfg.lab_thresholds.get(b.label)
                if t_b is None:
                    return True
                ix1, iy1, ix2, iy2 = inter
                hits = _find_blobs_unit_stride(
                    img, [tuple(t_b)], roi=(ix1, iy1, ix2 - ix1, iy2 - iy1), pixels_threshold=1, area_threshold=1, merge=False
                )
                if not hits:
                    return False
                others = [
                    o
                    for o in all_raw_by_label.get(b.label, [])
                    if o is not b and _rects_intersect(ix1, iy1, ix2 - ix1, iy2 - iy1, o.x, o.y, o.w, o.h)
                ]
                if not others:
                    return True
                parents = [b] + others
                ux1 = min(ix1, *(p.x for p in parents))
                uy1 = min(iy1, *(p.y for p in parents))
                ux2 = max(ix2, *(p.x2 for p in parents))
                uy2 = max(iy2, *(p.y2 for p in parents))
                exp_g = {(p.x, p.y, p.w, p.h, p.pixels) for p in parents}
                cid_grid, geoms, _ = _resolve_exact_label_components(b.label, t_b, (ux1, uy1, ux2, uy2), exp_g)
                b_geom = (b.x, b.y, b.w, b.h, b.pixels)
                b_cids = {cid for cid, g in enumerate(geoms) if g == b_geom}
                for yy in range(iy1, iy2):
                    row_c = cid_grid[yy - uy1]
                    for xx in range(ix1, ix2):
                        if row_c[xx - ux1] in b_cids:
                            return True
                return False

            for idx_b, b in enumerate(blobs):
                if b.label not in _OBJECT_LABELS or (b.in_corral is not None and b.in_sweep is not None):
                    continue
                blobs[idx_b] = Blob(
                    b.label,
                    b.x,
                    b.y,
                    b.w,
                    b.h,
                    b.pixels,
                    b.density,
                    _obj_blob_hits_roi_tier2(b, corral_clipped),
                    _obj_blob_hits_roi_tier2(b, sweep_clipped),
                )

            def _fill_unverified_comp_in_grid(
                c: Blob, box: Tuple[int, int, int, int], grid: List[List[bool]], ox: int, oy: int
            ) -> None:
                pre_pts = precomputed_pts_by_key.get((c.label, c.x, c.y, c.w, c.h, c.pixels))
                if pre_pts is not None:
                    bx1, by1, bx2, by2 = box
                    for x, y in pre_pts:
                        if bx1 <= x < bx2 and by1 <= y < by2:
                            grid[y - oy][x - ox] = True
                    return
                c_thresh = cfg.lab_thresholds.get(c.label)
                if c_thresh is None:
                    return
                inter = _rect_intersection(box[0], box[1], box[2] - box[0], box[3] - box[1], c.x, c.y, c.w, c.h)
                if inter is None:
                    return
                ix1, iy1, ix2, iy2 = inter
                others = [
                    o
                    for o in all_raw_by_label.get(c.label, [])
                    if o is not c and _rects_intersect(ix1, iy1, ix2 - ix1, iy2 - iy1, o.x, o.y, o.w, o.h)
                ]
                if not others:
                    _rle_fill_mask(img, [tuple(c_thresh)], inter, grid, ox, oy, True)
                    return
                parents = [c] + others
                ux1 = min(ix1, *(p.x for p in parents))
                uy1 = min(iy1, *(p.y for p in parents))
                ux2 = max(ix2, *(p.x2 for p in parents))
                uy2 = max(iy2, *(p.y2 for p in parents))
                exp_g = {(p.x, p.y, p.w, p.h, p.pixels) for p in parents}
                cid_grid, geoms, _ = _resolve_exact_label_components(c.label, c_thresh, (ux1, uy1, ux2, uy2), exp_g)
                v_geoms = verified_geoms_by_label.get(c.label, set())
                c_geom = (c.x, c.y, c.w, c.h, c.pixels)
                c_cids = {cid for cid, g in enumerate(geoms) if g == c_geom and g not in v_geoms}
                for yy in range(iy1, iy2):
                    row_c = cid_grid[yy - uy1]
                    row_g = grid[yy - oy]
                    for xx in range(ix1, ix2):
                        if row_c[xx - ux1] in c_cids:
                            row_g[xx - ox] = True

            def _clear_component_in_grid(
                comp: Blob, box: Tuple[int, int, int, int], grid: List[List[bool]], ox: int, oy: int
            ) -> None:
                pre_pts = precomputed_pts_by_key.get((comp.label, comp.x, comp.y, comp.w, comp.h, comp.pixels))
                if pre_pts is not None:
                    bx1, by1, bx2, by2 = box
                    for x, y in pre_pts:
                        if bx1 <= x < bx2 and by1 <= y < by2:
                            grid[y - oy][x - ox] = False
                    return
                c_thresh = cfg.lab_thresholds.get(comp.label)
                if c_thresh is None:
                    return
                inter = _rect_intersection(box[0], box[1], box[2] - box[0], box[3] - box[1], comp.x, comp.y, comp.w, comp.h)
                if inter is None:
                    return
                ix1, iy1, ix2, iy2 = inter
                if comp.pixels >= comp.w * comp.h:
                    for yy in range(iy1, iy2):
                        row_g = grid[yy - oy]
                        for xx in range(ix1, ix2):
                            row_g[xx - ox] = False
                    return
                others = [
                    o
                    for o in all_raw_by_label.get(comp.label, [])
                    if o is not comp and _rects_intersect(ix1, iy1, ix2 - ix1, iy2 - iy1, o.x, o.y, o.w, o.h)
                ]
                if not others:
                    _rle_fill_mask(img, [tuple(c_thresh)], inter, grid, ox, oy, False)
                    return
                parents = [comp] + others
                ux1 = min(ix1, *(p.x for p in parents))
                uy1 = min(iy1, *(p.y for p in parents))
                ux2 = max(ix2, *(p.x2 for p in parents))
                uy2 = max(iy2, *(p.y2 for p in parents))
                exp_g = {(p.x, p.y, p.w, p.h, p.pixels) for p in parents}
                cid_grid, geoms, _ = _resolve_exact_label_components(comp.label, c_thresh, (ux1, uy1, ux2, uy2), exp_g)
                c_geom = (comp.x, comp.y, comp.w, comp.h, comp.pixels)
                c_cids = {cid for cid, g in enumerate(geoms) if g == c_geom}
                for yy in range(iy1, iy2):
                    row_c = cid_grid[yy - uy1]
                    row_g = grid[yy - oy]
                    for xx in range(ix1, ix2):
                        if row_c[xx - ux1] in c_cids:
                            row_g[xx - ox] = False

            def _clear_verified_other_in_grid(
                other_label: str, box: Tuple[int, int, int, int], grid: List[List[bool]], ox: int, oy: int
            ) -> None:
                o_thresh = cfg.lab_thresholds.get(other_label)
                if o_thresh is None:
                    return
                v_hit = [
                    v
                    for v in verified_by_label.get(other_label, [])
                    if _rects_intersect(box[0], box[1], box[2] - box[0], box[3] - box[1], v.x, v.y, v.w, v.h)
                ]
                if not v_hit:
                    return
                u_hit = [
                    u
                    for u in non_verified_raw_by_label.get(other_label, [])
                    if _rects_intersect(box[0], box[1], box[2] - box[0], box[3] - box[1], u.x, u.y, u.w, u.h)
                ]
                if not u_hit:
                    for v in v_hit:
                        inter = _rect_intersection(box[0], box[1], box[2] - box[0], box[3] - box[1], v.x, v.y, v.w, v.h)
                        if inter is not None:
                            _rle_fill_mask(img, [tuple(o_thresh)], inter, grid, ox, oy, False)
                    return
                parents = v_hit + u_hit
                ux1 = min(box[0], *(p.x for p in parents))
                uy1 = min(box[1], *(p.y for p in parents))
                ux2 = max(box[2], *(p.x2 for p in parents))
                uy2 = max(box[3], *(p.y2 for p in parents))
                exp_g = {(p.x, p.y, p.w, p.h, p.pixels) for p in parents}
                cid_grid, geoms, _ = _resolve_exact_label_components(other_label, o_thresh, (ux1, uy1, ux2, uy2), exp_g)
                v_geoms = {(v.x, v.y, v.w, v.h, v.pixels) for v in v_hit}
                v_cids = {cid for cid, g in enumerate(geoms) if g in v_geoms}
                for yy in range(box[1], box[3]):
                    row_c = cid_grid[yy - uy1]
                    row_g = grid[yy - oy]
                    for xx in range(box[0], box[2]):
                        if row_c[xx - ux1] in v_cids:
                            row_g[xx - ox] = False

            def _extract_8conn_from_grid(
                grid: List[List[bool]], ox: int, oy: int, merge_small_flag: bool
            ) -> None:
                gh = len(grid)
                gw = len(grid[0]) if gh else 0
                vis_g = [[False] * gw for _ in range(gh)]
                small_g: List[Tuple[int, int, int, int, int, bool, bool]] = []
                for gy in range(gh):
                    row_g = grid[gy]
                    for gx in range(gw):
                        if vis_g[gy][gx] or not row_g[gx]:
                            continue
                        vis_g[gy][gx] = True
                        st_g = [(gx, gy)]
                        pts_g: List[Tuple[int, int]] = []
                        mn_x = mx_x = gx
                        mn_y = mx_y = gy
                        cnt = 0
                        while st_g:
                            cx, cy = st_g.pop()
                            pts_g.append((ox + cx, oy + cy))
                            cnt += 1
                            if cx < mn_x:
                                mn_x = cx
                            if cx > mx_x:
                                mx_x = cx
                            if cy < mn_y:
                                mn_y = cy
                            if cy > mx_y:
                                mx_y = cy
                            for dx, dy in (
                                (-1, -1), (0, -1), (1, -1),
                                (-1, 0), (1, 0),
                                (-1, 1), (0, 1), (1, 1),
                            ):
                                nx, ny = cx + dx, cy + dy
                                if 0 <= nx < gw and 0 <= ny < gh and not vis_g[ny][nx] and grid[ny][nx]:
                                    vis_g[ny][nx] = True
                                    st_g.append((nx, ny))
                        cw = mx_x - mn_x + 1
                        ch = mx_y - mn_y + 1
                        u_ic = _pts_hit_roi(pts_g, corral_clipped)
                        u_isw = _pts_hit_roi(pts_g, sweep_clipped)
                        if cnt >= cfg.min_hazard_pixels:
                            _emit_unknown(
                                Blob("unknown", ox + mn_x, oy + mn_y, cw, ch, cnt, cnt / float(cw * ch), u_ic, u_isw)
                            )
                        elif merge_small_flag:
                            small_g.append((ox + mn_x, oy + mn_y, ox + mx_x + 1, oy + mx_y + 1, cnt, u_ic, u_isw))
                if merge_small_flag and small_g:
                    tot = sum(c[4] for c in small_g)
                    if tot >= cfg.min_hazard_pixels:
                        mx1 = min(c[0] for c in small_g)
                        my1 = min(c[1] for c in small_g)
                        mx2 = max(c[2] for c in small_g)
                        my2 = max(c[3] for c in small_g)
                        mw, mh = max(1, mx2 - mx1), max(1, my2 - my1)
                        m_ic = any(c[5] for c in small_g)
                        m_isw = any(c[6] for c in small_g)
                        _emit_unknown(
                            Blob("unknown", mx1, my1, mw, mh, tot, min(1.0, tot / float(mw * mh)), m_ic, m_isw)
                        )

            for roi, merge_small in scan_rois:
                rx, ry, rw, rh = roi
                roi_img = img.crop(rx, ry, rw, rh)
                matched_mask = roi_img.binary(all_known_thresholds, invert=False, copy=True)
                uncal_raw = _find_blobs_unit_stride(matched_mask, black, pixels_threshold=1, area_threshold=1, merge=False)

                roi_set_b: List[Blob] = []
                seen_unverified_geoms = set()
                for c in unverified_set_b:
                    comp_key = (c.label, c.x, c.y, c.w, c.h, c.pixels)
                    if comp_key in seen_unverified_geoms:
                        continue
                    if not _rects_intersect(rx, ry, rw, rh, c.x, c.y, c.w, c.h):
                        continue
                    seen_unverified_geoms.add(comp_key)
                    roi_set_b.append(c)

                # Detect whether any overlapping custom LAB bands can share pixels inside this ROI
                # (either across two Set-B labels, with an overlapping verified class, or with already
                # emitted conflict/rejected object pixels).
                roi_b_labels = sorted({c.label for c in roi_set_b if c.label in cfg.lab_thresholds})
                has_band_overlap = False
                for i_lbl in range(len(roi_b_labels)):
                    ti = cfg.lab_thresholds[roi_b_labels[i_lbl]]
                    for j_lbl in range(i_lbl + 1, len(roi_b_labels)):
                        if _lab_ranges_overlap(ti, cfg.lab_thresholds[roi_b_labels[j_lbl]]):
                            has_band_overlap = True
                            break
                    if has_band_overlap:
                        break
                overlapping_verified_others: List[str] = []
                for other in (_SURFACE_LABELS + _OBJECT_LABELS):
                    if other not in cfg.lab_thresholds or not verified_by_label[other]:
                        continue
                    to = cfg.lab_thresholds[other]
                    if any(other != lbl and _lab_ranges_overlap(cfg.lab_thresholds[lbl], to) for lbl in roi_b_labels):
                        if any(_rects_intersect(rx, ry, rw, rh, v.x, v.y, v.w, v.h) for v in verified_by_label[other]):
                            overlapping_verified_others.append(other)
                            has_band_overlap = True
                roi_accounted_rej = [
                    rb
                    for rb in accounted_rejected_blobs
                    if _rects_intersect(rx, ry, rw, rh, rb.x, rb.y, rb.w, rb.h)
                    and any(
                        _lab_ranges_overlap(cfg.lab_thresholds[rb.label], cfg.lab_thresholds[lbl])
                        for lbl in roi_b_labels
                        if rb.label in cfg.lab_thresholds
                    )
                ]
                roi_has_accounted_pts = bool(roi_set_b) and any(
                    _rects_intersect(rx, ry, rw, rh, ax1, ay1, ax2 - ax1, ay2 - ay1)
                    for ax1, ay1, ax2, ay2 in accounted_obj_boxes
                )
                if roi_accounted_rej or roi_has_accounted_pts:
                    has_band_overlap = True

                if has_band_overlap:
                    # Exact unique-pixel boolean union across Set A + all active Set-B labels minus
                    # verified overlapping-band pixels and already-emitted conflict/rejected object pixels!
                    cand_boxes: List[Tuple[int, int, int, int]] = []
                    for item in uncal_raw:
                        ix1 = rx + int(item.x())
                        iy1 = ry + int(item.y())
                        cand_boxes.append((ix1, iy1, ix1 + max(1, int(item.w())), iy1 + max(1, int(item.h()))))
                    for c in roi_set_b:
                        inter = _rect_intersection(rx, ry, rw, rh, c.x, c.y, c.w, c.h)
                        if inter is not None:
                            cand_boxes.append(inter)
                    if cand_boxes:
                        bx1 = max(rx, min(b[0] for b in cand_boxes))
                        by1 = max(ry, min(b[1] for b in cand_boxes))
                        bx2 = min(rx + rw, max(b[2] for b in cand_boxes))
                        by2 = min(ry + rh, max(b[3] for b in cand_boxes))
                        if bx2 > bx1 and by2 > by1:
                            unk_grid = [[False] * (bx2 - bx1) for _ in range(by2 - by1)]
                            for item in uncal_raw:
                                mx1 = int(item.x())
                                my1 = int(item.y())
                                mx2 = mx1 + max(1, int(item.w()))
                                my2 = my1 + max(1, int(item.h()))
                                _rle_fill_mask(matched_mask, black, (mx1, my1, mx2, my2), unk_grid, bx1 - rx, by1 - ry, True)
                            for c in roi_set_b:
                                _fill_unverified_comp_in_grid(c, (bx1, by1, bx2, by2), unk_grid, bx1, by1)
                            for other in overlapping_verified_others:
                                _clear_verified_other_in_grid(other, (bx1, by1, bx2, by2), unk_grid, bx1, by1)
                            for rb in roi_accounted_rej:
                                _clear_component_in_grid(rb, (bx1, by1, bx2, by2), unk_grid, bx1, by1)
                            if roi_has_accounted_pts:
                                for ax, ay in accounted_obj_pts:
                                    if bx1 <= ax < bx2 and by1 <= ay < by2:
                                        unk_grid[ay - by1][ax - bx1] = False
                            _extract_8conn_from_grid(unk_grid, bx1, by1, merge_small)
                    continue

                roi_cands: List[Blob] = []
                for item in uncal_raw:
                    w = max(1, int(item.w()))
                    h = max(1, int(item.h()))
                    px = int(item.pixels())
                    roi_cands.append(Blob("unknown", rx + int(item.x()), ry + int(item.y()), w, h, px, px / float(w * h)))
                for c in roi_set_b:
                    inter = _rect_intersection(rx, ry, rw, rh, c.x, c.y, c.w, c.h)
                    if inter is None:
                        continue
                    ix1, iy1, ix2, iy2 = inter
                    iw, ih = max(1, ix2 - ix1), max(1, iy2 - iy1)
                    if ix1 == c.x and iy1 == c.y and iw == c.w and ih == c.h:
                        roi_cands.append(Blob("unknown", c.x, c.y, c.w, c.h, c.pixels, c.density))
                    else:
                        roi_cands.extend(_actual_unverified_subblobs_in_rect(c, ix1, iy1, iw, ih))

                def _unknown_hits_roi_tier2(
                    unk: Blob, target: Optional[Tuple[int, int, int, int]], require_min_comp: bool
                ) -> bool:
                    if target is None:
                        return False
                    if roi == target:
                        return True
                    ci = _rect_intersection(unk.x, unk.y, unk.w, unk.h, *target)
                    if ci is None:
                        return False
                    if unk.pixels >= unk.w * unk.h:
                        return True
                    cx1, cy1, cx2, cy2 = ci
                    has_raw_hit = bool(
                        _find_blobs_unit_stride(
                            matched_mask,
                            black,
                            roi=(cx1 - rx, cy1 - ry, cx2 - cx1, cy2 - cy1),
                            pixels_threshold=1,
                            area_threshold=1,
                            merge=False,
                        )
                    )
                    if not has_raw_hit:
                        for c in roi_set_b:
                            if _rects_intersect(cx1, cy1, cx2 - cx1, cy2 - cy1, c.x, c.y, c.w, c.h):
                                if _actual_unverified_subblobs_in_rect(c, cx1, cy1, cx2 - cx1, cy2 - cy1):
                                    has_raw_hit = True
                                    break
                    if not has_raw_hit:
                        return False
                    # Verify that the hit pixel inside ci actually belongs to unk's connected component
                    # (not a sub-threshold stray speck sitting inside a concave AABB corner).
                    bx1, by1, bx2, by2 = unk.x, unk.y, unk.x2, unk.y2
                    ugrid = [[False] * (bx2 - bx1) for _ in range(by2 - by1)]
                    _rle_fill_mask(
                        matched_mask, black, (bx1 - rx, by1 - ry, bx2 - rx, by2 - ry), ugrid, bx1 - rx, by1 - ry, True
                    )
                    for c in roi_set_b:
                        _fill_unverified_comp_in_grid(c, (bx1, by1, bx2, by2), ugrid, bx1, by1)
                    gh, gw = by2 - by1, bx2 - bx1
                    vis_u = [[False] * gw for _ in range(gh)]
                    for gy in range(gh):
                        for gx in range(gw):
                            if vis_u[gy][gx] or not ugrid[gy][gx]:
                                continue
                            vis_u[gy][gx] = True
                            st_u = [(gx, gy)]
                            cnt = 0
                            hit_ci = False
                            while st_u:
                                px_x, px_y = st_u.pop()
                                cnt += 1
                                gx_abs, gy_abs = bx1 + px_x, by1 + px_y
                                if cx1 <= gx_abs < cx2 and cy1 <= gy_abs < cy2:
                                    hit_ci = True
                                for dx, dy in (
                                    (-1, -1), (0, -1), (1, -1),
                                    (-1, 0), (1, 0),
                                    (-1, 1), (0, 1), (1, 1),
                                ):
                                    nx, ny = px_x + dx, px_y + dy
                                    if 0 <= nx < gw and 0 <= ny < gh and not vis_u[ny][nx] and ugrid[ny][nx]:
                                        vis_u[ny][nx] = True
                                        st_u.append((nx, ny))
                            if hit_ci and (not require_min_comp or cnt >= cfg.min_hazard_pixels):
                                return True
                    return False

                merged_cands = _merge_adjacent_unknowns(roi_cands)
                small_cands: List[Blob] = []
                for unk in merged_cands:
                    if unk.pixels >= cfg.min_hazard_pixels:
                        u_ic = _unknown_hits_roi_tier2(unk, corral_clipped, require_min_comp=True)
                        u_isw = _unknown_hits_roi_tier2(unk, sweep_clipped, require_min_comp=True)
                        _emit_unknown(Blob("unknown", unk.x, unk.y, unk.w, unk.h, unk.pixels, unk.density, u_ic, u_isw))
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
                        m_ic = any(_unknown_hits_roi_tier2(b, corral_clipped, require_min_comp=False) for b in small_cands)
                        m_isw = any(_unknown_hits_roi_tier2(b, sweep_clipped, require_min_comp=False) for b in small_cands)
                        _emit_unknown(
                            Blob("unknown", mx1, my1, mw, mh, tot, min(1.0, tot / float(mw * mh)), m_ic, m_isw)
                        )
            return blobs

        # Tier 3: Fallback for find_blobs-only mocks without pixel access or binary().
        # Always subtract verified/accounted component occupancy BEFORE deduplicating residuals so
        # an unexplained foreground patch sharing an AABB with a concave conflict is never skipped.
        emitted_tier3_residuals: set[Tuple[int, int, int, int]] = set()
        verified_parents = verified_full_surfaces + verified_full_objects

        def _parent_pixels(parent: Blob, clip: Tuple[int, int, int, int]) -> Iterable[Tuple[int, int]]:
            ix1, iy1, ix2, iy2 = clip
            geom = (parent.x, parent.y, parent.w, parent.h, parent.pixels)
            pre_pts = precomputed_pts_by_key.get((parent.label, *geom))
            if pre_pts is not None:
                return ((x, y) for x, y in pre_pts if ix1 <= x < ix2 and iy1 <= y < iy2)
            # Resolve the full parent before clipping: the intersection may contain enclosed siblings.
            box = (parent.x, parent.y, parent.x2, parent.y2)
            expected_geoms = {
                (b.x, b.y, b.w, b.h, b.pixels)
                for b in all_raw_by_label[parent.label]
                if parent.x <= b.x and parent.y <= b.y and b.x2 <= parent.x2 and b.y2 <= parent.y2
            }
            grid, geoms, _ = _resolve_exact_label_components(
                parent.label, cfg.lab_thresholds[parent.label], box, expected_geoms
            )
            parent_ids = [cid for cid, g in enumerate(geoms) if g == geom]
            if len(parent_ids) != 1:
                return ()  # No unique parent proof: leave these pixels unexplained.
            parent_id = parent_ids[0]
            return (
                (x, y) for y in range(iy1, iy2) for x in range(ix1, ix2)
                if grid[y - parent.y][x - parent.x] == parent_id
            )

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
                fx, fy, fg_w, fg_h = int(item.x()), int(item.y()), max(1, int(item.w())), max(1, int(item.h()))
                px = int(item.pixels())
                if px < cfg.min_hazard_pixels:
                    continue
                fg = Blob("unknown", fx, fy, fg_w, fg_h, px, px / float(fg_w * fg_h))
                fg_roi = (fx, fy, fg_w, fg_h)
                if fg_roi in emitted_tier3_residuals:
                    continue

                # Partition ALL non-floor pixels inside fg_roi into pairwise-disjoint connected
                # components so Tier 3 NEVER subtracts pixels belonging to a disconnected component
                # sitting inside a concave fg's AABB (e.g. a green block inside a C-shaped unknown).
                box_fg_comps = _find_blobs_unit_stride(
                    img,
                    [tuple(cfg.arena_floor_lab)],
                    roi=fg_roi,
                    pixels_threshold=1,
                    area_threshold=1,
                    invert=True,
                    merge=False,
                )
                other_fgs: List[Tuple[int, int, int, int, int]] = []
                solid_self_comps: List[Tuple[int, int, int, int]] = []
                matched_self = False
                total_box_fg_px = 0
                for c_item in box_fg_comps:
                    cx, cy = int(c_item.x()), int(c_item.y())
                    cw, ch = max(1, int(c_item.w())), max(1, int(c_item.h()))
                    cpx = int(c_item.pixels())
                    total_box_fg_px += cpx
                    if not merge_fg and not matched_self and (cx, cy, cw, ch, cpx) == (fx, fy, fg_w, fg_h, px):
                        matched_self = True
                        if cpx >= cw * ch:
                            solid_self_comps.append((cx, cy, cx + cw, cy + ch))
                    elif not merge_fg:
                        other_fgs.append((cx, cy, cw, ch, cpx))
                    elif cpx >= cw * ch:
                        solid_self_comps.append((cx, cy, cx + cw, cy + ch))
                membership_uncertain = (not merge_fg and not matched_self) or (merge_fg and total_box_fg_px != px)

                def _proven_sub_on_fg(
                    candidate: Blob, thresh: Optional[Sequence[int]], proven_non_floor: bool = False
                ) -> Optional[Blob]:
                    if membership_uncertain:
                        return None
                    fits_self = (
                        candidate.x >= fx
                        and candidate.y >= fy
                        and candidate.x2 <= fx + fg_w
                        and candidate.y2 <= fy + fg_h
                        and candidate.pixels <= px
                    )
                    if not fits_self:
                        return None
                    # If candidate lies wholly inside a 100% solid non-floor component of fg (zero floor or
                    # other-component pixels exist inside that box), every pixel of candidate belongs to fg.
                    if any(
                        sx1 <= candidate.x and sy1 <= candidate.y and candidate.x2 <= sx2 and candidate.y2 <= sy2
                        for sx1, sy1, sx2, sy2 in solid_self_comps
                    ):
                        return candidate
                    # Connected-component subgraph membership proof (R14 P2-5, R15 P2-4, R16 P2-4, R17 P2-2, R19 P2):
                    # Check floor disjointness BEFORE any early return unless `candidate` already came from an
                    # exact floor-excluded connected component (`precomputed_pts_by_key`).
                    if not proven_non_floor and (thresh is None or _lab_ranges_overlap(thresh, cfg.arena_floor_lab)):
                        return None
                    if not other_fgs:
                        return candidate
                    # Every pixel of single connected component `candidate` is non-floor, so `candidate`
                    # must lie wholly inside ONE connected component of `box_fg_comps` (`fg` or one of
                    # `other_fgs`). A component C can contain `candidate` only if AABB(C) encloses
                    # AABB(candidate) AND C.pixels >= candidate.pixels.
                    enclosing_others = [
                        (ox, oy, ow, oh, opx)
                        for ox, oy, ow, oh, opx in other_fgs
                        if ox <= candidate.x
                        and oy <= candidate.y
                        and ox + ow >= candidate.x2
                        and oy + oh >= candidate.y2
                        and opx >= candidate.pixels
                    ]
                    if not enclosing_others:
                        # `fg` is the UNIQUE connected foreground component that can contain `candidate`!
                        return candidate
                    # Ambiguous: another disconnected foreground component also encloses `candidate`'s AABB
                    # and pixel count; fail closed so a disconnected known block can never mask `fg`.
                    return None

                fx2, fy2 = fx + fg_w, fy + fg_h

                def _clip_parent_on_fg(parent: Blob, emit_label: str) -> List[_ProvenRegion]:
                    inter = _rect_intersection(fx, fy, fg_w, fg_h, parent.x, parent.y, parent.w, parent.h)
                    if inter is None:
                        return []
                    geom = (parent.x, parent.y, parent.w, parent.h, parent.pixels)
                    key = (parent.label, *geom)
                    thresh = cfg.lab_thresholds[parent.label]
                    non_floor = key in proven_non_floor_keys
                    ix1, iy1, ix2, iy2 = inter
                    if parent.pixels == parent.w * parent.h:
                        w, h = ix2 - ix1, iy2 - iy1
                        cand = Blob(emit_label, ix1, iy1, w, h, w * h, 1.0)
                    elif inter == (parent.x, parent.y, parent.x2, parent.y2):
                        cand = Blob(emit_label, *geom, parent.density)
                    else:
                        cand = None
                    if cand is not None:
                        proven = _proven_sub_on_fg(cand, thresh, proven_non_floor=non_floor)
                        return [_ProvenRegion(proven, parent=parent)] if proven is not None else []

                    # Clipping can disconnect parts of the parent. Check foreground membership
                    # for each connected fragment separately.
                    remaining_set = set(_parent_pixels(parent, inter))
                    out_subs: List[_ProvenRegion] = []
                    while remaining_set:
                        seed = remaining_set.pop()
                        st = [seed]
                        mn_x = mx_x = seed[0]
                        mn_y = mx_y = seed[1]
                        points: List[Tuple[int, int]] = []
                        while st:
                            cx, cy = st.pop()
                            points.append((cx, cy))
                            if cx < mn_x:
                                mn_x = cx
                            if cx > mx_x:
                                mx_x = cx
                            if cy < mn_y:
                                mn_y = cy
                            if cy > mx_y:
                                mx_y = cy
                            for nb_pt in ((cx - 1, cy), (cx + 1, cy), (cx, cy - 1), (cx, cy + 1)):
                                if nb_pt in remaining_set:
                                    remaining_set.remove(nb_pt)
                                    st.append(nb_pt)
                        sw, sh = mx_x - mn_x + 1, mx_y - mn_y + 1
                        cnt = len(points)
                        cand = Blob(emit_label, mn_x, mn_y, sw, sh, cnt, cnt / float(sw * sh))
                        proven = _proven_sub_on_fg(cand, thresh, proven_non_floor=non_floor)
                        if proven is not None:
                            out_subs.append(_ProvenRegion(proven, points=points))
                    return out_subs

                roi_verified = [
                    sub for parent in verified_parents
                    for sub in _clip_parent_on_fg(parent, parent.label)
                ]

                roi_accounted: List[_ProvenRegion] = []
                for cl_pts, (ax1, ay1, ax2, ay2), cl_floor_disjoint in accounted_clusters:
                    if membership_uncertain or not _rects_intersect(fx, fy, fg_w, fg_h, ax1, ay1, ax2 - ax1, ay2 - ay1):
                        continue
                    cand_in_fg = [
                        (x, y)
                        for x, y in cl_pts
                        if fx <= x < fx2
                        and fy <= y < fy2
                        and not any(ox <= x < ox + ow and oy <= y < oy + oh for ox, oy, ow, oh, _ in other_fgs)
                    ]
                    if not cand_in_fg:
                        continue
                    if cl_floor_disjoint:
                        in_fg = cand_in_fg
                    else:
                        in_fg = [
                            (x, y)
                            for x, y in cand_in_fg
                            if any(sx1 <= x < sx2 and sy1 <= y < sy2 for sx1, sy1, sx2, sy2 in solid_self_comps)
                        ]
                        rem_pts = [
                            (x, y)
                            for x, y in cand_in_fg
                            if not any(sx1 <= x < sx2 and sy1 <= y < sy2 for sx1, sy1, sx2, sy2 in solid_self_comps)
                        ]
                        if rem_pts:
                            rx1 = min(p[0] for p in rem_pts)
                            ry1 = min(p[1] for p in rem_pts)
                            rx2 = max(p[0] for p in rem_pts) + 1
                            ry2 = max(p[1] for p in rem_pts) + 1
                            fl_grid = [[False] * (rx2 - rx1) for _ in range(ry2 - ry1)]
                            _rle_fill_mask(
                                img, [tuple(cfg.arena_floor_lab)], (rx1, ry1, rx2, ry2), fl_grid, rx1, ry1, True
                            )
                            in_fg.extend((x, y) for x, y in rem_pts if not fl_grid[y - ry1][x - rx1])
                    if in_fg:
                        px1 = min(p[0] for p in in_fg)
                        py1 = min(p[1] for p in in_fg)
                        px2 = max(p[0] for p in in_fg) + 1
                        py2 = max(p[1] for p in in_fg) + 1
                        pw, ph = max(1, px2 - px1), max(1, py2 - py1)
                        pcnt = len(in_fg)
                        region = Blob("_accounted", px1, py1, pw, ph, pcnt, pcnt / float(pw * ph))
                        roi_accounted.append(_ProvenRegion(region, points=in_fg))
                for rb in accounted_rejected_blobs:
                    roi_accounted.extend(_clip_parent_on_fg(rb, "_accounted"))

                regions = roi_verified + roi_accounted
                region_blobs = [r.blob for r in regions]
                # A density estimate can count a patch twice when it lies on a sparse parent's
                # occupied pixels. Use actual occupancy in intersecting sparse/solid or sparse/sparse boxes.
                exact_union = any(
                    (a.pixels < a.w * a.h or b.pixels < b.w * b.h)
                    and _rects_intersect(a.x, a.y, a.w, a.h, b.x, b.y, b.w, b.h)
                    for i, a in enumerate(region_blobs)
                    for b in region_blobs[i + 1:]
                )
                if exact_union:
                    rows = [0] * fg_h
                    for region in regions:
                        b = region.blob
                        box = (b.x, b.y, b.x2, b.y2)
                        if b.pixels == b.w * b.h:
                            row_mask = ((1 << b.w) - 1) << (b.x - fx)
                            for y in range(b.y - fy, b.y2 - fy):
                                rows[y] |= row_mask
                        else:
                            points: Optional[Iterable[Tuple[int, int]]] = region.points
                            if points is None and region.parent is not None:
                                points = _parent_pixels(region.parent, box)
                            for x, y in points or ():
                                rows[y - fy] |= 1 << (x - fx)
                    explained_px = sum(row.bit_count() for row in rows)
                    union_area = _rect_union_area([(b.x, b.y, b.x2, b.y2) for b in region_blobs])
                else:
                    explained_px, union_area = _unique_explained_pixels(fg, region_blobs, cfg=cfg)
                unexplained_px = max(0, fg.pixels - explained_px)
                pixel_coverage = explained_px / float(max(1, fg.pixels))
                area_coverage = union_area / float(max(1, fg.w * fg.h))
                if (
                    membership_uncertain
                    or unexplained_px >= cfg.min_hazard_pixels
                    or pixel_coverage < 0.70
                    or (area_coverage < 0.70 and explained_px < fg.pixels)
                ):
                    emitted_tier3_residuals.add(fg_roi)
                    emit_px = (
                        unexplained_px
                        if (exact_union or roi_accounted) and unexplained_px >= cfg.min_hazard_pixels
                        else fg.pixels
                    )
                    blobs.append(
                        Blob("unknown", fg.x, fg.y, fg.w, fg.h, emit_px, min(1.0, emit_px / float(fg.w * fg.h)))
                    )

    return blobs
