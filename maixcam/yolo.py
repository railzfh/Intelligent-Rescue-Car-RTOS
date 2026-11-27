"""YOLO11/YOLOv8/YOLOv5 + lightweight LAB hybrid extractor for MaixCAM (SG2002).

Separates pure Python conflict resolution, box-to-Blob conversion, dual-buffer frame
alignment, and hybrid LAB fusion (fully testable on desktop Python) from the thin
MaixPy ``maix.nn`` NPU wrapper.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, List, Optional, Sequence, Tuple

from .config import RescueVisionConfig
from .vision import (
    Blob,
    _find_blobs_unit_stride,
    _merge_adjacent_unknowns,
    _min_area_for_label,
    _rect_intersection,
    _rects_intersect,
    classify_unknown_foreground,
    extract_blobs_from_maix_image,
)

_SURFACE_LABELS = ("red_zone", "blue_zone", "purple_ramp")
_BLOCK_LABELS = ("green", "black", "orange", "cyan")
_CONFLICT_LABELS = ("green", "black", "orange", "cyan", "unknown")


@dataclass(frozen=True)
class YoloDetection:
    """Normalized 2D detection in frame pixel coordinates."""

    x: int
    y: int
    w: int
    h: int
    label: str
    score: float
    class_id: int = -1
    raw_label: str = ""

    @property
    def x2(self) -> int:
        return self.x + self.w

    @property
    def y2(self) -> int:
        return self.y + self.h

    @property
    def area(self) -> int:
        return max(0, self.w) * max(0, self.h)

    @property
    def cx(self) -> float:
        return self.x + 0.5 * self.w

    @property
    def cy(self) -> float:
        return self.y + 0.5 * self.h


def _read_attr(obj: Any, name: str, default: Any = 0) -> Any:
    val = getattr(obj, name, default)
    return val() if callable(val) else val


def box_iou(ax: int, ay: int, aw: int, ah: int, bx: int, by: int, bw: int, bh: int) -> float:
    inter = _rect_intersection(ax, ay, aw, ah, bx, by, bw, bh)
    if inter is None:
        return 0.0
    ia = (inter[2] - inter[0]) * (inter[3] - inter[1])
    union = max(1, aw * ah + bw * bh - ia)
    return ia / float(union)


def normalize_yolo_label(raw_label: str, cfg: RescueVisionConfig) -> Optional[str]:
    key = raw_label.strip().lower().replace("-", "_").replace(" ", "_")
    mapped = cfg.yolo_label_map.get(key)
    if mapped is not None:
        return mapped
    return "unknown" if cfg.yolo_unknown_unmapped else None


def min_yolo_detect_conf(cfg: RescueVisionConfig) -> float:
    return min(
        cfg.yolo_conf_th,
        cfg.yolo_hazard_conf_th,
        cfg.yolo_opponent_conf_th,
        cfg.yolo_surface_conf_th,
    )


def threshold_for_label(label: str, cfg: RescueVisionConfig) -> float:
    if label in ("cyan", "unknown"):
        return cfg.yolo_hazard_conf_th
    if label == "opponent":
        return cfg.yolo_opponent_conf_th
    if label in _SURFACE_LABELS:
        return cfg.yolo_surface_conf_th
    return cfg.yolo_conf_th


def resolve_cross_class_conflicts(
    detections: Sequence[YoloDetection],
    cfg: RescueVisionConfig,
) -> List[YoloDetection]:
    """Filter per-class confidence thresholds and enforce fail-closed cross-class rules.

    Safety rules:
      1. Cyan hazard detections use the lower ``yolo_hazard_conf_th`` and NEVER lose
         to an overlapping green/black/orange box (IoU >= ``yolo_cross_class_iou_th``).
      2. If two different non-cyan block classes overlap strongly with nearly tied scores
         (margin < ``yolo_ambiguous_score_margin``), emit an ``unknown`` obstacle instead
         of guessing.
      3. Surfaces (zones/ramp) and opponent detections are resolved independently so
         blocks sitting on safe zones or purple ramps are never suppressed.
    """
    filtered: List[YoloDetection] = []
    for det in detections:
        if det.w <= 0 or det.h <= 0:
            continue
        if det.score < threshold_for_label(det.label, cfg):
            continue
        filtered.append(det)

    passthrough = [d for d in filtered if d.label not in _CONFLICT_LABELS]
    cyans = [d for d in filtered if d.label == "cyan"]
    unknowns = [d for d in filtered if d.label == "unknown"]
    non_cyan_blocks = [d for d in filtered if d.label in ("green", "black", "orange")]

    # Rule 1: Cyan hazard always overrides overlapping non-cyan block boxes.
    surviving_blocks: List[YoloDetection] = []
    for blk in non_cyan_blocks:
        if any(
            box_iou(blk.x, blk.y, blk.w, blk.h, cy.x, cy.y, cy.w, cy.h) >= cfg.yolo_cross_class_iou_th
            for cy in cyans
        ):
            continue
        surviving_blocks.append(blk)

    # Rule 2: Resolve cross-class overlaps among green/black/orange.
    ordered = sorted(surviving_blocks, key=lambda d: d.score, reverse=True)
    consumed = [False] * len(ordered)
    resolved_blocks: List[YoloDetection] = []
    resolved_unknowns: List[YoloDetection] = list(unknowns)

    for i, a in enumerate(ordered):
        if consumed[i]:
            continue
        ambiguous_group = [a]
        for j in range(i + 1, len(ordered)):
            if consumed[j]:
                continue
            b = ordered[j]
            if b.label == a.label:
                continue
            if box_iou(a.x, a.y, a.w, a.h, b.x, b.y, b.w, b.h) >= cfg.yolo_cross_class_iou_th:
                consumed[j] = True
                if (a.score - b.score) < cfg.yolo_ambiguous_score_margin:
                    ambiguous_group.append(b)
        consumed[i] = True
        if len(ambiguous_group) > 1:
            ux1 = min(d.x for d in ambiguous_group)
            uy1 = min(d.y for d in ambiguous_group)
            ux2 = max(d.x2 for d in ambiguous_group)
            uy2 = max(d.y2 for d in ambiguous_group)
            resolved_unknowns.append(
                YoloDetection(
                    x=ux1,
                    y=uy1,
                    w=max(1, ux2 - ux1),
                    h=max(1, uy2 - uy1),
                    label="unknown",
                    score=max(d.score for d in ambiguous_group),
                    raw_label="ambiguous:" + "/".join(sorted({d.label for d in ambiguous_group})),
                )
            )
        else:
            resolved_blocks.append(a)

    return passthrough + cyans + resolved_blocks + resolved_unknowns


def _box_in_corral(x: int, y: int, w: int, h: int, label: str, roi: Sequence[int]) -> bool:
    inter = _rect_intersection(x, y, w, h, roi[0], roi[1], roi[2], roi[3])
    if inter is None:
        return False
    # Hazards & unknowns fail closed on any overlap with the corral ROI.
    if label in ("cyan", "unknown"):
        return True
    cx = x + 0.5 * w
    cy = y + 0.5 * h
    if roi[0] <= cx < roi[0] + roi[2] and roi[1] <= cy < roi[1] + roi[3]:
        return True
    inter_area = (inter[2] - inter[0]) * (inter[3] - inter[1])
    min_overlap = min(25, max(1, int(round(0.15 * w * h))))
    return inter_area >= min_overlap


def detections_to_blobs(
    detections: Sequence[YoloDetection],
    cfg: RescueVisionConfig,
) -> List[Blob]:
    """Clamp YOLO boxes to frame bounds and convert to canonical ``Blob`` objects."""
    fw, fh = cfg.frame_width, cfg.frame_height
    corral_roi = cfg.near_corral_roi
    sweep_roi = cfg.near_sweep_roi
    blobs: List[Blob] = []

    for det in detections:
        x1 = max(0, min(fw, int(det.x)))
        y1 = max(0, min(fh, int(det.y)))
        x2 = max(0, min(fw, int(det.x + det.w)))
        y2 = max(0, min(fh, int(det.y + det.h)))
        w = x2 - x1
        h = y2 - y1
        if w <= 0 or h <= 0:
            continue
        area = w * h
        in_corral: Optional[bool] = None
        in_sweep: Optional[bool] = None
        if det.label in _CONFLICT_LABELS:
            in_corral = _box_in_corral(x1, y1, w, h, det.label, corral_roi)
            in_sweep = _rects_intersect(x1, y1, w, h, sweep_roi[0], sweep_roi[1], sweep_roi[2], sweep_roi[3])
        blobs.append(
            Blob(
                label=det.label,
                x=x1,
                y=y1,
                w=w,
                h=h,
                pixels=area,
                density=1.0,
                in_corral=in_corral,
                in_sweep=in_sweep,
                from_yolo=True,
                score=float(det.score),
            )
        )
    return blobs


class MaixYoloAdapter:
    """Thin adapter around MaixPy ``maix.nn.YOLO11`` / ``YOLOv8`` / ``YOLO5``."""

    def __init__(
        self,
        model_path: str,
        arch: str = "yolo11",
        dual_buff: bool = False,
        detector: Any = None,
    ) -> None:
        self.model_path = model_path
        self.arch = arch
        self.dual_buff = bool(dual_buff)
        if detector is not None:
            self._detector = detector
        else:
            from maix import nn

            cls_map = {
                "yolo11": "YOLO11",
                "yolov8": "YOLOv8",
                "yolo5": "YOLO5",
            }
            cls_name = cls_map.get(arch)
            if cls_name is None or not hasattr(nn, cls_name):
                raise RuntimeError(f"MaixPy maix.nn does not provide {cls_name!r} for arch {arch!r}")
            model_file = Path(model_path)
            if not model_file.exists():
                raise FileNotFoundError(f"YOLO model file not found: {model_path}")
            detector_cls = getattr(nn, cls_name)
            self._detector = detector_cls(model=str(model_file), dual_buff=self.dual_buff)

    @property
    def labels(self) -> Sequence[str]:
        return getattr(self._detector, "labels", ())

    def input_width(self) -> Optional[int]:
        fn = getattr(self._detector, "input_width", None)
        return int(fn()) if callable(fn) else None

    def input_height(self) -> Optional[int]:
        fn = getattr(self._detector, "input_height", None)
        return int(fn()) if callable(fn) else None

    def input_format(self) -> Any:
        fn = getattr(self._detector, "input_format", None)
        return fn() if callable(fn) else None

    def detect(self, img: Any, cfg: RescueVisionConfig) -> List[YoloDetection]:
        raw_objs = self._detector.detect(
            img,
            conf_th=min_yolo_detect_conf(cfg),
            iou_th=cfg.yolo_iou_th,
        )
        labels = self.labels
        out: List[YoloDetection] = []
        for obj in raw_objs or ():
            cid = int(_read_attr(obj, "class_id", -1))
            raw_lbl = str(_read_attr(obj, "label", ""))
            if not raw_lbl and 0 <= cid < len(labels):
                raw_lbl = str(labels[cid])
            mapped = normalize_yolo_label(raw_lbl, cfg)
            if mapped is None:
                continue
            out.append(
                YoloDetection(
                    x=int(_read_attr(obj, "x", 0)),
                    y=int(_read_attr(obj, "y", 0)),
                    w=int(_read_attr(obj, "w", 0)),
                    h=int(_read_attr(obj, "h", 0)),
                    label=mapped,
                    score=float(_read_attr(obj, "score", 0.0)),
                    class_id=cid,
                    raw_label=raw_lbl,
                )
            )
        return out


def extract_lab_surfaces(
    img: Any,
    cfg: RescueVisionConfig,
    labels: Sequence[str] = _SURFACE_LABELS,
) -> List[Blob]:
    """Fast C++ LAB extraction for safe zones and purple ramp (merge=False keeps own/opp separate)."""
    surfaces: List[Blob] = []
    for label in labels:
        thresh = cfg.lab_thresholds.get(label)
        if not thresh:
            continue
        min_px = _min_area_for_label(label, cfg)
        found = _find_blobs_unit_stride(
            img,
            [tuple(thresh)],
            pixels_threshold=min_px,
            area_threshold=min_px,
            merge=False,
        )
        for item in found:
            bx = int(item.x())
            by = int(item.y())
            bw = max(1, int(item.w()))
            bh = max(1, int(item.h()))
            px = int(item.pixels())
            if px < min_px:
                continue
            surfaces.append(Blob(label, bx, by, bw, bh, px, min(1.0, px / float(bw * bh))))
    return surfaces


def extract_lab_cyan_backup(
    img: Any,
    cfg: RescueVisionConfig,
    existing_cyan: Sequence[Blob],
) -> List[Blob]:
    """Near-view LAB cyan hazard safety net (OR-ed with YOLO cyan detections)."""
    cyan_thresh = cfg.lab_thresholds.get("cyan")
    if not cyan_thresh:
        return []
    corral_roi = cfg.near_corral_roi
    sweep_roi = cfg.near_sweep_roi
    scan_rois = (
        tuple(cfg.near_corral_roi),
        tuple(cfg.near_sweep_roi),
        tuple(cfg.near_drop_roi),
    )
    added: List[Blob] = []
    known_cyan = list(existing_cyan)
    for roi in scan_rois:
        found = _find_blobs_unit_stride(
            img,
            [tuple(cyan_thresh)],
            roi=roi,
            pixels_threshold=cfg.min_hazard_pixels,
            area_threshold=cfg.min_hazard_pixels,
            merge=False,
        )
        for item in found:
            bx = int(item.x())
            by = int(item.y())
            bw = max(1, int(item.w()))
            bh = max(1, int(item.h()))
            px = int(item.pixels())
            if px < cfg.min_hazard_pixels:
                continue
            # Avoid double-counting a cyan block already detected by YOLO or an earlier ROI.
            if any(box_iou(bx, by, bw, bh, kb.x, kb.y, kb.w, kb.h) >= 0.35 for kb in known_cyan):
                continue
            blob = Blob(
                label="cyan",
                x=bx,
                y=by,
                w=bw,
                h=bh,
                pixels=px,
                density=min(1.0, px / float(bw * bh)),
                in_corral=_rects_intersect(
                    bx, by, bw, bh, corral_roi[0], corral_roi[1], corral_roi[2], corral_roi[3]
                ),
                in_sweep=_rects_intersect(
                    bx, by, bw, bh, sweep_roi[0], sweep_roi[1], sweep_roi[2], sweep_roi[3]
                ),
                from_yolo=False,
            )
            added.append(blob)
            known_cyan.append(blob)
    return added


def extract_lab_unknown_backup(
    img: Any,
    cfg: RescueVisionConfig,
    known_objects: Sequence[Blob],
    surface_blobs: Sequence[Blob],
) -> List[Blob]:
    """Fast inverted-floor near-ROI check for obstacles YOLO was not trained on."""
    raw_fg: List[Blob] = []
    for roi in (tuple(cfg.near_corral_roi), tuple(cfg.near_sweep_roi)):
        found = _find_blobs_unit_stride(
            img,
            [tuple(cfg.arena_floor_lab)],
            roi=roi,
            pixels_threshold=cfg.min_hazard_pixels,
            area_threshold=cfg.min_hazard_pixels,
            invert=True,
            merge=False,
        )
        for item in found:
            fx = int(item.x())
            fy = int(item.y())
            fw = max(1, int(item.w()))
            fh = max(1, int(item.h()))
            px = int(item.pixels())
            if px >= cfg.min_hazard_pixels:
                raw_fg.append(Blob("unknown", fx, fy, fw, fh, px, min(1.0, px / float(fw * fh))))
    if not raw_fg:
        return []
    unknowns = classify_unknown_foreground(
        fg_blobs=raw_fg,
        known_blobs=known_objects,
        min_pixels=cfg.min_hazard_pixels,
        cfg=cfg,
        surface_blobs=surface_blobs,
    )
    if not unknowns:
        return []
    merged = _merge_adjacent_unknowns(unknowns)
    corral_roi = cfg.near_corral_roi
    sweep_roi = cfg.near_sweep_roi
    return [
        Blob(
            label="unknown",
            x=u.x,
            y=u.y,
            w=u.w,
            h=u.h,
            pixels=u.pixels,
            density=u.density,
            in_corral=_rects_intersect(
                u.x, u.y, u.w, u.h, corral_roi[0], corral_roi[1], corral_roi[2], corral_roi[3]
            ),
            in_sweep=_rects_intersect(
                u.x, u.y, u.w, u.h, sweep_roi[0], sweep_roi[1], sweep_roi[2], sweep_roi[3]
            ),
            from_yolo=False,
        )
        for u in merged
    ]


class HybridVisionExtractor:
    """Coordinates YOLO NPU detection, lightweight LAB safety layers, and dual_buff alignment."""

    def __init__(
        self,
        cfg: RescueVisionConfig,
        detector: Optional[MaixYoloAdapter] = None,
        load_error: Optional[str] = None,
        lab_extractor: Any = None,
    ) -> None:
        self.cfg = cfg
        self.detector = detector
        self.load_error = load_error
        self._lab_extractor = lab_extractor or extract_blobs_from_maix_image
        self._requested_backend = cfg.vision_backend
        self._prev_img: Any = None
        self._active_backend = self._resolve_backend(cfg.vision_backend)

    def _resolve_backend(self, requested: str) -> str:
        if requested not in ("hybrid", "yolo", "lab"):
            raise ValueError(f"unsupported vision backend: {requested!r}")
        if requested == "lab":
            return "lab"
        if self.detector is not None:
            return requested
        if self.cfg.yolo_fallback_to_lab:
            return "lab"
        raise RuntimeError(self.load_error or "YOLO detector unavailable and fallback disabled")

    @property
    def active_backend(self) -> str:
        return self._active_backend

    @property
    def requested_backend(self) -> str:
        return self._requested_backend

    @property
    def fallback_active(self) -> bool:
        return self._requested_backend != "lab" and self._active_backend == "lab"

    @property
    def uses_yolo(self) -> bool:
        return self._active_backend in ("hybrid", "yolo") and self.detector is not None

    @property
    def uses_dual_buff(self) -> bool:
        return self.uses_yolo and bool(self.detector is not None and self.detector.dual_buff)

    @property
    def pipeline_delay_frames(self) -> int:
        return 1 if self.uses_dual_buff else 0

    @property
    def status_tag(self) -> str:
        if self.fallback_active:
            return "LAB(FB)"
        return {"hybrid": "HYB", "yolo": "YOLO", "lab": "LAB"}[self._active_backend]

    def reset_pipeline(self) -> None:
        self._prev_img = None

    def set_backend(self, requested: str) -> str:
        self._requested_backend = requested
        next_backend = self._resolve_backend(requested)
        if next_backend != self._active_backend:
            self._active_backend = next_backend
            self.reset_pipeline()
        return self._active_backend

    def extract(
        self,
        img: Any,
        scan_unknown_near: bool = False,
    ) -> Tuple[List[Blob], Any, bool]:
        """Extract canonical ``Blob`` objects for the current frame.

        Returns ``(blobs, aligned_img, frame_ready)`` where ``aligned_img`` is the exact
        image that ``blobs`` correspond to (``prev_img`` under ``dual_buff=True``, else ``img``).
        Always runs ``detector.detect(img)`` when YOLO is active so settle drains genuinely
        flush the internal NPU pipeline buffer without dropping valid observations during
        gate-settle waits.
        """
        if not self.uses_yolo or self.detector is None:
            self._prev_img = None
            return self._lab_extractor(img, self.cfg, scan_unknown_near=scan_unknown_near), img, True

        raw_dets = self.detector.detect(img, self.cfg)
        if self.uses_dual_buff:
            aligned_img = self._prev_img
            self._prev_img = img
            if aligned_img is None:
                # Cold-start priming frame: result belongs to no valid frame yet.
                return [], img, False
        else:
            aligned_img = img

        resolved = resolve_cross_class_conflicts(raw_dets, self.cfg)
        yolo_blobs = detections_to_blobs(resolved, self.cfg)

        yolo_surface_labels = {b.label for b in yolo_blobs if b.label in _SURFACE_LABELS}
        if self.cfg.yolo_use_lab_surfaces:
            # Prefer pixel-accurate LAB boundaries for red/blue safe zones and purple ramp.
            yolo_blobs = [b for b in yolo_blobs if b.label not in _SURFACE_LABELS]
            surfaces = extract_lab_surfaces(aligned_img, self.cfg, _SURFACE_LABELS)
        else:
            missing = [lbl for lbl in _SURFACE_LABELS if lbl not in yolo_surface_labels]
            surfaces = extract_lab_surfaces(aligned_img, self.cfg, missing) if missing else []

        blobs: List[Blob] = list(yolo_blobs) + list(surfaces)

        if self._active_backend == "hybrid" and scan_unknown_near:
            existing_cyan = [b for b in blobs if b.label == "cyan"]
            if self.cfg.hybrid_lab_cyan_backup:
                blobs.extend(extract_lab_cyan_backup(aligned_img, self.cfg, existing_cyan))
            if self.cfg.hybrid_lab_unknown_backup:
                known_objects = [b for b in blobs if b.label in _BLOCK_LABELS]
                all_surfaces = [b for b in blobs if b.label in _SURFACE_LABELS]
                blobs.extend(extract_lab_unknown_backup(aligned_img, self.cfg, known_objects, all_surfaces))

        return blobs, aligned_img, True


def build_hybrid_extractor(
    cfg: RescueVisionConfig,
    detector: Optional[MaixYoloAdapter] = None,
    lab_extractor: Any = None,
) -> HybridVisionExtractor:
    """Create a ``HybridVisionExtractor``, gracefully handling missing/unloadable models."""
    if detector is not None:
        return HybridVisionExtractor(cfg, detector=detector, lab_extractor=lab_extractor)
    if cfg.vision_backend == "lab":
        return HybridVisionExtractor(cfg, detector=None, lab_extractor=lab_extractor)
    try:
        adapter = MaixYoloAdapter(
            model_path=cfg.yolo_model_path,
            arch=cfg.yolo_arch,
            dual_buff=cfg.yolo_dual_buff,
        )
        return HybridVisionExtractor(cfg, detector=adapter, lab_extractor=lab_extractor)
    except Exception as exc:
        if cfg.yolo_fallback_to_lab:
            return HybridVisionExtractor(cfg, detector=None, load_error=str(exc), lab_extractor=lab_extractor)
        raise
