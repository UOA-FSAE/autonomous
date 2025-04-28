from typing import List, Tuple, Dict, Any, Optional
import numpy as np
import matplotlib.pyplot as plt
import math

# ----------------------------------------------------------------------------------------------------------------------------------------------------------
def _fit_circle_algebraic(x: np.ndarray, y: np.ndarray) -> Tuple[float, float, float]:
    """
    Fit a circle x^2 + y^2 + A x + B y + C = 0
    to points (x, y) via linear least squares.

    Returns (cx, cy, R).
    """
    # build design matrix
    A_mat = np.column_stack([x, y, np.ones_like(x)])
    b = -(x**2 + y**2)
    # solve for [A_coef, B_coef, C_coef]
    coef, *_ = np.linalg.lstsq(A_mat, b, rcond=None)
    A_coef, B_coef, C_coef = coef

    cx = -A_coef / 2
    cy = -B_coef / 2
    # compute radius with numeric-clamp to avoid negative radicand
    radicand = cx**2 + cy**2 - C_coef
    if radicand < 0:
        # small negative due to noise: clamp to zero
        if radicand > -1e-6:
            radicand = 0.0
        else:
            # truly degenerate
            return np.nan, np.nan, np.inf
    R = np.sqrt(radicand)
    if not np.isfinite(R):
        return np.nan, np.nan, np.inf
    return cx, cy, R


def segment_centerline_and_fit_arcs(
    centre_pts: List[Tuple[float, float]],
    window_length: int,
    step_size: int
) -> List[Dict[str, Any]]:
    """
    Divide the sampled track centreline into small, overlapping segments and fit a geometric primitive
    (circle/arc) to each one so that later stages can decide whether the segment behaves like a curve or a straight.
    """
    pts = np.asarray(centre_pts)
    n_pts = len(pts)
    fits: List[Dict[str, Any]] = []

    # slide window
    for i in range(0, n_pts - window_length + 1, step_size):
        seg = pts[i : i + window_length]
        x, y = seg[:, 0], seg[:, 1]

        # fit circle
        try:
            cx, cy, R = _fit_circle_algebraic(x, y)
        except Exception:
            cx, cy, R = np.nan, np.nan, np.inf

        # compute distances to circle centre and RMS error
        d = np.sqrt((x - cx)**2 + (y - cy)**2)
        rms_err = float(np.sqrt(np.mean((d - R)**2)))

        # compute subtended angle by summing incremental angle changes (captures >180° correctly)
        thetas = np.arctan2(y - cy, x - cx)
        dtheta = np.diff(thetas)
        # wrap to [-pi, +pi]
        dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi
        arc_angle = float(abs(dtheta.sum()))

        fits.append({
            'idx_start': i,
            'idx_end':   i + window_length - 1,
            'R':         float(R),
            'centre':    (float(cx), float(cy)),
            'arc_angle': arc_angle,
            'rms_err':   rms_err,
        })

    return fits


def classify_arcs_vs_straights(
    fits: List[Dict[str, Any]],
    max_rms_err: float,
    min_arc_angle: float,
    max_R_for_corner: Optional[float] = None
) -> List[Dict[str, Any]]:
    """
    Decide, for every tested window, whether its geometry behaves like a corner (arc) or a straight segment.
    """
    for fit in fits:
        R = fit.get('R', float('inf'))
        arc_angle = fit.get('arc_angle', 0.0)
        rms_err = fit.get('rms_err', float('inf'))

        # Basic corner criteria: good fit error and enough curvature (min_arc_angle in radians)
        is_corner = (rms_err <= max_rms_err) and (arc_angle >= min_arc_angle)

        # If a maximum radius is specified, enforce it
        if max_R_for_corner is not None:
            is_corner = is_corner and (R <= max_R_for_corner)

        fit['is_corner'] = bool(is_corner)

    return fits


def consolidate_segments(
    fits: List[Dict[str, Any]],
    min_straight_len: int
) -> List[Dict[str, Any]]:
    # (unchanged)
    if not fits:
        return []
    runs = []
    curr_label = fits[0]['is_corner']
    run_start = fits[0]['idx_start']
    run_end = fits[0]['idx_end']
    for fit in fits[1:]:
        if fit['is_corner'] == curr_label:
            run_end = max(run_end, fit['idx_end'])
        else:
            runs.append((curr_label, run_start, run_end))
            curr_label = fit['is_corner']
            run_start = fit['idx_start']
            run_end = fit['idx_end']
    runs.append((curr_label, run_start, run_end))
    boundaries = []
    for i in range(len(runs) - 1):
        _, _, end_i = runs[i]
        _, start_j, _ = runs[i + 1]
        b = (end_i + start_j) // 2
        boundaries.append(b)
    segments = []
    for i, (label, r_start, r_end) in enumerate(runs):
        if i == 0:
            seg_start = r_start
            seg_end = boundaries[0] if boundaries else r_end
        elif i == len(runs) - 1:
            seg_start = boundaries[-1] + 1
            seg_end = r_end
        else:
            seg_start = boundaries[i - 1] + 1
            seg_end = boundaries[i]
        segments.append({
            'type': 'corner' if label else 'straight',
            'idx_start': seg_start,
            'idx_end': seg_end
        })
    # merge short boundary straights (unchanged)
    if len(segments) > 1:
        first = segments[0]
        if first['type'] == 'straight':
            length = first['idx_end'] - first['idx_start'] + 1
            if length < min_straight_len:
                segments[1]['idx_start'] = first['idx_start']
                segments.pop(0)
        last = segments[-1]
        if last['type'] == 'straight':
            length = last['idx_end'] - last['idx_start'] + 1
            if length < min_straight_len:
                segments[-2]['idx_end'] = last['idx_end']
                segments.pop(-1)
    return segments


def characterize_corners(
    segments: List[Dict[str, Any]],
    centre_pts: List[Tuple[float, float]]
) -> List[Dict[str, Any]]:
    # (unchanged) ...
    corners: List[Dict[str, Any]] = []
    for seg in segments:
        if seg.get('type') != 'corner':
            continue
        pts = np.array(centre_pts[seg['idx_start']: seg['idx_end'] + 1])
        x, y = pts[:, 0], pts[:, 1]
        cx, cy, R = _fit_circle_algebraic(x, y)
        thetas = np.arctan2(y - cy, x - cx)
        dtheta = np.diff(thetas)
        dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi
        arc_angle = abs(dtheta.sum())
        delta = dtheta.sum()
        turn_dir = 'L' if delta > 0 else 'R'
        corners.append({
            'idx_start': seg['idx_start'],
            'idx_end':   seg['idx_end'],
            'R':         float(R),
            'arc_angle': float(arc_angle),
            'turn_dir':  turn_dir
        })
    return corners


def extract_corner_metrics(
    corners: List[Dict[str, Any]],
    centre_pts: List[Tuple[float, float]]
) -> List[Dict[str, Any]]:
    # (unchanged)
    metrics = []
    n_corners = len(corners)
    for i, c in enumerate(corners):
        start, end = c['idx_start'], c['idx_end']
        theta = abs(c['arc_angle']) * 180.0 / math.pi
        L = sum(math.hypot(centre_pts[k+1][0] - centre_pts[k][0], centre_pts[k+1][1] - centre_pts[k][1])
                for k in range(start, end))
        if i < n_corners - 1:
            next_start = corners[i+1]['idx_start']
            next_gap = math.hypot(centre_pts[next_start][0] - centre_pts[end][0],
                                  centre_pts[next_start][1] - centre_pts[end][1])
        else:
            next_gap = None
        enriched = dict(c)
        enriched.update({'theta': theta, 'L': L, 'next_gap': next_gap})
        metrics.append(enriched)
    return metrics


def classify_corner_severity(
    corner_metrics: List[Dict[str, Any]],
    theta_hairpin: float,
    R_hairpin: float,
    theta_medium: float,
    R_medium: float
) -> List[Dict[str, Any]]:
    # (unchanged)
    classified = []
    for cm in corner_metrics:
        theta = cm.get('theta', 0.0)
        R = cm.get('R', float('inf'))
        if theta >= theta_hairpin and R <= R_hairpin:
            severity = "Hairpin"
        elif theta >= theta_medium and R <= R_medium:
            severity = "Medium"
        else:
            severity = "Sweeper"
        cm2 = dict(cm)
        cm2['severity'] = severity
        classified.append(cm2)
    return classified


def detect_compound_corners(
    corner_metrics: List[Dict[str, Any]],
    max_gap_for_chicane: float
) -> List[Dict[str, Any]]:
    # (unchanged)
    for cm in corner_metrics:
        cm['compound_id'] = None
    next_id = 1
    in_group = False
    for i in range(len(corner_metrics) - 1):
        curr, nxt = corner_metrics[i], corner_metrics[i+1]
        if curr['turn_dir'] != nxt['turn_dir'] and (curr['next_gap'] or float('inf')) <= max_gap_for_chicane:
            if not in_group:
                curr['compound_id'] = next_id
                nxt['compound_id'] = next_id
                in_group = True
                next_id += 1
            else:
                nxt['compound_id'] = next_id - 1
        else:
            in_group = False
    return corner_metrics


def print_track_detections(
    corner_list: List[Dict[str, Any]],
    total_pts: Optional[int] = None
) -> None:
    """
    Print the sequence of detected track elements, including starting/ending straights.
    """
    if not corner_list:
        print("Straight")
        return
    corners = sorted(corner_list, key=lambda c: c['idx_start'])
    # initial straight
    if corners[0]['idx_start'] > 0:
        print("Straight")
    i = 0
    n = len(corners)
    while i < n:
        curr = corners[i]
        # inter-corner straight
        if i > 0 and curr['idx_start'] - corners[i-1]['idx_end'] > 1:
            print("Straight")
        # compounds
        comp = curr.get('compound_id')
        if comp is not None:
            j = i
            while j < n and corners[j].get('compound_id') == comp:
                j += 1
            print("Chicane")
            i = j
        else:
            print(curr.get('severity', 'Corner'))
            i += 1
    # trailing straight
    if total_pts is not None:
        last_end = corners[-1]['idx_end']
        if last_end < total_pts - 1:
            print("Straight")


def visualize_track_segments(
    centre_pts: List[Tuple[float, float]],
    segments: List[Dict[str, Any]],
    line_width: float = 2.0,
    marker_size: float = 4.0,
    color_map: Optional[Dict[str, str]] = None
) -> None:
    # (unchanged)
    if color_map is None:
        color_map = {'straight': 'gray', 'L': 'blue', 'R': 'red'}
    xs, ys = zip(*centre_pts)
    plt.figure(figsize=(8, 8))
    ax = plt.gca()
    ax.set_aspect('equal', 'box')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Track Centreline Segments')
    for seg in segments:
        sx, sy = xs[seg['idx_start']:seg['idx_end']+1], ys[seg['idx_start']:seg['idx_end']+1]
        color = color_map['straight'] if seg['type']=='straight' else color_map.get(seg.get('turn_dir',''), 'black')
        ax.plot(sx, sy, linewidth=line_width, color=color)
        ax.scatter([sx[0], sx[-1]], [sy[0], sy[-1]], s=marker_size**2, color=color, zorder=3)
    plt.show()


if __name__ == "__main__":
    import numpy as np

    # --- Generate a circle with a sine-wave perturbation along its normal ---
    radius = 100
    num_pts = 500
    t = np.linspace(0, 2 * np.pi, num_pts)

    # Base circle
    x_center = radius * np.cos(t)
    y_center = radius * np.sin(t)

    # Sine-wave offset (number of oscillations around the circle)
    amplitude = 10
    n_oscillations = 12
    offset_mag = amplitude * np.sin(n_oscillations * t)

    # Compute unit normals (perpendicular to tangent)
    # Tangent: (dx, dy) = (−R sin t, R cos t)
    dx = -radius * np.sin(t)
    dy =  radius * np.cos(t)
    # Left-hand normal = (−dy, dx)
    nx = -dy
    ny =  dx
    L = np.hypot(nx, ny)
    nx /= L
    ny /= L

    # Offset the base circle along its normal by the sine wave
    x_vals = x_center + nx * offset_mag
    y_vals = y_center + ny * offset_mag

    centre_pts = list(zip(x_vals, y_vals))

    # --- Run the segmentation pipeline ---
    fits = segment_centerline_and_fit_arcs(centre_pts, window_length=20, step_size=1)
    fits = classify_arcs_vs_straights(fits,
                                      max_rms_err=1.0,
                                      min_arc_angle=0.15,
                                      max_R_for_corner=300)
    segs = consolidate_segments(fits, min_straight_len=5)
    corns = characterize_corners(segs, centre_pts)

    # Attach turn directions to segments
    for s in segs:
        if s['type'] == 'corner':
            for c in corns:
                if c['idx_start'] == s['idx_start'] and c['idx_end'] == s['idx_end']:
                    s['turn_dir'] = c['turn_dir']
                    break

    # Optional classification + detection
    metrics = extract_corner_metrics(corns, centre_pts)
    classified = classify_corner_severity(metrics,
                                          theta_hairpin=150,
                                          R_hairpin=50,
                                          theta_medium=60,
                                          R_medium=150)
    compound_tagged = detect_compound_corners(classified, max_gap_for_chicane=15)

    print("\nTrack Element Sequence:")
    print_track_detections(compound_tagged, total_pts=len(centre_pts))

    # --- Visualize ---
    visualize_track_segments(centre_pts, segs)






