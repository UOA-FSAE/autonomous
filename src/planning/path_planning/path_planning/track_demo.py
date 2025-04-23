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
    A = np.column_stack([x, y, np.ones_like(x)])
    b = -(x**2 + y**2)
    # solve for [A_coef, B_coef, C_coef]
    coef, *_ = np.linalg.lstsq(A, b, rcond=None)
    A_coef, B_coef, C_coef = coef
    cx = -A_coef / 2
    cy = -B_coef / 2
    # compute radius
    R = np.sqrt(cx**2 + cy**2 - C_coef)
    return cx, cy, R

def segment_centerline_and_fit_arcs(
    centre_pts: List[Tuple[float, float]],
    window_length: int,
    step_size: int
) -> List[Dict[str, Any]]:
    """
    Divide the sampled track centerline into small, overlapping segments and fit a geometric primitive
    (circle/arc) to each one so that later stages can decide whether the segment behaves like a curve or a straight.

    Parameters:
    - centre_pts: Ordered list of (x, y) tuples representing the track centreline.
    - window_length: Number of points per test segment (window size).
    - step_size: Advance between successive windows (number of points to slide the window each step).

    Returns:
    - fits: List of dictionaries, each containing:
        - idx_start: Start index of the window.
        - idx_end: End index of the window.
        - R: Fitted circle radius.
        - centre: Tuple (cx, cy) of fitted circle centre.
        - arc_angle: Subtended angle of the arc.
        - rms_err: Root-mean-square residual error of the fit.
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
        except Exception as e:
            # in case of degenerate (e.g. colinear) fallback to 'infinite' radius
            cx, cy, R = np.nan, np.nan, np.inf

        # compute distances to circle centre
        d = np.sqrt((x - cx)**2 + (y - cy)**2)
        # RMS error
        rms_err = float(np.sqrt(np.mean((d - R)**2)))

        # compute subtended angle:
        # angle at circle center from first to last point
        theta_start = np.arctan2(y[0] - cy, x[0] - cx)
        theta_end   = np.arctan2(y[-1] - cy, x[-1] - cx)
        # wrap to [-pi, +pi]
        delta = theta_end - theta_start
        delta = (delta + np.pi) % (2 * np.pi) - np.pi
        arc_angle = abs(delta)

        fits.append({
            'idx_start': i,
            'idx_end':   i + window_length - 1,
            'R':         float(R),
            'centre':    (float(cx), float(cy)),
            'arc_angle': float(arc_angle),
            'rms_err':   float(rms_err),
        })

    return fits
# ----------------------------------------------------------------------------------------------------------------------------------------------------------
def classify_arcs_vs_straights(
    fits: List[Dict[str, Any]],
    max_rms_err: float,
    min_arc_angle: float,
    max_R_for_corner: Optional[float] = None
) -> List[Dict[str, Any]]:
    """
    Decide, for every tested window, whether its geometry behaves like a corner (arc) or a straight segment.

    Parameters:
    - fits: List of fit dictionaries produced by segment_centerline_and_fit_arcs.
    - max_rms_err: Acceptable maximum circle-fit error to consider a valid arc.
    - min_arc_angle: Minimum subtended angle (in the same units as arc_angle) to qualify as a corner.
    - max_R_for_corner: Optional upper-bound radius; segments with radius larger than this are considered straight.

    Returns:
    - fits: The same list of dictionaries, each with an added key:
        - is_corner: Boolean indicating whether the segment is classified as a corner.
    """
    for fit in fits:
        R = fit.get('R', float('inf'))
        arc_angle = fit.get('arc_angle', 0.0)
        rms_err = fit.get('rms_err', float('inf'))

        # Basic corner criteria: good fit error and enough curvature
        is_corner = (rms_err <= max_rms_err) and (arc_angle >= min_arc_angle)

        # If a maximum radius is specified, enforce it
        if max_R_for_corner is not None:
            is_corner = is_corner and (R <= max_R_for_corner)

        fit['is_corner'] = bool(is_corner)

    return fits
# ----------------------------------------------------------------------------------------------------------------
def consolidate_segments(
    fits: List[Dict[str, Any]],
    min_straight_len: int
) -> List[Dict[str, Any]]:
    """
    Merge consecutive windows with the same label, split mixed sequences cleanly, and ensure rapid left–right
    switches (chicanes) appear as two corners with a short straight between.

    Parameters:
    - fits: List of fit dictionaries, each with is_corner, idx_start, idx_end.
    - min_straight_len: Minimum number of centreline points required to be considered a standalone straight segment.

    Returns:
    - segments: Ordered list of dictionaries, each containing:
        - type: "straight" or "corner"
        - idx_start: Start index of the segment in centre_pts.
        - idx_end: End index of the segment in centre_pts.
    """
    if not fits:
        return []

    # 1. Group consecutive windows into runs of the same label
    runs = []
    curr_label = fits[0]['is_corner']
    run_start = fits[0]['idx_start']
    run_end = fits[0]['idx_end']

    for fit in fits[1:]:
        if fit['is_corner'] == curr_label:
            # extend current run
            run_end = max(run_end, fit['idx_end'])
        else:
            # finish current run
            runs.append((curr_label, run_start, run_end))
            # start new run
            curr_label = fit['is_corner']
            run_start = fit['idx_start']
            run_end = fit['idx_end']
    # append last run
    runs.append((curr_label, run_start, run_end))

    # 2. Compute split boundaries between runs
    boundaries = []
    for i in range(len(runs) - 1):
        _, _, end_i = runs[i]
        _, start_j, _ = runs[i + 1]
        # midpoint boundary
        b = (end_i + start_j) // 2
        boundaries.append(b)

    # 3. Build preliminary segments using run boundaries
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

    # 4. Merge very short starting/ending straights (not chicane separators)
    #    if they are shorter than min_straight_len
    #    but preserve interior short straights (for chicanes)
    #    i.e., only adjust first or last segment if it's a short straight
    if len(segments) > 1:
        # first segment
        first = segments[0]
        if first['type'] == 'straight':
            length = first['idx_end'] - first['idx_start'] + 1
            if length < min_straight_len:
                # merge into next
                segments[1]['idx_start'] = first['idx_start']
                segments.pop(0)
        # last segment
        last = segments[-1]
        if last['type'] == 'straight':
            length = last['idx_end'] - last['idx_start'] + 1
            if length < min_straight_len:
                # merge into previous
                segments[-2]['idx_end'] = last['idx_end']
                segments.pop(-1)

    return segments
# -------------------------------------------------------------------------------------------------------------------------------------------------------------
def characterize_corners(
    segments: List[Dict[str, Any]],
    centre_pts: List[Tuple[float, float]]
) -> List[Dict[str, Any]]:
    """
    Attach geometry metrics to every confirmed corner so downstream planners know its “tightness” and “severity.”

    Parameters:
    - segments: List of segment dictionaries from consolidate_segments, where type == "corner".
    - centre_pts: Ordered list of (x, y) tuples representing the track centreline.

    Returns:
    - corners: List of dictionaries, each containing:
        - idx_start: Start index of the corner in centre_pts.
        - idx_end: End index of the corner in centre_pts.
        - R: Final radius of the re-fit circle.
        - arc_angle: Total subtended angle of the corner.
        - turn_dir: "L" or "R" indicating left or right turn direction.
    """
    corners: List[Dict[str, Any]] = []
    for seg in segments:
        if seg.get('type') != 'corner':
            continue

        # Extract the points for this corner
        pts = np.array(centre_pts[seg['idx_start']: seg['idx_end'] + 1])
        x, y = pts[:, 0], pts[:, 1]

        # Refit circle to the entire corner segment
        cx, cy, R = _fit_circle_algebraic(x, y)

        # Compute start/end angles relative to circle center
        theta_start = np.arctan2(y[0] - cy, x[0] - cx)
        theta_end   = np.arctan2(y[-1] - cy, x[-1] - cx)
        # Normalize delta to [-pi, pi]
        delta = theta_end - theta_start
        delta = (delta + np.pi) % (2 * np.pi) - np.pi
        arc_angle = abs(delta)

        # Determine turn direction (positive δ → counterclockwise → left)
        turn_dir = 'L' if delta > 0 else 'R'

        corners.append({
            'idx_start': seg['idx_start'],
            'idx_end':   seg['idx_end'],
            'R':         float(R),
            'arc_angle': float(arc_angle),
            'turn_dir':  turn_dir
        })

    return corners
# -------------------------------------------------------------------------------------------------------------------------------------------------------------


def visualize_track_segments(
    centre_pts: List[Tuple[float, float]],
    segments: List[Dict[str, Any]],
    line_width: float = 2.0,
    marker_size: float = 4.0,
    color_map: Optional[Dict[str, str]] = None
) -> None:
    """
    Provide a quick, intuitive visual of the entire lap, colouring each centre-line segment so engineers,
    drivers, or simulation tools can instantly distinguish straights, left-hand corners, and right-hand corners.
    """
    # Default colors if none provided
    if color_map is None:
        color_map = {
            'straight': 'gray',
            'L': 'blue',
            'R': 'red'
        }

    # Unpack centre points into arrays
    xs, ys = zip(*centre_pts)

    plt.figure(figsize=(8, 8))
    ax = plt.gca()
    ax.set_aspect('equal', 'box')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Track Centreline Segments')

    # Plot each segment with its designated color
    for seg in segments:
        start, end = seg['idx_start'], seg['idx_end']
        seg_x = xs[start:end+1]
        seg_y = ys[start:end+1]

        if seg['type'] == 'straight':
            color = color_map['straight']
        else:
            # corner: use turn direction key
            turn = seg.get('turn_dir', None)
            color = color_map.get(turn, 'black')

        # Draw the segment
        ax.plot(seg_x, seg_y, linewidth=line_width, color=color)

        # Optionally mark the boundaries of each segment
        ax.scatter([seg_x[0], seg_x[-1]], [seg_y[0], seg_y[-1]],
                   s=marker_size**2, color=color, zorder=3)

    plt.show()

if __name__ == "__main__":
    # --- Generate a simple chicane path ---
    # Shape: from left to right, quick R → L turn
    t1 = np.linspace(0, np.pi / 2, 50)
    x1 = 200 * np.cos(t1)
    y1 = 100 * np.sin(t1)

    # short straight
    x2 = np.linspace(x1[-1], x1[-1] + 40, 20)
    y2 = np.full_like(x2, y1[-1])

    # Left turn (mirrored arc)
    t3 = np.linspace(np.pi / 2, 0, 50)
    x3 = x2[-1] + 200 * np.cos(t3)
    y3 = -100 * np.sin(t3)

    # Combine path segments
    x = np.concatenate([x1, x2, x3])
    y = np.concatenate([y1, y2, y3])
    centre_pts = list(zip(x, y))

    # --- Run the segmentation pipeline ---
    fits = segment_centerline_and_fit_arcs(centre_pts, window_length=20, step_size=5)
    fits = classify_arcs_vs_straights(fits, max_rms_err=1.0, min_arc_angle=0.05, max_R_for_corner=300)
    segs = consolidate_segments(fits, min_straight_len=10)
    corns = characterize_corners(segs, centre_pts)

    # Attach turn directions
    for s in segs:
        if s['type'] == 'corner':
            for c in corns:
                if c['idx_start'] == s['idx_start'] and c['idx_end'] == s['idx_end']:
                    s['turn_dir'] = c['turn_dir']
                    break

    # --- Visualize ---
    visualize_track_segments(centre_pts, segs)



