from typing import List, Tuple, Dict, Any, Optional
import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.interpolate import splprep, splev
from scipy.optimize import minimize_scalar
from scipy.integrate import cumtrapz
from scipy.optimize import minimize_scalar
from scipy.special import fresnel

"""NOTES:
At present, if you use the oval track generator with a window length of 20, it detects
corners correctly but misidentifies the straights as corners. Its pretty bad.
Upon looking further into this, we find that for this oval track, the RADII of both
the actual corners and the straights are about the same (at around 50). This extends to even
the arc angles (aroudn 3.1). This is weird, and indicates something is wrong in the circle
fitting part specifically.
"""

# ----------------------------------------------------------------------------------------------------------------------------------------------------------
def _fit_circle_algebraic(x: np.ndarray, y: np.ndarray) -> Tuple[float, float, float]:
    """
    Fit a circle x^2 + y^2 + A x + B y + C = 0
    to points (x, y) via linear least squares.

    Returns (cx, cy, R).
    """
    """
    Divide the track centreline into overlapping windows of length `w`, stepping forward by `s`,
    and fit a circle to each segment using a least-squares method.

    Inputs:
    - centre_points: Ordered list of (x, y) tuples representing the track centreline.
    - w: Integer specifying the number of points in each window for fitting.
    - s: Integer step size that determines the overlap between windows.

    Output:
    - fits: A list of dictionaries, each containing:
        - 'idx_start': Index of the first point in the window.
        - 'idx_end': Index of the last point in the window.
        - 'R': Fitted circle radius.
        - 'centre': (cx, cy) coordinates of the fitted circle's centre.
        - 'arc_angle': Subtended angle of the arc (radians or degrees).
        - 'rms_err': Root-mean-square error of the fit.

    This function is foundational: it provides the geometric characterization
    of the road necessary for downstream corner detection.
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


def segment_centreline_and_fit_arcs(
    centre_points: List[Tuple[float, float]],
    window_length: int,
    step_size: int
) -> List[Dict[str, Any]]:
    """
    Divide the sampled track centreline into small, overlapping segments and fit a geometric primitive
    (circle/arc) to each one so that later stages can decide whether the segment behaves like a curve or a straight.
    """
    pts = np.asarray(centre_points)
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
    centre_points: List[Tuple[float, float]]
) -> List[Dict[str, Any]]:
    # (unchanged) ...
    corners: List[Dict[str, Any]] = []
    for seg in segments:
        if seg.get('type') != 'corner':
            continue
        pts = np.array(centre_points[seg['idx_start']: seg['idx_end'] + 1])
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
    centre_points: List[Tuple[float, float]]
) -> List[Dict[str, Any]]:
    # (unchanged)
    metrics = []
    n_corners = len(corners)
    for i, c in enumerate(corners):
        start, end = c['idx_start'], c['idx_end']
        theta = abs(c['arc_angle']) * 180.0 / math.pi
        L = sum(math.hypot(centre_points[k+1][0] - centre_points[k][0], centre_points[k+1][1] - centre_points[k][1])
                for k in range(start, end))
        if i < n_corners - 1:
            next_start = corners[i+1]['idx_start']
            next_gap = math.hypot(centre_points[next_start][0] - centre_points[end][0],
                                  centre_points[next_start][1] - centre_points[end][1])
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

class SegIslands():
    def __init__(self, points=[], max_overload=0):
        self.points = points
        self.length = len(points)
        self.mid_index = self.length//2
        self.first_half = points[:self.mid_index]
        self.second_half = points[self.mid_index+1:]
        self.fh_length = len(self.first_half)
        self.sh_length = len(self.second_half)
        self.max_overload = max_overload
        self.fh_gradient = max_overload / self.fh_length
        self.sh_gradient = max_overload / self.sh_length

    def linearize_laterals(self, half, relevant_cones):
        if half == 'f':
            centre_points = self.first_half
            gradient = self.fh_gradient
        else:
            centre_points = self.second_half
            gradient = self.sh_gradient

        planned_path = []
        for i in range(len(centre_points)):
            cone = relevant_cones[i]
            centre_point = centre_points[i]
            base_displacement = cone - centre_point
            final_pos = base_displacement * (self.max_overload - gradient*i) + centre_point
            planned_path.append(final_pos)

        return planned_path

#tofu's planning:


def find_angle(point1, mid_point, point2):
    # Convert to NumPy arrays (if not already)
    point1 = np.array(point1)
    mid_point = np.array(mid_point)
    point2 = np.array(point2)

    # Vectors from mid_point to point1 and point2
    a = point1 - mid_point
    b = point2 - mid_point

    # Dot product and magnitudes
    dot_prod = np.dot(a, b)
    mag_a = np.linalg.norm(a)
    mag_b = np.linalg.norm(b)

    if mag_a == 0 or mag_b == 0:
        return 0.0

    # Clamp to avoid NaNs from floating point errors
    cos_angle = np.clip(dot_prod / (mag_a * mag_b), -1.0, 1.0)
    angle_rad = np.arccos(cos_angle)
    angle_deg = np.degrees(angle_rad)

    return angle_deg
def lateralized_ideal_curves(centre_points, blue_cones, yellow_cones, segs, margin=0.5, track_width=3):
    """
    Refs: 
    - https://dspace.mit.edu/bitstream/handle/1721.1/64669/706825301-MIT.pdf
    - https://www.youtube.com/watch?v=aZlOkt1oU2k&list=PLVdC2-cdn7MWYL4gYNN-wHI1AYc_FT172&index=6&ab_channel=Driver61


    This doesn't use the geometric best line, but instead uses the IDEAL racing line.
    The ideal racing line turns a lot at first, apexes a bit later than the geomtric line but ends up with
    a faster straight entry speed (which is the goal here).
    It has been found that the greater the angle of the curve, (eg. a hairpin would be approx. 180 degrees),
    the further (later) the apex of the ideal racing line is compared ot the geomtric line. We can use this 
    basic observation to get the best realistic path.
    OK IDEAL RACING LINE is just a late apex and traditional apex trying to become one. We can find this
    analytically using EULER SPIRALS! Since we have done the cornber identificant, this will work great as
    Euler spirals work near perfectly for corners in isolation. We can tweak the use of Euler spirals to make
    straights easier to plan for, while also minimizing overal track time because late apex is generally a good idea.
    For a sequence of corners (later stages):
    - Find final corner in apex, Euler spiral for it.
    - For every corner before this in the sequence, just traditional apex it, as traditional apex maximises 
    We essentially want to:


    Assumes:
    - The track is 3m wide.
    - Margin is 0.5m from track boundaries if not specified is given.
    - Overload is the term given to how far in or out we go from the centreline divided by
      the distance (along the delaunay edge) to the margin from the centreline.
    """
    """each centre point has a (centre point index // 2) corresponding blue and yellow cone that formed it"""
    
    
    half_track_width = (track_width/2)
    max_overload = (half_track_width - margin) / half_track_width

    

    for seg in segs:
        if seg['type'] == 'corner':
            corners_centre_points = centre_points[seg['idx_start']:seg['idx_end']+1]
            mid_centre_point = corners_centre_points[len(corners_centre_points) // 2]
def lateral_linear_curves(centre_points, blue_cones, yellow_cones, segs, margin=0.5, track_width=3):

    """
    Assumes:
    - The track is 3m wide.
    - Margin is 0.5m from track boundaries if not specified is given.
    - Overload is the term given to how far in or out we go from the centreline divided by
      the distance (along the delaunay edge) to the margin from the centreline.
    """
    half_track_width = (track_width/2)
    max_overload = (half_track_width - margin) / half_track_width

    planned_curves = []

    for seg in segs:
        if seg['type'] == 'corner':
            """TradApex: go on the outer for the ends of the curve, and kiss the apex"""
            corner_si = SegIslands(centre_points[seg['idx_start']:seg['idx_end']+1])
            first_half_si = SegIslands(corner_si.first_half, max_overload)
            second_half_si = SegIslands(corner_si.second_half, max_overload)

            planned_path = []
            
class CornerRepr():
    def __init__(self, index, length):
        self.index = index
        self.length = length
def rank_corners(centre_points, segs):
    number_of_segs = len(segs)
    ranked_corners = [] #list of CornerReprs

    for i, seg in enumerate(segs):
        if seg['type'] != 'corner':
            continue  # Skip if it's not a corner

        # Look ahead to the next segment (with wraparound)
        next_i = (i + 1) % number_of_segs
        next_seg = segs[next_i]

        if next_seg['type'] == 'straight':
            # Find start and end points of the straight
            start_point = centre_points[next_seg['idx_start']]
            end_point = centre_points[next_seg['idx_end']]
            length = np.linalg.norm(end_point - start_point)
        else:
            length = 0.0

        ranked_corners.append(CornerRepr(i, length))

    ranked_corners = sorted(ranked_corners, key=lambda c: c.length, reverse=True)

    return ranked_corners

def vector_angle(v):
    return np.radians(np.arctan2(v[1], v[0])) % 360

def find_turn_angle(first_point, mid_point, second_point):
    """
    Computes the signed smallest angle between two vectors relative to the positive x-axis.
    
    Parameters:
    - v1: np.array, first vector
    - v2: np.array, second vector
    
    Returns:
    - angle_deg: float, angle in degrees from v1 to v2 (0 to 360)
    """
    v1 = mid_point - first_point
    v2 = second_point - mid_point

    angle1 = vector_angle(v1)
    angle2 = vector_angle(v2)
    angle_diff = abs((angle2 - angle1) % (2*math.pi))

    return angle_diff#, angle2, np.linalg.norm(v2)

def check_if_in_range(point, ref_point, threshold):
    if np.linalg.norm(point - ref_point) <= threshold:
        return True
    return False

def find_end_heading():
    #TODO
    pass

def euler_spiral_corner(centre_points, blue_cones, yellow_cones, turn_direction, 
                        mode='auto_apex', displacement_step=10, retry_displacement_coefficient=0.9, retry_angle_coefficient = 1.5,
                        angle_step_degrees=1, threshold_radius=1, end_length_scaler=1.4, end_resolution=10):

    n = len(centre_points)
    mid_index = n//2
    turn_angle, end_heading, end_length = find_turn_angle(centre_points[0], centre_points[mid_index], centre_points[-1])
    end_length *= end_length_scaler
    outside = []
    inside = []
    turn_polarity = 1
    if turn_direction == 'R':
        inside = yellow_cones
        outside = blue_cones
        turn_polarity = -1
    else:
        inside = blue_cones
        outside = yellow_cones


    if mode != 'auto_apex':
        end_heading = find_end_heading()



    #First section: Before the apex
    delta_angle = turn_polarity * np.radians(angle_step_degrees)
    apex_point = inside[mid_index]

    successful = False
    i = 0
    plt.figure(figsize=(12, 7))
    plt.scatter(centre_points[:, 0], centre_points[:, 1], color='red', marker='x', label='Centre Points')
    plt.scatter(blue_cones[:, 0], blue_cones[:, 1], color='blue', marker='o', label='Blue Cones')
    plt.scatter(yellow_cones[:, 0], yellow_cones[:, 1], color='yellow', marker='o', label='Yellow Cones')
    plt.axis('equal')
    plt.legend()
    plt.title('Bruh Graph with Segments')

    while not successful:
        path = [outside[0]]
        heading = vector_angle(outside[0] - outside[1])
        i += 1
        accumulated_delta_angle = 0
        plt.scatter(outside[0][0], outside[0][1], color='blue', zorder=5, s=70)
        plt.scatter(apex_point[0], apex_point[1], color='red', zorder=5, s=70)
        j = 0
        delta_angle *= retry_angle_coefficient
        while (abs(accumulated_delta_angle) < abs(turn_angle)):
            j += 1
            heading = heading + delta_angle*j
            offset = np.array([np.cos(heading), np.sin(heading)]) * displacement_step# * retry_displacement_coefficient**i
            path.append(path[-1] + offset)
            accumulated_delta_angle += delta_angle*j
            plt.scatter(path[-1][0], path[-1][1], color='green', marker='x', zorder=5, s=3)

            if check_if_in_range(path[-1], apex_point, threshold_radius):
                successful = True
                break
        


    #After apex, to end
    delta_angle = (end_heading - heading)
    gradient = delta_angle / end_length
    delta_disp = end_length / end_resolution

    for i in range(end_resolution):
        heading = gradient*delta_disp
        offset = np.array([np.cos(heading), np.sin(heading)]) * delta_disp
        path.append(path[-1] + offset)



    return path


    
        
#------------------------------------------------------------------------------

# def vector_angle(v):
#     return np.arctan2(v[1], v[0])

def generate_euler_spiral_segment(start_point, start_heading, turn_angle, apex_point, 
                                  initial_step=2.0, threshold_radius=1.0, max_iters=50, curvature_scale=0.01, turn_sign=1):
    
    plt.figure(figsize=(12, 7))
    plt.scatter(start_point[0], start_point[1], color='blue', zorder=5, s=70)
    plt.scatter(apex_point[0], apex_point[1], color='red', zorder=5, s=70)
    plt.scatter(apex_point[0], apex_point[1], color='red', zorder=5, s=70)
    plt.axis('equal')
    plt.legend()
    plt.title('Bruh Graph with Segments')
    # plt.show()


    for attempt in range(max_iters):
        step_size = initial_step * (0.9 ** attempt)
        path = [start_point]
        heading = start_heading
        accumulated_angle = 0.0
        s = 0.0

        while abs(accumulated_angle) < abs(turn_angle):
            s += step_size
            curvature = curvature_scale * s * turn_sign
            dtheta = curvature * step_size * np.sign(turn_angle)
            heading += dtheta
            accumulated_angle += dtheta

            dx = step_size * np.cos(heading)
            dy = step_size * np.sin(heading)
            new_point = path[-1] + np.array([dx, dy])
            path.append(new_point)

            plt.scatter(new_point[0], new_point[1], color='green', marker='x', zorder=5, s=2)

            if np.linalg.norm(new_point - apex_point) < threshold_radius:
                return path, heading  # success

    return None, None  # failed to hit apex within threshold

def generate_euler_spiral_to_heading(start_point, start_heading, end_heading, 
                                     segment_length=10.0, num_points=20, curvature_scale=0.01):
    path = [start_point]
    heading = start_heading
    delta_angle = end_heading - start_heading
    total_length = segment_length
    ds = total_length / num_points

    for i in range(1, num_points + 1):
        s = i * ds
        curvature = (curvature_scale * s) * np.sign(delta_angle)
        dtheta = curvature * ds
        heading += dtheta
        dx = ds * np.cos(heading)
        dy = ds * np.sin(heading)
        new_point = path[-1] + np.array([dx, dy])
        path.append(new_point)

    return path

def generate_euler_spiral_full(start_point, start_heading, turn_angle, apex_point, end_heading, threshold_radius=5, turn_sign=1):
    first_half, final_heading = generate_euler_spiral_segment(
        start_point, start_heading, turn_angle, apex_point, threshold_radius=threshold_radius, turn_sign=turn_sign
    )

    if first_half is None:
        raise ValueError("Failed to generate first half of Euler spiral to hit apex.")

    second_half = generate_euler_spiral_to_heading(
        first_half[-1], final_heading, end_heading
    )

    return first_half + second_half[1:]  # Remove duplicate connection point

def euler_spirals(centre_points, blue_cones, yellow_cones, ranked_corners, mode='auto'):
    spirals = []
    for ranked_corner in ranked_corners:
        corner = segs[ranked_corner.index]
        start = corner['idx_start']
        end = corner['idx_end'] + 1

        centre = centre_points[start : end]
        blue = blue_cones[start : end]
        yellow = yellow_cones[start : end]

        mid_index = len(centre)//2
        turn_angle = find_turn_angle(centre[0], centre[mid_index], centre[-1])
        inside, outside, turn_sign = (yellow, blue, 1) if (corner['turn_dir'] == 'R') else (blue, yellow, -1)
        start_point = outside[0]
        apex_point = inside[mid_index]
        start_heading = vector_angle(centre_points[1] - centre_points[0])
        end_heading = vector_angle(centre_points[-1] - centre_points[-2])
        spirals.append(generate_euler_spiral_full(start_point, start_heading, turn_angle, apex_point, end_heading, turn_sign=turn_sign))

    return spirals

#-------------------------------------------------------------------------------    
    
            
def euler_spiral_corners(centre_points, blue_cones, yellow_cones, segs, ranked_corners):
    spirals = []
    for i in range(len(ranked_corners)):
        corner = segs[ranked_corners[i].index]
        start = corner['idx_start']
        end = corner['idx_end'] + 1
        centre = centre_points[start : end]
        blue = blue_cones[start : end]
        yellow = yellow_cones[start : end]

        spirals.append(euler_spiral_corner(centre, blue, yellow, corner['turn_dir'], displacement_step=1))

    return spirals



            
                







#visualization, testing
def generate_cones(centre_points, offset=1.5):
    """
    Generates blue (left) and yellow (right) cones offset perpendicularly from the centreline.
    
    Parameters:
    - centre_points: list of np.array([x, y])
    - offset: float, distance from centerline to each cone (in meters)
    
    Returns:
    - blue_cones: list of np.array([x, y]) to the left of the centreline
    - yellow_cones: list of np.array([x, y]) to the right of the centreline
    """
    blue_cones = []
    yellow_cones = []

    for i in range(1, len(centre_points)-1):
        prev_pt = centre_points[i-1]
        next_pt = centre_points[i+1]
        direction = next_pt - prev_pt
        direction /= np.linalg.norm(direction)  # Normalize
        perp = np.array([-direction[1], direction[0]])  # 90° rotation for left

        center = centre_points[i]
        blue_cones.append(center + offset * perp)     # Left (blue)
        yellow_cones.append(center - offset * perp)   # Right (yellow)

    return np.array(blue_cones), np.array(yellow_cones)



def visualize_track_segments(
    centre_points: List[Tuple[float, float]],
    segments: List[Dict[str, Any]],
    blue_cones: List[Tuple[float, float]], 
    yellow_cones: List[Tuple[float, float]],
    line_width: float = 2.0,
    marker_size: float = 4.0,
    color_map: Optional[Dict[str, str]] = None
) -> None:
    # (unchanged)
    if color_map is None:
        color_map = {'straight': 'gray', 'L': 'blue', 'R': 'red'}
    xs, ys = zip(*centre_points)
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
    plt.scatter(centre_points[0][0], centre_points[0][1], color='black', zorder=5, s=70)
    plt.scatter(centre_points[10][0], centre_points[10][1], color='yellow', zorder=5, s=70)
    plt.scatter(blue_cones[:][0], blue_cones[:][1], color='blue', marker='o', label='Blue Cones')
    plt.scatter(yellow_cones[:][0], yellow_cones[:][1], color='yellow', marker='o', label='Yellow Cones')
    plt.show()

def testing_plotter(centre_points, blue_cones, yellow_cones, segs, spirals):
    plt.figure(figsize=(12, 7))
    plt.scatter(centre_points[:, 0], centre_points[:, 1], color='red', marker='x', label='Centre Points')
    plt.scatter(blue_cones[:, 0], blue_cones[:, 1], color='blue', marker='o', label='Blue Cones')
    plt.scatter(yellow_cones[:, 0], yellow_cones[:, 1], color='yellow', marker='o', label='Yellow Cones')

    for spiral in spirals:
        plt.scatter(spiral[:, 0], spiral[:, 1], color='black', marker='o')
    
    # plt.scatter(centre_points[0][0], centre_points[0][1], color='black', zorder=5, s=70)
    # plt.scatter(centre_points[10][0], centre_points[10][1], color='purple', zorder=5, s=70)
    # plt.scatter(centre_points[50][0], centre_points[50][1], color='black', zorder=5, s=70)
    # plt.scatter(centre_points[100][0], centre_points[100][1], color='purple', zorder=5, s=70)
    # plt.scatter(centre_points[150][0], centre_points[150][1], color='black', zorder=5, s=70)
    # plt.scatter(centre_points[190][0], centre_points[190][1], color='purple', zorder=5, s=70)
    # plt.scatter(centre_points[96][0], centre_points[96][1], color='red', zorder=5, s=100)
    # plt.scatter(centre_points[-1][0], centre_points[-1][1], color='blue', zorder=5, s=100)
    
    # Plot segments
    for seg in segs:
        idx_start = seg['idx_start']
        idx_end = seg['idx_end']
        seg_points = centre_points[idx_start:idx_end+1]  # +1 because slicing is exclusive
        
        if seg['type'] == 'corner':
            if seg['turn_dir'] == 'R':
                color = 'red'
            elif seg['turn_dir'] == 'L':
                color = 'cyan'
            else:
                color = 'gray'
        else:
            color = 'gray'
        
        plt.plot(seg_points[:, 0], seg_points[:, 1], color=color, linewidth=2)

    plt.axis('equal')
    plt.legend()
    plt.title('Bruh Graph with Segments')
    plt.show()

# Generation of tracks:

def generate_long_straight_track(num_points=2000, seed=42):
    """
    Generates a smooth, closed track with long straights and gentle corners.
    
    Parameters:
        num_points (int): Total number of interpolated points in the returned centreline.
        seed (int): Random seed for reproducibility of noise.
    
    Returns:
        List[Tuple[float, float]]: Ordered list of (x, y) tuples representing the track centreline.
    """
    np.random.seed(seed)

    track_outline = []

    # Define major segments: long straights + curves
    segments = [
        ((0, 0), (200, 0)),
        ((200, 0), (250, 100)),
        ((250, 100), (100, 250)),
        ((100, 250), (-20, 300)),
        ((-20, 300), (-200, 150)),
        ((-200, 150), (-220, 50)),
        ((-220, 50), (0, 0))
    ]

    for start, end in segments:
        p0 = np.array(start)
        p1 = np.array(end)
        for i in range(5):
            t = i / 4
            point = (1 - t) * p0 + t * p1
            point += np.random.normal(scale=2, size=2)  # Light positional noise
            track_outline.append(point)

    track_outline = np.array(track_outline)

    # Ensure the track is closed
    track_outline = np.vstack([track_outline, track_outline[0]])

    # B-spline interpolation
    tck, _ = splprep(track_outline.T, s=0, per=True)
    u_fine = np.linspace(0, 1, num_points)
    x_fine, y_fine = splev(u_fine, tck)

    # Convert to list of (x, y) tuples
    centre_points = list(zip(x_fine, y_fine))
    return centre_points

def generate_sine_perturbed_circle(radius=100, amplitude=10, n_oscillations=12, num_pts=500):
    """
    Generate a closed circular path with a sine-wave perturbation applied along the normal direction.

    Parameters:
        radius (float): Base radius of the circle.
        amplitude (float): Amplitude of the sine wave offset.
        n_oscillations (int): Number of sine wave cycles around the circle.
        num_pts (int): Number of points to generate along the path.

    Returns:
        List[Tuple[float, float]]: List of (x, y) tuples representing the perturbed path.
    """
    t = np.linspace(0, 2 * np.pi, num_pts)

    # Base circle coordinates
    x_centre = radius * np.cos(t)
    y_centre = radius * np.sin(t)

    # Magnitude of sine perturbation along normal
    offset_mag = amplitude * np.sin(n_oscillations * t)

    # Compute unit normals
    dx = -radius * np.sin(t)
    dy =  radius * np.cos(t)
    nx = -dy
    ny =  dx
    L = np.hypot(nx, ny)
    nx /= L
    ny /= L

    # Apply normal perturbation
    x_vals = x_centre + nx * offset_mag
    y_vals = y_centre + ny * offset_mag

    return list(zip(x_vals, y_vals))

def generate_oval_track(num_points=1000, straight_length=200, radius=50):
    # Half the points for straights, half for curves
    num_curve_points = num_points // 2
    num_straight_points = num_points - num_curve_points

    # First straight (bottom)
    straight1_x = np.linspace(-straight_length / 2, straight_length / 2, num_straight_points // 2)
    straight1_y = np.full_like(straight1_x, -radius)

    # Right curve (bottom to top, right side)
    theta1 = np.linspace(-np.pi/2, np.pi/2, num_curve_points // 2)
    curve1_x = radius * np.cos(theta1) + straight_length / 2
    curve1_y = radius * np.sin(theta1)

    # Second straight (top)
    straight2_x = np.linspace(straight_length / 2, -straight_length / 2, num_straight_points // 2)
    straight2_y = np.full_like(straight2_x, radius)

    # Left curve (top to bottom, left side)
    theta2 = np.linspace(np.pi/2, 3*np.pi/2, num_curve_points // 2)
    curve2_x = radius * np.cos(theta2) - straight_length / 2
    curve2_y = radius * np.sin(theta2)

    # Combine all parts
    x = np.concatenate([straight1_x, curve1_x, straight2_x, curve2_x])
    y = np.concatenate([straight1_y, curve1_y, straight2_y, curve2_y])

    # Convert to list of tuples
    points = list(zip(x, y))

    return points


if __name__ == "__main__":
    import test_and_scratch
    import tofu_delaunay


    # centre_points = generate_sine_perturbed_circle()
    centre_points = generate_long_straight_track()
    # centre_points = generate_oval_track()

    centre_points = np.array(centre_points)
    blue_cones, yellow_cones = generate_cones(centre_points)

    """
    remove delaunay cone pairs if its not needed. 
    Assumes blue cones are to the left, and yellow cones are to the right of the car at all times
    This will help in the planning paet
    """
    # centre_points, blue_cones, yellow_cones = tofu_delaunay.GimmeCanD(test_and_scratch.tester_oval(500))

    # --- Corner Identification---
    fits = segment_centreline_and_fit_arcs(centre_points, window_length=10, step_size=1)
    fits = classify_arcs_vs_straights(fits,
                                      max_rms_err=1.0,
                                      min_arc_angle=0.15,
                                      max_R_for_corner=300)
    segs = consolidate_segments(fits, min_straight_len=5)
    corns = characterize_corners(segs, centre_points)
    # Attach turn directions to segments
    for s in segs:
        if s['type'] == 'corner':
            for c in corns:
                if c['idx_start'] == s['idx_start'] and c['idx_end'] == s['idx_end']:
                    s['turn_dir'] = c['turn_dir']
                    break

    if False:
        # Optional classification + detection
        metrics = extract_corner_metrics(corns, centre_points)
        classified = classify_corner_severity(metrics,
                                              theta_hairpin=150,
                                              R_hairpin=50,
                                              theta_medium=60,
                                              R_medium=150)
        compound_tagged = detect_compound_corners(classified, max_gap_for_chicane=15)

        print("\nTrack Element Sequence:")
        print_track_detections(compound_tagged, total_pts=len(centre_points))
    


    ranked_corners = rank_corners(centre_points, segs)

    #BLUE CONES = left side, YELLOW CONES = right side
    spirals = euler_spirals(centre_points, blue_cones, yellow_cones, ranked_corners)
    # spirals = euler_spiral_corners(centre_points, blue_cones, yellow_cones, segs, ranked_corners)

    # black is first point, 11th point is yellow.
    testing_plotter(centre_points, blue_cones, yellow_cones, segs, spirals)
    # visualize_track_segments(centre_points, segs, blue_cones, yellow_cones)






