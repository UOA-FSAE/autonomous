from typing import List, Tuple, Dict, Any, Optional
import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.interpolate import splprep, splev
from pyclothoids import Clothoid
from scipy.interpolate import splprep, splev
from scipy.ndimage import gaussian_filter1d


"""NOTES:
At present, if you use the oval track generator with a window length of 20, it detects
corners correctly but misidentifies the straights as corners. Its pretty bad.
Upon looking further into this, we find that for this oval track, the RADII of both
the actual corners and the straights are about the same (at around 50). This extends to even
the arc angles (aroudn 3.1). This is weird, and indicates something is wrong in the circle
fitting part specifically.

BLUE CONES = left side, YELLOW CONES = right side
REFS:
https://github.com/TUMFTM/global_racetrajectory_optimization/blob/master/inputs/tracks/berlin_2018.csv

"""

# fitz's identification----------------------------------------------------------------------------------------------------------------------------------------------------------
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


# intermediataries------------------------------------------------------------------------------------------------------------------------------------------------------
def attach_turn_directions(segs, corns, min_corner_length=5):
    new_segs = []
    for s in segs:
        if s['type'] == 'corner':
            if (s['idx_end'] - s['idx_start']) > min_corner_length:
                for c in corns:
                    if c['idx_start'] == s['idx_start'] and c['idx_end'] == s['idx_end']:
                        s['turn_dir'] = c['turn_dir']
                        new_segs.append(s)
                        break

        else:
            new_segs.append(s)

    return new_segs

def break_apart_severe_corners_and_join_straights(centre_points, segs, split_angle=360):
    new_segs = []
    length = len(segs)
    i = 0
    while i < length:
        seg = segs[i]
        if seg['type'] == 'corner':
            start = seg['idx_start']
            end = seg['idx_end']
            start_vec = centre_points[start+3] - centre_points[start]
            end_vec = centre_points[end] - centre_points[end-3]
            start_heading = np.degrees(np.arctan2(start_vec[1], start_vec[0]))
            end_heading = np.degrees(np.arctan2(end_vec[1], end_vec[0]))
            if abs(end_heading - start_heading) > split_angle: 
                mid_index = (start + end)//2
                seg['idx_start'] = mid_index + 1

                new_segs.append({'type': 'corner', 'idx_start': start , 'idx_end': mid_index, 'turn_dir': seg['turn_dir']})
        else:
            i += 1
            iters = 0
            while iters < length and segs[i%length]['type'] == 'straight':
                i += 1
                iters += 1

            i -= 1    
            seg['idx_end'] = segs[i%length]['idx_end']
            
        new_segs.append(seg)     
        i += 1

    return new_segs


# tofu's planning-----------------------------------------------------------------------------------------------------------------------------------------------------
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

def connect_points_with_clothoid(A, B, theta_start_deg, theta_end_deg, n_points=500):
        """
        Connects two points A and B with specified start and end headings using a clothoid.

        Parameters:
        - A: Tuple (x0, y0) representing the start point.
        - B: Tuple (x1, y1) representing the end point.
        - theta_start_deg: Start heading in degrees.
        - theta_end_deg: End heading in degrees.
        - n_points: Number of points to sample along the clothoid.

        Returns:
        - x_vals: X coordinates of the sampled points.
        - y_vals: Y coordinates of the sampled points.
        - clothoid: The Clothoid object representing the curve.
        """
        # Convert angles from degrees to radians
        theta_start_rad = np.radians(theta_start_deg)
        theta_end_rad = np.radians(theta_end_deg)

        # Create the clothoid that connects the two points with specified headings
        clothoid = Clothoid.G1Hermite(A[0], A[1], theta_start_rad, B[0], B[1], theta_end_rad)

        # Sample points along the clothoid
        x_vals, y_vals = clothoid.SampleXY(n_points)
        points = np.column_stack((x_vals, y_vals))  # Combine into shape (n_points, 2)

        return points

class EulerSpiral():
    def __init__(self, spiral, seg_index):
        self.spiral = spiral
        self.seg_index = seg_index

def find_point_hand_side(ref_point, ref_heading_vec, compare_point):
    # Vector from A to the other point
    to_point = compare_point - ref_point

    # Compute the 2D cross product (scalar)
    cross = ref_heading_vec[0] * to_point[1] - ref_heading_vec[1] * to_point[0]
    return 'R' if cross < 0 else 'L'

def check_if_close(spiral, apex_points, threshold_distance):
        length = len(spiral)
        mid_index = length //2
        for i in range(mid_index):
            upper = min(mid_index + i, length-1)
            lower = max(mid_index - i - 1, 0)
            for i, apoint in enumerate(apex_points):
                if (np.linalg.norm(spiral[upper] - apoint) <= threshold_distance):
                    return True, upper, i
                if (np.linalg.norm(spiral[lower] - apoint) <= threshold_distance):
                    return True, lower, i
        
        return False, None, None

def check_if_on_correct_side(apex_point, apex_heading_vec, spiral_mid_point, correct_side):
    return correct_side == find_point_hand_side(apex_point, apex_heading_vec, spiral_mid_point)

def find_precut_direct(
        start_point, outside_end_point, inside_end_point, apex_points, 
        start_heading_deg, desired_end_heading_deg, correct_side, 
        slices=20, threshold_distance=5, clothoid_sample_size=30, angle_step=5, threshold_deg=5):

    iterations = 0
    step_vec = (np.array(inside_end_point) - np.array(outside_end_point)) / slices
    spiral = connect_points_with_clothoid(start_point, outside_end_point, start_heading_deg, desired_end_heading_deg, n_points=clothoid_sample_size)
    close, spiral_index, apex_index = check_if_close(spiral, apex_points, threshold_distance)
    while not close:
        outside_end_point += step_vec
        spiral = connect_points_with_clothoid(start_point, outside_end_point, start_heading_deg, desired_end_heading_deg, n_points=clothoid_sample_size)
        # plt.plot(spiral[:, 0], spiral[:, 1], color='orange', linewidth=3)
        # spiral_mid_point = spiral[len(spiral)//2]
        # plt.scatter(spiral_mid_point[0], spiral_mid_point[1], color='cyan', zorder=3, s=70)
        iterations += 1
        if iterations > slices:
            return spiral
        close, spiral_index, apex_index = check_if_close(spiral, apex_points, threshold_distance)


    apex_point = apex_points[apex_index]
    if apex_index != 0:
        apex_heading_vec = apex_points[apex_index] - apex_points[apex_index-1]
    else:
        apex_heading_vec = apex_points[apex_index+1] - apex_points[apex_index]

    spiral_mid_point = spiral[spiral_index]
    iterations = 0
    sign = 1 if correct_side == 'L' else -1
    end_heading = desired_end_heading_deg

    # plt.plot(spiral[:, 0], spiral[:, 1], color='green', linewidth=3)
    # plt.scatter(apex_point[0], apex_point[1], color='red', zorder=5, s=70, label='Apex Point')
    # plt.scatter(spiral_mid_point[0], spiral_mid_point[1], color='green', zorder=3, s=70)

    while not check_if_on_correct_side(apex_point, apex_heading_vec, spiral_mid_point, correct_side):
        end_heading += angle_step * sign
        spiral = connect_points_with_clothoid(start_point, outside_end_point, start_heading_deg, end_heading, n_points=clothoid_sample_size)
        spiral_mid_point = spiral[spiral_index]
        # plt.plot(spiral[:, 0], spiral[:, 1], color='cyan', linewidth=3)
        # plt.scatter(spiral_mid_point[0], spiral_mid_point[1], color='green', zorder=3, s=70)
        iterations += 1
        # if iterations > slices:
        #     break

    if iterations > 0:
        spiral = find_useful_part_of_spiral(spiral, desired_end_heading_deg, threshold_deg=threshold_deg)

    return spiral

def find_useful_part_of_spiral(spiral, desired_end_heading_deg, threshold_deg=5):
    lower_limit = desired_end_heading_deg - threshold_deg
    upper_limit = desired_end_heading_deg + threshold_deg

    length = len(spiral)
    mid_index = length //2
    for i in range(mid_index - 1):
        upper = min(mid_index + i, length-1)
        lower = max(mid_index - i - 1, 0)

        upper_vec = spiral[upper+1] - spiral[upper]
        lower_vec = spiral[lower+1] - spiral[lower]
        upper_heading = np.degrees(np.arctan2(upper_vec[1], upper_vec[0]))
        lower_heading = np.degrees(np.arctan2(lower_vec[1], lower_vec[0]))


        if lower_heading >= lower_limit and lower_heading <= upper_limit:
            return spiral[:lower+1]
        if upper_heading >= lower_limit and upper_heading <= upper_limit:
            return spiral[:upper+1]
    
    return spiral

def euler_spirals(centre_points, blue_cones, yellow_cones, segs, ranked_corners, apex_point_half_count=4, threshold_distance=0.2, threshold_deg=5):
    spirals = []
    finished_spirals = {}

    # plt.figure(figsize=(12, 7))
    # plt.scatter(centre_points[:, 0], centre_points[:, 1], color='red', marker='x', label='Centre Points')
    # plt.scatter(blue_cones[:, 0], blue_cones[:, 1], color='blue', marker='o', label='Blue Cones', s=2)
    # plt.scatter(yellow_cones[:, 0], yellow_cones[:, 1], color='#CCCC00', marker='o', label='Yellow Cones', s=2)
    # plt.scatter(centre_points[0][0], centre_points[0][1], color='black', zorder=5, s=70, label='Start Point')
    # plt.scatter(centre_points[-2][0], centre_points[-2][1], color='purple', zorder=5, s=70, label='End Point')

    for ranked_corner in ranked_corners:
        seg_index = ranked_corner.index
        corner_seg = segs[seg_index]
        start = corner_seg['idx_start']
        end = corner_seg['idx_end']

        centre = centre_points[start:end+1]
        if len(centre) > 4:
            blue = blue_cones[start:end+1]
            yellow = yellow_cones[start:end+1]
            inside, outside, correct_side = (yellow, blue, 'L') if corner_seg['turn_dir'] == 'R' else (blue, yellow, 'R')

            start_point = outside[0]
            apex_points = inside[(len(inside) // 2)-apex_point_half_count : (len(inside) // 2)+apex_point_half_count]
            
            start_vec = centre[3] - centre[0]
            start_heading = np.degrees(np.arctan2(start_vec[1], start_vec[0]))
            
            segs_length = len(segs)
            next_corner_seg_index = (seg_index + 1) % segs_length
            iters = 0
            while iters < segs_length and segs[next_corner_seg_index]['type'] != 'corner':
                next_corner_seg_index = (next_corner_seg_index + 1) % segs_length
                iters += 1

            if next_corner_seg_index not in finished_spirals:
                end_vec = centre[-1] - centre[-4]
            else:
                end_vec = finished_spirals[next_corner_seg_index] - centre[-1]
            
            end_heading = np.degrees(np.arctan2(end_vec[1], end_vec[0]))

            precut_spiral = find_precut_direct(start_point, outside[-1], inside[-1], inside, start_heading, end_heading, correct_side, 
                                               threshold_distance=threshold_distance, threshold_deg=threshold_deg)
            # spirals.append(EulerSpiral(find_useful_part_of_spiral(precut_spiral, end_heading, threshold_deg=threshold_deg), seg_index))
            spirals.append(EulerSpiral(precut_spiral, seg_index))

            finished_spirals[seg_index] = start_point



            # plt.plot(spirals[-1][0], spirals[-1][1], color='green', linewidth=2, label='Spiral Path')
            #assume no sequences for now, so we euler spiral everything
            #assume the first point is the start of the spiral

    return sorted(spirals, key=lambda es: es.seg_index)

def sample_straight_line(p1, p2, n=50):
    """
    Returns `n` points evenly spaced between points p1 and p2 (both np.array).
    """
    return np.linspace(p1, p2, n)

def stitch_path(centre_points, segs, spirals, n_straight_points=50, straight_sample_step=10):
    """
    Joins a list of EulerSpiral objects and samples straight lines between each spiral.
    
    Parameters:
    - spirals: List of EulerSpiral instances.
    - n_straight_points: Number of points to sample for each connecting straight line.
    
    Returns:
    - joined_path: NumPy array of shape (total_points, 2)
    """
    joined_path = []

    length = len(spirals)
    for i in range(length):
        spiral = spirals[i].spiral
        next_spiral = spirals[(i + 1) % length].spiral

        # Add the spiral path, excluding the last point (to prevent duplication)
        joined_path.extend(spiral[:-1])

        following_straight_index = (spirals[i].seg_index + 1) % len(segs)
        if segs[following_straight_index]['type'] == 'straight':
            #for a straight path that follows the track's slight curves
            start_idx = segs[following_straight_index]['idx_start']
            end_idx = segs[following_straight_index]['idx_end']
            if end_idx > start_idx:
                sampled_centre = centre_points[start_idx:end_idx:straight_sample_step]
            else:
                # Wrap-around case: concatenate two slices
                part1 = centre_points[start_idx::straight_sample_step]
                part2 = centre_points[:end_idx:straight_sample_step]
                sampled_centre = np.concatenate((part1, part2))

        
            if len(sampled_centre) > straight_sample_step:
                offset_start = spiral[-1] - sampled_centre[0]
                offset_end = next_spiral[0] - sampled_centre[-1]

                for j, centre_point in enumerate(sampled_centre[1:-1]):
                    t = j / (len(sampled_centre) - 2)  # Normalize from 0 to 1
                    interp_offset = (1 - t) * offset_start + t * offset_end
                    joined_path.append(centre_point + interp_offset)

        else:
            straight = sample_straight_line(spiral[-1], next_spiral[0], n=n_straight_points)
            joined_path.extend(straight[1:])

    return np.array(joined_path)

def smooth_path(stitched_path, sigma=2):
    """
    Smooths a path using a Gaussian filter.

    Parameters:
    - stitched_path: np.ndarray of shape (N, 2) representing [x, y] coordinates.
    - sigma: float, standard deviation for Gaussian kernel.

    Returns:
    - np.ndarray of same shape (N, 2), smoothed path.
    """
    if stitched_path.ndim != 2 or stitched_path.shape[1] != 2:
        raise ValueError("stitched_path must be a 2D array with shape (N, 2)")

    # Apply Gaussian filter separately to x and y coordinates
    smoothed_x = gaussian_filter1d(stitched_path[:, 0], sigma)
    smoothed_y = gaussian_filter1d(stitched_path[:, 1], sigma)

    return np.column_stack((smoothed_x, smoothed_y))


# visualization----------------------------------------------------------------------------------------------------------------------------------------------------------
def visualize(centre_points, blue_cones, yellow_cones, blue_margin, yellow_margin, segs, spirals, stitched_path, optimal_path):
    plt.figure(figsize=(12, 7))
    # plt.scatter(centre_points[:, 0], centre_points[:, 1], color='red', marker='x', label='Centre Points')
    plt.scatter(blue_cones[:, 0], blue_cones[:, 1], color='blue', marker='o', label='Blue Cones', s=0.2)
    plt.scatter(yellow_cones[:, 0], yellow_cones[:, 1], color='#CCCC00', marker='o', label='Yellow Cones', s=0.2)
    # plt.scatter(blue_margin[:, 0], blue_margin[:, 1], color='blue', marker='o', label='Blue Margin', s=1)
    # plt.scatter(yellow_margin[:, 0], yellow_margin[:, 1], color='yellow', marker='o', label='Yellow Margin', s=1)
    # plt.plot(stitched_path[:, 0], stitched_path[:, 1], color='green', label='Stitched Path', linewidth=3)
    plt.plot(optimal_path[:, 0], optimal_path[:, 1], color='red', label='Optimal Path', linewidth=2)
    # for spiral in spirals:
    #     plt.plot(spiral.spiral[:, 0], spiral.spiral[:, 1], color='green', linewidth=3, label='Spiral Path')
    
    plt.scatter(centre_points[0][0], centre_points[0][1], color='black', zorder=5, s=70, label='Start Point')
    plt.scatter(centre_points[-2][0], centre_points[-2][1], color='purple', zorder=5, s=70, label='End Point')
    
    # # Plot segments
    # for seg in segs:
    #     idx_start = seg['idx_start']
    #     idx_end = seg['idx_end']
    #     seg_points = centre_points[idx_start:idx_end+1]  # +1 because slicing is exclusive
        
    #     if seg['type'] == 'corner':
    #         if seg['turn_dir'] == 'R':
    #             color = 'red'
    #         elif seg['turn_dir'] == 'L':
    #             color = 'cyan'
    #         else:
    #             color = 'gray'
    #     else:
    #         color = 'gray'
        
    #     plt.plot(seg_points[:, 0], seg_points[:, 1], color=color, linewidth=2)

    plt.axis('equal')
    plt.legend()
    plt.title('Bruh Graph with Segments')
    plt.show()


# generation of tracks----------------------------------------------------------------------------------------------------------------------------------------------------------
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

def generate_square_track_with_rounded_corners(side_length=100, corner_radius=20, resolution=1.0):
    """
    Generate a square track with rounded corners.

    Parameters:
    - side_length: Length of one straight side of the square (in meters).
    - corner_radius: Radius of the rounded corners (in meters).
    - resolution: Distance between sampled points (in meters).

    Returns:
    - track_points: List of (x, y) tuples representing the track centerline.
    """
    if 2 * corner_radius >= side_length:
        raise ValueError("Corner radius too large compared to side length")

    track_points = []

    # Define the four corners (center of arc)
    corner_centers = [
        (corner_radius, corner_radius),  # Bottom-left
        (side_length - corner_radius, corner_radius),  # Bottom-right
        (side_length - corner_radius, side_length - corner_radius),  # Top-right
        (corner_radius, side_length - corner_radius)   # Top-left
    ]

    # Define the angles for each quarter circle
    corner_angles = [
        (np.pi, 1.5 * np.pi),    # Bottom-left (left to up)
        (1.5 * np.pi, 2.0 * np.pi),  # Bottom-right (down to right)
        (0, 0.5 * np.pi),        # Top-right (right to up)
        (0.5 * np.pi, np.pi)     # Top-left (up to left)
    ]

    # Generate arcs and straight segments
    for i in range(4):
        cx, cy = corner_centers[i]
        theta_start, theta_end = corner_angles[i]
        theta = np.arange(theta_start, theta_end, resolution / corner_radius)
        arc_x = cx + corner_radius * np.cos(theta)
        arc_y = cy + corner_radius * np.sin(theta)
        track_points.extend(zip(arc_x, arc_y))

        # Connect to next arc with a straight line
        next_i = (i + 1) % 4
        x0 = arc_x[-1]
        y0 = arc_y[-1]
        x1 = corner_centers[next_i][0] + corner_radius * np.cos(corner_angles[next_i][0])
        y1 = corner_centers[next_i][1] + corner_radius * np.sin(corner_angles[next_i][0])

        # Interpolate along straight segment
        length = np.hypot(x1 - x0, y1 - y0)
        steps = max(int(length / resolution), 2)
        straight_x = np.linspace(x0, x1, steps)
        straight_y = np.linspace(y0, y1, steps)
        track_points.extend(zip(straight_x, straight_y))

    return track_points

def generate_flower_track(num_points=500):
    r_large = 50  # radius of outer circles
    r_small = 25  # radius of inner loop
    straight_len = 2 * r_large

    points = []

    # Bottom straight
    for x in np.linspace(-straight_len/2, straight_len/2, num_points//5):
        points.append(np.array([x, -r_large]))

    # Right large semicircle
    for theta in np.linspace(-np.pi/2, np.pi/2, num_points//5):
        x = r_large * np.cos(theta) + straight_len/2
        y = r_large * np.sin(theta)
        points.append(np.array([x, y]))

    # Top small semicircle (inverted, center at 0, r_large - r_small)
    for theta in np.linspace(np.pi/2, -np.pi/2, num_points//5):
        x = r_small * np.cos(theta)
        y = r_small * np.sin(theta) + (r_large - r_small)
        points.append(np.array([x, y]))

    # Left large semicircle
    for theta in np.linspace(np.pi/2, 3*np.pi/2, num_points//5):
        x = r_large * np.cos(theta) - straight_len/2
        y = r_large * np.sin(theta)
        points.append(np.array([x, y]))

    # Close loop: return to start along bottom straight
    for x in np.linspace(-straight_len/2, straight_len/2, num_points//5):
        points.append(np.array([x, -r_large]))

    return points

def generate_ellipse_track(num_points=500):
    a = 100  # semi-major axis (horizontal)
    b = 50   # semi-minor axis (vertical)

    points = []
    for theta in np.linspace(0, 2*np.pi, num_points):
        x = a * np.cos(theta)
        y = b * np.sin(theta)
        points.append(np.array([x, y]))

    return points

def generate_real_track(file_name):
    x = []
    y = []

    # Open and read the file
    with open(file_name, 'r') as file:
        for line in file:
            parts = line.strip().split(',')
            if len(parts) == 4:
                x.append(float(parts[0]))
                y.append(float(parts[1]))

    return np.column_stack((x, y))


# lap time testing----------------------------------------------------------------------------------------------------------------------------------------------------------
def estimate_lap_time_realistic(
    path_points,
    max_lateral_accel_ms2=45.0,
    max_accel_ms2=13.0,
    max_brake_ms2=18.0,
    top_speed_ms=95.0  # ≈ 342 km/h
):
    n = len(path_points)
    distances = np.zeros(n - 1)
    curvatures = np.zeros(n - 2)
    speeds = np.zeros(n)

    # --- 1. Calculate distances between each point ---
    for i in range(n - 1):
        distances[i] = np.linalg.norm(path_points[i + 1] - path_points[i])

    # --- 2. Estimate curvature from angle between vectors ---
    for i in range(1, n - 1):
        vec1 = path_points[i] - path_points[i - 1]
        vec2 = path_points[i + 1] - path_points[i]
        angle = np.arccos(np.clip(np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2)), -1.0, 1.0))
        radius = np.linalg.norm(vec1) / angle if angle != 0 else np.inf
        curvatures[i - 1] = 1.0 / radius if radius != np.inf else 0.0

    # --- 3. Forward pass (acceleration-limited) ---
    speeds[0] = 0.0  # start from rest
    for i in range(n - 2):
        # Max speed from curvature
        corner_speed = min(top_speed_ms, np.sqrt(max_lateral_accel_ms2 / curvatures[i]) if curvatures[i] > 0 else top_speed_ms)
        # Accelerate toward that corner
        v_possible = np.sqrt(speeds[i]**2 + 2 * max_accel_ms2 * distances[i])
        speeds[i + 1] = min(v_possible, corner_speed)

    # --- 4. Backward pass (braking-limited) ---
    speeds[-1] = 0.0  # assume car stops at end
    for i in reversed(range(1, n - 1)):
        v_possible = np.sqrt(speeds[i + 1]**2 + 2 * max_brake_ms2 * distances[i])
        speeds[i] = min(speeds[i], v_possible)

    # --- 5. Compute time per segment ---
    total_time = 0.0
    for i in range(n - 1):
        if speeds[i] == 0 and speeds[i + 1] == 0:
            continue  # avoid div by zero
        avg_speed = (speeds[i] + speeds[i + 1]) / 2
        total_time += distances[i] / avg_speed

    return total_time


# main----------------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    # Generation----------------------------------------------------------------------------------------------------------------------------------------------------------
    if False:
        pass
        # centre_points = generate_sine_perturbed_circle()
        # centre_points = generate_long_straight_track()
        # centre_points = generate_oval_track()
        # centre_points = generate_square_track_with_rounded_corners()
        # centre_points = generate_flower_track()
        # centre_points = generate_ellipse_track()
        # centre_points = generate_real_track('berlin_2018.txt')
        # centre_points = generate_real_track('modena_2019.txt')
    # centre_points = generate_square_track_with_rounded_corners()
    centre_points = generate_real_track('modena_2019.txt')

    centre_points = np.array(centre_points)
    cone_distance_from_centre_points = 5
    margin = 0.85
    blue_cones, yellow_cones = generate_cones(centre_points, offset=cone_distance_from_centre_points)
    blue_margin, yellow_margin = generate_cones(centre_points, offset=cone_distance_from_centre_points-margin) # car is approx 1.7m in width, so half of that

    # Corner Identification----------------------------------------------------------------------------------------------------------------------------------------------------------
    fits = segment_centreline_and_fit_arcs(centre_points, window_length=20, step_size=1)
    fits = classify_arcs_vs_straights(fits,
                                      max_rms_err=1.0,
                                      min_arc_angle=0.15,
                                      max_R_for_corner=61)
    segs = consolidate_segments(fits, min_straight_len=5)
    corns = characterize_corners(segs, centre_points)
    segs = attach_turn_directions(segs, corns)
    segs = break_apart_severe_corners_and_join_straights(centre_points, segs)
    # Path Planning----------------------------------------------------------------------------------------------------------------------------------------------------------
    ranked_corners = rank_corners(centre_points, segs)
    spirals = euler_spirals(centre_points, blue_margin, yellow_margin, segs, ranked_corners, threshold_distance=1)
    stitched_path = stitch_path(centre_points, segs, spirals, straight_sample_step=3)
    optimal_path = smooth_path(stitched_path, sigma=3)
    # optimal_path = stitched_path

    # Visualization & Testing----------------------------------------------------------------------------------------------------------------------------------------------------------
    print(f"Optimal Path Lap Time: {estimate_lap_time_realistic(optimal_path)}s")
    print(f"Centreline Lap Time: {estimate_lap_time_realistic(centre_points)}s")
    visualize(centre_points, blue_cones, yellow_cones, blue_margin, yellow_margin, segs, spirals, stitched_path, optimal_path)






