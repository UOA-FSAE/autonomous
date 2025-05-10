import numpy as np
import matplotlib.pyplot as plt
import time
from typing import List, Tuple, Dict, Any, Optional
from joblib import Parallel, delayed
from collections import defaultdict
from scipy.ndimage import gaussian_filter1d


# Globals-----------------------------------------------------------------------------------------------------------------------------------------------------------
# car parameters-----------------------------------------------------------------------------------------------------------------------------------------------------------
max_lateral_accel_ms2=45.0
max_accel_ms2=13.0
max_brake_ms2=18.0
top_speed_ms=95.0  # ≈ 342 km/h
mass = 225 # kg
F_max = max_accel_ms2 * mass # Newtons
k_a= 0.1 # air drag coefficient, make 0 if not known

# thresholds-----------------------------------------------------------------------------------------------------------------------------------------------------------
max_direction_delta_deg = 70
cone_distance_from_centre_points = 5
margin = 0.85
angle_close_enough_threshold = max_direction_delta_deg//2 # degrees

# state sample parameters-----------------------------------------------------------------------------------------------------------------------------------------------------------
state_speed_sample = 6
state_direction_sample = 9
lateral_slices = 6
# longitudinal_slices = 100 # approximate

corner_longitudinal_slices = 10 # approximate
straight_longitudinal_slices = 10 # approximate

# Fitz's Identification----------------------------------------------------------------------------------------------------------------------------------------------------------
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

# Intermediataries------------------------------------------------------------------------------------------------------------------------------------------------------
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

def identify_segs(centre_points, window_length=20, max_R_for_corner=61):
    fits = segment_centreline_and_fit_arcs(centre_points, window_length=20, step_size=1)
    fits = classify_arcs_vs_straights(fits,
                                      max_rms_err=1.0,
                                      min_arc_angle=0.15,
                                      max_R_for_corner=61)
    segs = consolidate_segments(fits, min_straight_len=5)
    corns = characterize_corners(segs, centre_points)
    segs = attach_turn_directions(segs, corns)
    segs = break_apart_severe_corners_and_join_straights(centre_points, segs)

    return segs

# Tofu's Planning-----------------------------------------------------------------------------------------------------------------------------------------------------------

def setup_paramaters():
    state_speeds = np.linspace(0, top_speed_ms, state_speed_sample).tolist()
    state_relative_angles_deg = np.linspace(-max_direction_delta_deg, max_direction_delta_deg, state_direction_sample).tolist()

    return state_speeds, state_relative_angles_deg

def optimize_track(centre, left, right, segs):
    left_optimized = []
    right_optimized = []
    centre_optimized = []

    for seg in segs:
        start = seg['idx_start']
        end = seg['idx_end']
        slice_factor = corner_longitudinal_slices if seg['type'] == 'corner' else straight_longitudinal_slices
        sample = max((end - start) // slice_factor, 1)
        left_optimized.extend(left[start:end:sample])
        right_optimized.extend(right[start:end:sample])
        centre_optimized.extend(centre[start:end:sample])

    longitudinal_slices = len(centre_optimized)

    return np.array(centre_optimized), np.array(left_optimized), np.array(right_optimized), longitudinal_slices

class State:
    def __init__(self, position, speed, direction_deg, costs=None):
        self.position = position
        self.speed = speed
        self.direction = direction_deg
        self.costs = [] if costs is None else costs

def spawn_states(centre, left, right, speeds, angles, longitudinal_slices):
    sample = max(len(left) // longitudinal_slices, 1)
    left = left[::sample]
    right = right[::sample]
    centre = centre[::sample]

    states = {}  # (i, j, k, m) -> State
    global angle_vectors
    angle_vectors = {angle: np.array([np.cos(np.radians(angle)), np.sin(np.radians(angle))]) for angle in angles}

    for i in range(longitudinal_slices):
        displacement = right[i] - left[i]
        gradient = displacement / (lateral_slices - 1)

        if i < (longitudinal_slices - 1):
            tangent_vec = centre[i + 1] - centre[i]
        else:
            tangent_vec = centre[i] - centre[i - 1]
        tangent = np.degrees(np.arctan2(tangent_vec[1], tangent_vec[0]))

        for j in range(lateral_slices):
            position = left[i] + gradient * j
            for k, speed in enumerate(speeds):
                for m, angle in enumerate(angles):
                    direction = tangent + angle
                    states[(i, j, k, m)] = State(position, speed, direction)
    return states

def compute_single_state_costs(i, j, k, m, states, speeds, angles,
                               direction_threshold, F_max, mass, k_a,
                               lateral_slices):
    A = states[(i, j, k, m)]
    A_dir_rad = np.radians(A.direction)
    A_dir_vec = np.array([np.cos(A_dir_rad), np.sin(A_dir_rad)])
    A_velocity = A_dir_vec * A.speed
    cost_list = []

    for dj in [-1, 0, 1]:
        j2 = j + dj
        if not (0 <= j2 < lateral_slices): continue

        for dk in [-1, 0, 1]:
            k2 = k + dk
            if not (0 <= k2 < len(speeds)): continue

            for dm in [-1, 0, 1]:
                m2 = m + dm
                if not (0 <= m2 < len(angles)): continue

                B_index = (i + 1, j2, k2, m2)
                B = states.get(B_index)
                if B is None: continue

                B_dir_rad = np.radians(B.direction)
                B_dir_vec = np.array([np.cos(B_dir_rad), np.sin(B_dir_rad)])
                B_velocity = B_dir_vec * B.speed

                avg_velocity = (A_velocity + B_velocity) / 2
                ds_vector = B.position - A.position
                ds_mag = np.linalg.norm(ds_vector)
                avg_v_mag = np.linalg.norm(avg_velocity)

                if ds_mag == 0 or avg_v_mag == 0:
                    continue

                dt = ds_mag / avg_v_mag
                cos_theta = np.dot(ds_vector, avg_velocity) / (ds_mag * avg_v_mag)

                if cos_theta < direction_threshold:
                    continue

                dv = B_velocity - A_velocity
                F = (mass * dv / dt) + k_a * avg_velocity * avg_v_mag
                F_mag = np.linalg.norm(F)

                if F_mag > F_max:
                    continue

                cost = dt
                cost_list.append((cost, B_index))

    return ((i, j, k, m), cost_list)

def compute_transition_costs(states, longitudinal_slices, lateral_slices, speeds, angles,
                              angle_threshold_deg=45.0, F_max=10.0, mass=1.0, k_a=0.1):
    direction_threshold = np.cos(np.radians(angle_threshold_deg))

    results = Parallel(n_jobs=-1, prefer="threads")(
        delayed(compute_single_state_costs)(i, j, k, m, states, speeds, angles,
                                            direction_threshold, F_max, mass, k_a,
                                            lateral_slices)
        for i in range(longitudinal_slices - 1)
        for j in range(lateral_slices)
        for k in range(len(speeds))
        for m in range(len(angles))
    )

    for (i, j, k, m), costs in results:
        states[(i, j, k, m)].costs = costs

def compute_optimal_path(states, longitudinal_slices, lateral_slices, speeds, angles):
    for j in range(lateral_slices):
        for k in range(len(speeds)):
            for m in range(len(angles)):
                state = states[(longitudinal_slices - 1, j, k, m)]
                state.optimal_cost = 0
                state.next_state = None

    for i in reversed(range(longitudinal_slices - 1)):
        for j in range(lateral_slices):
            for k in range(len(speeds)):
                for m in range(len(angles)):
                    current_state = states[(i, j, k, m)]
                    best_cost = float('inf')
                    best_next = None

                    for link_cost, next_idx in current_state.costs:
                        next_state = states[next_idx]
                        total_cost = link_cost + getattr(next_state, 'optimal_cost', float('inf'))
                        if total_cost < best_cost:
                            best_cost = total_cost
                            best_next = next_idx

                    current_state.optimal_cost = best_cost
                    current_state.next_state = best_next

    min_cost = float('inf')
    best_start_idx = None

    for j in range(lateral_slices):
        for k in range(len(speeds)):
            for m in range(len(angles)):
                idx = (0, j, k, m)
                state = states[idx]
                if state.optimal_cost < min_cost:
                    min_cost = state.optimal_cost
                    best_start_idx = idx

    path = []
    current_idx = best_start_idx
    while current_idx is not None:
        state = states[current_idx]
        path.append(state.position)
        current_idx = state.next_state

    return np.array(path)

def smooth_path(stitched_path, sigma=2):
    """
    Smooths a path using a Gaussian filter.

    Parameters:
    - stitched_path: np.ndarray of shape (N, 2) representing [x, y] coordinates.
    - sigma: float, standard deviation for Gaussian kernel.

    Returns:
    - np.ndarray of same shape (N, 2), smoothed path.
    """
    try:
        if stitched_path.ndim != 2 or stitched_path.shape[1] != 2:
            raise ValueError("stitched_path must be a 2D array with shape (N, 2)")

        # Apply Gaussian filter separately to x and y coordinates
        smoothed_x = gaussian_filter1d(stitched_path[:, 0], sigma)
        smoothed_y = gaussian_filter1d(stitched_path[:, 1], sigma)

        return np.column_stack((smoothed_x, smoothed_y))

    except:
        return stitched_path



# Track Generation-----------------------------------------------------------------------------------------------------------------------------------------------------------
def generate_real_track(file_name):
    x = []
    y = []
    file_name = r'src/planning/path_planning/path_planning/tofu_fitz_path_planning_junior_project/test_tracks/' + file_name + r'.txt'

    # Open and read the file
    with open(file_name, 'r') as file:
        for line in file:
            parts = line.strip().split(',')
            if len(parts) > 1:
                x.append(float(parts[0]))
                y.append(float(parts[1]))

    return np.column_stack((x, y))


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


# Lap Time Testing----------------------------------------------------------------------------------------------------------------------------------------------------------
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


# Visualization----------------------------------------------------------------------------------------------------------------------------------------------------------
def visualize(path, centre, left, right, show_segs=False, show_debug=False, centre_optimized=[], left_optimized=[], right_optimized=[]):
    plt.figure(figsize=(12, 7))

    if show_debug:
        plt.plot(left_optimized[:, 0], left_optimized[:, 1], color='blue', label='Optimized Blue Cones', s=2)
        plt.plot(right_optimized[:, 0], right_optimized[:, 1], color='yellow', label='Optimized Yellow Cones', s=2)
        plt.plot(centre_optimized[:, 0], centre_optimized[:, 1], color='gray', label='Optimized Centreline', s=2)

    plt.scatter(left[:, 0], left[:, 1], color='blue', label='Blue Cones', s=2)
    plt.scatter(right[:, 0], right[:, 1], color='yellow', label='Yellow Cones', s=2)
    plt.plot(path[:, 0], path[:, 1], color='red', label='Stitched Path', linewidth=3)
    



    if show_segs:
        for seg in segs:
            idx_start = seg['idx_start']
            idx_end = seg['idx_end']
            seg_points = centre[idx_start:idx_end+1]  # +1 because slicing is exclusive
            
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
    plt.title('Dynamic Programming Optimal Path')
    plt.show()


# Main----------------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    start_time = time.time()

    # Generation----------------------------------------------------------------------------------------------------------------------------------------------------------
    centre = generate_real_track('BrandsHatch')
    left, right = generate_cones(centre, offset=cone_distance_from_centre_points)
    left_margin, right_margin = generate_cones(centre, offset=cone_distance_from_centre_points-margin) # car is approx 1.7m in width, so half of that

    state_speeds, state_relative_angles_deg = setup_paramaters()
    # segs = identify_segs(centre, window_length=20, max_R_for_corner=61)

    # centre_optimized, left_optimized, right_optimized, longitudinal_slices = optimize_track(centre, left_margin, right_margin, segs)
    # states = spawn_states(centre_optimized, left_optimized, right_optimized, state_speeds, state_relative_angles_deg, longitudinal_slices)
    longitudinal_slices = 100
    states = spawn_states(centre, left_margin, right_margin, state_speeds, state_relative_angles_deg, longitudinal_slices)
    compute_transition_costs(states, longitudinal_slices, lateral_slices, state_speeds, state_relative_angles_deg, angle_threshold_deg=angle_close_enough_threshold, F_max=F_max, mass=mass, k_a=k_a)
    optimal_path = compute_optimal_path(states, longitudinal_slices, lateral_slices, state_speeds, state_relative_angles_deg)
    
    # Visualization & Testing----------------------------------------------------------------------------------------------------------------------------------------------------------
    
    end_time = time.time()
    optimal_path_time = estimate_lap_time_realistic(optimal_path)
    centreline_time = estimate_lap_time_realistic(centre)
    reduction = (1-(optimal_path_time/centreline_time))*100
    print(f"Execution time: {end_time - start_time:.3g}s")
    print(f"Optimal Path Lap Time: {optimal_path_time}s")
    print(f"Centreline Lap Time: {centreline_time}s")
    print(f"Lap time optimized by {reduction:.3g}%.")
    visualize(optimal_path, centre, left, right, show_segs=False, show_debug=False)#, centre_optimized=centre_optimized, left_optimized=left_optimized, right_optimized=right_optimized)
