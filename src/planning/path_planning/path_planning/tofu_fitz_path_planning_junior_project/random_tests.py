import numpy as np
import matplotlib.pyplot as plt
from scipy.special import fresnel
from pyclothoids import Clothoid
from scipy.optimize import minimize_scalar

def first_one():
    
    # Total arc length
    L = 10

    # Number of points
    n_points = 1000

    # s: arc length parameter
    s = np.linspace(0, L, n_points)

    # Scale factor (a) for curvature κ(s) = s / a²
    a = 1.0
    b = 2.0

    # Compute scaled Fresnel integrals
    t = s / (np.sqrt(2 * np.pi) * a)
    S, C = fresnel(t)

    # Coordinates of the spiral
    x = a * np.sqrt(2 * np.pi) * C
    y = a * np.sqrt(2 * np.pi) * S

    # Plot the Euler spiral
    plt.figure(figsize=(8, 6))
    plt.plot(x, y, label='Euler Spiral A')
    # Compute scaled Fresnel integrals
    t = s / (np.sqrt(2 * np.pi) * b)
    S, C = fresnel(t)

    # Coordinates of the spiral
    x = b * np.sqrt(2 * np.pi) * C
    y = b * np.sqrt(2 * np.pi) * S
    plt.plot(x, y, label='Euler Spiral B')




    plt.axis('equal')
    plt.grid(True)
    plt.title('Euler (Clothoid) Spiral')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.show()

def second_one():
    def euler_spiral(L=10, n_points=1000, a=1.0, theta_deg=0):
        # s: arc length
        s = np.linspace(0, L, n_points)
        
        # Fresnel parameter
        t = s / (np.sqrt(2 * np.pi) * a)
        S, C = fresnel(t)

        # Base coordinates of Euler spiral
        scale = a * np.sqrt(2 * np.pi)
        x = scale * C
        y = scale * S

        # Convert angle to radians
        theta = np.radians(theta_deg)

        # Apply rotation
        x_rot = x * np.cos(theta) - y * np.sin(theta)
        y_rot = x * np.sin(theta) + y * np.cos(theta)

        return x_rot, y_rot

    # Change this angle to control starting direction (0 = right, 90 = up, etc.)

    # Get spiral
    x, y = euler_spiral(theta_deg=180)
    x2, y2 = euler_spiral(theta_deg=45)
    x3, y3 = euler_spiral(a=0.5, theta_deg=45)

    # Plot
    plt.figure(figsize=(8, 6))
    plt.plot(x, y, label=f'Euler Spiral @ {180}°')
    plt.plot(x2, y2, label=f'Euler Spiral @ {45}°')
    plt.plot(x3, y3, label=f'Euler Spiral @ {45}°')
    plt.axis('equal')
    plt.grid(True)
    plt.title('Euler (Clothoid) Spiral with Custom Starting Direction')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.show()

def third_one():


    def euler_spiral(L=10, n_points=1000, a=1.0, theta_deg=0, flip=False):
        # s: arc length
        s = np.linspace(0, L, n_points)

        # Fresnel parameter
        t = s / (np.sqrt(2 * np.pi) * a)
        S, C = fresnel(t)

        # Base coordinates
        scale = a * np.sqrt(2 * np.pi)
        x = scale * C
        y = scale * S

        # Flip the spiral across the tangent (horizontal axis)
        if flip:
            y = -y  # This reverses curvature (anticlockwise <-> clockwise)

        # Convert angle to radians
        theta = np.radians(theta_deg)

        # Apply rotation
        x_rot = x * np.cos(theta) - y * np.sin(theta)
        y_rot = x * np.sin(theta) + y * np.cos(theta)

        return x_rot, y_rot

    # === Parameters ===
    theta_deg = 90   # Start direction: 90° = up
    flip = True      # Flip: False = anticlockwise, True = clockwise

    # Generate spiral
    x, y = euler_spiral(theta_deg=theta_deg, flip=flip)

    # Plot
    plt.figure(figsize=(8, 6))
    plt.plot(x, y, label=f'Euler Spiral @ {theta_deg}° {"(clockwise)" if flip else "(anticlockwise)"}')
    plt.axis('equal')
    plt.grid(True)
    plt.title('Euler (Clothoid) Spiral with Flip and Heading Control')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.show()


def fourth_one():

    def euler_spiral(a=1.0, theta_start_deg=0, delta_theta_deg=60, n_points=1000, flip=False):
        # Convert to radians
        theta_start = np.radians(theta_start_deg)
        delta_theta = np.radians(delta_theta_deg)

        # Flip direction if needed (negate angle change)
        if flip:
            delta_theta = -delta_theta

        # Compute arc length required for this delta_theta
        s_stop = np.sqrt(4 * a**2 * abs(delta_theta))
        s = np.linspace(0, s_stop, n_points)

        # Normalized Fresnel parameter
        t = s / (np.sqrt(2 * np.pi) * a)
        S, C = fresnel(t)

        # Base spiral (unrotated)
        scale = a * np.sqrt(2 * np.pi)
        x = scale * C
        y = scale * S

        # If flipped, mirror across x-axis
        if flip:
            y = -y

        # Rotate spiral to desired starting heading
        x_rot = x * np.cos(theta_start) - y * np.sin(theta_start)
        y_rot = x * np.sin(theta_start) + y * np.cos(theta_start)

        return x_rot, y_rot

    # === Example Test ===
    theta_start_deg = 90          # Start pointing up (relative to positive x axis)
    delta_theta_deg = 90          # Turn left 90 deg (so end pointing left) reltaive to theta_start deg
    flip = True                  # Counterclockwise turn

    x, y = euler_spiral(theta_start_deg=theta_start_deg,
                        delta_theta_deg=delta_theta_deg,
                        flip=flip)

    # Plot
    plt.figure(figsize=(8, 6))
    plt.plot(x, y, label=f"Start: {theta_start_deg}°, Turn: {delta_theta_deg}° {'CW' if flip else 'CCW'}")
    plt.axis('equal')
    plt.grid(True)
    plt.title('Euler Spiral (Relative Heading Change)')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.show()

def fifth_one():

    from scipy.optimize import minimize_scalar

    def generate_unit_euler(delta_theta_rad, n_points=500):
        """Generate a unit Euler spiral for a given delta_theta (relative)."""
        a = 1.0  # unit scale
        s_stop = np.sqrt(4 * a**2 * abs(delta_theta_rad))
        s = np.linspace(0, s_stop, n_points)
        t = s / (np.sqrt(2 * np.pi) * a)
        S, C = fresnel(t)

        scale = a * np.sqrt(2 * np.pi)
        x = scale * C
        y = scale * S
        if delta_theta_rad < 0:
            y = -y  # flip for clockwise
        return x, y

    def transform_spiral(x, y, x_start, y_start, theta_start_rad, scale):
        """Scale, rotate, and translate the spiral to start from (x_start, y_start) with heading theta_start."""
        x_scaled = x * scale
        y_scaled = y * scale
        x_rot = x_scaled * np.cos(theta_start_rad) - y_scaled * np.sin(theta_start_rad)
        y_rot = x_scaled * np.sin(theta_start_rad) + y_scaled * np.cos(theta_start_rad)
        return x_rot + x_start, y_rot + y_start

    def connect_points_euler_relative(A, B, theta_start_deg, delta_theta_deg, n_points=500, tol=1e-3):
        # Convert to radians
        theta_start = np.radians(theta_start_deg)
        delta_theta = np.radians(delta_theta_deg)

        # Generate unit Euler spiral with relative angle change
        x_base, y_base = generate_unit_euler(delta_theta, n_points)

        # Optimize scale
        def objective(scale):
            x_trans, y_trans = transform_spiral(x_base, y_base, A[0], A[1], theta_start, scale)
            dx = x_trans[-1] - B[0]
            dy = y_trans[-1] - B[1]
            return dx**2 + dy**2

        result = minimize_scalar(objective, bounds=(0.01, 100), method='bounded')
        best_scale = result.x

        # Reconstruct final spiral
        x_final, y_final = transform_spiral(x_base, y_base, A[0], A[1], theta_start, best_scale)

        return x_final, y_final, best_scale, result.fun < tol

    # === Example ===
    A = (0, 0)
    B = (25, 35)
    C = (45, 87)
    theta_start_deg = 0           # Start facing up
    delta_theta_deg = 70          # Curve right (CW) to face left
    final_theta_deg = 20
    # x, y, scale_used, success = connect_points_euler_relative(A, B, theta_start_deg, delta_theta_deg)
    # BB = (x[-1], y[-1])
    x2, y2, scale_used2, success = connect_points_euler_relative(A, C, theta_start_deg, delta_theta_deg + final_theta_deg)


    # === Plot ===
    plt.figure(figsize=(8, 6))
    # plt.plot(x, y, label=f'Euler Spiral (Scale: {scale_used:.2f})')
    plt.plot(x2, y2, label=f'Euler Spiral (Scale: {scale_used2:.2f})')
    plt.plot(*A, 'go', label='Start A')
    plt.plot(*B, 'ro', label='Mid B')
    plt.plot(*C, 'bo', label='End C')
    plt.quiver(*A, np.cos(np.radians(theta_start_deg)), np.sin(np.radians(theta_start_deg)),
            scale=5, color='green', label='Heading A')
    theta_end_deg = theta_start_deg + delta_theta_deg
    plt.quiver(*B, np.cos(np.radians(theta_end_deg)), np.sin(np.radians(theta_end_deg)),
            scale=5, color='red', label='Heading B')
    final_theta_deg += theta_end_deg
    plt.quiver(*C, np.cos(np.radians(final_theta_deg)), np.sin(np.radians(final_theta_deg)),
            scale=5, color='blue', label='Heading C')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.title('Euler Spiral Connecting Two Points with Relative Angle')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.show()

    print("Success:", success)

def sixth_one():

    from pyclothoids import Clothoid
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

        return x_vals, y_vals, clothoid


    from scipy.optimize import minimize_scalar

    def generate_unit_euler(delta_theta_rad, n_points=500):
        """Generate a unit Euler spiral for a given delta_theta (relative)."""
        a = 1.0  # unit scale
        s_stop = np.sqrt(4 * a**2 * abs(delta_theta_rad))
        s = np.linspace(0, s_stop, n_points)
        t = s / (np.sqrt(2 * np.pi) * a)
        S, C = fresnel(t)

        scale = a * np.sqrt(2 * np.pi)
        x = scale * C
        y = scale * S
        if delta_theta_rad < 0:
            y = -y  # flip for clockwise
        return x, y

    def transform_spiral(x, y, x_start, y_start, theta_start_rad, scale):
        """Scale, rotate, and translate the spiral to start from (x_start, y_start) with heading theta_start."""
        x_scaled = x * scale
        y_scaled = y * scale
        x_rot = x_scaled * np.cos(theta_start_rad) - y_scaled * np.sin(theta_start_rad)
        y_rot = x_scaled * np.sin(theta_start_rad) + y_scaled * np.cos(theta_start_rad)
        return x_rot + x_start, y_rot + y_start

    def connect_points_euler_relative(A, B, theta_start_deg, delta_theta_deg, n_points=500, tol=1e-3):
        # Convert to radians
        theta_start = np.radians(theta_start_deg)
        delta_theta = np.radians(delta_theta_deg)

        # Generate unit Euler spiral with relative angle change
        x_base, y_base = generate_unit_euler(delta_theta, n_points)

        # Optimize scale
        def objective(scale):
            x_trans, y_trans = transform_spiral(x_base, y_base, A[0], A[1], theta_start, scale)
            dx = x_trans[-1] - B[0]
            dy = y_trans[-1] - B[1]
            return dx**2 + dy**2

        result = minimize_scalar(objective, bounds=(0.01, 100), method='bounded')
        best_scale = result.x

        # Reconstruct final spiral
        x_final, y_final = transform_spiral(x_base, y_base, A[0], A[1], theta_start, best_scale)

        return x_final, y_final, best_scale, result.fun < tol

    # Example usage
    if __name__ == "__main__":


        # Define start and end points with headings
        A = (0, 0)             # Start point
        B = (25, 35)           # End point
        C = (28, 87)
        theta_start_deg = 0   # Start heading in degrees
        theta_end_deg = 70    # End heading in degrees
        final_theta_deg = 90

        proportion = 0.5
        
        if True:
            pass
            # AVERAGE_BC = ((B[0]+C[0])/2,(B[1]+C[1])/2)
            # BBC = (25, 87) 
            
            # average_theta_deg = (final_theta_deg + theta_end_deg) / 2

            # # Generate the clothoid, first half, second half style
            # x_vals, y_vals, clothoid = connect_points_with_clothoid(A, B, theta_start_deg, theta_end_deg)
            # BB = (x_vals[-1], y_vals[-1])
            # x2, y2, clothoid2 = connect_points_with_clothoid(C, B, final_theta_deg+180, theta_end_deg + 180)

            # # generatre clothoid directly
            # x3, y3, clothoid3 = connect_points_with_clothoid(A, C, theta_start_deg, final_theta_deg, n_points=1000)

            # #average clothoid of the two:
            # x_halves = x_vals + x2
            # y_halves = y_vals + y2

            # average_x = [(x_halves[i]+x3[i])/2 for i in range(len(x_halves))]
            # average_y = [(y_halves[i]+y3[i])/2 for i in range(len(y_halves))]

            # #AIM AT AVERAGE:
            # # averx, avery, clothoid3 = connect_points_with_clothoid(A, AVERAGE_BC, theta_start_deg, average_theta_deg)
            # # divergex, divergey, clothoid4 = connect_points_with_clothoid(A, C, theta_start_deg, theta_end_deg)
            # # huggx, huggy, clothoid5 = connect_points_with_clothoid(A, BBC, theta_start_deg, final_theta_deg)

            # CBB = (30, 35) 
            # apexendx, apexendy, clothoid5 = connect_points_with_clothoid(A, CBB, theta_start_deg, theta_end_deg)
            # apexendx2, apexendy2, clothoid5 = connect_points_with_clothoid(C, CBB, (final_theta_deg + 180) % 360, (theta_end_deg + 180) % 360)
            

            # # axex, axey, scale_used, success = connect_points_euler_relative(A, B, theta_start_deg, theta_end_deg - theta_start_deg)
            # # axex2, axey2, scale_used, success = connect_points_euler_relative(C, B, 270, -20)
            # axex2, axey2, scale_used, success = connect_points_euler_relative(CBB, C, theta_end_deg, final_theta_deg-theta_end_deg)

            # # generatre clothoid directly, but changfes ending heading so it touches tangent
            # x4, y4, clothoid3 = connect_points_with_clothoid(A, C, theta_start_deg, final_theta_deg-70)
            # testx, testy, clothoid3 = connect_points_with_clothoid(A, B, theta_start_deg, theta_end_deg)
            # test2x, test2y, clothoid3 = connect_points_with_clothoid(A, B, theta_start_deg, 0)
        
        xfh, yfh, clothoidfh = connect_points_with_clothoid(A, B, theta_start_deg, theta_end_deg)
        xsh, ysh, clothoidsh = connect_points_with_clothoid(B, C, theta_end_deg, final_theta_deg)
        x_direct, y_direct, clothoid_direct = connect_points_with_clothoid(A, C, theta_start_deg, final_theta_deg, n_points=1000)

        x_full = xfh + xsh
        y_full = yfh + ysh

        x_complete = [(x_full[i] * proportion + x_direct[i] * (1-proportion)) for i in range(len(x_direct))]
        y_complete = [(y_full[i] * proportion + y_direct[i] * (1-proportion)) for i in range(len(x_direct))]




        # Plot the clothoid
        plt.figure(figsize=(12, 9))
        plt.plot(x_full, y_full, label='Partitioned')
        plt.plot(x_direct, y_direct, label='Direct')
        plt.plot(x_complete, y_complete, label='Proportioned')
        plt.plot(*A, 'go', label='Start Point A')
        plt.plot(*B, 'ro', label='Apex Point B')
        plt.plot(*C, 'bo', label='End Point C')

        # Plot start and end headings
        plt.quiver(*A, np.cos(np.radians(theta_start_deg)), np.sin(np.radians(theta_start_deg)),
                scale=5, color='green', label='Start Heading')
        plt.quiver(*B, np.cos(np.radians(theta_end_deg)), np.sin(np.radians(theta_end_deg)),
                scale=5, color='red', label='Apex Heading')
        plt.quiver(*C, np.cos(np.radians(final_theta_deg)), np.sin(np.radians(final_theta_deg)),
                scale=5, color='blue', label='End Heading')

        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.title('Clothoid Connecting Two Points with Specified Headings')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show()

        # Print clothoid parameters
        # print("Clothoid Parameters:")
        # print(f"  Length (L): {clothoid.L}")
        # print(f"  Start Curvature (kappa0): {clothoid.KappaStart}")
        # print(f"  End Curvature (kappa1): {clothoid.KappaEnd}")
        # print(f"  Curvature Rate (dk): {clothoid.dk}")


    # fifth_one()

sixth_one()