import numpy as np
import matplotlib.pyplot as plt
from scipy.special import fresnel

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
