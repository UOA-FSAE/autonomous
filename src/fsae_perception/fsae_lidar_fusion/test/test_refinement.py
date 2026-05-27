"""Unit tests for the ROS-free LiDAR refinement primitives (Component A)."""

import numpy as np

from fsae_lidar_fusion.refinement import crop_sphere


def _make_cloud(points):
    """Build an (N, 5) cloud (x, y, z, intensity, ring) from xyz triples."""
    pts = np.asarray(points, dtype=float)
    out = np.zeros((pts.shape[0], 5), dtype=float)
    out[:, :3] = pts
    return out


def test_crop_sphere_keeps_only_inside_points():
    """crop_sphere returns exactly the points within r of the seed."""
    seed = (0.0, 0.0, 0.0)
    inside = [
        (0.0, 0.0, 0.0),     # at the seed
        (0.5, 0.0, 0.0),     # 0.5 m away
        (0.0, -0.9, 0.0),    # 0.9 m away
        (0.3, 0.3, 0.3),     # ~0.52 m away
    ]
    outside = [
        (1.5, 0.0, 0.0),     # 1.5 m away
        (0.0, 0.0, 2.0),     # 2.0 m away
        (1.0, 1.0, 1.0),     # ~1.73 m away
    ]
    cloud = _make_cloud(inside + outside)

    cropped = crop_sphere(cloud, seed, r_sphere=1.0)

    assert cropped.shape[0] == len(inside)
    # every returned point is genuinely within the radius
    d = np.linalg.norm(cropped[:, :3] - np.asarray(seed), axis=1)
    assert np.all(d <= 1.0 + 1e-9)


def test_crop_sphere_offset_seed():
    """Cropping works when the seed is not at the origin."""
    seed = (2.0, -1.0, 0.5)
    cloud = _make_cloud([
        (2.0, -1.0, 0.5),    # at seed
        (2.4, -1.0, 0.5),    # 0.4 m
        (5.0, -1.0, 0.5),    # 3.0 m -> excluded
    ])
    cropped = crop_sphere(cloud, seed, r_sphere=1.0)
    assert cropped.shape[0] == 2


def test_crop_sphere_empty_cloud():
    """An empty cloud yields an empty (0, 5) result, not an error."""
    cloud = np.empty((0, 5), dtype=float)
    cropped = crop_sphere(cloud, (0.0, 0.0, 0.0), r_sphere=1.0)
    assert cropped.shape == (0, 5)


def test_crop_sphere_boundary_inclusive():
    """A point exactly on the radius is kept (<= comparison)."""
    cloud = _make_cloud([(1.0, 0.0, 0.0), (1.0001, 0.0, 0.0)])
    cropped = crop_sphere(cloud, (0.0, 0.0, 0.0), r_sphere=1.0)
    assert cropped.shape[0] == 1
