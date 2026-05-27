"""LiDAR cone-refinement algorithm (Component A) — pure numpy, no ROS imports.

This module is kept free of any ROS dependency so the refinement maths can be
unit-tested on synthetic point clouds with plain ``pytest``. The ROS-coupled
parts (parsing a ``PointCloud2`` into a numpy array, TF, publishing) live in
``fusion_node.py``.

It also deliberately avoids SciPy/scikit-learn: the target Jetson ships a
system SciPy built against NumPy 1.x while NumPy 2.x is installed, which makes
SciPy's compiled extensions unimportable. Everything here is plain NumPy, which
is fast enough — the sphere crop reduces tens of thousands of points to a small
local subset before any further work.

Point-cloud convention used throughout: an ``(N, 5)`` ``float`` array whose
columns are ``(x, y, z, intensity, ring)``.

Chunk 4 implements only the sphere crop (step A1). Ring bucketing, ground
removal, clustering and the circle fit are added in later chunks.
"""

import numpy as np

# Column indices for the (N, 5) point array.
X, Y, Z, INTENSITY, RING = 0, 1, 2, 3, 4


def crop_sphere(cloud: np.ndarray, seed_xyz, r_sphere: float) -> np.ndarray:
    """Return the subset of ``cloud`` within ``r_sphere`` of ``seed_xyz``.

    Keeps points satisfying ``(x-sx)^2 + (y-sy)^2 + (z-sz)^2 <= r_sphere^2``,
    using a vectorised squared-distance test (no kD-tree, no SciPy).

    Parameters
    ----------
    cloud : np.ndarray
        ``(N, 5)`` array of ``(x, y, z, intensity, ring)``.
    seed_xyz : array-like
        3D seed point ``(sx, sy, sz)`` already expressed in the LiDAR frame.
    r_sphere : float
        Sphere radius in metres.

    Returns
    -------
    np.ndarray
        ``(M, 5)`` subset of the input points inside the sphere (possibly empty).
    """
    cloud = np.asarray(cloud, dtype=float)
    if cloud.size == 0:
        return cloud.reshape(0, 5)
    seed = np.asarray(seed_xyz, dtype=float)
    diff = cloud[:, :3] - seed
    dist_sq = np.einsum('ij,ij->i', diff, diff)
    return cloud[dist_sq <= r_sphere * r_sphere]
