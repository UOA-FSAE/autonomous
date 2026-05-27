"""LiDAR cone-refinement algorithm (Component A) — pure numpy, no ROS imports.

This module is kept free of any ROS dependency so the refinement maths can be
unit-tested on synthetic point clouds with plain ``pytest``. The ROS-coupled
parts (parsing a ``PointCloud2`` into a numpy array, TF, publishing) live in
``fusion_node.py``.

Point-cloud convention used throughout: an ``(N, 5)`` ``float`` array whose
columns are ``(x, y, z, intensity, ring)``.

Chunk 4 implements only the sphere crop (step A1). Ring bucketing, ground
removal, clustering and the circle fit are added in later chunks.
"""

import numpy as np
from scipy.spatial import cKDTree

# Column indices for the (N, 5) point array.
X, Y, Z, INTENSITY, RING = 0, 1, 2, 3, 4


def build_kdtree(cloud: np.ndarray) -> cKDTree:
    """Build a kD-tree over the xyz columns of an (N, 5) point cloud.

    Building the tree once per LiDAR frame and reusing it across all seeds in
    that frame avoids rebuilding it per cone (see Chunk 9).
    """
    return cKDTree(np.asarray(cloud)[:, :3])


def crop_sphere(cloud: np.ndarray, seed_xyz, r_sphere: float,
                tree: cKDTree = None) -> np.ndarray:
    """Return the subset of ``cloud`` within ``r_sphere`` of ``seed_xyz``.

    Keeps points satisfying ``(x-sx)^2 + (y-sy)^2 + (z-sz)^2 <= r_sphere^2``.

    Parameters
    ----------
    cloud : np.ndarray
        ``(N, 5)`` array of ``(x, y, z, intensity, ring)``.
    seed_xyz : array-like
        3D seed point ``(sx, sy, sz)`` already expressed in the LiDAR frame.
    r_sphere : float
        Sphere radius in metres.
    tree : scipy.spatial.cKDTree, optional
        Pre-built tree over ``cloud[:, :3]``. Built on demand if not supplied.

    Returns
    -------
    np.ndarray
        ``(M, 5)`` subset of the input points inside the sphere (possibly empty).
    """
    cloud = np.asarray(cloud)
    if cloud.size == 0:
        return cloud.reshape(0, 5)
    if tree is None:
        tree = build_kdtree(cloud)
    idx = tree.query_ball_point(np.asarray(seed_xyz, dtype=float), r_sphere)
    return cloud[idx]
