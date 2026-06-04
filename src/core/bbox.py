from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from src.core.pointcloud import PointCloud


@dataclass(slots=True)
class BBox3D:
    """3D bounding box for one DBSCAN cluster in LiDAR coordinates.

    The box stores a rigid transform from local box coordinates to the sensor
    coordinate system. For the OBB used here, yaw is estimated in the X-Y plane;
    Z bounds are kept axis-aligned. This is usually more stable for road-scene
    objects than a fully free 3D PCA box.
    """

    center: np.ndarray
    extent: np.ndarray
    R: np.ndarray
    cluster_id: int
    n_points: int
    mean_velocity: float | None = None

    @property
    def size(self) -> np.ndarray:
        """Alias for compatibility with code that uses bbox.size."""
        return self.extent

    @property
    def yaw(self) -> float:
        """Box rotation around Z axis in radians."""
        return float(np.arctan2(self.R[1, 0], self.R[0, 0]))

    def corners(self) -> np.ndarray:
        """Return 8 box corners in sensor/world coordinates, shape=(8, 3)."""
        hx, hy, hz = self.extent / 2.0
        local = np.array(
            [
                [-hx, -hy, -hz],
                [hx, -hy, -hz],
                [hx, hy, -hz],
                [-hx, hy, -hz],
                [-hx, -hy, hz],
                [hx, -hy, hz],
                [hx, hy, hz],
                [-hx, hy, hz],
            ],
            dtype=np.float64,
        )
        return (self.R @ local.T).T + self.center


# Backward-compatible name: old code imported/printed OBB.
OBB = BBox3D


def _rotation_from_yaw(yaw: float) -> np.ndarray:
    c = float(np.cos(yaw))
    s = float(np.sin(yaw))
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def compute_aabb(
    points: np.ndarray,
    cluster_id: int,
    mean_velocity: float | None = None,
) -> BBox3D:
    """Build an axis-aligned 3D bounding box for cluster points."""
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if len(points) == 0:
        raise ValueError("cannot build bbox from an empty point set")

    pts = points.astype(np.float64, copy=False)
    p_min = pts.min(axis=0)
    p_max = pts.max(axis=0)
    center = (p_min + p_max) / 2.0
    extent = np.maximum(p_max - p_min, 1e-6)

    return BBox3D(
        center=center,
        extent=extent,
        R=np.eye(3, dtype=np.float64),
        cluster_id=int(cluster_id),
        n_points=int(len(points)),
        mean_velocity=mean_velocity,
    )


def compute_obb(
    points: np.ndarray,
    cluster_id: int,
    mean_velocity: float | None = None,
    min_points: int = 15,
) -> BBox3D | None:
    """Build a yaw-oriented 3D bounding box using PCA in the X-Y plane.

    Returns None for too-small or degenerate clusters. This function avoids a
    hard dependency on Open3D and is less likely to fail on nearly planar point
    groups than a full 3D oriented bounding box.
    """
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if len(points) < min_points:
        return None

    pts = points.astype(np.float64, copy=False)
    xy = pts[:, :2]
    xy_center = xy.mean(axis=0)
    xy_centered = xy - xy_center

    # Degenerate case: nearly one point/line after filtering.
    if np.linalg.matrix_rank(xy_centered) < 1:
        return None

    cov = np.cov(xy_centered, rowvar=False)
    eigvals, eigvecs = np.linalg.eigh(cov)
    order = np.argsort(eigvals)[::-1]
    eigvecs = eigvecs[:, order]

    # Keep a right-handed 2D basis.
    if np.linalg.det(eigvecs) < 0:
        eigvecs[:, 1] *= -1.0

    yaw = float(np.arctan2(eigvecs[1, 0], eigvecs[0, 0]))
    R = _rotation_from_yaw(yaw)

    # Rotate points into local box coordinates.
    pts_center_xy = np.array([xy_center[0], xy_center[1], 0.0], dtype=np.float64)
    local = (R.T @ (pts - pts_center_xy).T).T

    local_min = local.min(axis=0)
    local_max = local.max(axis=0)
    local_center = (local_min + local_max) / 2.0
    extent = np.maximum(local_max - local_min, 1e-6)
    center = (R @ local_center) + pts_center_xy

    return BBox3D(
        center=center,
        extent=extent,
        R=R,
        cluster_id=int(cluster_id),
        n_points=int(len(points)),
        mean_velocity=mean_velocity,
    )


def is_valid_bbox(
    bbox: BBox3D,
    min_extent: tuple[float, float, float] = (0.3, 0.15, 0.1),
    max_extent: tuple[float, float, float] = (8.0, 4.0, 3.5),
    max_aspect_ratio: float = 8.0,
) -> bool:
    """Return True if a bbox has plausible object-like dimensions."""
    length, width, height = sorted(map(float, bbox.extent), reverse=True)

    min_l, min_w, min_h = min_extent
    max_l, max_w, max_h = max_extent

    if length < min_l or width < min_w or height < min_h:
        return False
    if length > max_l or width > max_w or height > max_h:
        return False
    if length / max(width, 1e-6) > max_aspect_ratio:
        return False
    return True


def filter_bboxes(
    bboxes: list[BBox3D],
    min_extent: tuple[float, float, float] = (0.3, 0.15, 0.1),
    max_extent: tuple[float, float, float] = (8.0, 4.0, 3.5),
    max_aspect_ratio: float = 8.0,
) -> list[BBox3D]:
    """Filter bbox list by simple geometric validity checks."""
    return [
        bbox
        for bbox in bboxes
        if is_valid_bbox(
            bbox,
            min_extent=min_extent,
            max_extent=max_extent,
            max_aspect_ratio=max_aspect_ratio,
        )
    ]


def compute_cluster_obbs(
    pc: "PointCloud",
    cluster_ids: np.ndarray,
    min_cluster_points: int = 15,
    filter_invalid: bool = True,
    max_aspect_ratio: float = 8.0,
) -> list[BBox3D]:
    """Build OBBs for DBSCAN clusters with labels >= 0.

    Parameters
    pc:
        PointCloud containing xyz and optional radial velocity.
    cluster_ids:
        Array with labels for every point: -2 static, -1 moving noise,
        >=0 cluster id.
    min_cluster_points:
        Minimal number of points required to build an OBB.
    filter_invalid:
        If True, remove too small, too large and very thin boxes.
    max_aspect_ratio:
        Max length/width ratio after sorting extents.
    """
    if len(cluster_ids) != len(pc.xyz):
        raise ValueError("cluster_ids length must match point cloud length")

    obbs: list[BBox3D] = []
    valid_cluster_ids = np.unique(cluster_ids[cluster_ids >= 0])

    for cid in valid_cluster_ids:
        mask = cluster_ids == cid
        if int(mask.sum()) < min_cluster_points:
            continue

        pts = pc.xyz[mask].astype(np.float64, copy=False)
        mean_velocity = None
        if getattr(pc, "velocity", None) is not None:
            mean_velocity = float(np.mean(pc.velocity[mask]))

        bbox = compute_obb(
            pts,
            cluster_id=int(cid),
            mean_velocity=mean_velocity,
            min_points=min_cluster_points,
        )
        if bbox is None:
            continue
        if filter_invalid and not is_valid_bbox(
            bbox,
            max_aspect_ratio=max_aspect_ratio,
        ):
            continue
        obbs.append(bbox)

    return obbs
