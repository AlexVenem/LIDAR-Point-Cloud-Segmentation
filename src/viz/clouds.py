import open3d as o3d
import numpy as np
from src.core.pointcloud import PointCloud


def visualize_point_cloud(pc: PointCloud) -> None:
    """
    Визуализация облака точек без семантики.
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc.xyz)

    if pc.intensity is not None:
        i = pc.intensity
        i_norm = (i - i.min()) / (i.max() - i.min() + 1e-6)
        colors = np.zeros((len(i_norm), 3))
        colors[:, 0] = i_norm
        colors[:, 2] = 1.0 - i_norm
        pcd.colors = o3d.utility.Vector3dVector(colors)

    o3d.visualization.draw_geometries([pcd])