import numpy as np
import matplotlib.pyplot as plt
from src.core.pointcloud import PointCloud


def plot_velocity_vs_azimuth(pc: PointCloud) -> None:
    """
    График радиальной скорости от азимута.
    """
    if pc.velocity is None:
        raise ValueError("Velocity channel is missing")

    x, y = pc.xyz[:, 0], pc.xyz[:, 1]
    azimuth = np.degrees(np.arctan2(y, x))

    plt.figure(figsize=(8, 4))
    plt.scatter(azimuth, pc.velocity, s=1)
    plt.xlabel("Azimuth [deg]")
    plt.ylabel("Radial velocity [m/s]")
    plt.grid(True)
    plt.show()