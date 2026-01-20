import numpy as np
import struct
from typing import Optional
from src.core.pointcloud import PointCloud


def read_kitti_bin(path: str) -> PointCloud:
    """
    KITTI / HeLiMOS формат:
    float32 x, y, z, intensity
    """
    data = np.fromfile(path, dtype=np.float32).reshape(-1, 4)

    return PointCloud(
    xyz=data[:, :3],
    intensity=data[:, 3],
    velocity=None,
    )


def read_helipr_bin(path: str, typeLidar: str) -> PointCloud:
    xyz = []
    intensity = []
    velocity = []

    with open(path, "rb") as file:
        while True:
            if typeLidar == "Velodyne":
                data = file.read(22)
                if len(data) < 22:
                    break

                x, y, z, inten = struct.unpack("ffff", data[:16])
                xyz.append([x, y, z])
                intensity.append(inten)

            elif typeLidar == "Ouster":
                data = file.read(26)
                if len(data) < 26:
                    break

                x, y, z, inten = struct.unpack("ffff", data[:16])
                xyz.append([x, y, z])
                intensity.append(inten)

            elif typeLidar == "Aeva" and int(path.split("/")[-1].split(".")[0]) > 1691936557946849179:
                data = file.read(29)
                if len(data) < 29:
                    break

                x, y, z, reflectivity, vel = struct.unpack("fffff", data[:20])
                xyz.append([x, y, z])
                intensity.append(reflectivity)
                velocity.append(vel)

            elif typeLidar == "Aeva":
                data = file.read(25)
                if len(data) < 25:
                    break

                x, y, z, reflectivity, vel = struct.unpack("fffff", data[:20])
                xyz.append([x, y, z])
                intensity.append(reflectivity)
                velocity.append(vel)

            elif typeLidar == "Livox":
                data = file.read(19)
                if len(data) < 19:
                    break

                x, y, z = struct.unpack("fff", data[:12])
                xyz.append([x, y, z])

            else:
                raise ValueError(f"Unsupported LiDAR type: {typeLidar}")

    xyz = np.asarray(xyz, dtype=np.float32)

    intensity_arr: Optional[np.ndarray]
    velocity_arr: Optional[np.ndarray]

    intensity_arr = np.asarray(intensity, dtype=np.float32) if intensity else None
    velocity_arr = np.asarray(velocity, dtype=np.float32) if velocity else None

    return PointCloud(
        xyz=xyz,
        intensity=intensity_arr,
        velocity=velocity_arr,
    )