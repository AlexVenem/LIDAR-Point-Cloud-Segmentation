from src.core.pointcloud import PointCloud
from src.io.bin_reader import read_kitti_bin


def load_helimos_frame(bin_path: str) -> PointCloud:
    """
    Загружает один кадр HeLiMOS.
    """
    return read_kitti_bin(bin_path)