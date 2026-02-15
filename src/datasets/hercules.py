from src.core.pointcloud import PointCloud
from src.io.bin_reader import read_hercules_bin

def load_hercules_aeva(bin_path: str) -> PointCloud:
    """
    Загружает один кадр HeRCULES (Aeva).
    """
    return read_hercules_bin(bin_path)