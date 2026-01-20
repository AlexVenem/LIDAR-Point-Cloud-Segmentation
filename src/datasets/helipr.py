from src.core.pointcloud import PointCloud
from src.io.bin_reader import read_helipr_bin


def load_helipr_aeva(bin_path: str) -> PointCloud:
    """
    Загружает один кадр HeliPR (Aeva).
    """
    return read_helipr_bin(bin_path, 'Aeva')