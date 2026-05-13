import numpy as np
from pathlib import Path
from typing import Tuple


def read_label(path: str | Path) -> Tuple[np.ndarray, np.ndarray]:
    """
    Читает .label файл HeLiMOS / KITTI формата.

    Returns:
        semantic: ndarray (N,)
        instance: ndarray (N,)
    """
    path = Path(path)

    if not path.exists():
      raise FileNotFoundError(f"Файл не найден: {path}")

    labels = np.fromfile(path, dtype=np.uint32) # N беззнаковых 32 битных чисел

    semantic = labels & 0xFFFF # извлекаем из чисел метки
    instance = labels >> 16 # извлекаем индексы объектов меток

    return semantic, instance