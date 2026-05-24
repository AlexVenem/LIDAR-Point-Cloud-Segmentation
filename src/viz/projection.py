import numpy as np


BOX_EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 0),  # нижняя грань
    (4, 5), (5, 6), (6, 7), (7, 4),  # верхняя грань
    (0, 4), (1, 5), (2, 6), (3, 7),  # вертикальные ребра
]


def project_points_to_image(
    points_lidar: np.ndarray,
    k_matrix: np.ndarray,
    t_cam_lidar: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Проецирует 3D-точки из координат LiDAR в пиксели камеры.

    Parameters
    ----------
    points_lidar : np.ndarray
        Массив точек LiDAR, shape=(N, 3).
    k_matrix : np.ndarray
        Intrinsic matrix камеры, shape=(3, 3).
    t_cam_lidar : np.ndarray
        Матрица перехода LiDAR -> Camera, shape=(4, 4).

    Returns
    -------
    pixels : np.ndarray
        Координаты пикселей, shape=(N, 2).
    valid_mask : np.ndarray
        Маска точек, которые находятся перед камерой.
    """
    points_h = np.hstack([
        points_lidar,
        np.ones((len(points_lidar), 1), dtype=np.float64),
    ])

    points_cam = (t_cam_lidar @ points_h.T).T[:, :3]

    z = points_cam[:, 2]
    valid_mask = z > 1e-6

    pixels_h = (k_matrix @ points_cam.T).T
    pixels = pixels_h[:, :2] / pixels_h[:, 2:3]

    return pixels, valid_mask


def draw_projected_obb(
    ax,
    obb,
    k_matrix: np.ndarray,
    t_cam_lidar: np.ndarray,
    image_shape: tuple[int, int] | None = None,
    color="lime",
) -> None:
    """
    Рисует 3D OBB поверх изображения камеры.
    Пропускает боксы, которые плохо проецируются в изображение.
    """
    corners_lidar = obb.corners()

    pixels, valid = project_points_to_image(
        corners_lidar,
        k_matrix,
        t_cam_lidar,
    )

    # Бокс должен быть перед камерой.
    if valid.sum() < 8:
        return

    if image_shape is not None:
        height, width = image_shape[:2]

        x = pixels[:, 0]
        y = pixels[:, 1]

        # Сколько углов реально попали внутрь изображения.
        inside = (
            (x >= 0) & (x < width) &
            (y >= 0) & (y < height)
        )

        # Если почти весь бокс вне кадра — не рисуем.
        if inside.sum() < 4:
            return

        box_w = float(x.max() - x.min())
        box_h = float(y.max() - y.min())

        # Отсекаем слишком маленькие или гигантские проекции.
        if box_w < 5 or box_h < 5:
            return

        if box_w > width * 0.8 or box_h > height * 0.8:
            return

    for i, j in BOX_EDGES:
        x1, y1 = pixels[i]
        x2, y2 = pixels[j]
        ax.plot([x1, x2], [y1, y2], color=color, linewidth=1.5)

    cx, cy = pixels.mean(axis=0)
    ax.text(
        cx,
        cy,
        f"#{obb.cluster_id}",
        color=color,
        fontsize=8,
        fontweight="bold",
    )