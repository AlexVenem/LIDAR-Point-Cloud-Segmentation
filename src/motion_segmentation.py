import os
from collections import defaultdict
from typing import Iterator, List, Optional, Tuple

import joblib
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.ensemble import RandomForestClassifier
from sklearn.preprocessing import StandardScaler

from src.config import (
    MOS_STATIC_LABEL,
    MOS_MOVING_LABEL,
    RANSAC_N_ITERATIONS,
    RANSAC_INLIER_THRESHOLD,
    RANSAC_SEED,
    DBSCAN_EPS_XYZ,
    DBSCAN_EPS_VR,
    DBSCAN_MIN_SAMPLES,
    MOS_DEFAULT_THRESHOLD,
    MOS_DEFAULT_INLIER_THRESHOLD,
    MOS_MAX_TRAIN_POINTS,
    RF_N_ESTIMATORS,
    RF_MAX_DEPTH,
    RF_MIN_SAMPLES_LEAF,
    XGB_N_ESTIMATORS,
    XGB_MAX_DEPTH,
    XGB_LEARNING_RATE,
    XGB_SUBSAMPLE,
    XGB_COLSAMPLE_BYTREE,
    MOS_MIN_CLUSTER_POINTS,
    MOS_MIN_MEAN_RESIDUAL,
    MOS_MIN_MOVING_RATIO,
    MOS_RESIDUAL_THRESHOLD,
    MOS_MIN_ABS_MEAN_VR,
)
from src.core.pointcloud import PointCloud
from src.io.bin_reader import read_kitti_bin
from src.io.label_reader import read_label

def _extract_features(pc: PointCloud) -> np.ndarray:
    """[x, y, z, range, azimuth, elevation, intensity, velocity(opt)]"""
    x, y, z = pc.xyz[:, 0], pc.xyz[:, 1], pc.xyz[:, 2]
    xy = np.sqrt(x ** 2 + y ** 2)
    rng = np.sqrt(xy ** 2 + z ** 2)
    azimuth = np.arctan2(y, x)
    elevation = np.arctan2(z, xy + 1e-9)

    intensity = pc.intensity if pc.intensity is not None else np.zeros_like(x)
    feats = [x, y, z, rng, azimuth, elevation, intensity]

    if pc.velocity is not None:
        feats.append(pc.velocity)

    return np.stack(feats, axis=1).astype(np.float32)

def ransac_ego_motion(
    pc: PointCloud,
    n_iterations: int = RANSAC_N_ITERATIONS,
    inlier_threshold: float = RANSAC_INLIER_THRESHOLD,
    seed: int = RANSAC_SEED,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Оценка эго-скорости [Vx, Vy] по радиальным скоростям Доплера через RANSAC.
    Возвращает вектор скорости платформы и маску статичных точек.
    """
    assert pc.velocity is not None, "RANSAC requires per-point Doppler velocity."

    x, y = pc.xyz[:, 0], pc.xyz[:, 1]
    alpha = np.arctan2(y, x)
    v_r = pc.velocity.astype(np.float64)

    A = np.column_stack([-np.cos(alpha), -np.sin(alpha)])

    rng = np.random.default_rng(seed)
    n_pts = len(v_r)
    best_inliers: Optional[np.ndarray] = None
    best_count = 0

    for _ in range(n_iterations):
        idx = rng.choice(n_pts, 2, replace=False)
        try:
            params, *_ = np.linalg.lstsq(A[idx], v_r[idx], rcond=None)
        except np.linalg.LinAlgError:
            continue
        residuals = np.abs(A @ params - v_r)
        inliers = residuals < inlier_threshold
        n_in = int(inliers.sum())
        if n_in > best_count:
            best_count = n_in
            best_inliers = inliers

    if best_inliers is not None and best_inliers.sum() >= 2:
        params, *_ = np.linalg.lstsq(A[best_inliers], v_r[best_inliers], rcond=None)
        residuals = np.abs(A @ params - v_r)
        is_static = residuals < inlier_threshold
    else:
        params = np.zeros(2)
        is_static = np.ones(n_pts, dtype=bool)

    return params.astype(np.float32), is_static

def _pose_to_se3(row: np.ndarray) -> np.ndarray:
    """12-элементная строка из poses.txt -> матрица SE(3) 4x4."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :] = row.reshape(3, 4)
    return T

def temporal_consistency_segment(
    frames: List[PointCloud],
    poses: np.ndarray,
    n_context: int = 3,
    voxel_size: float = 0.5,
    moving_threshold: float = 0.35,
) -> List[np.ndarray]:
    """
    Классификация точек по воксельной занятости во временном окне.

    Для каждого кадра i трансформируем точки из +-n_context соседних кадров
    в его систему координат. Воксель, в который попали точки из многих кадров —
    статика; из одного-двух — скорее всего движущийся объект.
    """
    n = len(frames)
    assert len(poses) >= n, "Need one pose per frame."

    # упаковываем позы в SE(3) заранее, чтобы не пересчитывать внутри цикла
    STRIDE = 20001  # предполагаем не более +-10000 вокселей по каждой оси

    Ts = [_pose_to_se3(poses[i]) for i in range(n)]
    results: List[np.ndarray] = []

    for i in range(n):
        T_inv_i = np.linalg.inv(Ts[i])
        lo = max(0, i - n_context)
        hi = min(n, i + n_context + 1)
        n_window = hi - lo

        voxel_frames = defaultdict(set)

        for j in range(lo, hi):
            T_rel = T_inv_i @ Ts[j]
            pts = frames[j].xyz.astype(np.float64)
            pts_h = np.hstack([pts, np.ones((len(pts), 1))])
            pts_t = (T_rel @ pts_h.T)[:3].T

            vi = np.floor(pts_t / voxel_size).astype(np.int64)
            keys = vi[:, 0] * STRIDE * STRIDE + vi[:, 1] * STRIDE + vi[:, 2]
            for k in keys:
                voxel_frames[int(k)].add(j)

        pts_i = frames[i].xyz.astype(np.float64)
        vi_i = np.floor(pts_i / voxel_size).astype(np.int64)
        keys_i = vi_i[:, 0] * STRIDE * STRIDE + vi_i[:, 1] * STRIDE + vi_i[:, 2]

        occupancy = np.array(
            [len(voxel_frames.get(int(k), set())) / n_window for k in keys_i],
            dtype=np.float32,
        )
        results.append(occupancy < moving_threshold)

    return results

def cluster_moving_objects(
    pc: PointCloud,
    is_moving: np.ndarray,
    eps_xyz: float = 1.0,
    eps_vr: float = 1.0,
    min_samples: int = 4,
    two_stage: bool = True,
) -> np.ndarray:
    """
    DBSCAN-кластеризация движущихся точек.

    Returns:
        cluster_ids : int32 ndarray (N,)
            -2 -> static, -1 -> noise среди moving, ≥0 -> id кластера
    """
    cluster_ids = np.full(len(is_moving), -2, dtype=np.int32)
    moving_idx = np.where(is_moving)[0]

    if len(moving_idx) < min_samples:
        return cluster_ids

    cluster_ids[moving_idx] = -1  # noise по умолчанию, кластеры перезапишут

    xyz_mov = pc.xyz[moving_idx].astype(np.float64)
    has_vel = pc.velocity is not None
    vel_mov = pc.velocity[moving_idx].astype(np.float64) if has_vel else None

    if two_stage and has_vel:
        labels = _two_stage_dbscan(
            xyz_mov, vel_mov, eps_xyz, min_samples, eps_vr,
        )
    else:
        # Single-stage: 4D (xyz + Vr*weight). Чтобы Vr и xyz «жили» в одном
        # пространстве с понятным eps, домножаем Vr на eps_xyz/eps_vr.
        if has_vel:
            vr_scale = eps_xyz / max(eps_vr, 1e-6)
            features = np.column_stack([xyz_mov, vel_mov * vr_scale])
        else:
            features = xyz_mov
        db = DBSCAN(eps=eps_xyz, min_samples=min_samples, n_jobs=-1)
        labels = db.fit_predict(features)

    cluster_ids[moving_idx] = labels.astype(np.int32)
    return cluster_ids


def _two_stage_dbscan(
    xyz: np.ndarray,
    vel: np.ndarray,
    eps_xyz: float,
    min_samples: int,
    eps_vr: float,
) -> np.ndarray:
    """
    Two-stage DBSCAN in physical units.
    Stage 1: cluster by Vr [m/s] with eps=eps_vr.
    Stage 2: within each Vr group, spatial clustering by xyz [m] with eps=eps_xyz.
    """
    n = len(xyz)
    labels = np.full(n, -1, dtype=np.int32)

    # Stage 1: cluster by radial velocity (m/s)
    db_vel = DBSCAN(eps=eps_vr, min_samples=min_samples, n_jobs=-1)
    vel_labels = db_vel.fit_predict(vel.reshape(-1, 1))

    next_cluster = 0
    for vl in sorted(set(vel_labels)):
        if vl == -1:
            continue
        idx = np.where(vel_labels == vl)[0]
        if len(idx) < min_samples:
            continue

        # Stage 2: spatial clustering (in meters)
        db_xyz = DBSCAN(eps=eps_xyz, min_samples=min_samples, n_jobs=-1)
        sub_labels = db_xyz.fit_predict(xyz[idx])

        for sl in sorted(set(sub_labels)):
            if sl == -1:
                continue
            sub_mask = sub_labels == sl
            labels[idx[sub_mask]] = next_cluster
            next_cluster += 1

    return labels

def filter_motion_clusters(
    pc: PointCloud,
    cluster_ids: np.ndarray,
    ego_params: "np.ndarray | None" = None,
    min_cluster_points: int = MOS_MIN_CLUSTER_POINTS,
    min_mean_residual: float = MOS_MIN_MEAN_RESIDUAL,
    min_moving_ratio: float = MOS_MIN_MOVING_RATIO,
    residual_threshold: float = MOS_RESIDUAL_THRESHOLD,
    min_abs_mean_vr: float = MOS_MIN_ABS_MEAN_VR,
) -> np.ndarray:
    """
    Постфильтрация DBSCAN-кластеров по признакам движения.

    Удаляет кластеры, которые геометрически есть, но по Doppler/RANSAC
    больше похожи на статичные стены, фасады, бордюры или шум.

    Returns
    -------
    filtered_ids : np.ndarray
        cluster_ids той же формы:
        -2 -> static
        -1 -> noise
        >=0 -> валидный движущийся кластер
    """
    filtered_ids = cluster_ids.copy()

    if pc.velocity is None or ego_params is None:
        return filtered_ids

    x = pc.xyz[:, 0]
    y = pc.xyz[:, 1]
    alpha = np.arctan2(y, x)

    vr = pc.velocity.astype(np.float64)

    vr_expected = (
        -float(ego_params[0]) * np.cos(alpha)
        -float(ego_params[1]) * np.sin(alpha)
    )

    residual = np.abs(vr - vr_expected)

    valid_cids = np.unique(cluster_ids[cluster_ids >= 0])

    for cid in valid_cids:
        mask = cluster_ids == cid
        n_points = int(mask.sum())

        if n_points < min_cluster_points:
            filtered_ids[mask] = -1
            continue

        cluster_residual = residual[mask]
        cluster_vr = vr[mask]

        mean_residual = float(np.mean(cluster_residual))
        moving_ratio = float(np.mean(cluster_residual > residual_threshold))
        abs_mean_vr = float(abs(np.mean(cluster_vr)))

        if mean_residual < min_mean_residual:
            filtered_ids[mask] = -1
            continue

        if moving_ratio < min_moving_ratio:
            filtered_ids[mask] = -1
            continue

        if abs_mean_vr < min_abs_mean_vr:
            filtered_ids[mask] = -1
            continue

    return filtered_ids

def relabel_clusters(cluster_ids: np.ndarray) -> np.ndarray:
    """
    Renumber valid DBSCAN clusters sequentially: 0, 1, 2, ...
    Keeps -2 for static, -1 for noise.
    """
    relabeled = cluster_ids.copy()
    valid_cids = np.unique(cluster_ids[cluster_ids >= 0])

    for new_id, old_id in enumerate(valid_cids):
        relabeled[cluster_ids == old_id] = new_id

    return relabeled

class MotionSegmenter:
    """
    Moving Object Segmentation classifier for LiDAR point clouds.
    Uses RANSAC for Doppler-capable sensors, Random Forest or XGBoost otherwise.
    """

    def __init__(self, threshold: float = MOS_DEFAULT_THRESHOLD, inlier_threshold: float = MOS_DEFAULT_INLIER_THRESHOLD,
                 use_gpu: bool = False) -> None:
        self.classifier: Optional[RandomForestClassifier] = None
        self.scaler: Optional[StandardScaler] = None
        self.threshold = threshold
        self.inlier_threshold = inlier_threshold
        self.use_gpu = use_gpu

    def train_on_helimos(
        self,
        data_root: str,
        sensor: str = "Velodyne",
        split: str = "train",
        max_frames: Optional[int] = None,
    ) -> None:
        """Обучение RF/XGBoost на размеченных данных HeLiMOS."""
        split_file = os.path.join(data_root, f"{split}.txt")
        with open(split_file) as f:
            frame_ids = [int(ln.strip()) for ln in f if ln.strip()]

        if max_frames is not None:
            frame_ids = frame_ids[:max_frames]

        sensor_dir = os.path.join(data_root, sensor)
        vel_dir = os.path.join(sensor_dir, "velodyne") # путь к bin файлам
        lbl_dir = os.path.join(sensor_dir, "labels")

        all_feats: List[np.ndarray] = []
        all_labels: List[np.ndarray] = []
        skipped = 0

        print(f"[MOS] Loading {len(frame_ids)} frames | sensor={sensor} | split={split}")
        for i, fid in enumerate(frame_ids):
            bin_path = os.path.join(vel_dir, f"{fid:06d}.bin")
            lbl_path = os.path.join(lbl_dir, f"{fid:06d}.label")

            if not os.path.exists(bin_path) or not os.path.exists(lbl_path):
                skipped += 1
                continue

            pc = read_kitti_bin(bin_path)
            semantic, _ = read_label(lbl_path)

            mask = (semantic == MOS_STATIC_LABEL) | (semantic == MOS_MOVING_LABEL)
            if mask.sum() == 0:
                skipped += 1
                continue

            all_feats.append(_extract_features(pc)[mask])
            all_labels.append((semantic[mask] == MOS_MOVING_LABEL).astype(np.int8))

            if (i + 1) % 200 == 0:
                print(f"  {i + 1}/{len(frame_ids)} ({100*(i+1)/len(frame_ids):.0f}%)")

        if not all_feats:
            raise RuntimeError(
                "No labelled frames found. Check data_root / sensor path."
            )

        X = np.vstack(all_feats)
        y = np.concatenate(all_labels)
        print(
            f"[MOS] Loaded: {len(X):,} points | "
            f"{y.mean() * 100:.2f}% moving | {skipped} skipped"
        )

        if len(X) > MOS_MAX_TRAIN_POINTS:
            moving_idx = np.where(y == 1)[0]
            static_idx = np.where(y == 0)[0]

            n_moving = len(moving_idx)
            n_static_budget = MOS_MAX_TRAIN_POINTS - n_moving
            if n_static_budget < n_moving:
                n_static_budget = n_moving

            rng = np.random.default_rng(42)
            static_sample = rng.choice(
                static_idx, size=min(n_static_budget, len(static_idx)), replace=False
            )
            keep = np.concatenate([moving_idx, static_sample])
            rng.shuffle(keep)
            X = X[keep]
            y = y[keep]
            print(
                f"[MOS] Subsampled: {len(X):,} points "
                f"({(y == 1).sum():,} moving, {(y == 0).sum():,} static)"
            )

        self.scaler = StandardScaler()
        X_sc = self.scaler.fit_transform(X)

        if self.use_gpu:
            self.classifier = self._make_xgb_classifier(X_sc, y)
        else:
            self.classifier = RandomForestClassifier(
                n_estimators=RF_N_ESTIMATORS,
                max_depth=RF_MAX_DEPTH,
                min_samples_leaf=RF_MIN_SAMPLES_LEAF,
                n_jobs=-1,
                random_state=42,
                class_weight="balanced",
            )
            print("[MOS] Training Random Forest (CPU)")
            self.classifier.fit(X_sc, y)

        feat_names = ["x", "y", "z", "range", "azimuth", "elevation", "intensity"]
        if X.shape[1] > 7:
            feat_names.append("velocity")
        print("[MOS] Feature importances:")
        importances = self.classifier.feature_importances_
        for name, imp in zip(feat_names, importances):
            bar = "=" * int(imp * 40)
            print(f"  {name:10s} {imp:.3f}  {bar}")

    @staticmethod
    def _make_xgb_classifier(X_sc: np.ndarray, y: np.ndarray):
        """Train XGBoost classifier on GPU (CUDA)."""
        import xgboost as xgb

        n_pos = int((y == 1).sum())
        n_neg = int((y == 0).sum())
        scale = n_neg / max(n_pos, 1)

        clf = xgb.XGBClassifier(
            n_estimators=XGB_N_ESTIMATORS,
            max_depth=XGB_MAX_DEPTH,
            learning_rate=XGB_LEARNING_RATE,
            subsample=XGB_SUBSAMPLE,
            colsample_bytree=XGB_COLSAMPLE_BYTREE,
            scale_pos_weight=scale,
            eval_metric="logloss",
            device="cuda",
            random_state=42,
        )
        print("[MOS] Training XGBoost (GPU/CUDA)")
        clf.fit(X_sc, y)
        return clf

    def segment_frame(
        self,
        pc: PointCloud,
    ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """RANSAC если есть Doppler-скорость, иначе RF/XGBoost."""
        if pc.velocity is not None:
            ego_params, is_static = ransac_ego_motion(
                pc, inlier_threshold=self.inlier_threshold
            )
            return ~is_static, ego_params

        if self.classifier is None:
            raise RuntimeError(
                "No model loaded and no Doppler velocity available.\n"
                "Run train_on_helimos() or load() first."
            )
        feats = _extract_features(pc)
        feats_sc = self.scaler.transform(feats)
        proba = self.classifier.predict_proba(feats_sc)[:, 1]
        return proba > self.threshold, None

    def segment_frames(
        self,
        frames: List[PointCloud],
    ) -> List[np.ndarray]:
        """Покадровая сегментация для списка кадров."""
        return [self.segment_frame(pc)[0] for pc in frames]

    def segment_sequence(
        self,
        frames: List[PointCloud],
        poses: np.ndarray,
        n_context: int = 3,
        voxel_size: float = 0.5,
        moving_threshold: float = 0.35,
    ) -> List[np.ndarray]:
        """
        Сегментация последовательности HeLiMOS с позами (poses.txt).
        Точка считается moving только если оба метода согласны:
          1) воксельная занятость (temporal_consistency_segment) — низкая occupancy
          2) покадровый классификатор — RF/XGBoost (Velodyne/Ouster) или RANSAC (Aeva)
        """
        temporal = temporal_consistency_segment(
            frames, poses, n_context, voxel_size, moving_threshold
        )
        per_frame = self.segment_frames(frames)

        return [t & p for t, p in zip(temporal, per_frame)]

    def save(self, path: str) -> None:
        """Save trained model to disk."""
        if self.classifier is None:
            raise RuntimeError("Train the model before saving.")
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        joblib.dump({
            "classifier": self.classifier,
            "scaler": self.scaler,
            "threshold": self.threshold,
        }, path)
        print(f"[MOS] Model saved: {path} (threshold={self.threshold})")

    def load(self, path: str, override_threshold: bool = False) -> None:
        """Load a previously saved model from disk."""
        data = joblib.load(path)
        self.classifier = data["classifier"]
        self.scaler = data["scaler"]
        saved_threshold = data.get("threshold", 0.85)

        if not override_threshold:
            self.threshold = saved_threshold

        print(f"[MOS] Model loaded: {path} (threshold={self.threshold})")

# Dataset helpers (HeLiMOS)
def iter_helimos_labeled(
    data_root: str,
    sensor: str = "Velodyne",
    split: str = "train",
    max_frames: Optional[int] = None,
) -> Iterator[Tuple[int, PointCloud, np.ndarray]]:
    """
    Iterate over labelled HeLiMOS frames.
    """
    split_file = os.path.join(data_root, f"{split}.txt")
    with open(split_file) as f:
        frame_ids = [int(ln.strip()) for ln in f if ln.strip()]

    if max_frames is not None:
        frame_ids = frame_ids[:max_frames]

    sensor_dir = os.path.join(data_root, sensor)
    vel_dir = os.path.join(sensor_dir, "velodyne")
    lbl_dir = os.path.join(sensor_dir, "labels")

    for fid in frame_ids:
        bin_path = os.path.join(vel_dir, f"{fid:06d}.bin")
        lbl_path = os.path.join(lbl_dir, f"{fid:06d}.label")

        if not os.path.exists(bin_path) or not os.path.exists(lbl_path):
            continue

        pc = read_kitti_bin(bin_path)
        semantic, _ = read_label(lbl_path)
        yield fid, pc, semantic
