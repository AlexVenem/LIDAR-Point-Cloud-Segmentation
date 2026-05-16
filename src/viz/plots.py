import os as _os
import numpy as np
import matplotlib.pyplot as plt
import folium
import pandas as pd
from scipy.ndimage import uniform_filter1d
from src.core.pointcloud import PointCloud
from src.config import MAP_ZOOM_LEVEL, MAP_TILES, POLYLINE_COLOR, POLYLINE_WEIGHT, START_MARKER_COLOR, END_MARKER_COLOR


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


def plot_gps_on_map(gps_df, output_file='gps_map.html'):
    """
    Визуализация GPS траектории на интерактивной карте Google Maps (через Folium).
    
    Parameters:
        gps_df: DataFrame с GPS данными (должны быть колонки 'lat' и 'lon')
        output_file: путь для сохранения HTML файла карты
    """
    if 'lat' not in gps_df.columns or 'lon' not in gps_df.columns:
        raise ValueError("GPS DataFrame должен содержать колонки 'lat' и 'lon'")
    
    # Вычисляем центр карты
    center_lat = gps_df['lat'].mean()
    center_lon = gps_df['lon'].mean()
    
    # Создаем карту
    map_gps = folium.Map(
        location=[center_lat, center_lon],
        zoom_start=MAP_ZOOM_LEVEL,
        tiles=MAP_TILES
    )
    
    # Добавляем линию траектории
    coords = list(zip(gps_df['lat'], gps_df['lon']))
    folium.PolyLine(
        coords,
        color=POLYLINE_COLOR,
        weight=POLYLINE_WEIGHT,
        opacity=0.7,
        popup='GPS траектория'
    ).add_to(map_gps)
    
    # Добавляем стартовую точку
    folium.CircleMarker(
        location=[gps_df['lat'].iloc[0], gps_df['lon'].iloc[0]],
        radius=8,
        popup='Старт',
        color=START_MARKER_COLOR,
        fill=True,
        fillColor=START_MARKER_COLOR,
        fillOpacity=0.8
    ).add_to(map_gps)
    
    # Добавляем финальную точку
    folium.CircleMarker(
        location=[gps_df['lat'].iloc[-1], gps_df['lon'].iloc[-1]],
        radius=8,
        popup='Конец',
        color=END_MARKER_COLOR,
        fill=True,
        fillColor=END_MARKER_COLOR,
        fillOpacity=0.8
    ).add_to(map_gps)
    
    # Сохраняем карту
    map_gps.save(output_file)
    print(f"Карта сохранена в {output_file}")
    
    return map_gps

def _resample_reference(ts_est: np.ndarray, v_est: np.ndarray,
                        ts_ref: np.ndarray, v_ref: np.ndarray
                        ) -> "tuple[np.ndarray, np.ndarray, np.ndarray] | None":
    """
    Интерполирует v_ref на ts_est. Возвращает (t, v_est_aligned, v_ref_at_t)
    только для точек ts_est, попавших в диапазон ts_ref. None если пересечение пустое.
    """
    if len(ts_est) == 0 or len(ts_ref) < 2:
        return None
    order = np.argsort(ts_ref)
    ts_ref_s, v_ref_s = ts_ref[order], v_ref[order]
    mask = (ts_est >= ts_ref_s[0]) & (ts_est <= ts_ref_s[-1])
    if mask.sum() < 2:
        return None
    t = ts_est[mask]
    v_e = v_est[mask]
    v_r = np.interp(t, ts_ref_s, v_ref_s)
    return t, v_e, v_r


def _classification_metrics(v_est: np.ndarray, v_ref: np.ndarray,
                            threshold: float) -> dict:
    """
    Бинаризует обе скорости по порогу (>threshold = "движется") и считает
    confusion matrix + accuracy/precision/recall/F1/specificity.
    "Положительный" класс = движение.
    """
    y_pred = v_est > threshold
    y_true = v_ref > threshold
    tp = int(np.sum(y_pred & y_true))
    fp = int(np.sum(y_pred & ~y_true))
    fn = int(np.sum(~y_pred & y_true))
    tn = int(np.sum(~y_pred & ~y_true))
    n = tp + fp + fn + tn
    acc = (tp + tn) / n if n else float("nan")
    prec = tp / (tp + fp) if (tp + fp) else float("nan")
    rec = tp / (tp + fn) if (tp + fn) else float("nan")
    f1 = (2 * prec * rec / (prec + rec)
          if prec == prec and rec == rec and (prec + rec) > 0 else float("nan"))
    spec = tn / (tn + fp) if (tn + fp) else float("nan")
    return dict(n=n, tp=tp, fp=fp, fn=fn, tn=tn,
                accuracy=acc, precision=prec, recall=rec, f1=f1, specificity=spec)


def _regression_metrics(v_est: np.ndarray, v_ref: np.ndarray) -> dict:
    """Базовые регрессионные метрики ошибки скорости (м/с)."""
    err = v_est - v_ref
    abs_err = np.abs(err)
    ss_res = float(np.sum(err ** 2))
    ss_tot = float(np.sum((v_ref - v_ref.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return dict(
        mae=float(abs_err.mean()),
        rmse=float(np.sqrt((err ** 2).mean())),
        bias=float(err.mean()),
        max_abs=float(abs_err.max()),
        r2=r2,
    )


def _print_metrics_report(rows: "list[tuple[str, str, dict, dict]]",
                          threshold: float) -> None:
    """rows: [(estimator, reference, cls_metrics, reg_metrics), ...]"""
    if not rows:
        return
    print()
    print("=" * 96)
    print(f"Метрики качества оценки эго-скорости  (порог движения = {threshold:.2f} м/с)")
    print("=" * 96)
    print("Классификация 'движется vs стоит' (положительный класс = движение)")
    print("-" * 96)
    print(f"{'estimator':<16}{'reference':<10}{'N':>5}  "
          f"{'TP':>5}{'FP':>5}{'FN':>5}{'TN':>5}  "
          f"{'Acc':>7}{'Prec':>7}{'Rec':>7}{'F1':>7}{'Spec':>7}")
    for est, ref, cls, _reg in rows:
        if not cls:
            print(f"{est:<16}{ref:<10}  нет пересечения по времени")
            continue
        print(f"{est:<16}{ref:<10}{cls['n']:>5}  "
              f"{cls['tp']:>5}{cls['fp']:>5}{cls['fn']:>5}{cls['tn']:>5}  "
              f"{cls['accuracy']:>7.3f}{cls['precision']:>7.3f}"
              f"{cls['recall']:>7.3f}{cls['f1']:>7.3f}{cls['specificity']:>7.3f}")
    print()
    print("Регрессионные метрики ошибки |V|  (м/с, R² — безразмерное)")
    print("-" * 96)
    print(f"{'estimator':<16}{'reference':<10}{'MAE':>8}{'RMSE':>8}{'bias':>9}"
          f"{'max|err|':>10}{'R²':>8}")
    for est, ref, _cls, reg in rows:
        if not reg:
            continue
        print(f"{est:<16}{ref:<10}{reg['mae']:>8.3f}{reg['rmse']:>8.3f}"
              f"{reg['bias']:>+9.3f}{reg['max_abs']:>10.3f}{reg['r2']:>8.3f}")
    print("=" * 96)
    print()


def plot_velocity_comparison(gps_path: str, ins_path: str, output_path: str,
                             aeva_path: "str | None" = None,
                             radar_path: "str | None" = None,
                             inlier_threshold: float = 0.5,
                             motion_threshold: float = 0.5) -> None:
    """
    Графики модуля скорости и угловой скорости рыскания.

    Левый график — скорость, м/с:
        GPS  — красные точки (из конечных разностей ECEF-координат через GPS_to_V)
        ИНС  — чёрная линия (из NED-компонент скорости через INS_to_V)
        Aeva — синие точки (RANSAC ego-motion из Doppler LiDAR, если aeva_path задан)

    Правый график — угловая скорость рыскания, °/с:
        ИНС  — чёрные точки (производная азимута из INSPVA)

    Parameters:
        gps_path          : путь к GPS CSV
        ins_path          : путь к INSPVA CSV
        output_path       : путь для сохранения PNG
        aeva_path         : папка с .bin кадрами Aeva (опционально)
        inlier_threshold  : RANSAC inlier threshold [m/s]
    """
    import glob as _glob
    from src.odometry import GPS_to_V, INS_to_V

    # Загрузка
    gps_raw = pd.read_csv(gps_path, header=None,
                          names=["timestamp", "lat", "lon", "height",
                                 "c4", "c5", "c6", "c7", "c8", "c9", "c10", "c11", "c12"])
    gps_raw = gps_raw.reset_index(drop=True)

    ins_raw = pd.read_csv(ins_path, header=None,
                          names=["timestamp", "latitude", "longitude", "height",
                                 "north_velocity", "east_velocity", "up_velocity",
                                 "roll", "pitch", "azimuth", "status"])
    ins_raw = ins_raw.reset_index(drop=True)

    azimuth_raw = ins_raw["azimuth"].values

    # Вычисление скоростей. Модуль = sqrt(Vx²+Vy²+Vz²): нельзя отбрасывать Vz,
    # ECEF-оси наклонены относительно локальной горизонтали.
    Vx_gps, Vy_gps, Vz_gps, ts_gps = GPS_to_V(gps_raw[["timestamp", "lat", "lon", "height"]].copy())
    speed_gps = np.sqrt(Vx_gps**2 + Vy_gps**2 + Vz_gps**2)

    Vx_ins, Vy_ins, Vz_ins, ts_ins = INS_to_V(ins_raw[["timestamp", "latitude", "longitude",
                                                         "north_velocity", "east_velocity",
                                                         "up_velocity"]].copy())
    speed_ins = np.sqrt(Vx_ins**2 + Vy_ins**2 + Vz_ins**2)

    # Угловая скорость рыскания из азимута INSPVA
    az_unwrap = np.unwrap(np.radians(azimuth_raw)) * 180.0 / np.pi
    yaw_rate = np.gradient(az_unwrap, ts_ins)

    # Временная ось: начало с первого движения
    t0 = ts_ins[0]
    ts_ins_rel = ts_ins - t0
    ts_gps_rel = ts_gps - t0

    moving = speed_ins > 0.5
    t_start = ts_ins_rel[np.argmax(moving)] if moving.any() else 0.0

    gps_mask = (ts_gps_rel >= t_start) & (speed_gps < 12)
    ts_gps_plot    = ts_gps_rel[gps_mask] - t_start
    speed_gps_plot = speed_gps[gps_mask]

    ins_mask = ts_ins_rel >= t_start
    ts_ins_plot    = ts_ins_rel[ins_mask] - t_start
    speed_ins_plot = uniform_filter1d(speed_ins[ins_mask], size=3)
    yaw_rate_plot  = yaw_rate[ins_mask]

    # Aeva ego-velocity (RANSAC per frame, с кэшированием)
    plot_t_min, plot_t_max = 100, 250

    ts_aeva_plot = None
    speed_aeva_plot = None
    ts_icp_plot = None
    yaw_rate_icp_plot = None
    if aeva_path:
        cache_path = _os.path.join(aeva_path, "aeva_ego_cache.npz")

        if _os.path.exists(cache_path):
            cached = np.load(cache_path)
            ts_aeva = cached["timestamps_s"]
            speed_aeva = cached["speed"]
            print(f"[Aeva] Загружен кэш: {cache_path} ({len(ts_aeva)} кадров)")
        else:
            from src.datasets.hercules import load_hercules_aeva
            from src.motion_segmentation import ransac_ego_motion

            bin_files = sorted(_glob.glob(_os.path.join(aeva_path, "*.bin")))
            ts_list = []
            speed_list = []
            n = len(bin_files)
            for i, bf in enumerate(bin_files):
                stem = _os.path.splitext(_os.path.basename(bf))[0]
                if not stem.isdigit():
                    continue
                pc = load_hercules_aeva(bf)
                if pc.velocity is None:
                    continue
                try:
                    ego_params, _ = ransac_ego_motion(pc, inlier_threshold=inlier_threshold)
                except Exception:
                    continue
                ego_speed = float(np.sqrt(ego_params[0] ** 2 + ego_params[1] ** 2))
                ts_list.append(int(stem) / 1e9)
                speed_list.append(ego_speed)
                if (i + 1) % 50 == 0:
                    print(f"\r[Aeva] RANSAC: {i + 1}/{n}", end="", flush=True)

            ts_aeva = np.array(ts_list)
            speed_aeva = np.array(speed_list)
            np.savez(cache_path, timestamps_s=ts_aeva, speed=speed_aeva)
            print(f"\n[Aeva] Кэш сохранён: {cache_path} ({len(ts_aeva)} кадров)")

        if len(ts_aeva) > 0:
            ts_rel = ts_aeva - t0 - t_start
            mask = (ts_rel >= plot_t_min) & (ts_rel <= plot_t_max)
            ts_aeva_plot = ts_rel[mask]
            speed_aeva_plot = speed_aeva[mask]
            print(f"[Aeva] На графике: {int(mask.sum())} кадров")

        # ICP yaw rate между последовательными кадрами
        import open3d as o3d
        from src.datasets.hercules import load_hercules_aeva

        bin_files = sorted(_glob.glob(_os.path.join(aeva_path, "*.bin")))
        bin_files = [bf for bf in bin_files
                     if _os.path.splitext(_os.path.basename(bf))[0].isdigit()]

        # Фильтруем только кадры в окне графика
        icp_files = []
        icp_ts = []
        for bf in bin_files:
            stem = _os.path.splitext(_os.path.basename(bf))[0]
            t_rel = int(stem) / 1e9 - t0 - t_start
            if plot_t_min <= t_rel <= plot_t_max:
                icp_files.append(bf)
                icp_ts.append(t_rel)
        icp_ts = np.array(icp_ts)

        if len(icp_files) > 1:
            voxel_size = 0.5
            yaw_rates = []
            ts_mid = []
            prev_pcd = None
            for i, bf in enumerate(icp_files):
                pc = load_hercules_aeva(bf)
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(pc.xyz)
                pcd = pcd.voxel_down_sample(voxel_size)

                if prev_pcd is not None:
                    dt = icp_ts[i] - icp_ts[i - 1]
                    if dt > 0:
                        result = o3d.pipelines.registration.registration_icp(
                            pcd, prev_pcd, max_correspondence_distance=1.0,
                            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30),
                        )
                        R = result.transformation[:3, :3]
                        yaw = np.arctan2(R[1, 0], R[0, 0])
                        yaw_rates.append(-np.degrees(yaw) / dt)
                        ts_mid.append((icp_ts[i] + icp_ts[i - 1]) / 2)

                prev_pcd = pcd
                if (i + 1) % 100 == 0:
                    print(f"\r[ICP] {i + 1}/{len(icp_files)}", end="", flush=True)

            if yaw_rates:
                print(f"\n[ICP] Готово: {len(yaw_rates)} пар")
                ts_icp_plot = np.array(ts_mid)
                yaw_rate_icp_raw = np.array(yaw_rates)
                yaw_rate_icp_plot = uniform_filter1d(yaw_rate_icp_raw, size=5)

    # Radar ego-velocity (RANSAC) + ICP yaw rate
    ts_radar_plot = None
    speed_radar_plot = None
    ts_radar_icp_plot = None
    yaw_rate_radar_icp_plot = None
    if radar_path:
        import open3d as o3d
        from src.datasets.radar import load_radar_frame
        from src.motion_segmentation import ransac_ego_motion as _ransac

        radar_files = sorted(_glob.glob(_os.path.join(radar_path, "*.bin")))
        radar_files = [f for f in radar_files
                       if _os.path.splitext(_os.path.basename(f))[0].isdigit()]

        # RANSAC speed
        r_ts, r_speed = [], []
        for i, bf in enumerate(radar_files):
            stem = _os.path.splitext(_os.path.basename(bf))[0]
            pc = load_radar_frame(bf)
            if pc.velocity is None:
                continue
            try:
                ep, _ = _ransac(pc, inlier_threshold=inlier_threshold)
            except Exception:
                continue
            r_ts.append(int(stem) / 1e9)
            r_speed.append(float(np.sqrt(ep[0] ** 2 + ep[1] ** 2)))
            if (i + 1) % 50 == 0:
                print(f"\r[Radar] RANSAC: {i + 1}/{len(radar_files)}", end="", flush=True)

        if r_ts:
            print(f"\n[Radar] RANSAC готово: {len(r_ts)} кадров")
            r_ts = np.array(r_ts)
            r_speed = np.array(r_speed)
            r_rel = r_ts - t0 - t_start
            r_mask = (r_rel >= plot_t_min) & (r_rel <= plot_t_max)
            ts_radar_plot = r_rel[r_mask]
            speed_radar_plot = r_speed[r_mask]

        # ICP yaw rate
        radar_icp_files = []
        radar_icp_ts = []
        for bf in radar_files:
            stem = _os.path.splitext(_os.path.basename(bf))[0]
            t_rel = int(stem) / 1e9 - t0 - t_start
            if plot_t_min <= t_rel <= plot_t_max:
                radar_icp_files.append(bf)
                radar_icp_ts.append(t_rel)
        radar_icp_ts = np.array(radar_icp_ts)

        if len(radar_icp_files) > 1:
            voxel_size = 0.5
            r_yaw_rates, r_ts_mid = [], []
            prev_pcd = None
            for i, bf in enumerate(radar_icp_files):
                pc = load_radar_frame(bf)
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(pc.xyz)
                pcd = pcd.voxel_down_sample(voxel_size)

                if prev_pcd is not None:
                    dt = radar_icp_ts[i] - radar_icp_ts[i - 1]
                    if dt > 0:
                        result = o3d.pipelines.registration.registration_icp(
                            pcd, prev_pcd, max_correspondence_distance=1.0,
                            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30),
                        )
                        R = result.transformation[:3, :3]
                        yaw = np.arctan2(R[1, 0], R[0, 0])
                        r_yaw_rates.append(-np.degrees(yaw) / dt)
                        r_ts_mid.append((radar_icp_ts[i] + radar_icp_ts[i - 1]) / 2)

                prev_pcd = pcd
                if (i + 1) % 100 == 0:
                    print(f"\r[Radar ICP] {i + 1}/{len(radar_icp_files)}", end="", flush=True)

            if r_yaw_rates:
                print(f"\n[Radar ICP] Готово: {len(r_yaw_rates)} пар")
                ts_radar_icp_plot = np.array(r_ts_mid)
                yaw_rate_radar_icp_plot = uniform_filter1d(np.array(r_yaw_rates), size=5)

    # Метрики качества оценок |V| относительно INS/GPS (ground truth)
    # Адаптация под классификацию: для каждой пары интерполируем reference
    # на метки оценщика, бинаризуем по motion_threshold, считаем
    # confusion matrix + accuracy/precision/recall/F1 для класса "движется".
    # Реgress-метрики (MAE/RMSE/bias/max|err|/R²) считаем на тех же выровненных рядах.
    ins_win = (ts_ins_plot >= plot_t_min) & (ts_ins_plot <= plot_t_max)
    ts_ins_win, speed_ins_win = ts_ins_plot[ins_win], speed_ins_plot[ins_win]
    gps_win = (ts_gps_plot >= plot_t_min) & (ts_gps_plot <= plot_t_max)
    ts_gps_win, speed_gps_win = ts_gps_plot[gps_win], speed_gps_plot[gps_win]

    def _eval_pair(name_est, ts_e, v_e, name_ref, ts_r, v_r):
        aligned = _resample_reference(ts_e, v_e, ts_r, v_r)
        if aligned is None:
            return (name_est, name_ref, {}, {})
        _t, v_e_a, v_r_a = aligned
        return (name_est, name_ref,
                _classification_metrics(v_e_a, v_r_a, motion_threshold),
                _regression_metrics(v_e_a, v_r_a))

    metric_rows = []
    if ts_aeva_plot is not None and len(ts_aeva_plot) > 0:
        metric_rows.append(_eval_pair("Aeva (RANSAC)", ts_aeva_plot, speed_aeva_plot,
                                      "ИНС", ts_ins_win, speed_ins_win))
        metric_rows.append(_eval_pair("Aeva (RANSAC)", ts_aeva_plot, speed_aeva_plot,
                                      "GPS", ts_gps_win, speed_gps_win))
    if ts_radar_plot is not None and len(ts_radar_plot) > 0:
        metric_rows.append(_eval_pair("Radar (RANSAC)", ts_radar_plot, speed_radar_plot,
                                      "ИНС", ts_ins_win, speed_ins_win))
        metric_rows.append(_eval_pair("Radar (RANSAC)", ts_radar_plot, speed_radar_plot,
                                      "GPS", ts_gps_win, speed_gps_win))
    # sanity-check согласованности двух источников ground truth
    metric_rows.append(_eval_pair("GPS", ts_gps_win, speed_gps_win,
                                  "ИНС", ts_ins_win, speed_ins_win))
    _print_metrics_report(metric_rows, motion_threshold)

    # Детекция пиков угловой скорости (только N самых выраженных)
    from scipy.signal import find_peaks
    import contextily as ctx

    MAX_PEAKS = 5

    window_mask = (ts_ins_plot >= plot_t_min) & (ts_ins_plot <= plot_t_max)
    yr_abs = np.abs(yaw_rate_plot[window_mask])
    ts_window = ts_ins_plot[window_mask]

    peak_indices, peak_props = find_peaks(yr_abs, prominence=2.0, distance=80)

    # Оставляем только MAX_PEAKS самых выраженных по prominence
    if len(peak_indices) > MAX_PEAKS:
        top = np.argsort(peak_props["prominences"])[-MAX_PEAKS:]
        top = np.sort(top)  # сохраняем хронологический порядок
        peak_indices = peak_indices[top]

    # INS lat/lon (под ins_mask)
    lat_ins = ins_raw["latitude"].values[ins_mask]
    lon_ins = ins_raw["longitude"].values[ins_mask]

    peak_times = ts_window[peak_indices]
    peak_yaw = yaw_rate_plot[window_mask][peak_indices]
    peak_lat = np.interp(peak_times, ts_ins_plot, lat_ins)
    peak_lon = np.interp(peak_times, ts_ins_plot, lon_ins)

    # Траектория в пределах окна графика
    traj_t = np.linspace(plot_t_min, plot_t_max, 500)
    traj_lat = np.interp(traj_t, ts_ins_plot, lat_ins)
    traj_lon = np.interp(traj_t, ts_ins_plot, lon_ins)

    _peak_colors = ["#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4"]

    # Построение (2 строки: верх — скорость + yaw rate, низ — карта)
    fig = plt.figure(figsize=(14, 11))
    gs = fig.add_gridspec(2, 2, height_ratios=[0.8, 1.2], hspace=0.30, wspace=0.35)
    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[0, 1])
    ax3 = fig.add_subplot(gs[1, :])

    # Скорость
    ax1.scatter(ts_gps_plot, speed_gps_plot, s=4, c="red", alpha=0.6, zorder=2, label="GPS")
    ax1.plot(ts_ins_plot, speed_ins_plot, "k-", linewidth=1.5, zorder=3, label="ИНС")
    if ts_aeva_plot is not None and len(ts_aeva_plot) > 0:
        ax1.scatter(ts_aeva_plot, speed_aeva_plot, s=6, c="blue", alpha=0.7,
                    zorder=4, label="Aeva (RANSAC)")
    if ts_radar_plot is not None and len(ts_radar_plot) > 0:
        ax1.scatter(ts_radar_plot, speed_radar_plot, s=6, c="green", alpha=0.7,
                    zorder=4, label="Radar (RANSAC)")
    ax1.set_xlabel("время, с", fontsize=11)
    ax1.set_ylabel("скорость, м/с", fontsize=11)
    ax1.set_xlim(plot_t_min, plot_t_max)
    ax1.set_ylim(0, 10)
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend(fontsize=10, loc="upper right")
    sensors = []
    if ts_aeva_plot is not None: sensors.append("Aeva")
    if ts_radar_plot is not None: sensors.append("Radar")
    title_suffix = " + " + " + ".join(sensors) if sensors else ""
    ax1.set_title(f"a) Скорость (м/с) от времени (с) (GPS + INS{title_suffix})",
                  loc="left", fontsize=11, fontweight="bold")

    # Угловая скорость + пронумерованные пики
    ax2.plot(ts_ins_plot, yaw_rate_plot, "k.", markersize=1.5, zorder=2, label="ИНС")
    if ts_icp_plot is not None and len(ts_icp_plot) > 0:
        ax2.plot(ts_icp_plot, yaw_rate_icp_plot, "b-", linewidth=1.5,
                 alpha=0.7, zorder=3, label="Aeva (ICP)")
    if ts_radar_icp_plot is not None and len(ts_radar_icp_plot) > 0:
        ax2.plot(ts_radar_icp_plot, yaw_rate_radar_icp_plot, "g-", linewidth=1.5,
                 alpha=0.7, zorder=3, label="Radar (ICP)")
    for j, (pt, py) in enumerate(zip(peak_times, peak_yaw)):
        color = _peak_colors[j % len(_peak_colors)]
        ax2.plot(pt, py, "o", color=color, markersize=9, zorder=5,
                 markeredgecolor="white", markeredgewidth=0.8)
        ax2.annotate(
            str(j + 1), (pt, py),
            textcoords="offset points", xytext=(5, 8),
            fontsize=11, fontweight="bold", color=color, zorder=6,
        )
    ax2.set_xlabel("время, с", fontsize=11)
    ax2.set_ylabel("угол, град/с", fontsize=11)
    ax2.set_xlim(plot_t_min, plot_t_max)
    ax2.axhline(0, color="gray", linewidth=0.8)
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(fontsize=10, loc="upper right")
    ax2.set_title("b) Угловая скорость рыскания (°/с)", loc="left", fontsize=11, fontweight="bold")

    # Карта с траекторией и пронумерованными поворотами
    ax3.plot(traj_lon, traj_lat, "-", color="#e63333", linewidth=2.5,
             alpha=0.85, zorder=3, label="траектория")
    ax3.plot(traj_lon[0], traj_lat[0], "s", color="#00aa00",
             markersize=10, zorder=5, markeredgecolor="white",
             markeredgewidth=1.2, label="старт")
    ax3.plot(traj_lon[-1], traj_lat[-1], "s", color="#cc0000",
             markersize=10, zorder=5, markeredgecolor="white",
             markeredgewidth=1.2, label="конец")
    for j, (plat, plon) in enumerate(zip(peak_lat, peak_lon)):
        color = _peak_colors[j % len(_peak_colors)]
        ax3.plot(plon, plat, "o", color=color, markersize=13, zorder=6,
                 markeredgecolor="white", markeredgewidth=1.5)
        ax3.annotate(
            str(j + 1), (plon, plat),
            ha="center", va="center",
            fontsize=9, fontweight="bold", color="white", zorder=7,
        )

    # Подложка карты (OpenStreetMap)
    try:
        ctx.add_basemap(ax3, crs="EPSG:4326",
                        source=ctx.providers.OpenStreetMap.Mapnik)
    except Exception as e:
        print(f"[Карта] Не удалось загрузить тайлы: {e}")

    ax3.set_xlabel("долгота", fontsize=11)
    ax3.set_ylabel("широта", fontsize=11)
    ax3.legend(fontsize=9, loc="upper right")
    ax3.set_title("c) Траектория с местами поворотов", loc="left", fontsize=11, fontweight="bold")

    _os.makedirs(_os.path.dirname(output_path), exist_ok=True)
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    print(f"График сохранён: {output_path}")
    print(f"Отмечено пиков: {len(peak_times)}")
    for j, (pt, py, plat, plon) in enumerate(
            zip(peak_times, peak_yaw, peak_lat, peak_lon)):
        print(f"  #{j+1}: t={pt:.1f}с, yaw_rate={py:.1f}°/с, "
              f"lat={plat:.6f}, lon={plon:.6f}")
    plt.show()


def plot_ins_track(ins_df: pd.DataFrame) -> None:
    plt.figure()
    plt.plot(ins_df["longitude"], ins_df["latitude"])
    plt.grid(True)
    plt.xlabel("longitude")
    plt.ylabel("latitude")
    plt.title("INS track")
    plt.show()

def plot_imu_accel(imu_df: pd.DataFrame) -> None:
    plt.figure()
    plt.plot(imu_df["timestamp"], imu_df["acc_x"], label="acc_x")
    plt.plot(imu_df["timestamp"], imu_df["acc_y"], label="acc_y")
    plt.plot(imu_df["timestamp"], imu_df["acc_z"], label="acc_z")
    plt.grid(True)
    plt.xlabel("timestamp")
    plt.ylabel("acc (m/s^2)")
    plt.title("IMU acceleration")
    plt.legend()
    plt.show()


# MOS 2-D plots

def _get_cluster_colors(n: int):
    """Сгенерировать различные цвета для n кластеров, используя tab20 цветовую карту."""
    if n == 0:
        return []
    cmap = plt.cm.get_cmap("tab20", max(n, 1))
    return [cmap(i) for i in range(n)]


def _draw_obb_bev(ax, obb, color):
    """Спроецировать OBB на плоскость X-Y и нарисовать как полигон с подписью."""
    from matplotlib.patches import Polygon
    from scipy.spatial import ConvexHull

    corners_xy = obb.corners()[:, :2]  # (8, 2)
    try:
        hull = ConvexHull(corners_xy)
        poly_pts = corners_xy[hull.vertices]
    except Exception:
        # Вырожденная проекция (точки коллинеарны) — fallback на AABB
        mn, mx = corners_xy.min(0), corners_xy.max(0)
        poly_pts = np.array([
            [mn[0], mn[1]], [mx[0], mn[1]], [mx[0], mx[1]], [mn[0], mx[1]],
        ])

    ax.add_patch(Polygon(poly_pts, closed=True, fill=False,
                         edgecolor=color, linewidth=1.8))
    ax.text(
        obb.center[0], poly_pts[:, 1].max() + 0.3,
        f"#{obb.cluster_id}",
        fontsize=7, color=color, ha="center", va="bottom",
        fontweight="bold",
        bbox=dict(boxstyle="round,pad=0.15", facecolor="white", alpha=0.7, edgecolor="none"),
    )


def plot_mos(
    pc: PointCloud,
    is_moving: np.ndarray,
    ego_params: "np.ndarray | None" = None,
    title: str = "",
    camera_img: "np.ndarray | None" = None,
    cluster_ids: "np.ndarray | None" = None,
    obbs: "list | None" = None,
) -> None:
    """
    2D-графики MOS-результата (static vs moving) с опциональным кадром камеры.

    Панели (без камеры):
      Левый  — radial velocity vs azimuth (только при наличии velocity)
      Правый — bird's-eye view (x, y) с раскраской кластеров

    Панели (с камерой):
      Верхний ряд — velocity vs azimuth + bird's-eye view
      Нижний ряд  — изображение с камеры (на всю ширину)

    Parameters:
        pc          : PointCloud (xyz обязательно, velocity — опционально)
        is_moving   : bool mask
        ego_params  : [Vx, Vy] для отрисовки RANSAC-кривой (опционально)
        title       : заголовок окна
        camera_img  : RGB-массив (H, W, 3) с кадром камеры (опционально)
        cluster_ids : int32 array (N,), >=0 = cluster id, -1 = noise, -2 = static
        obbs        : список OBB из compute_cluster_obbs() — проекции на BEV
    """
    x, y, z = pc.xyz[:, 0], pc.xyz[:, 1], pc.xyz[:, 2]
    azimuth_deg = np.degrees(np.arctan2(y, x))
    has_velocity = pc.velocity is not None
    unique_cluster_ids = (
        np.unique(cluster_ids[cluster_ids >= 0]) if cluster_ids is not None else np.array([], dtype=np.int32)
    )
    has_clusters = cluster_ids is not None and len(unique_cluster_ids) > 0

    static_mask = ~is_moving

    # Build figure layout
    if camera_img is not None:
        if has_velocity:
            fig = plt.figure(figsize=(16, 12))
            gs = fig.add_gridspec(2, 2, height_ratios=[1, 0.65], hspace=0.35, wspace=0.3)
            ax_vel = fig.add_subplot(gs[0, 0])
            ax_bev = fig.add_subplot(gs[0, 1])
            ax_cam = fig.add_subplot(gs[1, :])
        else:
            fig = plt.figure(figsize=(14, 10))
            gs = fig.add_gridspec(2, 1, height_ratios=[1, 0.65], hspace=0.35)
            ax_bev = fig.add_subplot(gs[0])
            ax_cam = fig.add_subplot(gs[1])
    else:
        if has_velocity:
            fig, (ax_vel, ax_bev) = plt.subplots(1, 2, figsize=(16, 7))
        else:
            fig, ax_bev = plt.subplots(1, 1, figsize=(9, 7))

    if title:
        fig.suptitle(title, fontsize=13)

    # Cluster colors
    if has_clusters:
        n_clusters = len(unique_cluster_ids)
        cluster_colors = _get_cluster_colors(n_clusters)
        cid_to_color = {int(cid): cluster_colors[i] for i, cid in enumerate(unique_cluster_ids)}

    # Velocity vs Azimuth
    if has_velocity:
        v = pc.velocity
        ax_vel.scatter(
            azimuth_deg[static_mask], v[static_mask],
            s=0.4, c="#999999", alpha=0.5, label="static", rasterized=True,
        )

        if has_clusters:
            # Раскрашиваем moving точки по кластерам
            noise_mask = is_moving & (cluster_ids < 0)
            ax_vel.scatter(
                azimuth_deg[noise_mask], v[noise_mask],
                s=1.5, c="#e63333", alpha=0.3, label="noise", rasterized=True,
            )
            for cid in unique_cluster_ids:
                cmask = cluster_ids == cid
                c = cid_to_color[int(cid)]
                ax_vel.scatter(
                    azimuth_deg[cmask], v[cmask],
                    s=4, color=c, alpha=0.8, rasterized=True,
                )
        else:
            ax_vel.scatter(
                azimuth_deg[is_moving], v[is_moving],
                s=3, c="#e63333", alpha=0.7, label="moving", rasterized=True,
            )

        if ego_params is not None:
            alpha_sweep = np.linspace(azimuth_deg.min(), azimuth_deg.max(), 500)
            alpha_rad = np.radians(alpha_sweep)
            vr_model = -ego_params[0] * np.cos(alpha_rad) - ego_params[1] * np.sin(alpha_rad)
            ax_vel.plot(alpha_sweep, vr_model, "k-", lw=1.8, label="ego-motion model")

            ego_speed = float(np.sqrt(ego_params[0] ** 2 + ego_params[1] ** 2))
            ax_vel.text(
                0.98, 0.97, f"|V_ego| = {ego_speed:.2f} m/s",
                transform=ax_vel.transAxes, ha="right", va="top",
                fontsize=10, fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
            )

        ax_vel.set_xlabel("Azimuth [deg]")
        ax_vel.set_ylabel("Radial velocity [m/s]")
        ax_vel.set_title("Radial velocity vs Azimuth")
        ax_vel.legend(loc="lower left", fontsize=8, markerscale=3)
        ax_vel.grid(True, alpha=0.3)

    # Bird's-eye view (x, y)
    ax_bev.scatter(
        x[static_mask], y[static_mask],
        s=0.3, c="#999999", alpha=0.4, label="static", rasterized=True,
    )

    if has_clusters:
        noise_mask = is_moving & (cluster_ids < 0)
        ax_bev.scatter(
            x[noise_mask], y[noise_mask],
            s=1, c="#e63333", alpha=0.3, label="noise", rasterized=True,
        )
        for cid in unique_cluster_ids:
            cmask = cluster_ids == cid
            c = cid_to_color[int(cid)]
            ax_bev.scatter(
                x[cmask], y[cmask],
                s=4, color=c, alpha=0.8, rasterized=True,
            )
        if obbs:
            for obb in obbs:
                _draw_obb_bev(ax_bev, obb, cid_to_color.get(obb.cluster_id, "black"))
    else:
        ax_bev.scatter(
            x[is_moving], y[is_moving],
            s=3, c="#e63333", alpha=0.7, label="moving", rasterized=True,
        )

    ax_bev.set_xlabel("x, m")
    ax_bev.set_ylabel("y, m")
    ax_bev.set_title("Bird's-eye view")
    ax_bev.set_aspect("equal")
    ax_bev.legend(loc="upper right", fontsize=8, markerscale=3)
    ax_bev.grid(True, alpha=0.3)

    # Cluster summary
    if has_clusters:
        summary = f"{len(unique_cluster_ids)} clusters detected"
        ax_bev.text(
            0.02, 0.98, summary,
            transform=ax_bev.transAxes, ha="left", va="top",
            fontsize=9, fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
        )

    # Camera image
    if camera_img is not None:
        ax_cam.imshow(camera_img)
        ax_cam.set_title("Stereo Left Camera", fontsize=10)
        ax_cam.axis("off")

    plt.tight_layout()
    plt.show()

# Velocity from GPS / INS
def plot_ego_velocity(Vx: np.ndarray, Vy: np.ndarray, Vz: np.ndarray,
                      ts: np.ndarray, source: str = "GPS") -> None:
    """
    График компонент эго-скорости (Vx, Vy, Vz) и абсолютной скорости от времени.

    Parameters:
        Vx     : np.ndarray — скорость по оси X (ECEF), м/с
        Vy     : np.ndarray — скорость по оси Y (ECEF), м/с
        Vz     : np.ndarray — скорость по оси Z (ECEF), м/с
        ts     : np.ndarray — временные метки (секунды)
        source : str — источник данных ('GPS' или 'INS')
    """
    V = np.sqrt(Vx**2 + Vy**2 + Vz**2)
    t_rel = ts - ts[0]  # относительное время от начала

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.suptitle(f"Ego-velocity from {source}", fontsize=13)

    ax1.plot(t_rel, Vx, label="Vx", linewidth=0.8)
    ax1.plot(t_rel, Vy, label="Vy", linewidth=0.8)
    ax1.plot(t_rel, Vz, label="Vz", linewidth=0.8)
    ax1.set_ylabel("Velocity [m/s]")
    ax1.set_title("ECEF velocity components")
    ax1.legend()
    ax1.grid(True, alpha=0.3)
 
    ax2.plot(t_rel, V, color="#e63333", linewidth=0.8)
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("|V| [m/s]")
    ax2.set_title("Absolute velocity")
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


# MOS sequence rendering (Hercules / Aeva)

import os as _os


def _build_camera_index(camera_dir: str):
    """Return sorted list of (timestamp_ns, filepath) for all .png files."""
    entries = []
    for f in _os.listdir(camera_dir):
        if not f.lower().endswith(".png"):
            continue
        stem = _os.path.splitext(f)[0]
        if stem.isdigit():
            entries.append((int(stem), _os.path.join(camera_dir, f)))
    entries.sort()
    return entries


def _find_closest_camera(cam_index, lidar_ts: int):
    """Binary search for the closest camera timestamp."""
    if not cam_index:
        return None
    lo, hi = 0, len(cam_index) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if cam_index[mid][0] < lidar_ts:
            lo = mid + 1
        else:
            hi = mid
    best = lo
    if lo > 0 and abs(cam_index[lo - 1][0] - lidar_ts) < abs(cam_index[lo][0] - lidar_ts):
        best = lo - 1
    return cam_index[best][1]


def _compute_fixed_limits(bin_files, loader_fn, n_probe=20):
    """
    Scan first n_probe frames to determine stable axis limits.
    Returns dict with azimuth, velocity, x, y ranges (with padding).
    """
    az_min, az_max = np.inf, -np.inf
    v_min, v_max = np.inf, -np.inf
    x_min, x_max = np.inf, -np.inf
    y_min, y_max = np.inf, -np.inf

    step = max(1, len(bin_files) // n_probe)
    probes = bin_files[::step][:n_probe]

    for path in probes:
        pc = loader_fn(path)
        x, y = pc.xyz[:, 0], pc.xyz[:, 1]
        az = np.degrees(np.arctan2(y, x))

        az_min = min(az_min, float(np.percentile(az, 1)))
        az_max = max(az_max, float(np.percentile(az, 99)))
        x_min = min(x_min, float(np.percentile(x, 1)))
        x_max = max(x_max, float(np.percentile(x, 99)))
        y_min = min(y_min, float(np.percentile(y, 1)))
        y_max = max(y_max, float(np.percentile(y, 99)))

        if pc.velocity is not None:
            v_min = min(v_min, float(np.percentile(pc.velocity, 1)))
            v_max = max(v_max, float(np.percentile(pc.velocity, 99)))

    def pad(lo, hi):
        margin = (hi - lo) * 0.1
        return lo - margin, hi + margin

    lim = {
        "az": pad(az_min, az_max),
        "v": pad(v_min, v_max) if v_min < np.inf else (-10, 10),
        "x": pad(x_min, x_max),
        "y": pad(y_min, y_max),
    }
    zoom_r = 30
    lim["x_zoom"] = (-zoom_r, zoom_r)
    lim["y_zoom"] = (-zoom_r, zoom_r)
    return lim


def _draw_mos_frame(fig, axes, pc, is_moving, ego_params, camera_img,
                    frame_idx, n_frames, lim,
                    cluster_ids=None):
    """Clear axes and redraw a single MOS frame with fixed axis limits."""
    ax_cam, ax_bev, ax_vel, ax_zoom = axes

    for ax in axes:
        ax.clear()

    x, y = pc.xyz[:, 0], pc.xyz[:, 1]
    azimuth_deg = np.degrees(np.arctan2(y, x))
    static = ~is_moving
    unique_cluster_ids = (
        np.unique(cluster_ids[cluster_ids >= 0]) if cluster_ids is not None else np.array([], dtype=np.int32)
    )
    has_clusters = cluster_ids is not None and len(unique_cluster_ids) > 0

    # Cluster colors
    if has_clusters:
        n_clusters = len(unique_cluster_ids)
        cluster_colors = _get_cluster_colors(n_clusters)
        cid_to_color = {int(cid): cluster_colors[i] for i, cid in enumerate(unique_cluster_ids)}

    # ── Top-left: Camera
    if camera_img is not None:
        ax_cam.imshow(camera_img)
    ax_cam.set_title("Stereo Left Camera", fontsize=9)
    ax_cam.axis("off")

    # ── Helper: draw clustered or plain moving points on an axis
    def _scatter_moving(ax, xvals, yvals):
        if has_clusters:
            noise_mask = is_moving & (cluster_ids < 0)
            ax.scatter(xvals[noise_mask], yvals[noise_mask],
                       s=1, c="#e63333", alpha=0.3, rasterized=True)
            for cid in unique_cluster_ids:
                cmask = cluster_ids == cid
                c = cid_to_color[int(cid)]
                ax.scatter(xvals[cmask], yvals[cmask],
                           s=4, color=c, alpha=0.8, rasterized=True)
        else:
            ax.scatter(xvals[is_moving], yvals[is_moving],
                       s=3, c="#e63333", alpha=0.7, rasterized=True)

    # ── Top-right: BEV full
    ax_bev.scatter(x[static], y[static],
                   s=0.3, c="#999999", alpha=0.4, rasterized=True)
    _scatter_moving(ax_bev, x, y)
    ax_bev.set_xlim(lim["x"])
    ax_bev.set_ylim(lim["y"])
    ax_bev.set_xlabel("x, m")
    ax_bev.set_ylabel("y, m")
    ax_bev.set_title(
        f"Bird's-eye view ({len(unique_cluster_ids)} clusters)" if has_clusters else "Bird's-eye view",
        fontsize=9,
    )
    ax_bev.set_aspect("equal")
    ax_bev.grid(True, alpha=0.3)

    # ── Bottom-left: Velocity vs Azimuth
    if pc.velocity is not None:
        v = pc.velocity
        ax_vel.scatter(azimuth_deg[static], v[static],
                       s=0.4, c="#999999", alpha=0.5, rasterized=True)
        _scatter_moving(ax_vel, azimuth_deg, v)
        if ego_params is not None:
            a_sweep = np.linspace(lim["az"][0], lim["az"][1], 500)
            vr = -ego_params[0] * np.cos(np.radians(a_sweep)) \
                 - ego_params[1] * np.sin(np.radians(a_sweep))
            ax_vel.plot(a_sweep, vr, "k-", lw=1.5)

            ego_speed = float(np.sqrt(ego_params[0] ** 2 + ego_params[1] ** 2))
            ax_vel.text(
                0.98, 0.97, f"|V_ego| = {ego_speed:.2f} m/s",
                transform=ax_vel.transAxes, ha="right", va="top",
                fontsize=10, fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
            )
    ax_vel.set_xlim(lim["az"])
    ax_vel.set_ylim(lim["v"])
    ax_vel.set_xlabel("azimuth, deg")
    ax_vel.set_ylabel("radial velocity, m/s")
    ax_vel.set_title("Radial velocity vs Azimuth", fontsize=9)
    ax_vel.grid(True, alpha=0.3)

    # ── Bottom-right: BEV zoom
    ax_zoom.scatter(x[static], y[static],
                    s=0.5, c="#999999", alpha=0.4, rasterized=True)
    _scatter_moving(ax_zoom, x, y)
    ax_zoom.set_xlim(lim["x_zoom"])
    ax_zoom.set_ylim(lim["y_zoom"])
    ax_zoom.set_xlabel("x, m")
    ax_zoom.set_ylabel("y, m")
    ax_zoom.set_title("BEV (near range)", fontsize=9)
    ax_zoom.set_aspect("equal")
    ax_zoom.grid(True, alpha=0.3)

    n_mov = int(is_moving.sum())
    n_tot = len(is_moving)
    n_clu = len(unique_cluster_ids) if has_clusters else 0
    fig.suptitle(
        f"Frame {frame_idx + 1}/{n_frames}  |  "
        f"moving {n_mov}/{n_tot} ({100 * n_mov / n_tot:.1f}%)"
        + (f"  |  {n_clu} clusters" if n_clu else ""),
        fontsize=12, y=0.98,
    )


def render_mos_sequence(
    bin_files: list,
    loader_fn,
    segmenter,
    output_dir: str,
    camera_dir: "str | None" = None,
    inlier_threshold: float = 0.5,
    dpi: int = 120,
) -> None:
    """
    Покадровый рендеринг MOS-результатов в PNG (2x2: camera, BEV, velocity, BEV-zoom).

    Parameters:
        bin_files         : отсортированный список путей к .bin кадрам
        loader_fn         : функция загрузки кадра (path -> PointCloud)
        segmenter         : MotionSegmenter (уже загруженный, если нужна модель)
        output_dir        : папка для сохранения PNG
        camera_dir        : папка stereo_left с .png кадрами камеры (или None)
        inlier_threshold  : RANSAC порог [m/s]
        dpi               : DPI сохраняемых изображений
    """
    import matplotlib.image as mpimg
    from src.motion_segmentation import ransac_ego_motion, cluster_moving_objects

    _os.makedirs(output_dir, exist_ok=True)

    cam_index = []
    if camera_dir:
        cam_index = _build_camera_index(camera_dir)
        print(f"Найдено кадров камеры: {len(cam_index)}")

    print("Вычисляю диапазоны осей...")
    lim = _compute_fixed_limits(bin_files, loader_fn)
    print(f"  azimuth: [{lim['az'][0]:.0f}, {lim['az'][1]:.0f}] deg")
    print(f"  velocity: [{lim['v'][0]:.1f}, {lim['v'][1]:.1f}] m/s")
    print(f"  BEV x: [{lim['x'][0]:.0f}, {lim['x'][1]:.0f}] m")
    print(f"  BEV y: [{lim['y'][0]:.0f}, {lim['y'][1]:.0f}] m")

    fig, ((ax_cam, ax_bev), (ax_vel, ax_zoom)) = plt.subplots(
        2, 2, figsize=(14, 9),
        gridspec_kw={"hspace": 0.35, "wspace": 0.3},
    )
    axes = (ax_cam, ax_bev, ax_vel, ax_zoom)

    n = len(bin_files)
    for i, bin_path in enumerate(bin_files):
        pc = loader_fn(bin_path)
        [is_moving] = segmenter.segment_frames([pc])

        ego_params = None
        if pc.velocity is not None:
            ego_params, _ = ransac_ego_motion(pc, inlier_threshold=inlier_threshold)

        # DBSCAN кластеризация движущихся точек
        cluster_ids = cluster_moving_objects(pc, is_moving)

        camera_img = None
        if cam_index:
            stem = _os.path.splitext(_os.path.basename(bin_path))[0]
            if stem.isdigit():
                img_path = _find_closest_camera(cam_index, int(stem))
                if img_path:
                    camera_img = mpimg.imread(img_path)

        _draw_mos_frame(fig, axes, pc, is_moving, ego_params, camera_img, i, n, lim,
                        cluster_ids=cluster_ids)

        out_path = _os.path.join(output_dir, f"{i:06d}.png")
        fig.savefig(out_path, dpi=dpi)
        n_clu = int(np.unique(cluster_ids[cluster_ids >= 0]).size)
        print(f"\r[{i + 1}/{n}] {n_clu} clusters | saved {out_path}", end="", flush=True)

    plt.close(fig)
    print(f"\n\nГотово! {n} кадров сохранено в {output_dir}/")
    print(f"\nСобрать GIF:   magick convert -delay 10 -loop 0 {output_dir}/*.png mos.gif")
    print(f"Собрать видео: ffmpeg -framerate 10 -i {output_dir}/%06d.png -c:v libx264 -pix_fmt yuv420p mos.mp4")