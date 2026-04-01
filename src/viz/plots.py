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

def plot_velocity_comparison(gps_path: str, ins_path: str, output_path: str,
                             aeva_path: "str | None" = None,
                             radar_path: "str | None" = None,
                             inlier_threshold: float = 0.5) -> None:
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

    # Вычисление скоростей
    Vx_gps, Vy_gps, ts_gps = GPS_to_V(gps_raw[["timestamp", "lat", "lon", "height"]].copy())
    speed_gps = np.sqrt(Vx_gps**2 + Vy_gps**2)

    Vx_ins, Vy_ins, ts_ins = INS_to_V(ins_raw[["timestamp", "latitude", "longitude",
                                                 "north_velocity", "east_velocity",
                                                 "up_velocity"]].copy())
    speed_ins = np.sqrt(Vx_ins**2 + Vy_ins**2)

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
    ax1.set_title(f"Скорость (м/с) от времени (с) (GPS + INS{title_suffix})",
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
    ax2.set_title("Угловая скорость рыскания (°/с)", loc="left", fontsize=11, fontweight="bold")

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
    ax3.set_title("Траектория с местами поворотов", loc="left", fontsize=11, fontweight="bold")

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
# Palette matching the Open3D one in clouds.py
_MOS_PALETTE = [
    "#ff3333",  # red
    "#ff9900",  # orange
    "#ffff00",  # yellow
    "#00e633",  # green
    "#00ccff",  # cyan
    "#9933ff",  # violet
    "#ff66cc",  # pink
    "#ccff66",  # lime
    "#66ccff",  # sky
    "#ffcc66",  # peach
]


def plot_mos(
    pc: PointCloud,
    is_moving: np.ndarray,
    ego_params: "np.ndarray | None" = None,
    title: str = "",
    camera_img: "np.ndarray | None" = None,
) -> None:
    """
    2D-графики MOS-результата (static vs moving) с опциональным кадром камеры.

    Панели (без камеры):
      Левый  — radial velocity vs azimuth (только при наличии velocity)
      Правый — bird's-eye view (x, y)

    Панели (с камерой):
      Верхний ряд — velocity vs azimuth + bird's-eye view
      Нижний ряд  — изображение с камеры (на всю ширину)

    Parameters:
        pc          : PointCloud (xyz обязательно, velocity — опционально)
        is_moving   : bool mask
        ego_params  : [Vx, Vy] для отрисовки RANSAC-кривой (опционально)
        title       : заголовок окна
        camera_img  : RGB-массив (H, W, 3) с кадром камеры (опционально)
    """
    x, y, z = pc.xyz[:, 0], pc.xyz[:, 1], pc.xyz[:, 2]
    azimuth_deg = np.degrees(np.arctan2(y, x))
    has_velocity = pc.velocity is not None

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

    # Velocity vs Azimuth
    if has_velocity:
        v = pc.velocity
        ax_vel.scatter(
            azimuth_deg[static_mask], v[static_mask],
            s=0.4, c="#999999", alpha=0.5, label="static", rasterized=True,
        )
        ax_vel.scatter(
            azimuth_deg[is_moving], v[is_moving],
            s=3, c="#e63333", alpha=0.7, label="moving", rasterized=True,
        )
        if ego_params is not None:
            alpha_sweep = np.linspace(azimuth_deg.min(), azimuth_deg.max(), 500)
            alpha_rad = np.radians(alpha_sweep)
            vr_model = -ego_params[0] * np.cos(alpha_rad) - ego_params[1] * np.sin(alpha_rad)
            ax_vel.plot(alpha_sweep, vr_model, "k-", lw=1.8, label="ego-motion model")

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

    # Camera image
    if camera_img is not None:
        ax_cam.imshow(camera_img)
        ax_cam.set_title("Stereo Left Camera", fontsize=10)
        ax_cam.axis("off")

    plt.tight_layout()
    plt.show()

# ── Velocity from GPS / INS ─────────────────────────────────────────────────
 
def plot_ego_velocity(Vx: np.ndarray, Vy: np.ndarray, ts: np.ndarray,
                      source: str = "GPS") -> None:
    """
    График компонент эго-скорости (Vx, Vy) и абсолютной скорости от времени.
 
    Parameters:
        Vx     : np.ndarray — скорость по оси X (ECEF), м/с
        Vy     : np.ndarray — скорость по оси Y (ECEF), м/с
        ts     : np.ndarray — временные метки (секунды)
        source : str — источник данных ('GPS' или 'INS')
    """
    V = np.sqrt(Vx**2 + Vy**2)
    t_rel = ts - ts[0]  # относительное время от начала
 
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.suptitle(f"Ego-velocity from {source}", fontsize=13)
 
    ax1.plot(t_rel, Vx, label="Vx", linewidth=0.8)
    ax1.plot(t_rel, Vy, label="Vy", linewidth=0.8)
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