import numpy as np
import pyproj
 
 
def geodetic_to_cartesian(lat: float, lon: float, alt: float):
    """
    Преобразование геодезических координат в декартовы (ECEF).
 
    Parameters:
        lat : широта (градусы)
        lon : долгота (градусы)
        alt : высота (метры)
 
    Returns:
        (x, y, z) в метрах (ECEF)
    """
    # Убеждаемся, что используется lon, lat порядок (always_xy=True) во избежание свапа осей
    geodetic_crs = pyproj.CRS("EPSG:4326")
    ecef_crs = pyproj.CRS("EPSG:4978")
    transformer = pyproj.Transformer.from_crs(geodetic_crs, ecef_crs, always_xy=True)
    x, y, z = transformer.transform(lon, lat, alt)
    return x, y, z
 
 
def GPS_to_V(GPS):
    """
    Вычисление скорости по GPS-координатам методом конечных разностей.
 
    Parameters:
        GPS : dict-like (DataFrame) с ключами:
              'timestamp' (нс), 'lat', 'lon', 'height'

    Returns:
        Vx : np.ndarray — скорость по оси X (ECEF), м/с
        Vy : np.ndarray — скорость по оси Y (ECEF), м/с
        Vz : np.ndarray — скорость по оси Z (ECEF), м/с
        ts : np.ndarray — временные метки (секунды)
    """
    # Vectorized, robust computation using correct axis ordering
    geodetic_crs = pyproj.CRS("EPSG:4326")
    ecef_crs = pyproj.CRS("EPSG:4978")
    transformer = pyproj.Transformer.from_crs(geodetic_crs, ecef_crs, always_xy=True)

    # timestamps -> seconds
    ts_all = np.asarray(GPS['timestamp'], dtype=float) / 1e9

    lons = np.asarray(GPS['lon'], dtype=float)
    lats = np.asarray(GPS['lat'], dtype=float)
    alts = np.asarray(GPS['height'], dtype=float)

    # трансформирует массивы (lon, lat, alt) -> (x, y, z) в ECEF
    x_all, y_all, z_all = transformer.transform(lons, lats, alts)

    # конечные разности между последовательными кадрами
    dt = ts_all[1:] - ts_all[:-1]
    valid = dt > 0
    if not valid.any():
        return np.array([]), np.array([]), np.array([]), np.array([])

    dx = x_all[1:] - x_all[:-1]
    dy = y_all[1:] - y_all[:-1]
    dz = z_all[1:] - z_all[:-1]

    Vx_arr = dx[valid] / dt[valid]
    Vy_arr = dy[valid] / dt[valid]
    Vz_arr = dz[valid] / dt[valid]
    ts_arr = ts_all[1:][valid]

    return np.array(Vx_arr), np.array(Vy_arr), np.array(Vz_arr), np.array(ts_arr)
 