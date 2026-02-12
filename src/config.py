"""
Конфиг файл с константами путей и параметров проекта.
"""

import os
import numpy as np

# Получаем корневую директорию проекта
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ============================================================================
# ПУТИ К ДАННЫМ
# ============================================================================

# Директория с исходными данными
DATA_DIR = os.path.join(PROJECT_ROOT, '03_Day')

# Путь к папке с сенсорными данными
SENSOR_DATA_DIR = os.path.join(DATA_DIR, 'sensor_data')

# GPS данные
GPS_DATA_FILE = os.path.join(SENSOR_DATA_DIR, 'gps.csv')

# INS/IMU данные (INSPVA)
INSPVA_DATA_FILE = os.path.join(SENSOR_DATA_DIR, 'inspva.csv')

# Другие сенсорные данные
AEVA_DATA_FILE = os.path.join(SENSOR_DATA_DIR, 'aeva_stamp.csv')
CONTINENTAL_DATA_FILE = os.path.join(SENSOR_DATA_DIR, 'continental_stamp.csv')
NAVTECH_DATA_FILE = os.path.join(SENSOR_DATA_DIR, 'navtech_stamp.csv')
STEREO_DATA_FILE = os.path.join(SENSOR_DATA_DIR, 'stereo_stamp.csv')
XSENS_IMU_FILE = os.path.join(SENSOR_DATA_DIR, 'xsens_imu.csv')

# Директория с калибровкой
CALIBRATION_DIR = os.path.join(DATA_DIR, 'Calibration')

# Директория с ground truth
GT_DIR = os.path.join(DATA_DIR, 'PR_GT')

# ============================================================================
# ПУТИ К ВЫХОДНЫМ ФАЙЛАМ
# ============================================================================

# Директория для сохранения результатов (по умолчанию корневая директория)
OUTPUT_DIR = PROJECT_ROOT

# GPS карты
GPS_MAP_FILE = os.path.join(OUTPUT_DIR, 'gps_map.html')
GPS_TRAJECTORY_MAP_FILE = os.path.join(OUTPUT_DIR, 'gps_trajectory_map.html')

# Другие выходные файлы
POINT_CLOUD_VIS_FILE = os.path.join(OUTPUT_DIR, 'point_cloud_vis.html')
VELOCITY_PLOT_FILE = os.path.join(OUTPUT_DIR, 'velocity_plot.png')

# ============================================================================
# ПАРАМЕТРЫ ВИЗУАЛИЗАЦИИ
# ============================================================================

# Параметры для GPS карт
MAP_ZOOM_LEVEL = 18  # Уровень приближения на карте (1-19)
MAP_TILES = 'OpenStreetMap'  # Тип карты (OpenStreetMap, Stamen, CartoDB и т.д.)
POLYLINE_COLOR = 'blue'  # Цвет линии траектории
POLYLINE_WEIGHT = 2  # Толщина линии
START_MARKER_COLOR = 'green'  # Цвет маркера начала
END_MARKER_COLOR = 'red'  # Цвет маркера конца

# Параметры для облаков точек
POINT_SIZE = 1  # Размер точек при визуализации
COLORMAP = 'viridis'  # Колормап для интенсивности

# ============================================================================
# ПАРАМЕТРЫ ОБРАБОТКИ
# ============================================================================

# Точность прочтения бинарных файлов
FLOAT_PRECISION = np.float32
