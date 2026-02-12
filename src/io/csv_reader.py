import numpy as np
import struct
import matplotlib.pyplot as plt
import pandas as pd

def read_GPS(filename):
    """
    Чтение GPS данных из CSV файла.
    
    Args:
        filename: путь к GPS CSV файлу
        
    Returns:
        DataFrame с колонками: timestamp, lat, lon, height, 
        velocity_north, velocity_east, velocity_up,
        roll, pitch, azimuth, status и другие
    """
    columns = ['timestamp', 'lat', 'lon', 'height', 
               'velocity_north', 'velocity_east', 'velocity_up',
               'roll', 'pitch', 'azimuth', 'status', '12', '13']
    
    try:
        gps_df = pd.read_csv(filename, names=columns)
        print(f"GPS данные загружены. Всего записей: {len(gps_df)}")
        return gps_df
    except FileNotFoundError:
        print(f"Ошибка: файл {filename} не найден")
        return None
    except Exception as e:
        print(f"Ошибка при чтении GPS файла: {e}")
        return None