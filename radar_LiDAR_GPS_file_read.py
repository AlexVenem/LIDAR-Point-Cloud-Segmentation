import numpy as np
import struct
import matplotlib.pyplot as plt
import pandas as pd

'''SENSORS READ FUCTION'''
def read_GPS(filename):
    columns = ['timestamp', 'lat', 'lon', 'height', 
               'velocity_north', 'velocity_east', 'velocity_up',
               'roll', 'pitch', 'azimuth', 'status', '12', '13']
    gps_df = pd.read_csv(filename, names = columns)
    return gps_df

def read_INS(filename):
    columns = ['timestamp', 'latitude', 'longitude', 'height', 
               'north velocity', 'east velocity', 'up velocity', 
               'roll', 'pitch', 'azimuth', 'status']
    gps_df = pd.read_csv(filename, names = columns)
    return gps_df

def read_IMU(filename):
    columns = ['timestamp', 'qx', 'qy', 'qz', 'qw', 
               'eul x', 'eul y', 'eul z', 'gyr x', 'gyr y', 'gyr z', 
               'acc x', 'acc y', 'acc z', 'mag x', 'mag y', 'mag z']
    gps_df = pd.read_csv(filename, names = columns)
    return gps_df
    
def read_radar_bin_file(filename):
    points = []
    timestamp = int(filename[-23:-4])
    with open(filename, "rb") as file:
        while True:
            data = file.read(29)
            if len(data) < 29:
                break
            x, y, z, v, r = struct.unpack('fffff', data[:20])
            RCS = struct.unpack('B', data[20:21])[0]
            az, el = struct.unpack('ff', data[21:29])            
           
            points.append([x, y, z, v, r, RCS, az, el])
    return points, timestamp

def read_LiDAR_bin_file(filename):
    points = []
    timestamp = int(filename[-23:-4])
    with open(filename, "rb") as file:
        while True:
            if int(filename.split("/")[-1].split('.')[0]) > 1691936557946849179: 
                data = file.read(29)
                if len(data) < 29:
                    break
                x, y, z, reflectivity, velocity = struct.unpack('fffff', data[:20])
                time_offset_ns = struct.unpack('I', data[20:24])[0]
                line_index = struct.unpack('B', data[24:25])
                intensity = struct.unpack('f', data[25:29])
            elif int(filename.split("/")[-1].split('.')[0]) <= 1691936557946849179: 
                data = file.read(25)
                if len(data) < 25:
                    break
                x, y, z, reflectivity, velocity = struct.unpack('fffff', data[:20])
                time_offset_ns = struct.unpack('I', data[20:24])[0]
                line_index = struct.unpack('B', data[24:25])            
            points.append([x, y, z, reflectivity, velocity, time_offset_ns])
    
    return points, timestamp

    

'''----------------------MAIN------------------------'''
''' RADAR '''  
radar_bin_file = '03_Day/Radar/Continental/1738300449254828899.bin'
pointcloud, radar_TS = read_radar_bin_file(radar_bin_file)
PC = np.array(pointcloud)
radar_PC_df = pd.DataFrame(PC, columns = ['x', 'y', 'z', 'Vr', 'range', 'RCS', 'azimuth', 'elevation'])


fig = plt.figure(1, figsize=(32, 16))
plt.subplot(2,1,1)
plt.scatter(radar_PC_df['x'], radar_PC_df['y'],8)  # reflectivity in log scale 
plt.grid()
plt.xlabel('x, m', fontsize=26)
plt.ylabel('y, m', fontsize=26)
plt.title('RADAR. xy', fontsize=26)
plt.axis([0, 150, -30, 30])
plt.xticks(fontsize=26)
plt.yticks(fontsize=26)

plt.subplot(2,1,2)
plt.scatter(radar_PC_df['azimuth']*180/np.pi, radar_PC_df['Vr'],8)
plt.grid()
plt.xlabel('azimuth, deg', fontsize=26)
plt.ylabel('velocity, m/s', fontsize=26)
plt.title('radial velocity', fontsize=26)
plt.xticks(fontsize=26)
plt.yticks(fontsize=26)
plt.axis([-60, 60, -15, 10])


''' LiDAR ''' 
LiDAR_bin_file = '03_Day/Aeva/1738300381892782006.bin'
pointcloud, LiDAR_TS = read_LiDAR_bin_file(LiDAR_bin_file)
PC = np.array(pointcloud)

LiDAR_PC_df = pd.DataFrame(PC, columns = ['x', 'y', 'z', 'reflectivity', 'Vr', 'time_ns'])
az = np.arctan2(LiDAR_PC_df['y'], LiDAR_PC_df['x'])*180/np.pi


fig = plt.figure(2, figsize=(32, 16))
plt.subplot(2,1,1)
plt.scatter(LiDAR_PC_df['x'], LiDAR_PC_df['y'],8, np.ceil(2*np.log10(LiDAR_PC_df['reflectivity'])))  # reflectivity in log scale 
plt.grid()
plt.xlabel('x, m', fontsize=26)
plt.ylabel('y, m', fontsize=26)
plt.title('LiDAR. xy', fontsize=26)
plt.axis([0, 150, -30, 30])
plt.xticks(fontsize=26)
plt.yticks(fontsize=26)

plt.subplot(2,1,2)
plt.scatter(az, LiDAR_PC_df['Vr'],8)
plt.grid()
plt.xlabel('azimuth, deg', fontsize=26)
plt.ylabel('velocity, m/s', fontsize=26)
plt.title('radial velocity', fontsize=26)
plt.xticks(fontsize=26)
plt.yticks(fontsize=26)
plt.axis([-60, 60, -15, 10])


'''GPS'''
'''
GPS = read_GPS('D:/datasets/Radar_FMCW_LiDAR/Sports Complex_03_Day/sensor_data_20250308T225318Z_001/sensor_data/gps.csv')
fig = plt.figure(3, figsize=(32, 16))
plt.scatter(GPS['lat'], GPS['lon'],8)
plt.grid()
plt.xlabel('lat', fontsize=26)
plt.ylabel('lon', fontsize=26)
'''

'''INS'''
INS = read_INS('03_Day/sensor_data/inspva.csv')
fig = plt.figure(4, figsize=(32, 16))
plt.scatter(INS['latitude'], INS['longitude'],8)
plt.grid()
plt.xlabel('latitude', fontsize=26)
plt.ylabel('longitude', fontsize=26)


'''IMU'''
IMU = read_IMU('03_Day/sensor_data/xsens_imu.csv')


plt.show()