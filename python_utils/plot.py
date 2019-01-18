import pandas as pd
import matplotlib.pyplot as plt
hist_camera_path = '/home/phyorch/Data/HistCamera.csv'
hist_lidar_path = '/home/phyorch/Data/HistLiDAR.csv'


df_camera = pd.read_csv(hist_camera_path)
df_lidar = pd.read_csv(hist_lidar_path)
# df.plot()   #kind='bar'
plt.hist(df_camera.values, bins=1000)
plt.hist(df_lidar.values, bins=1000)
plt.show()
a = 1