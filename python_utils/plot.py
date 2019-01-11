import pandas as pd
import matplotlib.pyplot as plt
hist_lidar_path = '/home/phyorch/Data/lidar_distance_test.csv'


df_lidar = pd.read_csv(hist_lidar_path)
# df.plot()   #kind='bar'
#plt.hist(df_camera.values, bins=1000)
plt.hist(df_lidar.values, bins=1000)
plt.show()
a = 1