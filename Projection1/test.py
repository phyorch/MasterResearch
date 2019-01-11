import numpy as np

points_path = "/home/phyorch/MasterResearch/Data/2011_09_26/data/2011_09_26_drive_0048_sync/velodyne_points/data/0000000000.bin"
points = np.fromfile(points_path, dtype=np.float32)#.reshape(-1, 4)
a = 1