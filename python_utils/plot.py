# import pandas as pd
# import matplotlib.pyplot as plt
import numpy as np
import cv2

import xml.dom.minidom

hist_camera_path = '/home/phyorch/Data/HistCamera.csv'
hist_lidar_path = '/home/phyorch/Data/HistLiDAR.csv'
depth_image_path = '/home/phyorch/PROJECT/DORN/result/KITTI/demo_04_pred.png'
depth_xml_path = '/home/phyorch/Data/2019_01_03/2019_01_03_4/ZEDData/DepthImage/depth_map1546525040.xml'

# csv show
# df_camera = pd.read_csv(hist_camera_path)
# df_lidar = pd.read_csv(hist_lidar_path)
# plt.hist(df_camera.values, bins=1000)
# plt.hist(df_lidar.values, bins=1000)
# plt.show()
# a = 1


#image numpy show

xml_file = xml.dom.minidom.parse(depth_xml_path)
root = xml_file.documentElement
depth = root.getElementsByTagName('data')[0]
depth_text = depth.childNodes[0].data
depth_list = depth_text.split(' ')

depth_list2 = []
for elem in depth_list:
    if(elem!='\n' and elem!=''):
        if(elem.find('\n')):
            depth_list2.append(elem.split('\n')[0])
        else:
            depth_list2.append(elem)

depth_np = np.array(depth_list2)
depth_np = depth_np.reshape((720,1280))
#depth_np = depth_np.astype(float)
#depth_np = depth_np / 1000


depth_image = cv2.imread(depth_image_path, cv2.IMREAD_ANYDEPTH)
projected_depths = np.float32(depth_image/256.0)
a = 1
