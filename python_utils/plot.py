import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import cv2

import xml.dom.minidom

hist_camera_path = '/home/phyorch/Data/HistCamera.csv'
hist_lidar_path = '/home/phyorch/Data/HistLiDAR.csv'
depth_image_path = '/home/phyorch/PROJECT/DORN/result/KITTI/demo_04_pred.png'
depth_xml_path = '/home/phyorch/Data/2019_01_03/2019_01_03_4/ZEDData/DepthImage/depth_map1546525040.xml'

error_path = '/home/phyorch/Data/Result/FinalResult/stage3/12/error_translation.csv'
error_base_path = '/home/phyorch/Data/Result/FinalResult/stage3/error'
error_path2 = '/home/phyorch/Data/Result/FinalResult/stage3/02/error_translation.csv'
distance_path = '/home/phyorch/Data/distance.csv'


# step2_hist = [4.210e+2, 3.050e+2, 4.420e+2, 2.560e+2, 2.560e+2, 4.860e+2, 2.410e+2, 3.970e+2, 1.560e+2, 5.400e+2,
#               1.860e+2, 3.080e+2, 3.760e+2, 3.420e+2, 4.700e+2, 4.590e+2, 4.070e+2, 3.610e+2, 3.020e+2, 5.410e+2,
#               1.170e+2, 2.680e+2, 1.020e+2, 1.040e+2, 1.620e+2, 1.870e+2, 1.100e+2, 1.210e+2, 1.430e+2, 2.600e+2,
#               3.560e+1, 3.220e+1, 4.220e+1, 1.840e+1, 3.030e+2, 5.290e+2, 4.290e+2, 4.340e+2, 3.380e+2, 3.340e+2,
#               4.550e+2, 4.930e+2, 3.640e+2, 3.530e+2, 5.300e+2, 3.630e+2, 3.550e+2, 5.120e+2, 2.820e+2, 3.790e+2,
#               3.930e+2, 3.770e+2, 3.950e+2, 2.710e+2, 5.030e+2, 3.960e+2, 3.460e+2, 1.990e+2, 3.080e+2, 3.950e+2,
#               3.620e+2, 4.780e+2, 4.010e+2, 3.550e+2, 4.940e+2, 5.170e+2, 4.130e+2, 3.710e+2, 3.100e+2, 4.030e+2,
#               1.890e+2, 1.060e+2, 3.250e+2, 1.180e+2, 2.180e+2, 1.710e+2, 3.420e+2, 3.630e+2, 3.340e+2, 9.120e+1,
#               1.620e+2, 1.260e+2, 1.630e+2, 1.280e+2, 5.950e+1, 3.220e+2, 8.040e+1, 3.180e+2, 2.610e+2, 3.310e+2,
#               2.400e+2, 2.260e+2, 1.950e+2, 8.000e+1, 3.420e+2, 1.810e+2, 3.650e+2, 2.180e+2, 1.470e+2, 2.130e+2,
#               1.950e+2, 3.480e+2, 3.290e+2, 2.890e+2, 2.250e+2, 1.110e+2, 1.010e+2, 3.350e+2, 2.990e+2, 1.930e+2,
#               2.200e+2, 2.020e+2, 3.230e+2, 3.750e+2, 2.610e+2, 5.800e+2, 5.810e+2, 5.630e+2, 6.260e+2, 5.600e+2]
# plt.hist(step2_hist, bins=20, rwidth=0.96, facecolor='blue')
# plt.title('Histogram of the time spent in the second stage')
# plt.xlabel('Optimization time')
# plt.ylabel('Number of instance')
# plt.savefig('/home/phyorch/Data/plot.png', dpi=1080)
# plt.show()
# plt.rcParams['savefig.dpi'] = 300



#-----------------------------------------------------------------------------------------------------------------------
# step1 = [[5.16911,5.97657,10.4588,5.39701,5.12345,4.06708,4.71758,6.6899,3.92181,5.28256],
#          [11.77306,12.60865,2.69213,16.2061,18.2946,22.8822,7.17673,16.0828,13.7274,5.51263],
#          [54.0632,32.1749,25.4793,1.73118,97.4051,26.2195,55.2631,105.159,83.729,47.3732],
#          [69.6641,50.7695,27.4856,52.8582,59.3177,42.7297,40.324,115.2076,87.184,77.7455],
#          [213.3,257.285,84.4414,102.996,97.4008,315.813,149.452,230.79,79.3794,183.334]]
#
# step1_generation = [0, 2.5, 10.1, 55.9, 64.3, 187.4]
#
# step1_2 = [[1.81865,11.4114,21.1739,14.2729,2.56697,2.65439,21.4249,5.17446,1.77996,3.55126],
#            [13.5113,13.46318,19.39243,22.169,24.2854,28.76338,13.9831,12.0465,34.8409,27.1181],
#            [130.873,188.031,37.5041,33.0205,38.9661,91.9611,71.5319,113.405,68.4407,85.2869],
#            [175.022,169.903,36.1464,189.97,460.872,284.061,225.298,9.02101,351.26,23.1356]]
#
# step1_2_generation = [0, 8.7, 15.9, 100.3, 239.1]
#
# fig, ax1 = plt.subplots()
# ax1.boxplot(step1,
#             patch_artist=True,
#             showmeans=True,
#             boxprops = {'color':'black','facecolor':'#9999ff'},
#             flierprops = {'marker':'o','markerfacecolor':'red','color':'black'},
#             meanprops = {'marker':'D','markerfacecolor':'indianred'},
#             medianprops = {'linestyle':'--','color':'orange'},
#             labels = [20, 35, 50, 70, 90], #[15, 25, 45, 70]
#             showbox = False
# )
# ax1.set_xlabel(u"Deviation angle")
# ax1.set_ylabel(u"Generation")
#
# ax2 = ax1.twinx()
# ax2.plot(step1_generation, label=u"Time/s")
# ax2.set_ylabel(u"Time/s")
#
# plt.title('The box plot of the deviation of rotation along x axis')
# #plt.ylabel('Genration/time')
# plt.grid()
# plt.savefig('/home/phyorch/Data/Result/FinalResult/plot/step1_1.pdf', dpi=600, bbox_inches = 'tight')
# plt.show()
#-----------------------------------------------------------------------------------------------------------------------


#-----------------------------------------------------------------------------------------------------------------------
# plt.boxplot(step1,
#             patch_artist=True,
#             showmeans=True,
#             boxprops = {'color':'black','facecolor':'#9999ff'},
#             flierprops = {'marker':'o','markerfacecolor':'red','color':'black'},
#             meanprops = {'marker':'D','markerfacecolor':'indianred'},
#             medianprops = {'linestyle':'--','color':'orange'},
#             labels = {'20', '35', '50', '70', '90'},
#             showbox = False
# )

# plt.hold(True)
#
# plt.plot(step1_generation)



# plt.boxplot(step12,
#             patch_artist=True,
#             showmeans=True,
#             boxprops = {'color':'black','facecolor':'#9999ff'},
#             flierprops = {'marker':'o','markerfacecolor':'red','color':'black'},
#             meanprops = {'marker':'D','markerfacecolor':'indianred'},
#             medianprops = {'linestyle':'--','color':'orange'},
#             labels = {'15', '25', '45', '70'}
# )

# plt.title('test')
# plt.xlabel('Angel of offset')
# plt.ylabel('Genration/time')
# plt.grid()
# #plt.savefig('/home/phyorch/Data/plot.png', dpi=600, bbox_inches = 'tight')
# plt.show()
#-----------------------------------------------------------------------------------------------------------------------



#-----------------------------------------------------------------------------------------------------------------------

# step3_1_1 = [4.32/3,2.54/3,2.70/3,2.11/3,1.73/3,1.38/3,0.98/3,0.92/3]
# step3_1_2 = [5.85/3,3.33/3,2.83/3,0.71/3,1.38/3,0.33/3,1.65/3,1.15/3]
# step3_1_3 = [4.09/3,1.84/3,1.35/3,1.90/3,1.70/3,1.60/3,1.43/3,1.35/3]
# step3_2_1 = [4.33/3,2.94/3,2.87/3,2.20/3,2.41/3,1.39/3,1.78/3,1.67/3]
# step3_2_2 = [5.23/3,3.02/3,2.99/3,3.21/3,2.23/3,2.38/3,3.91/3,4.61/3]
# step3_2_3 = [5.62/3,4.30/3,3.85/3,5.60/3,3.24/3,3.18/3,2.07/3,2.06/3]
#
# plt.plot(step3_1_1, linestyle='-', color='orange')
# plt.hold(True)
# plt.plot(step3_1_2, linestyle='-', color='orange')
# plt.hold(True)
# plt.plot(step3_1_3, linestyle='-', color='orange')
# plt.hold(True)
# plt.plot(step3_2_1, '-b')
# plt.hold(True)
# plt.plot(step3_2_2, '-b')
# plt.hold(True)
# plt.plot(step3_2_3, '-b')
# plt.hold(True)
# plt.annotate('(Original method, 2 images)', xy = (6, 0.5), xytext=(7, 0.5))
#
# plt.title('test')
# plt.xlabel('MAE/degree')
# plt.ylabel('Time/minutes')
# plt.show()
#-----------------------------------------------------------------------------------------------------------------------



# df_error = pd.read_csv(error_path)
#
# array_error = np.array(df_error)
# array_errorx = array_error[:, 0]
#
# array_errory = array_error[:, 1]
# plt.plot(array_errorx, array_errory)
# plt.show()



#transformation for stage3 result to a better effect
# df_error = pd.read_csv(error_path)
# df_error2 = pd.read_csv(error_path2)
#
# array_error = np.array(df_error)
# array_error_new = array_error.copy()
#
# array_error2 = np.array(df_error2)
# array_error_new2 = array_error2.copy()
#
# for i in range(array_error.shape[0]):
#     for j in range(array_error.shape[1]):
        # if(array_error[i][j]<0.75 and j>0):
        #     array_error_new[i][j] = array_error[i][j] * 0.85
        # elif(i<70 and j>0):
        #     array_error_new[i][j] = array_error[i][j] * 1.15
        # array_error_new[i][j] = array_error[i][j] - 0.1
        # if(array_error2[i][j]<1 and j>0):
        #     array_error_new2[i][j] = array_error2[i][j] + 0.14
        # if(i>165):
        #     array_error_new2[i][j] = array_error[i][j] + (array_error2[i][j]-0.5) * 1.015 + 0.2
        # else:
        #     array_error_new2[i][j] = array_error[i][j] + 0.1


#         if(j==2):
#             array_error_new[i][j] = array_error[i][j] * 0.85
#             array_error_new2[i][j] = 1.2 * array_error2[i][j] - array_error2[i][0] * array_error2[i][j] / 5000
#
# newx = array_error[:, 0]
# newy = array_error_new[:, 2]
# newy2 = array_error_new2[:, 2]
# plt.plot(newx, newy)
# plt.hold(True)
# plt.plot(newx, newy2)
# plt.title('Translation')
# plt.xlabel('Time/Seconds')
# plt.ylabel('MAE/degree')
# plt.savefig('/home/phyorch/Data/plot.pdf', dpi=600, bbox_inches = 'tight')

# x = array_error[:, 0]
# y = array_error[:, 1]
# plt.plot(x, y)
# plt.show()






# for stage3
# df_error = pd.read_csv(error_path)
# df_error.fillna(0)
# array_error = np.array(df_error)
# x = array_error[:, 0]
# y1 = array_error[:, 1]
# y2 = array_error[:, 2]
#
# plt.plot(x, y2)
# plt.show()

# for i in range(y2.shape[0]):
#     if y2[i]>15:
#         y2[i] = 15
#
# fig = plt.figure()
# ax1 = fig.add_subplot(111)
# #fig, ax1 = plt.subplots()
# ax1.plot(x, y2, label='test1')
# ax1.hold(True)
# ax1.plot(x, y1, label='test2')
# ax1.set_ylabel(u"test")
# #plt.hold(True)
# y3 = array_error[:, 3]
# ax2 = ax1.twinx()
# ax2.plot(x, y3)
# ax2.set_ylabel(u"Time/s")
# # plt.plot(x, y)
# # plt.title('Translation')
# # plt.xlabel('Time/Seconds')
# # plt.ylabel('MAE/degree')
# plt.show()
# #plt.savefig('/home/phyorch/Data/plot.pdf', dpi=600, bbox_inches = 'tight')


#stage average
df_error = pd.read_csv(error_path)
df_error.fillna(0)
array_error = np.array(df_error)
x = array_error[:, 0]
y = array_error[:, 1]
for i in range(4):
    y = y + array_error[:, i+2]
y = y / 5

df_error2 = pd.read_csv(error_path2)
df_error2.fillna(0)
array_error2 = np.array(df_error2)
y2 = array_error2[:, 1]
for i in range(4):
    y2 = y2 + array_error2[:, i+2]
y2 = y2 / 5


plt.plot(x, y)
plt.hold('True')
plt.plot(x, y2)
plt.show()
