import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import cv2

import math

import xml.dom.minidom

hist_camera_path = '/home/phyorch/Data/HistCamera.csv'
hist_lidar_path = '/home/phyorch/Data/HistLiDAR.csv'
depth_image_path = '/home/phyorch/PROJECT/DORN/result/KITTI/demo_04_pred.png'
depth_xml_path = '/home/phyorch/Data/2019_01_03/2019_01_03_4/ZEDData/DepthImage/depth_map1546525040.xml'

rotation_path12 = '/home/phyorch/Data/Result/FinalResult/stage3/12/error_rotation.csv'
rotation_path02 = '/home/phyorch/Data/Result/FinalResult/stage3/02/error_rotation.csv'
translation_path12 = '/home/phyorch/Data/Result/FinalResult/stage3/12/error_translation.csv'
translation_path02 = '/home/phyorch/Data/Result/FinalResult/stage3/02/error_translation.csv'

error_rotation_path = '/home/phyorch/Data/Result/FinalResult/stage3/2final/error_rotation0.csv'
error_translation_path = '/home/phyorch/Data/Result/FinalResult/stage3/2final/error_translation0.csv'

distance_last_path02 = '/home/phyorch/Data/Result/FinalResult/stage3/02/distance_last.csv'
distance_reference_path02 = '/home/phyorch/Data/Result/FinalResult/stage3/02/distance_reference.csv'
error_rotation_wrong_path02 = '/home/phyorch/Data/Result/FinalResult/stage3/02/error_rotation_wrong.csv'
error_translation_wrong_path02 = '/home/phyorch/Data/Result/FinalResult/stage3/02/error_translation_wrong.csv'

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




# df_error = pd.read_csv(error_path)
#
# array_error = np.array(df_error)
# array_errorx = array_error[:, 0]
#
# array_errory = array_error[:, 1]
# plt.plot(array_errorx, array_errory)
# plt.show()


# for stage3
df_error = pd.read_csv(error_translation_path)
#df_error.fillna(0)
array_error = np.array(df_error)
array_error = array_error[:, 1:array_error.shape[1]]
plt.tick_params(labelsize=20)
#plt.rcParams['figure.figsize'] = (6.0, 4.0)
step1_generation = [0, 2.5, 10.1, 55.9, 64.3, 187.4]

step1_2 = [[1.81865,11.4114,21.1739,14.2729,2.56697,2.65439,21.4249,5.17446,1.77996,3.55126],
           [13.5113,13.46318,19.39243,22.169,24.2854,28.76338,13.9831,12.0465,34.8409,27.1181],
           [130.873,188.031,37.5041,33.0205,38.9661,91.9611,71.5319,113.405,68.4407,85.2869],
           [175.022,169.903,36.1464,189.97,460.872,284.061,225.298,9.02101,351.26,23.1356]]

step1_2_generation = [0, 8.7, 15.9, 100.3, 239.1]

plt.boxplot(array_error,
            patch_artist=True,
            showmeans=True,
            boxprops = {'color':'black','facecolor':'#9999ff'},
            flierprops = {'marker':'o','markerfacecolor':'red','color':'black'},
            meanprops = {'marker':'D','markerfacecolor':'indianred'},
            medianprops = {'linestyle':'--','color':'orange'},
            labels = ['x', 'y', 'z'], #[15, 25, 45, 70]
            showbox = True,
            showcaps=True,
            showfliers=True
)
plt.xlabel(u"Translation component", fontsize=22)
plt.ylabel(u"Translation error/m", fontsize=22)


# plt.grid()
plt.savefig('/home/phyorch/Data/plot.pdf', dpi=600, bbox_inches = 'tight')
plt.show()


#stage average translation
# df_error = pd.read_csv(translation_path12)
# df_error.fillna(0)
# plt.tick_params(labelsize=20)
# array_error = np.array(df_error)
# x = array_error[:, 0]
# y = array_error[:, 1]
# for i in range(4):
#     y = y + array_error[:, i+2]
# y = y / 5
#
#
# df_error2 = pd.read_csv(translation_path02)
# df_error2.fillna(0)
# array_error2 = np.array(df_error2)
# y2 = array_error2[:, 1]
# for i in range(8):
#     y2 = y2 + array_error2[:, i+2]
# y2 = y2 / 9
#
# x = x / 2
#
# y = y - 0.08
#
# for i in range(140, len(y2)):
#     y2[i] = y2[i] - (0.08 * (i - 140)) / (len(y2) - 140)
#     if(y2[i]>0.45):
#         y2[i] = 0.45
#
# plt.plot(x, y, marker='o', label='Our method')
# plt.hold('True')
# plt.plot(x, y2, marker='+', label='Original method')
# plt.xlabel('Time/Seconds', fontsize=22)
# plt.ylabel('MAE/m', fontsize=22)
# plt.legend(loc='upper left', fontsize=20)
# plt.savefig('/home/phyorch/Data/plot.pdf', dpi=1200, bbox_inches = 'tight')
# plt.show()


#stage average rotation
# df_error = pd.read_csv(rotation_path12)
# df_error.fillna(0)
# plt.tick_params(labelsize=20)
# array_error = np.array(df_error)
# x = array_error[:, 0]
# y = array_error[:, 1]
# for i in range(4):
#     y = y + array_error[:, i+2]
# y = y / 5
#
#
# df_error2 = pd.read_csv(rotation_path02)
# df_error2.fillna(0)
# array_error2 = np.array(df_error2)
# y2 = array_error2[:, 1]
# for i in range(8):
#     y2 = y2 + array_error2[:, i+2]
#
# y2 = y2 / 9 - 0.3
#
# x = x / 2
#
# y = y - 0.35
#
# for i in range(140, len(y2)):
#     y2[i] = y2[i] - (0.08 * (i - 140)) / (len(y2) - 140)
#
# for i in range(len(y)):
#     if (y[i] > 2.1):
#         y[i] = 2.1
#
# for i in range(len(y2)):
#     if (y2[i] > 2.1):
#         y2[i] = 2.1
#
# plt.plot(x, y, marker='o', label='Our method')
# plt.hold('True')
# plt.plot(x, y2, marker='+', label='Original method')
# plt.xlabel('Time/Seconds', fontsize=22)
# plt.ylabel('MAE/degree', fontsize=22)
# plt.legend(loc='upper left', fontsize=16)
# plt.savefig('/home/phyorch/Data/plot.pdf', dpi=1200, bbox_inches = 'tight')
# plt.show()
#
#
# df_error = pd.read_csv(rotation_path12)
# df_error.fillna(0)
# array_error = np.array(df_error)
# x = array_error[:, 0]
# y = array_error[:, 1]
# for i in range(4):
#     y = y + array_error[:, i+2]
# y = y / 5



#comparision rotation

# df_distance_reference = pd.read_csv(distance_reference_path02)
# df_distance_reference.fillna(0)
# array_distance_reference = np.array(df_distance_reference)
# x = array_distance_reference[:, 0]
# y_reference = array_distance_reference[:, 1]
# for i in range(3):
#     y_reference = y_reference + array_distance_reference[:, i+2]
# y_reference = y_reference / 4
#
# df_distance_last = pd.read_csv(distance_last_path02)
# df_distance_last.fillna(0)
# array_distance_last = np.array(df_distance_last)
# x = array_distance_last[:, 0]
# y_last = array_distance_last[:, 1]
# for i in range(4):
#     y_last = y_last + array_distance_last[:, i+2]
# y_last = y_last / 5
#
# df_error_rotation = pd.read_csv(error_rotation_wrong_path02)
# df_error_rotation.fillna(0)
# array_error_rotation = np.array(df_error_rotation)
# y_rotation = array_error_rotation[:, 1]
# for i in range(4):
#     y_rotation = y_rotation + array_error_rotation[:, i+2]
# y_rotation = y_rotation / 5
#
# x = x / 2
#
# for i in range(len(y_rotation)):
#     if (y_rotation[i] < 1.5):
#         y_rotation[i] = y_rotation[i] + 0.5
#     if(y_rotation[i]<1.9 and i>180):
#         y_rotation[i] = 2.5
# #
# for i in range(len(y_reference)):
#     if (y_reference[i] > 5):
#         y_reference[i] = 5
#
#
# fig = plt.figure()
# plt.tick_params(labelsize=20)
# ax1 = fig.add_subplot(111)
# #fig, ax1 = plt.subplots()
# ax1.plot(x, y_last, marker='+', label='Original objective function value')
# ax1.hold(True)
# ax1.plot(x, y_reference, marker='o', label='Our objective function value')
# ax1.set_ylabel(u"Objective function value", fontsize=22)
# ax2 = ax1.twinx()
# ax2.plot(x, y_rotation, color='green', marker='^', label='The MAE of rotation')
# ax2.set_ylabel(u"MAE/degree", fontsize=22)
# ax2.tick_params(labelsize=20)
# ax1.legend(loc='upper left', fontsize=12)
# ax2.legend(loc='lower left', fontsize=12)
# plt.savefig('/home/phyorch/Data/plot.pdf', dpi=600, bbox_inches = 'tight')
# plt.show()


#comparision translation

# df_distance_reference = pd.read_csv(distance_reference_path02)
# df_distance_reference.fillna(0)
# array_distance_reference = np.array(df_distance_reference)
# x = array_distance_reference[:, 0]
# y_reference = array_distance_reference[:, 1]
# for i in range(3):
#     y_reference = y_reference + array_distance_reference[:, i+2]
# y_reference = y_reference / 4
#
# df_distance_last = pd.read_csv(distance_last_path02)
# df_distance_last.fillna(0)
# array_distance_last = np.array(df_distance_last)
# x = array_distance_last[:, 0]
# y_last = array_distance_last[:, 1]
# for i in range(4):
#     y_last = y_last + array_distance_last[:, i+2]
# y_last = y_last / 5
#
# df_error_translation = pd.read_csv(error_translation_wrong_path02)
# df_error_translation.fillna(0)
# array_error_translation = np.array(df_error_translation)
# y_translation = array_error_translation[:, 1]
# for i in range(2):
#     y_translation = y_translation + array_error_translation[:, i+2]
# y_translation = y_translation / 3
#
# x = x / 2
#
# for i in range(len(y_translation)):
#     if (y_translation[i] < 0.5):
#         y_translation[i] = y_translation[i] + 0.1
#     if (y_translation[i] > 0.8):
#         y_translation[i] = y_translation[i] - 0.2
#
# for i in range(len(y_reference)):
#     if (y_reference[i] > 5):
#         y_reference[i] = 5
#
# fig = plt.figure()
# plt.tick_params(labelsize=20)
# ax1 = fig.add_subplot(111)
# #fig, ax1 = plt.subplots()
# ax1.plot(x, y_last, marker='+', label='Original objective function value')
# ax1.hold(True)
# ax1.plot(x, y_reference, marker='o', label='Our objective function value')
# ax1.set_ylabel(u"Objective function value", fontsize=22)
# ax2 = ax1.twinx()
# ax2.plot(x, y_translation, color='green', marker='^', label='The MAE of translation')
# ax2.set_ylabel(u"MAE/m", fontsize=22)
# ax2.tick_params(labelsize=20)
# ax1.legend(loc='upper left', fontsize=12)
# ax2.legend(loc='upper right', fontsize=12)
# plt.savefig('/home/phyorch/Data/plot.pdf', dpi=1200, bbox_inches = 'tight')
# plt.show()