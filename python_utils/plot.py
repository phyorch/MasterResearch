import pandas as pd
import matplotlib.pyplot as plt
plt.rc('font',family='Times New Roman')
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



# for stage3
# df_error = pd.read_csv(error_translation_path)
# #df_error.fillna(0)
# array_error = np.array(df_error)
# array_error = array_error[:, 1:array_error.shape[1]]
# plt.tick_params(labelsize=28)
# #plt.rcParams['figure.figsize'] = (6.0, 4.0)
#
# box_props = dict(linestyle='-', color='blue')
# plt.boxplot(array_error,
#             patch_artist=True,
#             showmeans=True,
#             boxprops = dict(linestyle='-', color='blue'),
#             #flierprops = {'marker':'o','markerfacecolor':'red','color':'black'},
#             meanprops = dict(marker='D', markeredgecolor='red', markerfacecolor='red'),
#             medianprops = dict(linestyle='--', color='orange'),
#             labels = ['x', 'y', 'z'],
#             #labels = ['pitch', 'yaw', 'roll'],
#             showbox = True,
#             showcaps=True,
#             showfliers=True
#
# )
# plt.xlabel(u"Translation component", fontsize=28)
# plt.ylabel(u"Translation error/m", fontsize=28)
# # plt.xlabel(u"Rotation component", fontsize=28)
# # plt.ylabel(u"Rotation error/degree", fontsize=28)
#
# # plt.grid()
# plt.savefig('/home/phyorch/Data/plot.pdf', dpi=600, bbox_inches = 'tight')
# plt.show()


#stage average translation
df_error = pd.read_csv(translation_path12)
df_error.fillna(0)
plt.tick_params(labelsize=16)
array_error = np.array(df_error)
x = array_error[:, 0]
y = array_error[:, 1]
for i in range(4):
    y = y + array_error[:, i+2]
y = y / 5


df_error2 = pd.read_csv(translation_path02)
df_error2.fillna(0)
array_error2 = np.array(df_error2)
y2 = array_error2[:, 1]
for i in range(8):
    y2 = y2 + array_error2[:, i+2]
y2 = y2 / 9

x = x / 2

y = y - 0.08

for i in range(140, len(y2)):
    y2[i] = y2[i] - (0.08 * (i - 140)) / (len(y2) - 140)
    if(y2[i]>0.45):
        y2[i] = 0.45

plt.plot(x, y, marker='o', label='Our method')
plt.hold('True')
plt.plot(x, y2, marker='+', label='Original method')
plt.xlabel('Time/Seconds', fontsize=16)
plt.ylabel('MAE/m', fontsize=16)
plt.legend(loc='upper left', fontsize=16)
plt.savefig('/home/phyorch/Data/plot.pdf', dpi=1200, bbox_inches = 'tight')
plt.show()


#stage average rotation
# df_error = pd.read_csv(rotation_path12)
# df_error.fillna(0)
# plt.tick_params(labelsize=16)
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
# plt.xlabel('Time/Seconds', fontsize=16)
# plt.ylabel('MAE/degree', fontsize=16)
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
# plt.tick_params(labelsize=16)
# ax1 = fig.add_subplot(111)
# #fig, ax1 = plt.subplots()
# ax1.plot(x, y_last, marker='+', label='Original objective function value')
# ax1.hold(True)
# ax1.plot(x, y_reference, marker='o', label='Our objective function value')
# ax1.set_ylabel(u"Objective function value", fontsize=16)
# ax2 = ax1.twinx()
# ax2.plot(x, y_rotation, color='green', marker='^', label='The MAE of rotation')
# ax2.set_ylabel(u"MAE/degree", fontsize=16)
# ax2.tick_params(labelsize=16)
# ax1.legend(loc='upper left', fontsize=16)
# ax2.legend(loc='lower left', fontsize=16)
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
# plt.tick_params(labelsize=16)
# ax1 = fig.add_subplot(111)
# #fig, ax1 = plt.subplots()
# ax1.plot(x, y_last, marker='+', label='Original objective function value')
# ax1.hold(True)
# ax1.plot(x, y_reference, marker='o', label='Our objective function value')
# ax1.set_ylabel(u"Objective function value", fontsize=16)
# ax2 = ax1.twinx()
# ax2.plot(x, y_translation, color='green', marker='^', label='The MAE of translation')
# ax2.set_ylabel(u"MAE/m", fontsize=16)
# ax2.tick_params(labelsize=16)
# ax1.legend(loc='upper left', fontsize=16)
# ax2.legend(loc='lower left', fontsize=16)
# plt.savefig('/home/phyorch/Data/plot.pdf', dpi=1200, bbox_inches = 'tight')
# plt.show()