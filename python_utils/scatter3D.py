import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

from sklearn.cluster import DBSCAN
from mpl_toolkits.mplot3d import Axes3D

translations_path = '/home/phyorch/Data/translations.csv'

df_trans = pd.read_csv(translations_path)
array_trans = np.array(df_trans)
plt.ion()

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
array = array_trans[:, 0]

dbscan = DBSCAN(eps=0.5, min_samples=5).fit(array_trans)
ax.scatter(array_trans[:, 0], array_trans[:, 1], array_trans[:, 2], s=200, c=dbscan.labels_, marker=".")
plt.show()

labels = dbscan.labels_
translation_final = np.zeros(3, dtype=float)
for i in range(array_trans.shape[0]):
    if(labels[i]==0):
        translation_final[0] = translation_final[0] + array_trans[i][0]
        translation_final[1] = translation_final[1] + array_trans[i][1]
        translation_final[2] = translation_final[2] + array_trans[i][2]
translation_final = translation_final / array_trans.shape[0]
a = 1

