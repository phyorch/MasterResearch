import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

from sklearn.cluster import DBSCAN
from mpl_toolkits.mplot3d import Axes3D

translations_path = '/home/phyorch/Data/candidates.csv'

df_trans = pd.read_csv(translations_path)
array_trans = np.array(df_trans)
plt.ion()

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
# ax.set_xlabel('tx/m')
# ax.set_ylabel('ty/m')
# ax.set_zlabel('tz/m')
ax.set_xlabel('rx')
ax.set_ylabel('ry')
ax.set_zlabel('rz')
array = array_trans[:, 0]



dbscan = DBSCAN(eps=0.5, min_samples=3).fit(array_trans)
ax.scatter(array_trans[:, 0], array_trans[:, 1], array_trans[:, 2], s=200, c=dbscan.labels_, marker=".")
plt.show()
fig.savefig('/home/phyorch/Data/plot.pdf', dpi=1200, bbox_inches = 'tight')

labels = dbscan.labels_
translation_final = np.zeros(3, dtype=float)
cnt = 0
for i in range(array_trans.shape[0]):
    if(labels[i]==0):
        cnt = cnt + 1
        translation_final[0] = translation_final[0] + array_trans[i][0]
        translation_final[1] = translation_final[1] + array_trans[i][1]
        translation_final[2] = translation_final[2] + array_trans[i][2]
translation_final = translation_final / cnt #array_trans.shape[0]
a = 1

