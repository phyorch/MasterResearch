import numpy as np
import cv2

def neighborDyeingElem(input_pos, input_val, size, output):
    for i in range(size[0]):
        for j in range(size[1]):
            output[int((input_pos[0]-size[0])/2 + i)][int((input_pos[1]-size[1])/2 + j)] = input_val

def neighborDyeing(input, size, output):
    for i in range(input.shape[0]):
        for j in range(input.shape[1]):
            if(input[i][j] > 0):
                input_pos = []
                input_pos.append(i)
                input_pos.append(j)
                input_val = input[i][j]
                neighborDyeingElem(input_pos, input_val, size, output)




depth_image = cv2.imread('/home/phyorch/Data/0.png', cv2.IMREAD_ANYDEPTH)
projected_depths = np.float32(depth_image / 256.0)
size = []
size.append(2)
size.append(6)
output = np.zeros(projected_depths.shape)
neighborDyeing(projected_depths, size, output)
cv2.imwrite('/home/phyorch/Data/test1.png', output)
a = 1