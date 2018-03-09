#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Mar  9 11:04:08 2018

@author: isabelle
"""

## import two consecutive images

import cv2 
import numpy as np
import matplotlib.pyplot as plt

plt.close("all")

########## Parameters to input ##############
percentage_height = 0.3

img = cv2.imread('snap1.png')
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
gray = np.float32(gray)

rot_gray = np.rot90(gray)


percentage_height = 0.3
n_horizontal_portions = 3



def portion_segment_image(percentage_height,n_horizontal_portions, image):
    shape_image = np.shape(image)
    top_left_corner_y = int(np.ceil(0.5*(1- percentage_height)*shape_image[0]))    
    bottom_right_corner_y = int(np.ceil(0.5*(1 + percentage_height)*shape_image[0]))
    middle_section_image = image[top_left_corner_y:bottom_right_corner_y,:]
    
    all_corners = []
    
    width_segment = int(np.ceil(shape_image[1]/n_horizontal_portions))
    for i in range(n_horizontal_portions):
        if i != n_horizontal_portions - 1:
            width = width_segment
            start_segment_x = i*width
            end_segment_x = (i+1)*width
        else:
            width = shape_image[1] - (n_horizontal_portions - 1)*width_segment
            start_segment_x = i*width_segment
            end_segment_x = (i+1)*width
        segment = middle_section_image[:,start_segment_x:end_segment_x] #top_left_corner_y:bottom_right_corner_y
        plt.figure()
        plt.imshow(segment, cmap='gray')
        plt.show()
        # apply corner detector to segments
        maxCorners = 20
        outputs = cv2.goodFeaturesToTrack(segment, maxCorners, 0.0001, 10)
        segment_corners = []
        for j in range(maxCorners):
            try:        
#                print j
                xy = outputs[j,0,:]
                x_corner = xy[0] + i*width
                y_corner = xy[1] # since scatter is from bottom to top
                corner = [x_corner,y_corner]
                all_corners.append(np.array(corner))
                segment_corners.append(np.array(xy))
            except:
                pass
        segment_corners = np.array(segment_corners)
        plt.scatter(x = segment_corners[:,0],y = segment_corners[:,1], c='r', s=40)
        plt.show()
                
    return middle_section_image, all_corners


middle_section_image, corners = portion_segment_image(percentage_height,n_horizontal_portions, gray)


# finally output the result through pyplot in the current window
plt.figure()
plt.imshow(middle_section_image, cmap='gray')
plt.show()


all_corners = np.array(corners)
shape_corners = np.shape(all_corners)
plt.scatter(x = all_corners[:,0],y = all_corners[:,1], c='r', s=40)
plt.show()



