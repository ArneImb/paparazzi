#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Mar  9 11:04:08 2018

@author: isabelle
"""

import cv2 
import numpy as np
import matplotlib.pyplot as plt
import math

plt.close("all")

########## Parameters to input ##############
percentage_height = 0.5
n_horizontal_portions = 3
focal_length = 1.0 #Be sure to enter this in pixels

########## Import two consecutive images/converting gray scale ##############
img1 = cv2.imread('Optic_flow_2m.jpg')
gray1 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)
gray1 = np.float32(gray1)

img2 = cv2.imread('Optic_flow_1m80.jpg')
gray2 = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
gray2 = np.float32(gray2)

# rot_gray = np.rot90(gray)
##### Rotation will be needed when implementing to work with bebop camera input frames, but not now

########## Corner detection function ##############
# =============================================================================
# Function will apply corner detection of opencv to different segments of a 
# middle portion of the image. 
# =============================================================================
def portion_segment_image(percentage_height,n_horizontal_portions, image):
    shape_image = np.shape(image)
    # segmenting middle of the image out
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
        outputs = cv2.goodFeaturesToTrack(segment, maxCorners, 0.01, 50)
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
        
    all_corners = np.array(all_corners)
                
    return middle_section_image, all_corners
#######################################################

########## Distance to feature calculation ##############
# =============================================================================
# The function will produce a range map based on the optical flow data between 
# two consecutive images. It uses the detected corners in the images as optical 
# flow points. Assuming that the focus of expansion is in the middle of the
# image due to a straight path of the drone.
# =============================================================================
def distToFeatures(corners1, corners2, img_width, img_height, focal_length, velocity = 1.0, d_frames = 0.2): #!!!! later distance between frames can be made a function of velocity and frame rate of the camera
    shape_corners1 = np.shape(corners1)
    shape_corners2 = np.shape(corners2)
    
    if shape_corners1[0] != shape_corners2[0]: # not the same amount of corners tracked in both images
        if shape_corners1[0] > shape_corners2[0]:
            corners1 = corners1[0:(shape_corners2[0]-1),:]
            shape_corners1 = np.shape(corners1)
        else:
            corners2 = corners2[0:(shape_corners1[0]-1),:]
            shape_corners2 = np.shape(corners2)
    else: pass
        
    # transforming coordinate frame for corner location in the image
    corners1[:,0] = -(corners1[:,0]) + np.ceil(img_width/2)
    corners1[:,1] = -(corners1[:,1]) + np.ceil(img_height/2)
    corners2[:,0] = -(corners2[:,0]) + np.ceil(img_width/2)
    corners2[:,1] = -(corners2[:,1]) + np.ceil(img_height/2)
    
    # time difference between frames
    dt = d_frames/velocity
    
    TTC_all_corners = []
    for i in range(shape_corners1[0]):
        x_corner1 = corners1[i,0] # current corner in image1
        y_corner1 = corners1[i,1]
        x_corner2 = corners2[i,0] # current corner in image2
        y_corner2 = corners2[i,1]
        
        l_corner1 = math.sqrt(x_corner1**2 + y_corner1**2) # distance of corner to center of image
        l_corner2 = math.sqrt(x_corner2**2 + y_corner2**2)
        
        phi1 = math.atan2(l_corner1, focal_length) # radians
        phi2 = math.atan2(l_corner2, focal_length)
        dphi = abs(phi2-phi1)
        phi_dot = dphi/dt
        
        # time to contact/passing calculations for the current corner
        TTC = (math.cos(phi1)*math.sin(phi1))/phi_dot
        TTC_all_corners.append(np.array([TTC]))
        
    TTC_all_corners = np.array(TTC_all_corners)
    
    # calculate the positions of the detected corners
    Z = velocity*TTC_all_corners # optical axis of the camera frame (distance of corner)
    X = np.reshape(corners1[:,0], (60,1))*Z/focal_length # positive to the left of the camera
    Y = np.reshape(corners1[:,1], (60,1))*Z/focal_length # positive to the top of the camera
    
    return 

middle_section_image1, corners1 = portion_segment_image(percentage_height,n_horizontal_portions, gray1)
middle_section_image2, corners2 = portion_segment_image(percentage_height,n_horizontal_portions, gray2)
shape_image = np.shape(middle_section_image1)

# finally output the result through pyplot in the current window
plt.figure('Image1')
plt.imshow(middle_section_image1, cmap='gray')
plt.scatter(x = corners1[:,0],y = corners1[:,1], c='r', s=40)
plt.show()

plt.figure('Image2')
plt.imshow(middle_section_image2, cmap='gray')
plt.scatter(x = corners2[:,0],y = corners2[:,1], c='r', s=40)
plt.show()

distToFeatures(corners1, corners2, shape_image[1], shape_image[0], focal_length)


