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
from mpl_toolkits.mplot3d import Axes3D


plt.close("all")

########## Parameters to input ##############
percentage_height = 0.5
n_horizontal_portions = 3
focal_length_x = 1188 #Be sure to enter this in pixels-> ...photo def 3800x3188 
focal_length_y = 1203
focal_length = np.mean([focal_length_x, focal_length_y])
########## Import two consecutive images/converting gray scale ##############
img1 = cv2.imread('Optic_flow_2m.jpg')
grayscale1 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)
gray1 = np.float32(grayscale1)

img2 = cv2.imread('Optic_flow_1m80.jpg')
grayscale2 = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
gray2 = np.float32(grayscale2)

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
    
    width_segment = int(np.ceil(shape_image[1]/n_horizontal_portions))
    
 
    segment_coordinates_array = np.empty((0,2), float)
    print(type(segment_coordinates_array))
    for i in range(n_horizontal_portions):
        if i != n_horizontal_portions - 1:
            width = width_segment
            start_segment_x = i*width
            end_segment_x = (i+1)*width
        else:
            width = shape_image[1] - (n_horizontal_portions - 1)*width_segment
            start_segment_x = i*width_segment
            end_segment_x = (i+1)*width
                            
        segment_coordinates = np.array([start_segment_x, end_segment_x])
        segment_coordinates_array = np.vstack((segment_coordinates_array,segment_coordinates))
                
    return middle_section_image, segment_coordinates_array

middle_section_image1, segment_coordinates_array1 = portion_segment_image(percentage_height,n_horizontal_portions, grayscale1)
middle_section_image2, segment_coordinates_array2 = portion_segment_image(percentage_height,n_horizontal_portions, grayscale2)







def get_orb_matches_segments(middle_image_portion1, middle_image_portion2, segment_coordinates_array):
    
    n_segments = len(segment_coordinates_array)
    kp1_coordinates = np.empty((0,2), float)
    kp2_coordinates = np.empty((0,2), float)
    all_matches_list = []
    ratios_list = []

    for i in range(n_segments):
    
        start_segment_x = segment_coordinates_array[i,0]
        end_segment_x = segment_coordinates_array[i,1]
        segment1 = middle_image_portion1[:,int(start_segment_x):int(end_segment_x)]
        segment2 = middle_image_portion2[:,int(start_segment_x):int(end_segment_x)]
        # Initiate ORB detector
        orb = cv2.ORB_create()
        
        # find the keypoints and descriptors with SIFT
        kp1, des1 = orb.detectAndCompute(segment1,None) #watch out for the floats!
        kp2, des2 = orb.detectAndCompute(segment2,None)
        
        # create BFMatcher object
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        # Match descriptors.
        matches = bf.match(des1,des2)
        
        # Sort them in the order of their distance.
        matches = sorted(matches, key = lambda x:x.distance)


        

        # For each pair of points we have between both images
        # draw circles, then connect a line between them
        for mat in matches:
    
            # Get the matching keypoints for each of the images
            img1_idx = mat.queryIdx
            img2_idx = mat.trainIdx
    
            # x - columns
            # y - rows
            (x1,y1) = kp1[img1_idx].pt
            x1 = x1 + start_segment_x
            size_kp1 = kp1[img1_idx].size
            (x2,y2) = kp2[img2_idx].pt
            x2 = x2 + start_segment_x
            size_kp2 = kp2[img2_idx].size
            ratio = size_kp2/size_kp1
            if ratio > 1:
                kp1_coordinates = np.vstack((kp1_coordinates,np.array([x1,y1])))
                kp2_coordinates = np.vstack((kp2_coordinates,np.array([x2,y2])))
                all_matches_list.extend([mat])
                ratios_list.extend([ratio])
                
    return kp1, kp2, all_matches_list, ratios_list, kp1_coordinates, kp2_coordinates
        
kp1, kp2, all_matches_list, ratios_list, kp1_coordinates, kp2_coordinates = get_orb_matches_segments(middle_section_image1, middle_section_image2, segment_coordinates_array1)        

##################################################################################
##################################################################################
##################################################################################
##################################################################################
# Draw first 10 matches.
def drawMatches(img1, img2, matches, kp1_coordinates, kp2_coordinates):
    """
    My own implementation of cv2.drawMatches as OpenCV 2.4.9
    does not have this function available but it's supported in
    OpenCV 3.0.0

    This function takes in two images with their associated 
    keypoints, as well as a list of DMatch data structure (matches) 
    that contains which keypoints matched in which images.

    An image will be produced where a montage is shown with
    the first image followed by the second image beside it.

    Keypoints are delineated with circles, while lines are connected
    between matching keypoints.

    img1,img2 - Grayscale images
    kp1,kp2 - Detected list of keypoints through any of the OpenCV keypoint 
              detection algorithms
    matches - A list of matches of corresponding keypoints through any
              OpenCV keypoint matching algorithm
    """
    # Create a new output image that concatenates the two images together
    # (a.k.a) a montage
    rows1 = img1.shape[0]
    cols1 = img1.shape[1]
    rows2 = img2.shape[0]
    cols2 = img2.shape[1]

    # Create the output image
    # The rows of the output are the largest between the two images
    # and the columns are simply the sum of the two together
    # The intent is to make this a colour image, so make this 3 channels
    out = np.zeros((max([rows1,rows2]),cols1+cols2,3), dtype='uint8')

    # Place the first image to the left
    out[:rows1,:cols1] = np.dstack([img1, img1, img1])

    # Place the next image to the right of it
    out[:rows2,cols1:] = np.dstack([img2, img2, img2])

    kpx_1 = kp1_coordinates[:,0]
    kpx_2 = kp2_coordinates[:,0]
    kpy_1 = kp1_coordinates[:,1]
    kpy_2 = kp2_coordinates[:,1]

    # For each pair of points we have between both images
    # draw circles, then connect a line between them
    for i in range(len(matches)):


        # x - columns
        # y - rows
        x1 = kpx_1[i]
        y1 = kpy_1[i]
        x2 = kpx_2[i]
        y2 = kpy_2[i]


        # Draw a small circle at both co-ordinates
        # radius 4
        # colour blue
        # thickness = 1
        cv2.circle(out, (int(x1),int(y1)), 4, (255, 0, 0), 1)   
        cv2.circle(out, (int(x2)+cols1,int(y2)), 4, (255, 0, 0), 1)

        # Draw a line in between the two points
        # thickness = 1
        # colour blue
        cv2.line(out, (int(x1),int(y1)), (int(x2)+cols1,int(y2)), (255,0,0), 1)


    # Show the image
    cv2.imshow('Matched Features', out)
    cv2.waitKey(0)
    cv2.destroyWindow('Matched Features')

    # Also return the image if you'd like a copy
    return out , kpx_1, kpx_2, kpy_1, kpy_2

img3, kpx_1, kpx_2, kpy_1, kpy_2 = drawMatches(middle_section_image1, middle_section_image2, all_matches_list[:], kp1_coordinates, kp2_coordinates)

plt.imshow(img3),plt.show()


########## Distance to feature calculation ##############
# =============================================================================
# The function will produce a range map based on the optical flow data between 
# two consecutive images. It uses the detected corners in the images as optical 
# flow points. Assuming that the focus of expansion is in the middle of the
# image due to a straight path of the drone.
# =============================================================================
def distToFeaturesv2(kpx_1, kpx_2, kpy_1, kpy_2, img_width, img_height, focal_length, velocity = 1.0, d_frames = 0.2): #!!!! later distance between frames can be made a function of velocity and frame rate of the camera
    
    n_keypoints = len(kpx_1)             

    # transforming coordinate frame for corner location in the image
    kpx_1 = -1.0*np.array(kpx_1) + np.ceil(img_width/2)*np.ones(np.shape(kpx_1))
    kpy_1 = -1*np.array(kpy_1) + np.ceil(img_height/2)*np.ones(np.shape(kpx_1))
    kpx_2 = -1*np.array(kpx_2) + np.ceil(img_width/2)*np.ones(np.shape(kpx_1))
    kpy_2 = -1*np.array(kpy_2) + np.ceil(img_height/2)*np.ones(np.shape(kpx_1))
    a = kpx_1
    
    # time difference between frames
    dt = d_frames/velocity
    
    TTC_all_corners = []
    i = 0
    n_keypoints_changing = n_keypoints
    while n_keypoints_changing>0:
   
        x_corner1 = kpx_1[i] # current corner in image1
        y_corner1 = kpy_1[i]
        x_corner2 = kpx_2[i] # current corner in image2
        y_corner2 = kpy_2[i]
        
        l_corner1 = math.sqrt(x_corner1**2 + y_corner1**2) # distance of corner to center of image
        l_corner2 = math.sqrt(x_corner2**2 + y_corner2**2)
        
        phi1 = math.atan2(l_corner1, focal_length) # radians
        phi2 = math.atan2(l_corner2, focal_length)
        dphi = abs(phi2-phi1)
        phi_dot = dphi/dt

        
        try:
            # time to contact/passing calculations for the current corner            
            TTC = (math.cos(phi1)*math.sin(phi1))/phi_dot
            TTC_all_corners.append(TTC)  
            # update index
            i = i + 1
            n_keypoints_changing = n_keypoints_changing - 1
            
 
        except:
            # add the index of the 
            n_keypoints_changing = n_keypoints_changing - 1
            # Remove the elements that do not 
            kpx_1 = np.delete(kpx_1, i)
            kpy_1 = np.delete(kpy_1, i)
            kpx_2 = np.delete(kpx_2, i)
            kpy_2 = np.delete(kpy_2, i)
            pass

    # calculate the positions of the detected corners
    Z = velocity*np.array(TTC_all_corners) # optical axis of the camera frame (distance of corner)
    X = kpx_1*Z/focal_length # positive to the left of the camera
    Y = kpy_1*Z/focal_length # positive to the top of the camera

    kpx_1_new = kpx_1
    kpy_1_new = kpy_1

    return X, Y, Z, a,TTC_all_corners, kpx_1_new, kpy_1_new

shape_image = np.shape(middle_section_image1)
X, Y, Z, a,TTC_all_corners, kpx_1_new, kpy_1_new = distToFeaturesv2(kpx_1, kpx_2, kpy_1, kpy_2, shape_image[1], shape_image[0], focal_length, 0.5, 0.2)

# annotate



# finally output the result through pyplot in the current window
fig = plt.figure('Positions of features 3D')
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-5,5])
ax.set_ylim([-5,5])
ax.scatter(X, Y, Z)

fig = plt.figure('Positions of features 2D ')
ax = fig.add_subplot(111)
ax.set_xlim([-2,2])
ax.set_ylim([0,2])
ax.scatter(X, Z)


plt.figure()
for i in range(len(kpx_1_new)):
    x = int(kpx_1[i])
    y = int(kpy_1[i])
    z = np.round(float(Z[i]))
    fontface = cv2.FONT_HERSHEY_SIMPLEX
    fontscale = 1
    fontcolor = (255, 255, 255)
    cv2.putText(middle_section_image1,str(z),(x,y), fontface, fontscale, fontcolor)
plt.imshow(middle_section_image1, cmap='gray')
plt.show()

#
#plt.figure('Image2')
#plt.imshow(middle_section_image2, cmap='gray')
#plt.scatter(x = corners2[:,0],y = corners2[:,1], c='r', s=40)
#plt.show()

#Z = distToFeatures(corners1, corners2, shape_image[1], shape_image[0], focal_length)

