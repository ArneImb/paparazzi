# -*- coding: utf-8 -*-
"""
Created on Sun Mar  4 08:07:23 2018

@author: dennis
"""

import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons

def convert_BGR_YCbCr(img_bgr):
    #Transformation [Y, Cb, Cr]^T = A[B. G, R]^T+B
    A = np.matrix([[0.114,  0.587,  0.299],
                   [0.499,  -0.331, -0.169],
                   [0.0813, -0.418, 0.499]])
    B = np.transpose(np.matrix([0., 128., 128.]))
    
    h, w = img_bgr.shape[:2]
    img_YCbCr = np.zeros((h,w,3), dtype=np.uint8)
    
    for i in range(h):
        for j in range(w):
            YCbCr_vect = A*np.transpose(np.matrix(img_bgr[i,j,:]))+B
            img_YCbCr[i,j,:] = np.array(np.transpose(YCbCr_vect))
    return img_YCbCr

def YCbCr_mask(img_YCbCr):
    min_th = np.array([0, 0, 0])
    max_th = np.array([255,128,128])
    
    mask = cv2.inRange(img_YCbCr, min_th, max_th)
    return mask

def YCbCr_mask_interactive(img_YCbCr, minY=26, maxY=100, minCb=99, maxCb=131, minCr=111, maxCr=140):
    min_th = np.array([np.uint8(minY), np.uint8(minCb), np.uint8(minCr)])
    max_th = np.array([np.uint8(maxY), np.uint8(maxCb), np.uint8(maxCr)])
    
    mask = cv2.inRange(img_YCbCr, min_th, max_th)
    return mask    

img_bgr = cv2.imread('snap4.png')
img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB) 
img_YCbCr = convert_BGR_YCbCr(img_bgr)
#mask = YCbCr_mask(img_YCbCr)
mask = YCbCr_mask_interactive(img_YCbCr)
masked_rgb = cv2.bitwise_and(img_rgb,img_rgb,mask=mask)

#Plot figures and compare
fig, (rgb_ax, mask_ax, masked_ax) = plt.subplots(3,1)
fig.subplots_adjust(left=0.3)
show_rgb = rgb_ax.imshow(img_rgb)
show_mask = mask_ax.imshow(cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB))
show_masked = masked_ax.imshow(masked_rgb)

#Define slider classes
ax_minY = fig.add_axes([0.075, 0.95, 0.2, 0.02])
ax_maxY = fig.add_axes([0.075, 0.9, 0.2, 0.02])
ax_minCb = fig.add_axes([0.075, 0.85, 0.2, 0.02])
ax_maxCb = fig.add_axes([0.075, 0.8, 0.2, 0.02])
ax_minCr = fig.add_axes([0.075, 0.75, 0.2, 0.02])
ax_maxCr = fig.add_axes([0.075, 0.7, 0.2, 0.02])

#Create sliders
sminY = Slider(ax_minY, 'minY', 0, 255, valinit=26)
smaxY = Slider(ax_maxY, 'maxY', 0, 255, valinit=100) 
sminCb = Slider(ax_minCb, 'minCb', 0, 255, valinit=99)
smaxCb = Slider(ax_maxCb, 'maxCb', 0, 255, valinit=131) 
sminCr = Slider(ax_minCr, 'minCr', 0, 255, valinit=111)
smaxCr = Slider(ax_maxCr, 'maxCr', 0, 255, valinit=140) 

#Check for changes sliders
def update(val):
    mask = YCbCr_mask_interactive(img_YCbCr, sminY.val, smaxY.val, sminCb.val, smaxCb.val, sminCr.val, smaxCr.val)
    show_mask.set_data(cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB))
    masked_rgb = cv2.bitwise_and(img_rgb,img_rgb,mask=mask)
    show_masked.set_data(masked_rgb)

#Initialise
update(1)

sminY.on_changed(update)
smaxY.on_changed(update)
sminCb.on_changed(update)
smaxCb.on_changed(update)
sminCr.on_changed(update)
smaxCr.on_changed(update)

# Look in forward direction for obstacles
#First define a function to calculate the integral mask and area of white dots
def positives_integral_image(integral_img, left, right, up, down):
    positives = integral_img[up,left]+integral_img[down,right]-(integral_img[up,right]+integral_img[down,left])
    return positives

def pix_no_obstacle(mask, square_size, threshold=0.99): #Returns the number of pixels in the videoframe without obstacles
    h, w = mask.shape[:2]
    area = square_size**2
    mask = mask/255
    integral_mask = cv2.integral(mask)
    for up in np.arange(h-square_size,0,-square_size):
        down = up+square_size
        positives = positives_integral_image(integral_mask, (w-square_size)/2, (w+square_size)/2, up, down)
        if (np.float(positives)/np.float(area)) < threshold:
            break
    return h-down #pixels to cover safely

# Draw line
h, w = img_rgb.shape[:2]  
pix_to_go = pix_no_obstacle(mask, 50)    
show_rgb.set_data(cv2.line(img_rgb,(w/2,h),(w/2,h-pix_to_go),(255,0,0),15))
plt.show()

print 'Pixels to go: ', pix_to_go