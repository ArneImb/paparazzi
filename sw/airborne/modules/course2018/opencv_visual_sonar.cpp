/*
 * Copyright (C) Dennis van Wijngaarden
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/course2018/visual_sonar.c"
 * @author Dennis van Wijngaarden
 * Object detection, range and avoidance algorithm using green floor in cyberzoo
 */

#include "opencv_visual_sonar.h"
#include "opencv_visual_sonar.hpp"
#include "visual_sonar.h"
#include "state.h"
#include <math.h>

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "modules/computer_vision/opencv_image_functions.h"

float aggression = 0.8;
uint8_t screen_height = 245;
uint16_t min_square_height = 50;
uint16_t max_square_height = 150;
uint16_t pitch_to_pix = 100;

//Calculates the height of a pixel square
uint16_t func_square_height(uint16_t pos)
{
	if (pos==0){pos=1;}
	float theta = stateGetNedToBodyEulers_f()->theta;
	uint16_t square_height = (float)max_square_height-((float)max_square_height-(float)min_square_height)/((float)screen_height/2.)*((float)pos+(float)pitch_to_pix*theta);
	return square_height;
}

// Calculates the number of accessible pixels in a square using the integral image of a masked image
uint16_t number_positives_square(Mat integral_mask, uint16_t left, uint16_t right, uint16_t top, uint16_t bottom)
{
	uint16_t n_positives; 									//number of positive pixels in a square block
	n_positives = (integral_mask.at<uint32_t>(top,left)+integral_mask.at<uint32_t>(bottom,right)) -
				  (integral_mask.at<uint32_t>(top,right)+integral_mask.at<uint32_t>(bottom,left));   //(y,x)

	return n_positives;
}

//Decides to go left or right based on positive pixels
uint8_t left_or_right(Mat integral_img, int width, int height)
{
	uint16_t positives_left;
	uint16_t positives_right;
	positives_left = integral_img.at<uint32_t>((int)((float)height/2.),width);
	positives_right = integral_img.at<uint32_t>(height,width) - integral_img.at<uint32_t>((int)((float)height/2.),width);
	if (positives_right>=positives_left){preferred_dir=GO_RIGHT;}
	else {preferred_dir=GO_LEFT;}
	return preferred_dir;
}

//Calculates the number of pixels up to an obstacle by stepping forward in a masked image with pixel blocks
uint16_t pixels_to_go(Mat mask, uint8_t square_width = square_width, float threshold = square_th)
{
	//Define masked image properties
	int w = mask.size().width; 								//Width of mask in pixels
	int h = mask.size().height; 							//Height of mask in pixels
	uint16_t left_pos; 										//Left position of pixel block
	Mat bin_mask;											//Define binary mask
	Mat integral_mask; 										//Define integral image of mask
	cv::threshold(mask, bin_mask, 127, 1, THRESH_BINARY); 	//Set threshold of mask
	integral(bin_mask,integral_mask); 						//Define integral image of mask

	//Step forward and check if the number of accessible pixels stays above a certain threshold
	for(left_pos = 0; left_pos<=w+square_width; left_pos += square_width)
	{
		uint16_t square_height = func_square_height(left_pos);
		uint16_t square_area = square_width*square_height; 		//Area of pixel block
		uint16_t top_pos = (h-square_height)/2;					//Top position of pixel block
		uint16_t bottom_pos = (h+square_height)/2;				//Bottom position of pixel block
		uint16_t min_area = threshold*square_area;				//Minimal area of pixel block
		uint16_t right_pos = left_pos + square_width;
		uint16_t n_postives = number_positives_square(integral_mask, left_pos, right_pos, top_pos, bottom_pos);
		if(n_postives < min_area)
		{
			break;
		}
	}
	if(left_pos==0) return 0; else return left_pos - square_width;
}

//Converts accessible pixels to accessible distance
float pix_to_m(uint16_t pixels)
{
	float meters = 0.; //Initialize distance at zero, since the function is only valid for pixels>0
	if (pixels>0)
	{
		float theta = stateGetNedToBodyEulers_f()->theta;	//Pitch angle in radians
		meters = 0.03328*(pixels+pitch_to_pix*theta)+2.12956;	//Calculate distance
		meters *= aggression;
	}
	return meters;
}

//Generates YUV image from original bebop image
int opencv_YCbCr_filter(char *img, int width, int height)
{
	// Create a new image, using the original bebop image.
	Mat M(height, width, CV_8UC2, img); //YCrCb
	Mat M_RGB;
	Mat masked_RGB;

	//Convert UYUV in paparazzi to YUV opencv
	cvtColor(M, M_RGB, CV_YUV2RGB_Y422);
	cvtColor(M_RGB, M, CV_RGB2YUV);
	Mat mask(M.size(), CV_8UC1, Scalar(0));

	// Filter YCrCb values
	inRange(M,
			Scalar(color_lum_min, color_cb_min, color_cr_min),
			Scalar(color_lum_max, color_cb_max, color_cr_max),
			mask);

	//Calculate distance to go
	pix_to_go = pixels_to_go(mask);
	m_to_go = pix_to_m(pix_to_go);

	bitwise_and(M_RGB,M_RGB,masked_RGB,mask); //in, in, out (cooy to inimg frame)

	uint16_t disp_pos;
	for(disp_pos = 0; disp_pos<=pix_to_go; disp_pos += square_width)
	{
		uint16_t square_height = func_square_height(disp_pos);
		line(masked_RGB, Point(disp_pos,(height-square_height)/2), Point(disp_pos,(height+square_height)/2), Scalar(255,0,0),1);
	}

	cvtColor(masked_RGB, M, CV_RGB2YUV);
	//Convert back and save in original image position
	coloryuv_opencv_to_yuv422(M, img, width, height);

	return 0;
}

