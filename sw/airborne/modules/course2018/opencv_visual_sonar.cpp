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
 * Object detection, range and avoidance algoritm using green floor in cyberzoo
 */

#include "opencv_visual_sonar.h"
#include "opencv_visual_sonar.hpp"
#include "visual_sonar.h"

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "modules/computer_vision/opencv_image_functions.h"

uint16_t number_positives_square(Mat integral_img, uint16_t left, uint16_t right, uint16_t top, uint16_t bottom)
{
	uint16_t n_positives;
	n_positives = (integral_img.at<uint32_t>(top,left)+integral_img.at<uint32_t>(bottom,right)) -
				  (integral_img.at<uint32_t>(top,right)+integral_img.at<uint32_t>(bottom,left));   //(y,x)

	return n_positives;
}

uint16_t pixels_to_go(Mat mask, uint8_t square_width = squarewidth, uint8_t square_height = squareheight, float threshold = square_th)
{
	int w = mask.size().width;
	int h = mask.size().height;
	uint16_t left_pos;
	uint16_t square_area = square_width*square_height;
	Mat bin_mask;
	Mat integral_mask;
	cv::threshold(mask, bin_mask, 127, 1, THRESH_BINARY);
	integral(bin_mask,integral_mask);

	uint16_t top_pos = (h-square_height)/2;
	uint16_t bottom_pos = (h+square_height)/2;
	uint16_t min_area = threshold*square_area;
	for(left_pos = 0; left_pos<=w+square_width; left_pos += square_width)
	{
		uint16_t right_pos = left_pos + square_width;
		uint16_t n_postives = number_positives_square(integral_mask, left_pos, right_pos, top_pos, bottom_pos);
		if(n_postives < min_area)
		{
			break;
		}
	}
	if(left_pos==0) return 0; else return left_pos - square_width;
}

float pix_to_m(uint16_t pixels)
{
	float meters = 0.03328*pixels+2.12956;
	return meters;
}

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

	//line(M, Point(0,width/2), Point(height,width/2),Scalar(255,255,255), 20);
	//circle(M,Point(height/2,width/2), 50, Scalar(255,255,255), 10);
	// Filter YCrCb values
	inRange(M,
			Scalar(color_lum_min, color_cb_min, color_cr_min),
			Scalar(color_lum_max, color_cb_max, color_cr_max),
			mask);

	pix_to_go = pixels_to_go(mask);
	m_to_go = pix_to_m(pix_to_go);

	//cvtColor(mask, mask, CV_GRAY2RGB);
	//cvtColor(M, M, CV_YUV2RGB);
	//cvtColor(M, M, CV_RGB2GRAY);

	//mask.copyTo(M);
	bitwise_and(M_RGB,M_RGB,masked_RGB,mask); //in, in, out (cooy to inimg frame)

	rectangle(masked_RGB, Point(0,(height+squareheight)/2), Point(pix_to_go,(height-squareheight)/2), Scalar(255,0,0),2);

	cvtColor(masked_RGB, M, CV_RGB2YUV);
	//Convert back and save in original image position
	coloryuv_opencv_to_yuv422(M, img, width, height);
	//coloryuv_opencv_to_yuv422(mask, img, width, height);

	return 0;
}

