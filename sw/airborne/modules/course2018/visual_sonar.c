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
 * Obstacle avoid and detect module using a single camera. The module uses opencv functions.
 */

#include "modules/course2018/visual_sonar.h"
#include "modules/computer_vision/cv.h"
#include "modules/course2018/opencv_visual_sonar.h"

#ifndef VISUAL_SONAR_FPS
#define VISUAL_SONAR_FPS 0      ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(OPENCVDEMO_FPS)

struct video_listener *listener = NULL;

// Filter Settings
uint8_t color_lum_min = VISUAL_SONAR_MINY;
uint8_t color_lum_max = VISUAL_SONAR_MAXY;
uint8_t color_cb_min  = VISUAL_SONAR_MINCB;
uint8_t color_cb_max  = VISUAL_SONAR_MAXCB;
uint8_t color_cr_min  = VISUAL_SONAR_MINCR;
uint8_t color_cr_max  = VISUAL_SONAR_MAXCR;

// Function
struct image_t *opencv_func(struct image_t *img);
struct image_t *opencv_func(struct image_t *img)
{
  if (img->type == IMAGE_YUV422) {
    // Call OpenCV (C++ from paparazzi C function)
	  opencv_YCbCr_filter((char *) img->buf, img->w, img->h);
  }
  return NULL;
}

void visual_sonar_init()
{
	listener = cv_add_to_device(&VISUAL_SONAR_CAMERA, opencv_func, VISUAL_SONAR_FPS); //Define camera in module xml
}

// void visual_sonar_periodic() {}


