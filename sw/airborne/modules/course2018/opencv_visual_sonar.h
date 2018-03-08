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
 * @file "modules/course2018/visual_sonar.h"
 * @author Dennis van Wijngaarden
 * Object detection, range and avoidance algoritm using green floor in cyberzoo
 */



#ifndef OPENCV_VISUAL_SONAR_H
#define OPENCV_VISUAL_SONAR_H

#include <stdint.h>

extern float square_th;

#ifdef __cplusplus
extern "C" {
#endif


//extern uint16_t number_positives_square(Mat integral_img, uint16_t left, uint16_t right, uint16_t top, uint16_t bottom);
extern int opencv_YCbCr_filter(char *img, int width, int height);

#ifdef __cplusplus
}
#endif

#endif
