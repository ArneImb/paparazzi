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

#include "visual_sonar_tuner.h"

#include "modules/computer_vision/colorfilter.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "state.h"
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define VISUAL_SONAR_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[visual_sonar->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if VISUAL_SONAR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

#ifndef VISUAL_SONAR_LUM_MIN
#define VISUAL_SONAR_LUM_MIN 80
#endif

#ifndef VISUAL_SONAR_LUM_MAX
#define VISUAL_SONAR_LUM_MAX 101
#endif

#ifndef VISUAL_SONAR_CB_MIN
#define VISUAL_SONAR_CB_MIN 80
#endif

#ifndef VISUAL_SONAR_CB_MAX
#define VISUAL_SONAR_CB_MAX 90
#endif

#ifndef VISUAL_SONAR_CR_MIN
#define VISUAL_SONAR_CR_MIN 121
#endif

#ifndef VISUAL_SONAR_CR_MAX
#define VISUAL_SONAR_CR_MAX 125
#endif

void visual_sonar_init()
{
	// Initialise the variables of the colorfilter to accept orange
	color_lum_min = VISUAL_SONAR_LUM_MIN;
	color_lum_max = VISUAL_SONAR_LUM_MAX;
	color_cb_min  = VISUAL_SONAR_CB_MIN;
	color_cb_max  = VISUAL_SONAR_CB_MAX;
	color_cr_min  = VISUAL_SONAR_CR_MIN;
	color_cr_max  = VISUAL_SONAR_CR_MAX;
	// Initialise random values
	srand(time(NULL));
}

// void visual_sonar_periodic {}


