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
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "state.h"
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef VISUAL_SONAR_FPS
#define VISUAL_SONAR_FPS 0      ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(OPENCVDEMO_FPS)

#define VISUAL_SONAR_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[visual_sonar->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if VISUAL_SONAR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

struct video_listener *listener = NULL;

// Filter Settings
uint8_t color_lum_min = VISUAL_SONAR_MINY;
uint8_t color_lum_max = VISUAL_SONAR_MAXY;
uint8_t color_cb_min  = VISUAL_SONAR_MINCB;
uint8_t color_cb_max  = VISUAL_SONAR_MAXCB;
uint8_t color_cr_min  = VISUAL_SONAR_MINCR;
uint8_t color_cr_max  = VISUAL_SONAR_MAXCR;

//Initialize other settings
uint16_t square_height_min = VISUAL_SONAR_SQUARE_HEIGHT_MIN;
uint16_t square_height_max = VISUAL_SONAR_SQUARE_HEIGHT_MAX;
uint8_t square_width = VISUAL_SONAR_SQUARE_WIDTH;
float square_th = VISUAL_SONAR_TH;

// Create number of pixels to travel variable
uint16_t pix_to_go;
float m_to_go;

// navigation settings
float incrementForAvoidance;
uint8_t safeToGoForwards = false;
uint8_t at_goal = false;
float best_distance = 0;
uint8_t static_running = false;

// Function
struct image_t *opencv_func(struct image_t *img);
struct image_t *opencv_func(struct image_t *img)
{
  if (img->type == IMAGE_YUV422) {
    // Call OpenCV (C++ from paparazzi C function)
	  opencv_YCbCr_filter((char *) img->buf, img->w, img->h);
	  VERBOSE_PRINT("pixels to go = %d \n", pix_to_go);
	  VERBOSE_PRINT("meters to go = %f \n", m_to_go);
  }
  return NULL;
}


// define all navigation functions below
void visual_sonar_init()
{
	listener = cv_add_to_device(&VISUAL_SONAR_CAMERA, opencv_func, VISUAL_SONAR_FPS); //Define camera in module xml
	chooseRandomIncrementAvoidance();
	srand(time(NULL));
}

void visual_sonar_periodic()
{
	// Check the amount of orange. If this is above a threshold
	// you want to turn a certain amount of degrees
	safeToGoForwards = m_to_go > 0.5;
	//VERBOSE_PRINT("Pixel count threshold: %d safe: %d \n", color_count, tresholdColorCount, safeToGoForwards);
	if(!at_goal && static_running){
		if(pix_to_go==0){
			waypoint_set_here_2d(WP_ATGOAL);
			waypoint_set_here_2d(WP_GOAL);
			at_goal = true;
		}
	}
	if(at_goal && (sqrt(pow(stateGetSpeedNed_f()->x,2)+pow(stateGetSpeedNed_f()->y,2))) <0.15){
		if(safeToGoForwards)
		{
			int r = rand()%10;
			if(r!=1){
				if(m_to_go > best_distance){
					best_distance = m_to_go;
					moveWaypointForward(WP_GOAL, best_distance);
					increase_nav_heading(&nav_heading, incrementForAvoidance);
				}
			}
			else{
				if(m_to_go >= best_distance){
					best_distance = m_to_go;
					moveWaypointForward(WP_GOAL, best_distance);
				}
				nav_set_heading_towards_waypoint(WP_GOAL);
				chooseRandomIncrementAvoidance();
				best_distance = 0;
				at_goal = false;
				}
			}
		else
		{
			increase_nav_heading(&nav_heading, incrementForAvoidance);
		}
	}
	return;
}

//Strategic navigation functions called in flightplan


/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(int32_t *heading, float incrementDegrees)
{
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  int32_t newHeading = eulerAngles->psi + ANGLE_BFP_OF_REAL(RadOfDeg(incrementDegrees));
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(newHeading); // HEADING HAS INT32_ANGLE_FRAC....
  *heading = newHeading;
  VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(ANGLE_FLOAT_OF_BFP(*heading)));
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  struct EnuCoor_i *pos             = stateGetPositionEnu_i(); // Get your current position
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  // Calculate the sine and cosine of the heading the drone is keeping
  float sin_heading                 = sinf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
  float cos_heading                 = cosf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
  // Now determine where to place the waypoint you want to go to
  new_coor->x                       = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
  new_coor->y                       = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,
                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y), POS_FLOAT_OF_BFP(pos->x), POS_FLOAT_OF_BFP(pos->y),
                DegOfRad(ANGLE_FLOAT_OF_BFP(eulerAngles->psi)) );
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}


uint8_t chooseRandomIncrementAvoidance()
{
  // Randomly choose CW or CCW avoiding direction
  int r = rand() % 2;
  if (r == 0) {
    incrementForAvoidance = 10.0;
    //VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidance);
  } else {
    incrementForAvoidance = -10.0;
    //VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidance);
  }
  return false;
}


