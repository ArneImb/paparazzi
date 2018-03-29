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
uint8_t safe_heading = false;
uint8_t forward_heading = false;
uint8_t set_heading = false;
uint8_t stabalized = false;
float dist2_goal;
uint8_t status;
float ground_speed;
uint8_t preferred_dir;
uint8_t scan_direction;
uint8_t confidence_level;

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


// define all navigation functions below
void visual_sonar_init()
{
	listener = cv_add_to_device(&VISUAL_SONAR_CAMERA, opencv_func, VISUAL_SONAR_FPS); //Define camera in module xml
	choose_next_direction();
	confidence_level = 0;
	srand(time(NULL));
	status = STATUS_STANDBY;
}

// Periodic navigation function that runs modules if the drone has a predefined navigation status defined in the visual_sonar.h file
void visual_sonar_periodic()
{
	safeToGoForwards = m_to_go > 0.5;

	switch(status){
	case STATUS_STANDBY :
		break;
	case STATUS_STABALIZING :
		break;
	case STATUS_SET_SCAN_HEADING : // Selecting scan heading called once in flight plan
		check_scan_heading(5);
		break;
	case STATUS_AT_GOAL :
		compute_ground_speed();
		look_around();
		break;
	case STATUS_SET_HEADING : // Setting heading called once in flight plan
		compute_dist2_to_goal();
		check_goal_heading(5);
		break;
	case STATUS_GO_GOAL :
		compute_dist2_to_goal(); //Update distance to goal variable
		compute_ground_speed();
		check_at_goal(); // Check if status should be changed to AT_GOAL
		stop_obstacle(); //Check if it has to stop for obstacles
		break;
	case STATUS_STOP :
		compute_dist2_to_goal(); //Update distance to goal variable
		compute_ground_speed();
		check_at_goal();
		break;
	}
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

// Function to point nose of drone towards the goal waypoint
void nav_set_heading_towards_goal(void)
{
  struct FloatVect2 target = {WaypointX(WP_GOAL), WaypointY(WP_GOAL)};
  struct FloatVect2 pos_diff;
  VECT2_DIFF(pos_diff, target, *stateGetPositionEnu_f());
  float heading_f = atan2f(pos_diff.x, pos_diff.y);
  nav_heading = ANGLE_BFP_OF_REAL(heading_f);
}

// Equation that computes the distance to the goal waypoint with respect to the position of the drone.
void compute_dist2_to_goal(void)
{
  dist2_goal =  sqrt(get_dist2_to_waypoint(WP_GOAL));
}

void check_goal_heading(float heading_diff_limit)
{
	if(dist2_goal<0.3){          //Needed to overcome lock
		status = STATUS_AT_GOAL;
	}

	struct FloatVect2 target = {WaypointX(WP_GOAL), WaypointY(WP_GOAL)};
	struct FloatVect2 pos_diff;
	VECT2_DIFF(pos_diff, target, *stateGetPositionEnu_f());
	float heading_f = DegOfRad(atan2f(pos_diff.x, pos_diff.y));
	float heading_actual = DegOfRad(ANGLE_FLOAT_OF_BFP(stateGetNedToBodyEulers_i()->psi));

	float heading_diff = heading_f-heading_actual;
	VERBOSE_PRINT("hdg diff = %f \n", heading_diff);

	if(heading_diff > 180){
		heading_diff = -1*(heading_diff-360);
	}

	if(heading_diff < -180){
		heading_diff = heading_diff+360;
	}

	if(heading_diff < 0){
		heading_diff = heading_diff*-1;
	}

	if(heading_diff < heading_diff_limit){
		status = STATUS_GO_GOAL;
	}
}

void stop_obstacle(void){
	if(pix_to_go <= 0){
		moveWaypointForward(WP_GOAL, 0.5*ground_speed);
		//moveWaypointForward(WP_ATGOAL, 0.25*ground_speed);
		//waypoint_set_here_2d(WP_GOAL);
		//waypoint_set_here_2d(WP_ATGOAL);
		//status = STATUS_STABALIZING;
		status = STATUS_STOP;
		VERBOSE_PRINT("Stop!! \n");
	}
	else{
		if(m_to_go <= dist2_goal){
			if(m_to_go > 1){
				moveWaypointForward(WP_GOAL, m_to_go-0.5);
				VERBOSE_PRINT("Replace goal %f meters forward in stead of %f meters forward \n", m_to_go-0.5, dist2_goal);
			}
			else{
				moveWaypointForward(WP_GOAL, m_to_go);
				VERBOSE_PRINT("Replace goal %f meters forward in stead of %f meters forward \n", m_to_go, dist2_goal);
			}
		}
	}
}

void compute_ground_speed(void){
	ground_speed = sqrtf(powf(stateGetSpeedNed_f()->x,2)+powf(stateGetSpeedNed_f()->y,2));
}

void check_at_goal(void){
	if(dist2_goal<0.3 && ground_speed <0.15){
		choose_next_direction();
		status = STATUS_SET_SCAN_HEADING;
	}
}

void look_around(void){
	if(ground_speed < 0.15){
		int r = rand()%5; //Change 1 out of 5 that r == 1

		if(safeToGoForwards){
			safe_heading = true;
			VERBOSE_PRINT("pixels to go = %d \n", pix_to_go);
			VERBOSE_PRINT("meters to go = %f \n", m_to_go);
			if(r==1 || confidence_level >= 1){
				if(m_to_go >= best_distance){
					best_distance = m_to_go;
					moveWaypointForward(WP_GOAL, best_distance);
				}
				best_distance = 0;
				safe_heading = false;
				confidence_level = 0;
				status = STATUS_SET_HEADING;
				//choose_next_direction();
			}
			else{
				if(m_to_go > 3){
					confidence_level = confidence_level + 1;
				}
				if(m_to_go > best_distance){
					best_distance = m_to_go;
					moveWaypointForward(WP_GOAL, best_distance);
					}
				increase_nav_heading(&nav_heading, incrementForAvoidance);
			}
		}

		else{
			if(r == 1 && safe_heading){
				best_distance = 0;
				safe_heading = false;
				confidence_level = 0;
				status = STATUS_SET_HEADING;
				//choose_next_direction();
			}
			else{
				increase_nav_heading(&nav_heading, incrementForAvoidance);
			}
		}
	}
}

// Function that initialises next turning direction at goal waypoint
void choose_next_direction(void){
	int r = rand() % 2;
	if(r==0){
		scan_direction = GO_RIGHT;
		incrementForAvoidance = 10.0;
	} else{
		scan_direction = GO_LEFT;
		incrementForAvoidance = -10.0;
	}
}

// Function to set the heading of the drone a offset of 90 to 180 deg when arriving at the goal
void set_scan_heading(void){
	int r = rand() % 91;
	if(scan_direction == GO_RIGHT){
		increase_nav_heading(&nav_heading, 90+r);
		VERBOSE_PRINT("set heading RIGHT to %d \n", nav_heading);
	} else {
		increase_nav_heading(&nav_heading, -90-r);
		VERBOSE_PRINT("set heading LEFT to %d \n", nav_heading);
	}
}

void check_scan_heading(float hdg_diff_limit){
	float heading_actual = DegOfRad(ANGLE_FLOAT_OF_BFP(stateGetNedToBodyEulers_i()->psi));

	float heading_diff = DegOfRad(ANGLE_FLOAT_OF_BFP(nav_heading))-heading_actual;
	VERBOSE_PRINT("hdg diff = %f \n", heading_diff);

		if(heading_diff > 180){
			heading_diff = -1*(heading_diff-360);
		}

		if(heading_diff < -180){
			heading_diff = heading_diff+360;
		}

		if(heading_diff < 0){
			heading_diff = heading_diff*-1;
		}

		if(heading_diff < hdg_diff_limit){
			status = STATUS_AT_GOAL;
		}
}
