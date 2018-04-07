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


// Import Modules
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

// Set framerate
#ifndef VISUAL_SONAR_FPS
#define VISUAL_SONAR_FPS 0      ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(OPENCVDEMO_FPS)

// Enable text messages to be send to terminal
#define VISUAL_SONAR_VERBOSE TRUE


// Define macro to print within functions
#define PRINT(string,...) fprintf(stderr, "[visual_sonar->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if VISUAL_SONAR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// Setting to enable/disable camera
struct video_listener *listener = NULL;

// YCbCr Filter Settings
uint8_t color_lum_min = VISUAL_SONAR_MINY;
uint8_t color_lum_max = VISUAL_SONAR_MAXY;
uint8_t color_cb_min  = VISUAL_SONAR_MINCB;
uint8_t color_cb_max  = VISUAL_SONAR_MAXCB;
uint8_t color_cr_min  = VISUAL_SONAR_MINCR;
uint8_t color_cr_max  = VISUAL_SONAR_MAXCR;

//Initialize other settings
uint8_t square_width = VISUAL_SONAR_SQUARE_WIDTH; 	// Initialize the discrete step the visual looks forward in image pixels
float square_th = VISUAL_SONAR_TH;					// Define threshold for part of pixels that must be ground color in a visual block in order to be positive

// Create number of pixels to travel variable
uint16_t pix_to_go;		// Initialization of variable that contains the obstacle free pixels forward (Updated by vision function)
float m_to_go;			// Initialization of variable that contains the obstacle free meters forward (Updated by vision function)

// Variables used by opencv cpp file
uint16_t pitch_to_pix = PITCH_TO_PIX;									// Effect of pitch on screen pixel rotation
uint16_t basic_min_square_height = VISUAL_SONAR_MIN_HEIGHT;        		// Set Square height at screen horizon
uint16_t basic_max_square_height = VISUAL_SONAR_MAX_HEIGHT;				// Set Square height at bottom of the screen
uint16_t min_square_height = VISUAL_SONAR_MIN_HEIGHT;					// Actual Square height at top of the screen
uint16_t max_square_height = VISUAL_SONAR_MAX_HEIGHT;					// Actual Square height at bottom of the screen

// navigation settings
float incrementForAvoidance;				// Direction for scanning (-10 or 10 deg)
uint8_t safeToGoForwards = false;			// Variable returning true if m_to_go is above a set threshold
float best_distance = 0;      				// The furthest dstance to go in a scan loop will be saved
uint8_t safe_heading = false;				// True if a direction without obstacles is detected in a scan loop
uint8_t forward_heading = false;			// True if actual heading is equal to desired heading
uint8_t set_heading = false;
float dist2_goal;							// Distance to goal (m)
uint8_t status;								// Status of drone in flight plan (Defined as macro's in visual_sonar.h)
float ground_speed;							// Ground speed in (m/s)
uint8_t scan_direction;						// Left or right
uint8_t confidence_level;					// 1 if direction is found free of obstacles for a certain threshold distance
uint8_t first_look_around_loop = true;		// set to false when second loop of obstacle scanning starts
float square_height_factor = 1.15;			// Increase in min and max square_height during scanning
float safety_m_to_go = 0.;					// Safety factor for stop_obstacle function in meters
uint8_t go_goal_chance = 2;					// The 1/number chance to directly go to a goal during scanning


// Declaration of CPP openCV function
struct image_t *opencv_func(struct image_t *img); // Create struct for openCV function
struct image_t *opencv_func(struct image_t *img)
{
  if (img->type == IMAGE_YUV422) {
    // Call OpenCV (C++ from paparazzi C function)
	  opencv_YCbCr_filter((char *) img->buf, img->w, img->h);
  }
  return NULL;
}


// initialization of the opencv function and navigation parameters
void visual_sonar_init()
{
	listener = cv_add_to_device(&VISUAL_SONAR_CAMERA, opencv_func, VISUAL_SONAR_FPS); //Define camera in module xml
	choose_next_direction(); 	// Select direction to turn during first obstacle scan
	confidence_level = 0;		// Set confidence level to 0
	srand(time(NULL));			// Initialize randomizer
	status = STATUS_STANDBY;   	// Set status to Standby
}

// Periodic navigation function that runs modules if the drone has a predefined navigation status defined in the visual_sonar.h file
void visual_sonar_periodic()
{
	safeToGoForwards = m_to_go > 0.5;  	// Checks if no obstacles are present in forward direction

	switch(status){
	case STATUS_STANDBY :				// During start and landing
		break;
	case STATUS_STABALIZING : 			// Switched to in flight plan to in flight plan
		break;
	case STATUS_SET_SCAN_HEADING : 		// Selecting scan heading called once in flight plan.
		check_scan_heading(5); 			// check if heading is at the scan heading before changing status
		break;
	case STATUS_AT_GOAL :				// Start scanning until status is changed by look_around() function
		compute_ground_speed(); 		// Update ground speed
		look_around(); 					// Look around and set goal in furthest direction.
		break;
	case STATUS_SET_HEADING : 			// Setting heading called once in flight plan
		compute_dist2_to_goal(); 		// Calculate distance to goal needed (for small distances, the function is prevented from getting locked)
		check_goal_heading(5); 			// Check if the heading is set to goal before changing status to goal
		break;
	case STATUS_GO_GOAL :				// Move towards the goal waypoint until stop_obstacle() or check_at_goal() changes status
		compute_dist2_to_goal(); 		// Update distance to goal variable
		compute_ground_speed(); 		// Update ground speed
		check_at_goal(); 				// Check if status should be changed to AT_GOAL
		stop_obstacle(); 				// Check if it has to stop for obstacles
		break;
	case STATUS_STOP :					// Status triggered when pix_to_go == 0 by stop_obstacle() function
		compute_dist2_to_goal(); 		//Update distance to goal variable
		compute_ground_speed(); 		// Update ground speed (Used to overcome overshoot)
		check_at_goal(); 				// Check if the drone is at the goal before changing status
		break;
	default :
		break;
	}
}

//Strategic navigation functions called in flightplan and periodic function


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

// Check if the heading is set to the goal heading before going to goal.
void check_goal_heading(float heading_diff_limit)
{
	if(dist2_goal<0.3){          															//Needed to overcome lock
		status = STATUS_AT_GOAL;
	}

	struct FloatVect2 target = {WaypointX(WP_GOAL), WaypointY(WP_GOAL)}; 					// Convert goal x and y into 2D vector
	struct FloatVect2 pos_diff; 															// Initialize position difference vector
	VECT2_DIFF(pos_diff, target, *stateGetPositionEnu_f()); 								// Calculate relative position difference of goal in ENU reference frame.
	float heading_f = DegOfRad(atan2f(pos_diff.x, pos_diff.y)); 							// Convert goal heading to degrees
	float heading_actual = DegOfRad(ANGLE_FLOAT_OF_BFP(stateGetNedToBodyEulers_i()->psi)); 	// Convert actual heading to degrees

	float heading_diff = heading_f-heading_actual; 											// Calculate heading difference
	VERBOSE_PRINT("hdg diff = %f \n", heading_diff);

	// if statements to normalize heading to compensate for circular values
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

// Check if the drone has to stop for an obstacle or has to adjust it's forward position of the obstacle.
void stop_obstacle(void){
	// Hard stop if pixels to go <= 0
	if(pix_to_go <= 0){
		moveWaypointForward(WP_GOAL, 0.6*ground_speed); // Move waypoint close to drone (0.6*ground speed to overcome overshoot)
		status = STATUS_STOP;							// Trigger Stop status
		VERBOSE_PRINT("Stop!! \n");
	}

	// Soft stop if obstacle pops up closer than initially calculated
	else{
		if(m_to_go-safety_m_to_go <= dist2_goal){
			if(m_to_go > 1){
				moveWaypointForward(WP_GOAL, m_to_go-0.5);																	// Update wp position with 0.5m safety margin
				VERBOSE_PRINT("Replace goal %f meters forward in stead of %f meters forward \n", m_to_go-0.5, dist2_goal);
			}
			else{
				moveWaypointForward(WP_GOAL, m_to_go);																		// Update wp position
				VERBOSE_PRINT("Replace goal %f meters forward in stead of %f meters forward \n", m_to_go, dist2_goal);
			}
		}
	}
}

// Function to compute the ground speed based on the NED reference system
void compute_ground_speed(void){
	ground_speed = sqrtf(powf(stateGetSpeedNed_f()->x,2)+powf(stateGetSpeedNed_f()->y,2));
}


// Check if the drone is at the goal by checking speed and relative distance. Then the new scan heading is set.
void check_at_goal(void){
	if(dist2_goal<0.4 && ground_speed <0.2){
		choose_next_direction();
		status = STATUS_SET_SCAN_HEADING;
	}
}

// Function to look around for free flight paths and set the goal to the furthest one.
void look_around(void){
	// square height is increased the first look_around loop
	if(first_look_around_loop){
		min_square_height = basic_min_square_height * square_height_factor;
		max_square_height = basic_max_square_height * square_height_factor;
		first_look_around_loop = false;
	}
	// Look around for free trajectories
	if(ground_speed < 0.15){
		int r = rand()%go_goal_chance; // Chance 1 out of go_goal_change() that r == 0

		if(safeToGoForwards){
			safe_heading = true;
			VERBOSE_PRINT("pixels to go = %d \n", pix_to_go);
			VERBOSE_PRINT("meters to go = %f \n", m_to_go);
			// If free trajectory is identified and r==0 or previous trajectory was more than 3 meters (confidence level increased)
			if(r==0 || confidence_level >= 1){
				if(m_to_go >= best_distance){
					best_distance = m_to_go;
					moveWaypointForward(WP_GOAL, best_distance);
				}
				set_goal();
			}
			// If r!=0 but free trajectory
			else{
				// If trajectory is long, increase confidence level
				if(m_to_go > 3){
					confidence_level = confidence_level + 1;
				}
				// If new trajectory is longer than previous ones, update the Goal waypoint
				if(m_to_go > best_distance){
					best_distance = m_to_go;
					moveWaypointForward(WP_GOAL, best_distance);
					}
				increase_nav_heading(&nav_heading, incrementForAvoidance);
			}
		}
		// If a obstacle is identified in forward direction
		else{
			// If already other free trajectories are found and r==0
			if(r == 0 && safe_heading){
				set_goal();
			}
			// If no free trajectories are found yet
			else{
				increase_nav_heading(&nav_heading, incrementForAvoidance);
			}
		}
	}
}

// Function to initialize all variables for the next loop_around() loop before setting the goal
void set_goal(void){
	best_distance = 0;
	safe_heading = false;
	confidence_level = 0;
	first_look_around_loop = true;
	min_square_height = basic_min_square_height;
	max_square_height = basic_max_square_height;
	status = STATUS_SET_HEADING;
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

// Function to set the heading of the drone a offset of 45 to 90 deg when arriving at the goal
void set_scan_heading(void){
	int r = rand() % 91;
	if(scan_direction == GO_RIGHT){
		increase_nav_heading(&nav_heading, 45+r);
		VERBOSE_PRINT("set heading RIGHT to %d \n", nav_heading);
	} else {
		increase_nav_heading(&nav_heading, -45-r);
		VERBOSE_PRINT("set heading LEFT to %d \n", nav_heading);
	}
}

// Check if the heading is set to the scan heading before scanning around.
void check_scan_heading(float hdg_diff_limit){
	float heading_actual = DegOfRad(ANGLE_FLOAT_OF_BFP(stateGetNedToBodyEulers_i()->psi));

	float heading_diff = DegOfRad(ANGLE_FLOAT_OF_BFP(nav_heading))-heading_actual;
	VERBOSE_PRINT("hdg diff = %f \n", heading_diff);
		// Normalize ciruclar heading
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
