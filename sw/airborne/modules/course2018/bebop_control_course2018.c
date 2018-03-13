/*
 * Copyright (C) Arne Imbrechts
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/course2018/bebop_control_course2018.c"
 * @author Arne Imbrechts
 * This module will include some navigation functions that come in handy during the project of the MAV course 2018
 */

#include "modules/course2018/bebop_control_course2018.h"

#include "math/pprz_algebra_int.h"

#include "state.h"
#include "generated/flight_plan.h"

/**
 * Function moves a waypoint relative to drone's position and heading by providing the distance to travel to the front (x) and to the
 * right (y) of the drone. The z value does not matter since we are dealing with 2D, so just put it at zero.
 */
void placeRelative(uint8_t wp_id, struct Int32Vect3 relativeMove)
{
	struct Int32Vect3 relPos_wp_ned;

	// body frame <-> NED frame
	struct Int32Rmat *ned_to_body_rmat = stateGetNedToBodyRMat_i();
	int32_rmat_transp_vmult(&relPos_wp_ned, ned_to_body_rmat, &relativeMove);

	// relative positions of the waypoint wrt the drone in ned frame
	int32_t relPos_wp_n = relPos_wp_ned.x; // north pos
	int32_t relPos_wp_e = relPos_wp_ned.y; // east pos

	struct NedCoor_i *pos = stateGetPositionNed_i(); // current position of the drone in local ned frame

	// placing the waypoint relative to the drone and make the drone change heading towards waypoint
	waypoint_set_xy_i(wp_id, pos->y + relPos_wp_e, pos->x + relPos_wp_n);
	nav_set_heading_towards_waypoint(wp_id); //This makes the drone yaw to the new waypoint (optional, you can delete)
}



