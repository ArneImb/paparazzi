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
 * Obstacle avoid and detect module using a single camera. The module uses opencv functions.
 */

#ifndef VISUAL_SONAR_H
#define VISUAL_SONAR_H

#ifndef VISUAL_SONAR_MINY
#define VISUAL_SONAR_MINY 65
#endif

#ifndef VISUAL_SONAR_MAXY
#define VISUAL_SONAR_MAXY 117
#endif

#ifndef VISUAL_SONAR_MINCB
#define VISUAL_SONAR_MINCB 111
#endif

#ifndef VISUAL_SONAR_MAXCB
#define VISUAL_SONAR_MAXCB 132
#endif

#ifndef VISUAL_SONAR_MINCR
#define VISUAL_SONAR_MINCR 124
#endif

#ifndef VISUAL_SONAR_MAXCR
#define VISUAL_SONAR_MAXCR 138
#endif

#ifndef VISUAL_SONAR_SQUARE_WIDTH
#define VISUAL_SONAR_SQUARE_WIDTH 5
#endif

#ifndef VISUAL_SONAR_TH
#define VISUAL_SONAR_TH 0.95
#endif

#ifndef STATUS_STANDBY
#define STATUS_STANDBY 0
#endif

#ifndef STATUS_STABALIZING
#define STATUS_STABALIZING 1
#endif

#ifndef STATUS_SET_SCAN_HEADING
#define STATUS_SET_SCAN_HEADING 2
#endif

#ifndef STATUS_AT_GOAL
#define STATUS_AT_GOAL 3
#endif

#ifndef STATUS_SET_HEADING
#define STATUS_SET_HEADING 4
#endif

#ifndef STATUS_GO_GOAL
#define STATUS_GO_GOAL 5
#endif

#ifndef STATUS_STOP
#define STATUS_STOP 6
#endif

#ifndef GO_RIGHT
#define GO_RIGHT 0
#endif

#ifndef GO_LEFT
#define GO_LEFT 1
#endif

#ifndef GO_RANDOM
#define GO_RANDOM 2
#endif

#ifndef PITCH_TO_PIX
#define PITCH_TO_PIX 25
#endif

#ifndef VISUAL_SONAR_MAX_HEIGHT
#define VISUAL_SONAR_MAX_HEIGHT 170
#endif

#ifndef VISUAL_SONAR_MIN_HEIGHT
#define VISUAL_SONAR_MIN_HEIGHT 45
#endif

#include <inttypes.h>
#include "math/pprz_geodetic_int.h"

extern void visual_sonar_init(void);
extern void visual_sonar_periodic(void);
extern uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
extern uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
extern uint8_t increase_nav_heading(int32_t *heading, float incrementDegrees);
extern void nav_set_heading_towards_goal(void);
extern void compute_dist2_to_goal(void);
extern void check_goal_heading(float heading_diff_limit);
extern void stop_obstacle(void);
extern void compute_ground_speed(void);
extern void check_at_goal(void);
extern void look_around(void);
extern void choose_next_direction(void);
extern void set_scan_heading(void);
extern void check_scan_heading(float hdg_diff_limit);
extern void set_goal(void);

extern uint8_t color_lum_min;
extern uint8_t color_lum_max;
extern uint8_t color_cb_min;
extern uint8_t color_cb_max;
extern uint8_t color_cr_min;
extern uint8_t color_cr_max;

extern uint16_t pix_to_go;

extern uint16_t pitch_to_pix;
extern uint16_t basic_min_square_height;
extern uint16_t basic_max_square_height;
extern uint16_t min_square_height;
extern uint16_t max_square_height;

extern uint8_t square_width;
extern float square_th;
extern float m_to_go;
extern float best_distance;
extern uint8_t forward_heading;
extern uint8_t set_heading;
extern float dist2_goal;
extern uint8_t status;
extern float ground_speed;
extern uint8_t scan_direction;
extern uint8_t confidence_level;
extern uint8_t first_look_around_loop;
extern float square_height_factor;
extern uint8_t go_goal_chance;

extern float safety_m_to_go;

extern struct video_listener *listener;

#endif

