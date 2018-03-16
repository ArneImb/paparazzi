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
#define VISUAL_SONAR_MINY 70
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

#ifndef VISUAL_SONAR_SQUARE_HEIGHT_MAX
#define VISUAL_SONAR_SQUARE_HEIGHT_MAX 150
#endif

#ifndef VISUAL_SONAR_SQUARE_HEIGHT_MIN
#define VISUAL_SONAR_SQUARE_HEIGHT_MIN 50
#endif

#ifndef VISUAL_SONAR_SQUARE_WIDTH
#define VISUAL_SONAR_SQUARE_WIDTH 5
#endif

#ifndef VISUAL_SONAR_TH
#define VISUAL_SONAR_TH 0.95
#endif

#include <inttypes.h>
#include "math/pprz_geodetic_int.h"

extern void visual_sonar_init(void);
extern void visual_sonar_periodic(void);
extern uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
extern uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
extern uint8_t increase_nav_heading(int32_t *heading, float incrementDegrees);
extern uint8_t chooseRandomIncrementAvoidance(void);
// extern void visual_sonar_periodic();


extern uint8_t color_lum_min;
extern uint8_t color_lum_max;
extern uint8_t color_cb_min;
extern uint8_t color_cb_max;
extern uint8_t color_cr_min;
extern uint8_t color_cr_max;

extern uint16_t pix_to_go;

extern uint16_t square_height_min;
extern uint16_t square_height_max;
extern uint8_t square_width;
extern float square_th;
extern float m_to_go;
extern uint8_t at_goal;
extern float best_distance;
extern uint8_t static_running;

extern struct video_listener *listener;

#endif

