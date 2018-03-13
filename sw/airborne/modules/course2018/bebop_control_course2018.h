/*
 * Copyright (C) Arne Imbrechts
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/course2018/bebop_control_course2018.h"
 * @author Arne Imbrechts
 * This module will include some navigation functions that come in handy during the project of the MAV course 2018
 */

#ifndef BEBOP_CONTROL_COURSE2018_H
#define BEBOP_CONTROL_COURSE2018_H

#include "std.h"
#include <stdint.h>

extern void placeRelative(uint8_t wp_id, struct Int32Vect3 relativeMove);

#endif

