/*
 * Copyright (C) Shuo Li
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
 * @file "modules/flight_plan_in_guided_mode/flight_plan_in_guided_mode.c"
 * @author Shuo Li
 * This module is used to generate flight plan in guided mode
 */

#include <stdio.h>
//#include <std.h>
#include "flight_plan_in_guided_mode.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_in_guided_mode.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "state.h"
#include "modules/guidance_loop_velocity_autonomous_race/guidance_loop_velocity_autonomous_race.h"
#include <math.h>
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
//#include "firmwares/rotorcraft/stabilization/stabilization_attitude_euler_float.h"


float psi0;//
float psi_omega;

bool primitive_mask[5] = {0,0,0,0,0}; // 0 take off;1 hover;2 go straight;3 change_heading_hover;
// 5 circle
uint8_t previous_mode;
uint8_t current_mode;
uint8_t previous_guidance_h_mode;
uint8_t current_guidance_h_mode;

void flight_plan_in_guided_mode_init() {
    previous_mode = autopilot_mode;
    current_mode = autopilot_mode;
    previous_guidance_h_mode = guidance_h.mode;
    current_guidance_h_mode = guidance_h.mode;
    set_bit_ls(primitive_mask,1);
}


void display_information()
{
    if (autopilot_mode == AP_MODE_MODULE) {
        if(bit_is_set_ls(primitive_mask,0))
            printf("It is in take of mode\n");
        else if(bit_is_set_ls(primitive_mask,1))
            printf("It is in hover mode\n");
        else if (bit_is_set_ls(primitive_mask,2))
            printf("It is in go straight mode\n");
        else if (bit_is_set_ls(primitive_mask,3))
            printf("It is in change_heading_hover mode\n");

        //if (bit_is_set_ls(clock_mask,2))
            printf("Time in primitive is %f\n",time_primitive);

        printf("Altitude now is %f !\n",stateGetPositionNed_f()->z);
        printf("Z setpoint is %f !\n",(float)guidance_v_z_sp*pow(2,-INT32_POS_FRAC));
        printf("Z reference is %f \n", (float)guidance_v_z_ref*pow(2,-INT32_POS_FRAC));
        printf("\n");
        printf("\n");
        printf("\n");
    }
}

void take_off(float altitude);
void hover(float planned_time);
void go_straight(float planned_time, float velocity);
void change_heading_hover(float derta_psi,float planned_time);
void circle(float radius, float planned_time);

void take_off(float altitude){
    if (!bit_is_set_ls(clock_mask,2))
    {
        set_bit_ls(clock_mask,2);
        counter_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        guidance_loop_set_velocity(0,0);
        guidance_v_set_guided_z(altitude);
       // guidance_loop_set_heading(stateGetNedToBodyEulers_f()->psi);
        guidance_loop_set_heading(0);
    }

    else if(fabs(altitude-stateGetPositionNed_f()->z)<0.3)
    {
        clear_bit_ls(primitive_mask,0);
        set_bit_ls(primitive_mask,1);
        clear_bit_ls(clock_mask,2);
    }

}

void hover(float planned_time)
{
    if (!bit_is_set_ls(clock_mask,2))
    {
        set_bit_ls(clock_mask,2);
        counter_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        guidance_loop_set_velocity(0,0);
        guidance_v_set_guided_z(stateGetPositionNed_f()->z);
        guidance_loop_set_heading(stateGetNedToBodyEulers_f()->psi);
    }
    if(time_primitive > planned_time)
    {
        clear_bit_ls(primitive_mask,1);
        set_bit_ls(primitive_mask,2);
        clear_bit_ls(clock_mask,2);
    }
}

void go_straight(float planned_time, float velocity){
    if (!bit_is_set_ls(clock_mask,2)){
        set_bit_ls(clock_mask,2);
        counter_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        float psi = stateGetNedToBodyEulers_f()->psi;
        float vx_earth = cosf(psi)*velocity;
        float vy_earth = sinf(psi)*velocity;
        guidance_loop_set_velocity(vx_earth,vy_earth);   // earth coordinate
        guidance_v_set_guided_z(stateGetPositionNed_f()->z);
    }
    if(time_primitive > planned_time)
    {
        clear_bit_ls(primitive_mask,2);
        set_bit_ls(primitive_mask,1);
        clear_bit_ls(clock_mask,2);
    }
}

void change_heading_hover(float derta_psi,float planned_time){
    psi_omega = derta_psi/planned_time;
    if (!bit_is_set_ls(clock_mask,2))
    {
        set_bit_ls(clock_mask,2);
        counter_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        psi0 = stateGetNedToBodyEulers_f()->psi;
        psi_omega = derta_psi/planned_time;
        guidance_loop_set_velocity(0,0);
        guidance_v_set_guided_z(stateGetPositionNed_f()->z);
        guidance_loop_set_heading(psi0);
    }
    else
    {
        guidance_loop_set_velocity(0,0);
        guidance_v_set_guided_z(stateGetPositionNed_f()->z);
        guidance_loop_set_heading(psi0+psi_omega*time_primitive);
    }
    if(time_primitive > planned_time)      //(time_primitive>planned_time)
    {
        clear_bit_ls(primitive_mask,2);
        set_bit_ls(primitive_mask,1);
        clear_bit_ls(clock_mask,2);
    }
}

void circle(float radius, float planned_time){
    float omega = 2*3.14/planned_time;
    if (!bit_is_set_ls(clock_mask,2))
    {
        set_bit_ls(clock_mask,2);
        counter_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        psi0 = stateGetNedToBodyEulers_f()->psi;
        guidance_loop_set_velocity(0,omega*radius);
        guidance_v_set_guided_z(stateGetPositionNed_f()->z);
        guidance_loop_set_heading(stateGetNedToBodyEulers_f()->psi);
    }
    else
    {
        float vx = -omega*radius*sinf(omega*time_primitive);
        float vy = omega*radius*cosf(omega*time_primitive);
        guidance_loop_set_velocity(vx,vy);
        guidance_v_set_guided_z(stateGetPositionNed_f()->z);
        float heading = atan2(vy,vx);
        guidance_loop_set_heading(heading);
    }
    if(time_primitive > planned_time)      //(time_primitive>planned_time)
    {
        clear_bit_ls(primitive_mask,2);
        set_bit_ls(primitive_mask,1);
        clear_bit_ls(clock_mask,2);
    }
}

void flight_plan_run() {        // 10HZ
    current_mode = autopilot_mode;
    //current_guidance_h_mode = guidance_h.mode;
    if (current_mode != previous_mode)
    {
        counter_autopilot_mode = 0;
        clear_bit_ls(clock_mask,2);
        set_bit_ls(primitive_mask,1);
    }

    if (autopilot_mode != AP_MODE_GUIDED)
        clear_bit_ls(clock_mask,3);

    if (autopilot_mode != AP_MODE_ATTITUDE_DIRECT && bit_is_set_ls(primitive_mask,0))
    {
        take_off(-1.5);
    }
    if (autopilot_mode != AP_MODE_ATTITUDE_DIRECT && bit_is_set_ls(primitive_mask,1))
    {
        hover(2);
    }
    if (autopilot_mode != AP_MODE_ATTITUDE_DIRECT && bit_is_set_ls(primitive_mask,2))
    {
        go_straight(3,1);
    }
    if (autopilot_mode != AP_MODE_ATTITUDE_DIRECT &&  bit_is_set_ls(primitive_mask,3))
    {
        change_heading_hover(90/180*3.14,5);
    }
    previous_mode = current_mode;
}

void set_bit_ls(bool *mask,int bit){
    mask[bit] = 1;
}

void clear_bit_ls(bool *mask,int bit){
    mask[bit] = 0;
}

bool bit_is_set_ls(bool *mask,int bit){
    return(mask[bit]);
}