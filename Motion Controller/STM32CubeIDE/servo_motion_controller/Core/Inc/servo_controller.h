/*
 * servo_controller.h
 *
 *  Created on: Feb 2, 2025
 *      Author: asus
 */

#ifndef INC_SERVO_CONTROLLER_H_
#define INC_SERVO_CONTROLLER_H_

#include <stdio.h>
#include <math.h>

#include "servo.h"

#define AXES 4
#define SAMPLING_TIME 1 	// milliseconds
#define SAMPLING_TIME_SETTLING 50

typedef struct {
	servo_t *axis_0;
	servo_t *axis_1;
	servo_t *axis_2;
	servo_t *axis_3;
} servo_controller_t;

typedef struct {
	servo_t *axis_0;
	servo_t *axis_1;
	servo_t *axis_2;
	servo_t *axis_3;
	float settling_time;
	float P_0; float I_0; float D_0;
	float P_1; float I_1; float D_1;
} servo_controller_scara_t;

void servo_controller_init(servo_controller_t *_servo_controller,
						   servo_t *_axis_0, servo_t *_axis_1, servo_t *_axis_2, servo_t *_axis_3);

void move_j(servo_controller_t *_servo_controller, int _axis, float _angle, float _T);
void move_js(servo_controller_t *_servo_controller,
			 float _angle_0, float _angle_1, float _angle_2, float _angle_3, float _T);

void servo_controller_scara_init(servo_controller_scara_t *_servo_controller_scara,
						   	     servo_t *_axis_0, servo_t *_axis_1, servo_t *_axis_2, servo_t *_axis_3,
						   	   	 float _settling_time,
								 float _P_0, float _I_0, float _D_0,
							 	 float _P_1, float _I_1, float _D_1);

void servo_controller_scara_reinit(servo_controller_scara_t *_servo_controller_scara,
								   float _settling_time,
								   float _P_0, float _I_0, float _D_0,
								   float _P_1, float _I_1, float _D_1);

void servo_controller_scara_auto_calibrate_axis(servo_controller_scara_t *_servo_controller_scara, int _axis);

void move_j_scara(servo_controller_scara_t *_servo_controller_scara, int _axis, float _angle, float _T);
void move_js_scara(servo_controller_scara_t *_servo_controller_scara,
		     	   float _angle_0, float _angle_1, float _angle_2, float _angle_3, float _T);

#endif /* INC_SERVO_CONTROLLER_H_ */
