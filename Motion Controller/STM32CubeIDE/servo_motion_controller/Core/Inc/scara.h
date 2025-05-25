/*
 * scara.h
 *
 *  Created on: Feb 2, 2025
 *      Author: asus
 */

#ifndef INC_SCARA_H_
#define INC_SCARA_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//#include "servo.h"
#include "servo_controller.h"
//#include "main.h"

#define PI 3.141592653589793

typedef struct {
	servo_t *joint_0;
	servo_t *joint_1;
	servo_t *joint_2;
	servo_t *joint_3;
    float link_1;
    float link_2;
    float z_min;
    float z_max;
    servo_controller_scara_t *servo_controller_scara;
} scara_t;

typedef struct {
	float x;
	float y;
	float angle_z;
	float z;
} scara_position_t;

void scara_init(scara_t *_scara, servo_t *_joint_0, servo_t *_joint_1, servo_t *_joint_2, servo_t *_joint_3,
                float _link_1, float _link_2, float _z_min, float _z_max,
				float _settling_time,
				float _P_0, float _I_0, float _D_0,
				float _P_1, float _I_1, float _D_1);

void scara_reinit(scara_t *_scara,
				  float _link_1, float _link_2, float _z_min, float _z_max,
				  float _settling_time,
				  float _P_0, float _I_0, float _D_0,
				  float _P_1, float _I_1, float _D_1);

void scara_auto_calibrate(scara_t *_scara);

void scara_move_j(scara_t *_scara, int _axis, float _angle, float _T);
void scara_move_js(scara_t *_scara, float _angle_0, float _angle_1, float _angle_2, float _angle_3, float _T);
void scara_move_coord(scara_t *_scara, float _x, float _y, float _z_angle, float _z, float _T);

scara_position_t scara_get_position(scara_t *_scara);

#endif /* INC_SCARA_H_ */
