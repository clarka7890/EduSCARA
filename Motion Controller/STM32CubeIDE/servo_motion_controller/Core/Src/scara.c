/*
 * scara.c
 *
 *  Created on: Feb 2, 2025
 *      Author: asus
 */

#include "scara.h"
#include "potentiometers.h"

//#include <windows.h>

// function prototypes
// ----------------------------------------------------------------------------------------------------

void calc_forward_kinematics(scara_t *_scara, float *_x, float *_y, float *_angle_z, float *_z,
											  float _theta_1, float _theta_2, float _theta_3, float _theta_4);

void calc_inverse_kinematics(scara_t *_scara, float *_theta_1, float *_theta_2, float *_theta_3, float *_theta_4,
                                              float _x, float _y, float _angle_z, float _z);

// ----------------------------------------------------------------------------------------------------

void scara_init(scara_t *_scara, servo_t *_joint_0, servo_t *_joint_1, servo_t *_joint_2, servo_t *_joint_3,
                float _link_1, float _link_2, float _z_min, float _z_max,
				float _settling_time,
				float _P_0, float _I_0, float _D_0,
				float _P_1, float _I_1, float _D_1)
{
	potentiometers_init();

    if (_scara->servo_controller_scara == NULL) {
        _scara->servo_controller_scara = malloc(sizeof(servo_controller_scara_t));
    }

    _scara->joint_0 = _joint_0;
    _scara->joint_1 = _joint_1;
    _scara->joint_2 = _joint_2;
    _scara->joint_3 = _joint_3;

    servo_controller_scara_init(_scara->servo_controller_scara,
    							_joint_0, _joint_1, _joint_2, _joint_3,
								_settling_time,
								_P_0, _I_0, _D_0, _P_1, _I_1, _D_1);

    _scara->link_1 = _link_1;
    _scara->link_2 = _link_2;
    _scara->z_min = _z_min;
    _scara->z_max = _z_max;
}

void scara_reinit(scara_t *_scara,
				  float _link_1, float _link_2, float _z_min, float _z_max,
				  float _settling_time,
				  float _P_0, float _I_0, float _D_0,
				  float _P_1, float _I_1, float _D_1)
{
    servo_controller_scara_reinit(_scara->servo_controller_scara,
								  _settling_time,
								  _P_0, _I_0, _D_0, _P_1, _I_1, _D_1);

    _scara->link_1 = _link_1;
    _scara->link_2 = _link_2;
    _scara->z_min = _z_min;
    _scara->z_max = _z_max;
}

void scara_auto_calibrate(scara_t *_scara)
{
	servo_controller_scara_auto_calibrate_axis(_scara->servo_controller_scara, 0);
	HAL_Delay(500);
	servo_controller_scara_auto_calibrate_axis(_scara->servo_controller_scara, 1);
	HAL_Delay(500);
}

void scara_move_j(scara_t *_scara, int _axis, float _angle, float _T)
{
	move_j_scara(_scara->servo_controller_scara, _axis, _angle, _T);
}

void scara_move_js(scara_t *_scara, float _angle_0, float _angle_1, float _angle_2, float _angle_3, float _T)
{
    move_js_scara(_scara->servo_controller_scara, _angle_0, _angle_1, _angle_2, _angle_3, _T);
}

void scara_move_coord(scara_t *_scara, float _x, float _y, float _z_angle, float _z, float _T)
{
    float theta_1, theta_2, theta_3, theta_4;
    calc_inverse_kinematics(_scara, &theta_1, &theta_2, &theta_3, &theta_4, _x, _y, _z_angle, _z);
    move_js_scara(_scara->servo_controller_scara, theta_1, theta_2, theta_3, theta_4, _T);
}

scara_position_t scara_get_position(scara_t *_scara)
{
	float theta_1 = potentiometers_read_angle(0);
	float theta_2 = potentiometers_read_angle(1);
	float theta_3 = _scara->joint_2->cpos;
	float theta_4 = _scara->joint_3->cpos;

	float x, y, angle_z, z;

	calc_forward_kinematics(_scara, &x, &y, &angle_z, &z, theta_1, theta_2, theta_3, theta_4);

	scara_position_t pos;
	pos.x = x;
	pos.y = y;
	pos.angle_z = angle_z;
	pos.z = z;

	return pos;
}

// calculations
// ----------------------------------------------------------------------------------------------------

void calc_forward_kinematics(scara_t *_scara, float *_x, float *_y, float *_angle_z, float *_z,
											  float _theta_1, float _theta_2, float _theta_3, float _theta_4)
{
    float l1 = _scara->link_1;
    float l2 = _scara->link_2;

    // convert angles from degrees to radians
    float t1 = _theta_1 * (PI / 180.0);
    float t2 = _theta_2 * (PI / 180.0);

    // compute X and Y position
    *_x = l1 * sin(t1) + l2 * sin(t1 + t2);
    *_y = l1 * cos(t1) + l2 * cos(t1 + t2);

    // compute total rotation angle around Z
    *_angle_z = _theta_1 + _theta_2 + _theta_3;

    // convert theta_4 (the servo angle) back to Z height
    *_z = _scara->z_min + ((_theta_4 - _scara->joint_3->min_angle) *
          (_scara->z_max - _scara->z_min) /
          (_scara->joint_3->max_angle - _scara->joint_3->min_angle));
}

void calc_inverse_kinematics(scara_t *_scara, float *_theta_1, float *_theta_2, float *_theta_3, float *_theta_4,
                                              float _x, float _y, float _angle_z, float _z)
{
    float l1 = _scara->link_1;
    float l2 = _scara->link_2;
    float x = _x;
    float y = _y;

    float r = sqrt((x * x) + (y * y));

    float phi1 = acos(((l2 * l2) - (r * r) - (l1 * l1)) / (-2 * r * l1));
    float phi2 = atan2(x, y);
    float phi3 = acos(((r * r) - (l1 * l1) - (l2 * l2)) / (-2 * l1 * l2));
    // convert z displacement to servo angle in degrees
    float phi4 = _scara->joint_3->min_angle + ((_z - _scara->z_min)
    			 * (_scara->joint_3->max_angle - _scara->joint_3->min_angle) / (_scara->z_max - _scara->z_min));

    *_theta_1 = (phi2 - phi1) * (180.0 / PI);   // convert to degrees
    *_theta_2 = (PI - phi3) * (180.0 / PI);     // convert to degrees
    *_theta_3 = _angle_z - (*_theta_1 + *_theta_2);
    *_theta_4 = phi4;
}

// ----------------------------------------------------------------------------------------------------
