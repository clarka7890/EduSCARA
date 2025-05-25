/*
 * servo_controller.c
 *
 *  Created on: Feb 2, 2025
 *      Author: asus
 */

#include "servo_controller.h"
#include "potentiometers.h"

// function prototypes
// ----------------------------------------------------------------------------------------------------

void calc_double_s_parameters(float T, float h, float alpha, float beta,
							  float *T_a, float *T_j,
							  float *v_max, float *a_max, float *j_max);

float calc_double_s_value(float T, float h, float T_a, float T_j,
						  float v_max, float a_max, float j_max,
						  float t);

// ----------------------------------------------------------------------------------------------------

void servo_controller_init(servo_controller_t *_servo_controller,
						   servo_t *_axis_0, servo_t *_axis_1, servo_t *_axis_2, servo_t *_axis_3)
{
    _servo_controller->axis_0 = _axis_0;
    _servo_controller->axis_1 = _axis_1;
    _servo_controller->axis_2 = _axis_2;
    _servo_controller->axis_3 = _axis_3;

    servo_init(_axis_0, 2500, 500, 1500, -90, 90, 0, 0, 0);
	servo_init(_axis_1, 2500, 500, 1500, -90, 45, 90, 0, 1);
	servo_init(_axis_2, 2500, 500, 1500, -180, 180, 0, 0, 2);
	servo_init(_axis_3, 500, 2500, 1500, -90, 90, 0, 0, 3);

    printf("servo controller initialised\n");
}

void move_j(servo_controller_t *_servo_controller, int _axis, float _angle, float _T)
{
    // assign axis
    servo_t *axis = _servo_controller->axis_0;
    switch (_axis)
    {
        case 1:
            axis = _servo_controller->axis_1;
            break;
        case 2:
            axis = _servo_controller->axis_2;
            break;
        case 3:
			axis = _servo_controller->axis_3;
			break;
        default:
            break;
    }

	// input variables
    float cpos = axis->cpos;
	float tpos = _angle;        // target position in degrees

	float h = tpos - cpos;     	// displacement in degrees
	float T = _T;				// time to complete move
	float alpha = 0.3;         	// acceleration phase duration as fraction of T
	float beta = 0.3;          	// jerk phase duration as fraction of T_a

	// calculate double-S profile parameters
	float T_a, T_j, v_max, a_max, j_max;
	calc_double_s_parameters(T, h, alpha, beta, &T_a, &T_j, &v_max, &a_max, &j_max);

	// calculate samples
	int samples = (int)(T * 1000.0 / SAMPLING_TIME);

	for (int i = 0; i < samples; i++) {
		double t = i * SAMPLING_TIME / 1000.0; // convert sampling time to seconds
        float dpos = cpos + calc_double_s_value(T, h, T_a, T_j, v_max, a_max, j_max, t);

        servo_set_angle(axis, dpos);

        HAL_Delay(SAMPLING_TIME);
	}
}

void move_js(servo_controller_t *_servo_controller,
		     float _angle_0, float _angle_1, float _angle_2, float _angle_3, float _T)
{
    servo_t *axis[AXES] = {_servo_controller->axis_0,
                           _servo_controller->axis_1,
						   _servo_controller->axis_2,
    					   _servo_controller->axis_3};
	// input variables
    float cpos[AXES];
    float tpos[AXES] = {_angle_0, _angle_1, _angle_2, _angle_3};

    float h[AXES];     	        // displacement in degrees
	float T = _T;				// time to complete move
	float alpha = 0.3;         	// acceleration phase duration as fraction of T
	float beta = 0.3;          	// jerk phase duration as fraction of T_a

    // calculate double-S profile parameters
	float T_a[AXES], T_j[AXES], v_max[AXES], a_max[AXES], j_max[AXES];

    for (int i = 0; i < AXES; i++)
    {
        cpos[i] = axis[i]->cpos;
        h[i] = tpos[i] - cpos[i];
    	calc_double_s_parameters(T, h[i], alpha, beta, &T_a[i], &T_j[i], &v_max[i], &a_max[i], &j_max[i]);
    }

	// calculate samples
	int samples = (int)(T * 1000.0 / SAMPLING_TIME);

	for (int i = 0; i < samples; i++) {
        double t = i * SAMPLING_TIME / 1000.0; // convert sampling time to seconds
        for (int j = 0; j < AXES; j++)
        {
            float dpos = cpos[j]
                         + calc_double_s_value(T, h[j], T_a[j], T_j[j], v_max[j], a_max[j], j_max[j], t);

            servo_set_angle(axis[j], dpos);
        }

        HAL_Delay(SAMPLING_TIME);
	}
}

// ----------------------------------------------------------------------------------------------------

void servo_controller_scara_init(servo_controller_scara_t *_servo_controller_scara,
						   	     servo_t *_axis_0, servo_t *_axis_1, servo_t *_axis_2, servo_t *_axis_3,
						   	   	 float _settling_time,
								 float _P_0, float _I_0, float _D_0,
							 	 float _P_1, float _I_1, float _D_1)
{
	_servo_controller_scara->axis_0 = _axis_0;
	_servo_controller_scara->axis_1 = _axis_1;
	_servo_controller_scara->axis_2 = _axis_2;
	_servo_controller_scara->axis_3 = _axis_3;

	servo_init(_axis_0, 2500, 500, 1500, -90, 90, 0, 0, 0);
	servo_init(_axis_1, 2500, 500, 1500, -90, 30, 90, 0, 1);
	servo_init(_axis_2, 500, 2500, 1550, -180, 180, 0, 0, 2);
	servo_init(_axis_3, 500, 2300, 1500, 90, -60, 0, 0, 3);

	_servo_controller_scara->settling_time = _settling_time;

	_servo_controller_scara->P_0 = _P_0;
	_servo_controller_scara->I_0 = _I_0;
	_servo_controller_scara->D_0 = _D_0;

	_servo_controller_scara->P_1 = _P_1;
	_servo_controller_scara->I_1 = _I_1;
	_servo_controller_scara->D_1 = _D_1;

	printf("servo controller initialised\n");
}

void servo_controller_scara_reinit(servo_controller_scara_t *_servo_controller_scara,
								   float _settling_time,
								   float _P_0, float _I_0, float _D_0,
								   float _P_1, float _I_1, float _D_1)
{
	_servo_controller_scara->settling_time = _settling_time;

	_servo_controller_scara->P_0 = _P_0;
	_servo_controller_scara->I_0 = _I_0;
	_servo_controller_scara->D_0 = _D_0;

	_servo_controller_scara->P_1 = _P_1;
	_servo_controller_scara->I_1 = _I_1;
	_servo_controller_scara->D_1 = _D_1;
}

void servo_controller_scara_auto_calibrate_axis(servo_controller_scara_t *_servo_controller_scara, int _axis)
{
	servo_t *axis = _servo_controller_scara->axis_0;
	switch (_axis)
	{
		case 1:
			axis = _servo_controller_scara->axis_1;
			break;
		default:
			break;
	}

	float pulse_width_us = axis->zero_pulse_width;
	float pot_angle = potentiometers_read_angle(_axis);

	// reset to zero position
	servo_set_pulse_width_us(axis, pulse_width_us);
	HAL_Delay(500);

	// check whether the axis is inverted
	// ==================== AXIS IS NOT INVERTED ====================
	if (axis->max_pulse_width >= axis->min_pulse_width)
	{
		// sweep from 0 to max pulse width (deadband = 4), stopping when max angle reached
		for (int i = axis->zero_pulse_width; i <= axis->max_pulse_width; i+=5)
		{
			pulse_width_us = i;
			servo_set_pulse_width_us(axis, pulse_width_us);

			// check position using encoder
			pot_angle = potentiometers_read_angle(_axis);
			// if angle within 20% of target angle, move slower
			if (pot_angle >= (axis->max_angle * 0.5) + axis->offset_angle)
			{
				HAL_Delay(50);
			}
			else // otherwise move fast
			{
				HAL_Delay(5);
			}

			// check if target angle is reached
			if (pot_angle >= (axis->max_angle + axis->offset_angle))
			{
				axis->calib_max_pulse_width = pulse_width_us;
				break;
			}
		}
		// sweep from max angle to min angle
		for (int i = pulse_width_us; i >= axis->min_pulse_width; i-=5)
		{
			pulse_width_us = i;
			servo_set_pulse_width_us(axis, pulse_width_us);

			// check position using encoder
			pot_angle = potentiometers_read_angle(_axis);
			if (pot_angle <= (axis->min_angle * 0.5) + axis->offset_angle)
			{
				HAL_Delay(50);
			}
			else
			{
				HAL_Delay(5);
			}

			if (pot_angle <= (axis->min_angle + axis->offset_angle))
			{
				axis->calib_min_pulse_width = pulse_width_us;
				break;
			}
		}
		// sweep towards max pulse width again, but stopping at 0 degrees
		for (int i = pulse_width_us; i <= axis->max_pulse_width; i+=5)
		{
			pulse_width_us = i;
			servo_set_pulse_width_us(axis, pulse_width_us);

			pot_angle = potentiometers_read_angle(_axis);
			if (pot_angle >= (axis->min_angle * 0.5) + axis->offset_angle)
			{
				HAL_Delay(50);
			}
			else
			{
				HAL_Delay(5);
			}

			if (pot_angle >= (0 + axis->offset_angle))
			{
				axis->calib_zero_pulse_width = pulse_width_us;
				break;
			}
		}
	}
	// ==================== AXIS IS INVERTED ====================
	else
	{
		// sweep towards max angle
		for (int i = axis->zero_pulse_width; i >= axis->max_pulse_width; i-=5)
		{
			pulse_width_us = i;
			servo_set_pulse_width_us(axis, pulse_width_us);

			pot_angle = potentiometers_read_angle(_axis);
			if (pot_angle >= (axis->max_angle * 0.5) + axis->offset_angle)
			{
				HAL_Delay(50);
			}
			else
			{
				HAL_Delay(5);
			}

			if (pot_angle >= (axis->max_angle + axis->offset_angle))
			{
				axis->calib_max_pulse_width = pulse_width_us;
				break;
			}
		}
		// sweep from max angle to min angle
		for (int i = pulse_width_us; i <= axis->min_pulse_width; i+=5)
		{
			pulse_width_us = i;
			servo_set_pulse_width_us(axis, pulse_width_us);

			pot_angle = potentiometers_read_angle(_axis);
			if (pot_angle <= (axis->min_angle * 0.5) + axis->offset_angle)
			{
				HAL_Delay(50);
			}
			else
			{
				HAL_Delay(5);
			}

			if (pot_angle <= (axis->min_angle + axis->offset_angle))
			{
				axis->calib_min_pulse_width = pulse_width_us;
				break;
			}
		}
		// return to 0
		for (int i = pulse_width_us; i >= axis->max_pulse_width; i-=5)
		{
			pulse_width_us = i;
			servo_set_pulse_width_us(axis, pulse_width_us);

			pot_angle = potentiometers_read_angle(_axis);
			if (pot_angle >= (axis->min_angle * 0.5) + axis->offset_angle)
			{
				HAL_Delay(50);
			}
			else
			{
				HAL_Delay(5);
			}

			if (pot_angle >= (0 + axis->offset_angle))
			{
				axis->calib_zero_pulse_width = pulse_width_us;
				break;
			}
		}
	}
	axis->cpos = pot_angle;
}

void move_j_scara(servo_controller_scara_t *_servo_controller_scara, int _axis, float _angle, float _T)
{
    // assign axis
    servo_t *axis = _servo_controller_scara->axis_0;
    switch (_axis)
    {
        case 1:
            axis = _servo_controller_scara->axis_1;
            break;
        case 2:
            axis = _servo_controller_scara->axis_2;
            break;
        case 3:
			axis = _servo_controller_scara->axis_3;
			break;
        default:
            break;
    }

    // compensate for adjustment from previous move
    float cpos = axis->cpos;
	if (_axis == 0 || _axis == 1)
	{
		float cpos_adjustment;
		if (axis->last_angle_adjustment < 0)
		{
			cpos_adjustment = -(axis->last_angle_adjustment) * 0.5;
		}
		else
		{
			cpos_adjustment = axis->last_angle_adjustment * 0.5;
		}

		if (_angle - cpos > cpos_adjustment)
		{
			cpos += cpos_adjustment;
		}
		else if (_angle - cpos < cpos_adjustment)
		{
			cpos -= cpos_adjustment;
		}
	}

	float tpos = _angle;        // target position in degrees
	float h = tpos - cpos;     	// displacement in degrees
	float T = _T;				// time to complete move
	float alpha = 0.3;         	// acceleration phase duration as fraction of T
	float beta = 0.3;          	// jerk phase duration as fraction of T_a

	// calculate double-S profile parameters
	float T_a, T_j, v_max, a_max, j_max;
	calc_double_s_parameters(T, h, alpha, beta, &T_a, &T_j, &v_max, &a_max, &j_max);

	// ============================== MOVE ==============================

	float P;
	float I;
	float D;
	if (_axis == 0)
	{
		P = _servo_controller_scara->P_0;
		I = _servo_controller_scara->I_0;
		D = _servo_controller_scara->D_0;
	}
	else
	{
		P = _servo_controller_scara->P_1;
		I = _servo_controller_scara->I_1;
		D = _servo_controller_scara->D_1;
	}

	float error_sum = 0.0;
	float last_error = 0.0;
	float max_integral = 20.0;

	// calculate samples
	int samples = (int)(T * 1000.0 / SAMPLING_TIME);

	for (int i = 0; i < samples; i++)
	{
	    double t = i * SAMPLING_TIME / 1000.0; // Convert sampling time to seconds

	    // desired position using double S-curve profile
	    float dpos = cpos + calc_double_s_value(T, h, T_a, T_j, v_max, a_max, j_max, t);

	    // ----- control loop -----
	    if (_axis == 0 || _axis == 1)
	    {
	        // error calculation
	        float pot_read = potentiometers_read_angle(_axis);
	        float error = dpos - pot_read;

	        // PID terms
	        float P_term_move = error * P;

	        // integral term (with anti-windup)
	        error_sum += error * (SAMPLING_TIME / 1000.0);
	        if (error_sum > max_integral)
	            error_sum = max_integral;
	        else if (error_sum < -max_integral)
	            error_sum = -max_integral;

	        float I_term_move = error_sum * I;

	        // derivative term
	        float D_term_move = ((error - last_error) / (SAMPLING_TIME / 1000.0)) * D;
	        last_error = error;

	        // PID output
	        float pid_output = P_term_move + I_term_move + D_term_move;

	        // limit PID output to +-5 degrees
	        float adjusted_error;
	        if (pid_output > 5)
	        {
	            adjusted_error = 5;
	        }
	        else if (pid_output < -5)
	        {
	            adjusted_error = -5;
	        }
	        else
	        {
	            adjusted_error = pid_output;
	        }

	        // set adjusted position with PID correction
	        servo_set_angle(axis, dpos + adjusted_error);
	    }
	    // ------------------------
	    else
	    {
	        // no PID correction for other axes
	        servo_set_angle(axis, dpos);
	    }

	    HAL_Delay(SAMPLING_TIME);
	}

	// ============================== SETTLING ==============================

	// get angle before settling
	float angle_before_settling = 0.0;
	if (_axis == 0 || _axis == 1)
	{
		angle_before_settling = potentiometers_read_angle(_axis);
	}
    float pulse_width_correction = 0.0;

	// settling time (seconds) to reduce backlash
	int settling_samples = (int)(_servo_controller_scara->settling_time * 1000.0 / SAMPLING_TIME_SETTLING);

	for (int i = 0; i < settling_samples; i++)
	{
	    // ----- control loop -----
	    if (_axis == 0 || _axis == 1)
	    {
	        float pot_read = potentiometers_read_angle(_axis);

	        if (tpos - pot_read < -0.5) {
	        	pulse_width_correction = 5;
	        } else if (tpos - pot_read > 0.5) {
	        	pulse_width_correction = -5;
	        } else {
	        	pulse_width_correction = 0;
	        }

	        float current_pulse_width = servo_get_pulse_width_us(axis);
	        float adjusted_pulse_width = current_pulse_width + pulse_width_correction;

	        servo_set_pulse_width_us(axis, adjusted_pulse_width);
	    }

	    HAL_Delay(SAMPLING_TIME_SETTLING);
	}

	// set last angle adjustment from settling
	if (_axis == 0 || _axis == 1)
	{
		axis->last_angle_adjustment = angle_before_settling - potentiometers_read_angle(_axis);
	}
}

void move_js_scara(servo_controller_scara_t *_servo_controller_scara,
		     	   float _angle_0, float _angle_1, float _angle_2, float _angle_3, float _T)
{
    servo_t *axis[AXES] = {_servo_controller_scara->axis_0,
						   _servo_controller_scara->axis_1,
						   _servo_controller_scara->axis_2,
						   _servo_controller_scara->axis_3};
	// input variables
    float cpos[AXES];
    float tpos[AXES] = {_angle_0, _angle_1, _angle_2, _angle_3};

    float h[AXES];     	        // displacement in degrees
	float T = _T;				// time to complete move
	float alpha = 0.3;         	// acceleration phase duration as fraction of T
	float beta = 0.3;          	// jerk phase duration as fraction of T_a

    // calculate double-S profile parameters
	float T_a[AXES], T_j[AXES], v_max[AXES], a_max[AXES], j_max[AXES];

    for (int i = 0; i < AXES; i++)
    {
    	cpos[i] = axis[i]->cpos;

    	if (i == 0 || i == 1)
    	{
    		float cpos_adjustment;
			if (axis[i]->last_angle_adjustment < 0)
			{
				cpos_adjustment = -(axis[i]->last_angle_adjustment) * 0.5;
			}
			else
			{
				cpos_adjustment = axis[i]->last_angle_adjustment * 0.5;
			}

			if (tpos[AXES] - cpos[i] > cpos_adjustment)
			{
				cpos[i] += cpos_adjustment;
			}
			else if (tpos[AXES] - cpos[i] < cpos_adjustment)
			{
				cpos[i] -= cpos_adjustment;
			}
    	}

        h[i] = tpos[i] - cpos[i];
    	calc_double_s_parameters(T, h[i], alpha, beta, &T_a[i], &T_j[i], &v_max[i], &a_max[i], &j_max[i]);
    }

	// ============================== MOVE ==============================

    // PID for axis 0 and 1
    float P[2] = {_servo_controller_scara->P_0, _servo_controller_scara->P_1};
	float I[2] = {_servo_controller_scara->I_0, _servo_controller_scara->I_1};
	float D[2] = {_servo_controller_scara->D_0, _servo_controller_scara->D_1};

	float error_sum[2] = {0.0, 0.0};
	float last_error[2] = {0.0, 0.0};
	float max_integral[2] = {20.0, 20.0};

	// calculate samples
	int samples = (int)(T * 1000.0 / SAMPLING_TIME);

	for (int i = 0; i < samples; i++) {
        double t = i * SAMPLING_TIME / 1000.0; // convert sampling time to seconds
        for (int j = 0; j < AXES; j++)
        {
            float dpos = cpos[j]
                         + calc_double_s_value(T, h[j], T_a[j], T_j[j], v_max[j], a_max[j], j_max[j], t);

            // ----- control loop -----
            if (j == 0 || j == 1)
            {
            	// error calculation
				float pot_read = potentiometers_read_angle(j);
				float error = dpos - pot_read;

				// PID terms
				float P_term_move = error * P[j];

				// integral term (with anti-windup)
				error_sum[j] += error * (SAMPLING_TIME / 1000.0);
				if (error_sum[j] > max_integral[j])
					error_sum[j] = max_integral[j];
				else if (error_sum[j] < -max_integral[j])
					error_sum[j] = -max_integral[j];

				float I_term_move = error_sum[j] * I[j];

				// derivative term
				float D_term_move = ((error - last_error[j]) / (SAMPLING_TIME / 1000.0)) * D[j];
				last_error[j] = error;

				// PID output
				float pid_output = P_term_move + I_term_move + D_term_move;

				// limit PID output to +-5 degrees
				float adjusted_error;
				if (pid_output > 5)
				{
					adjusted_error = 5;
				}
				else if (pid_output < -5)
				{
					adjusted_error = -5;
				}
				else
				{
					adjusted_error = pid_output;
				}

				// set adjusted position with PID correction
            	servo_set_angle(axis[j], dpos + adjusted_error);
            }
            // ------------------------
            else
            {
            	servo_set_angle(axis[j], dpos);
            }
        }

        //Sleep(SAMPLING_TIME);
        HAL_Delay(SAMPLING_TIME);
	}

	// ============================== SETTLING ==============================

	// get angle before settling
	float angle_before_settling[2] = {0.0, 0.0};
	angle_before_settling[0] = potentiometers_read_angle(0);
	angle_before_settling[1] = potentiometers_read_angle(1);

	float pulse_width_correction[2] = {0.0, 0.0};

	// settling time (seconds) to reduce backlash
	int settling_samples = (int)(_servo_controller_scara->settling_time * 1000.0 / SAMPLING_TIME_SETTLING);

	for (int i = 0; i < settling_samples; i++)
	{
		// apply to only axis 0 and 1 ...
		for (int j = 0; j < 2; j++)
		{
			float pot_read = potentiometers_read_angle(j);

			if (tpos[j] - pot_read < -0.5)
			{
				pulse_width_correction[j] = 5; // minimum pulse width as deadband = 4ms
			}
			else if (tpos[j] - pot_read > 0.5)
			{
				pulse_width_correction[j] = -5;
			}
			else
			{
				pulse_width_correction[j] = 0;
			}

			float current_pulse_width = servo_get_pulse_width_us(axis[j]);
			float adjusted_pulse_width = current_pulse_width + pulse_width_correction[j];

			servo_set_pulse_width_us(axis[j], adjusted_pulse_width);
		}

		HAL_Delay(SAMPLING_TIME_SETTLING);
	}

	// set last angle adjustment from settling
	axis[0]->last_angle_adjustment = angle_before_settling[0] - potentiometers_read_angle(0);
	axis[1]->last_angle_adjustment = angle_before_settling[1] - potentiometers_read_angle(1);
}

// calculations
// ----------------------------------------------------------------------------------------------------

void calc_double_s_parameters(float T, float h, float alpha, float beta,
							  float *T_a, float *T_j,
							  float *v_max, float *a_max, float *j_max)
{
    // accel / decel duration    0 < alpha < 1/2
    *T_a = alpha * T;
    // jerk duration             0 < beta < 1/2
    *T_j = beta * (*T_a);
    // calculate maximum velocity
    *v_max = h / ((1 - alpha) * T);
    // calculate maximum acceleration
    *a_max = h / (alpha * (1 - alpha) * (1 - beta) * T * T);
    // calculate maximum jerk
    *j_max = h / (alpha * alpha * beta * (1 - alpha) * (1 - beta) * T * T * T);
}


float calc_double_s_value(float T, float h, float T_a, float T_j,
						  float v_max, float a_max, float j_max,
						  float t)
{
    // acceleration phase
    if (t >= 0 && t <= T_j)
    {
        return j_max * (pow(t, 3) / 6.0);
    }
    else if (t > T_j && t <= T_a - T_j)
    {
    	return (a_max / 6.0) * ((3 * pow(t, 2)) - (3 * T_j * t) + pow(T_j, 2));
    }
    else if (t > T_a - T_j && t <= T_a)
    {
    	return (v_max * (T_a / 2.0)) - (v_max * (T_a - t)) - (-j_max * (pow(T_a - t, 3) / 6.0));
    }
    // constant velocity phase
    else if (t > T_a && t <= T - T_a)
    {
    	return v_max * (T_a / 2.0) + (v_max * (t - T_a));
    }
    // deceleration phase
    else if (t > T - T_a && t <= T - T_a + T_j)
    {
    	return h - (v_max * (T_a / 2.0)) + v_max * (t - T + T_a) - (j_max * pow(t - T + T_a, 3) / 6.0);
    }
    else if (t > T - T_a + T_j && t <= T - T_j)
    {
    	return h - (v_max * (T_a / 2.0)) + v_max * (t - T + T_a)
               - ((a_max / 6.0) * (3 * pow(t - T + T_a, 2) - 3 * T_j * (t - T + T_a) + pow(T_j, 2)));
    }
    else if (t > T - T_j && t <= T)
    {
    	return h - (j_max * (pow(T - t, 3) / 6.0));
    }
    else
    {
    	return 999;
    }
}

// ----------------------------------------------------------------------------------------------------
