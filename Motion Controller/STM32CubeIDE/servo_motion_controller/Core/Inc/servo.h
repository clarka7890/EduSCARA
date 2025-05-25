/*
 * servo.h
 *
 *  Created on: Feb 4, 2025
 *      Author: asus
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include <stdio.h>
#include <math.h>

#include "stm32f4xx_hal.h"

typedef struct {
	float min_pulse_width;
	float max_pulse_width;
	float zero_pulse_width;

	float calib_min_pulse_width;
	float calib_max_pulse_width;
	float calib_zero_pulse_width;

	float min_angle;
	float max_angle;
	float offset_angle;

	float cpos;
	float last_angle_adjustment;

	TIM_HandleTypeDef *pwm_timer;
	uint32_t pwm_timer_channel;
} servo_t;

void servo_init(servo_t *_servo, float _min_pulse_width, float _max_pulse_width, float _zero_pulse_width,
				float _min_angle, float _max_angle, float _offset_angle, float _cpos, int _channel);

float servo_get_pulse_width_us(servo_t *_servo);
void servo_set_pulse_width_us(servo_t *_servo, float _pulse_width_us);

float servo_get_angle_from_pulse_width(servo_t *_servo);
void servo_set_angle(servo_t *_servo, float _angle);

#endif /* INC_SERVO_H_ */
