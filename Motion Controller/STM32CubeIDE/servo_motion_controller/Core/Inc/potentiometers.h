/*
 * potentiometers.h
 *
 *  Created on: Mar 27, 2025
 *      Author: asus
 */

#ifndef INC_POTENTIOMETERS_H_
#define INC_POTENTIOMETERS_H_

#include <stdio.h>
#include <math.h>

#include "stm32f4xx_hal.h"
#include "adc.h"

#define POTS 2

typedef struct {
    float min_angle;
    float max_angle;
    float offset_angle;

    float min_raw_value;
    float max_raw_value;
} potentiometer_t;

extern potentiometer_t pots[POTS];

void potentiometers_init();

float potentiometers_read_raw_value(int _channel);
float potentiometers_read_angle(int _channel);

#endif /* INC_POTENTIOMETERS_H_ */
