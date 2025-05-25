#ifndef OHMMETER_H
#define OHMMETER_H

#include <Arduino.h>

#define NUM_REF_RESISTORS 8
#define NUM_SELECT_PINS   3
#define MAX_ANALOG_VALUE  1023
#define SWITCH_RESISTANCE 75           // Internal resistance of the multiplexer in ohms

void initOhmmeter();
float readResistance();

#endif

