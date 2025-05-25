#include "ohmmeter.h"

float rRef[NUM_REF_RESISTORS] = {51, 100.0, 1000.0, 10000.0, 100000.0, 1000000.0, 5100000.0, 10000000.0};
const byte rSelPins[NUM_SELECT_PINS] = {3, 4, 5};
const byte enableMux = 6;

void initOhmmeter()
{
  pinMode(enableMux, OUTPUT);
  digitalWrite(enableMux, HIGH);

  for (int i = 0; i < NUM_SELECT_PINS; i++) {
    pinMode(rSelPins[i], OUTPUT);
    digitalWrite(rSelPins[i], HIGH);
  }
}

float readResistance()
{
  int analogReading;
  float delta, deltaBest1 = MAX_ANALOG_VALUE, deltaBest2 = MAX_ANALOG_VALUE;
  float rBest1 = -1, rBest2 = -1;
  float rR, rX = -1;

  for (byte i = 0; i < NUM_REF_RESISTORS; i++) {
    digitalWrite(rSelPins[0], i & 1);
    digitalWrite(rSelPins[1], i & 2);
    digitalWrite(rSelPins[2], i & 4);

    digitalWrite(enableMux, LOW);
    delay(i + 1);
    analogReading = analogRead(A0);
    digitalWrite(enableMux, HIGH);
    delay(NUM_REF_RESISTORS - i);

    if (analogReading < MAX_ANALOG_VALUE) {
      rR = rRef[i] + SWITCH_RESISTANCE;
      float rThis = (rR * analogReading) / (MAX_ANALOG_VALUE - analogReading);
      delta = (MAX_ANALOG_VALUE / 2.0) - analogReading;

      if (fabs(delta) < fabs(deltaBest1)) {
        deltaBest2 = deltaBest1;
        rBest2 = rBest1;
        deltaBest1 = delta;
        rBest1 = rThis;
      } else if (fabs(delta) < fabs(deltaBest2)) {
        deltaBest2 = delta;
        rBest2 = rThis;
      }
    }
  }

  if (rBest1 >= 0 && rBest2 >= 0) {
    if (deltaBest1 * deltaBest2 < 0) {
      rX = rBest1 - deltaBest1 * (rBest2 - rBest1) / (deltaBest2 - deltaBest1);
    } else {
      rX = rBest1;
    }
  } else {
    rX = -1.0;
  }

  return rX;
}
