#include "servo_gripper.h"

servo_gripper::servo_gripper(int servoPin, int openPulse, int closePulse) {
    pin = servoPin;
    openPulseWidth = openPulse;
    closePulseWidth = closePulse;
}

void servo_gripper::begin() {
    servo.attach(pin);
    close(); // default to closed
}

void servo_gripper::open() {
    servo.writeMicroseconds(openPulseWidth);
}

void servo_gripper::close() {
    servo.writeMicroseconds(closePulseWidth);
}

void servo_gripper::idle() {
    servo.writeMicroseconds((openPulseWidth + closePulseWidth) / 2);
}
