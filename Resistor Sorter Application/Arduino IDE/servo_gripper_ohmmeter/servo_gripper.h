#ifndef SERVO_GRIPPER_H
#define SERVO_GRIPPER_H

#include <Arduino.h>
#include <Servo.h>

class servo_gripper {
public:
    servo_gripper(int servoPin, int openPulse, int closePulse);
    void open();
    void close();
    void idle();
    void begin();
private:
    Servo servo;
    int pin;
    int openPulseWidth;
    int closePulseWidth;
};

#endif

