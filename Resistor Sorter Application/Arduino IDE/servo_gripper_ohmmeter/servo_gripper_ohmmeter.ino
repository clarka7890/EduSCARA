#include "servo_gripper.h"
#include "ohmmeter.h"

const int SERVO_PIN = 9;
const int OPEN_PULSE = 2000;
const int CLOSE_PULSE = 1575;

servo_gripper gripper(SERVO_PIN, OPEN_PULSE, CLOSE_PULSE);

float resistance = -1.0;

void setup() {
    Serial.begin(9600);
    gripper.begin();
    initOhmmeter();
}

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command == "OPEN") {
            gripper.open();
            Serial.println("Gripper opened.");
        } else if (command == "CLOSE") {
            gripper.close();
            Serial.println("Gripper closed.");
        } else if (command == "IDLE") {
            gripper.idle();
            Serial.println("Gripper idle.");
        } else if (command == "TEST") {
            resistance = readResistance();
            Serial.println(resistance);
        } else {
            Serial.println("Unknown command.");
        }
    }
}


    // resistance = readResistance();
    // Serial.println(resistance);
    // delay(100);