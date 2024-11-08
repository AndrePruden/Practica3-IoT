#ifndef LASER_SENSOR_H
#define LASER_SENSOR_H

#include "ServoController.h"
#include "MQTTHandler.h"

class LaserSensor {
private:
    int sensorPin;
    unsigned long lastLaserTime;
    unsigned long laserCutDuration;
    bool laserCutFlag;
    ServoController& servoController;
    MQTTHandler& mqttHandler;

public:
    LaserSensor(int pin, ServoController& servo, MQTTHandler& mqtt)
        : sensorPin(pin), lastLaserTime(0), laserCutDuration(500), laserCutFlag(false),
          servoController(servo), mqttHandler(mqtt) {
    }

    void checkLaser() {
        int analogValue = analogRead(sensorPin);
        float voltage = analogValue * (3.3 / 4095.0);

        if (voltage > 3) {
            if (!laserCutFlag) {
                if (millis() - lastLaserTime >= laserCutDuration) {
                    laserCutFlag = true;
                    servoController.setPosition(90);
                    Serial.println("Laser cortado durante 1 segundo. Mensaje publicado.");
                }
            }
        } else {
            if (laserCutFlag) {
                laserCutFlag = false;
                lastLaserTime = 0;
                Serial.println("Laser se apagó antes de 1 segundo. Reiniciando temporizador.");
            }
            lastLaserTime = millis();
        }
    }
};

#endif