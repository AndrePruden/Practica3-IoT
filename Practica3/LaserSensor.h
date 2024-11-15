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
    float voltage = analogValue * (3.3 / 4095.0);  // Conversión de valor analógico a voltaje

    if (voltage > 3) {  // Umbral para detectar si el láser está siendo cortado
        if (!laserCutFlag) {
            // Solo publicamos el estado cuando el láser se corta
            if (millis() - lastLaserTime >= laserCutDuration) {
                laserCutFlag = true;
                
                // Publicar el estado de "LaserCortado": 1
                String jsonPayload = "{\"state\": {\"reported\": {\"LaserCortado\": 1}}}";
                mqttHandler.publish("$aws/things/MyThing/shadow/update", jsonPayload.c_str());
                
                Serial.println("Laser cortado durante 1 segundo. Mensaje publicado.");
            }
        }
    } else {  // Si el láser no está cortado
        if (laserCutFlag) {
            laserCutFlag = false;
            lastLaserTime = 0;
            // Publicar el estado de "LaserCortado": 0 cuando el láser se apaga


            Serial.println("Laser se apagó antes de 1 segundo. Mensaje publicado.");
        }
        lastLaserTime = millis();  // Reiniciar temporizador
    }
}
};

#endif