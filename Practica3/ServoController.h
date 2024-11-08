#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <ESP32Servo.h>
#include "MQTTHandler.h"
#include "LEDHandler.h"

class ServoController {
private:
    Servo servo;
    int servoPin;
    int position;
    MQTTHandler& mqttHandler;
    LEDHandler& ledHandler;

public:
    ServoController(int pin, MQTTHandler& mqtt, LEDHandler& led) 
        : servoPin(pin), position(0), mqttHandler(mqtt), ledHandler(led) {
    }

    void setup() {
        ESP32PWM::allocateTimer(0);
        ESP32PWM::allocateTimer(1);
        ESP32PWM::allocateTimer(2);
        ESP32PWM::allocateTimer(3);
        servo.setPeriodHertz(50);
        servo.attach(servoPin, 500, 2400);
    }

    void setPosition(int newPosition) {
        position = newPosition;
        servo.write(position);
        updateLEDs();
        reportPosition();
    }

    void updateLEDs() {
        int currentPos = servo.read();
        if (currentPos > 10 && currentPos != 8880) {
            ledHandler.setRedLED(true);
            ledHandler.setBlueLED(false);
        } else {
            ledHandler.setRedLED(false);
            ledHandler.setBlueLED(true);
        }
    }

    void reportPosition() {
        String jsonPayload = "{\"state\": {\"desired\": {\"servoPosition\": ";
        jsonPayload += String(position);
        jsonPayload += "}, \"reported\": {\"servoPosition\": ";
        jsonPayload += String(position);
        jsonPayload += "}}}";
        
        mqttHandler.publish(mqttHandler.getUpdateTopic(), jsonPayload.c_str());
    }

    int getPosition() const { return position; }
    
    int getCurrentPosition() { return servo.read(); }
};

#endif