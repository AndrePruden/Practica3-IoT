#ifndef LED_HANDLER_H
#define LED_HANDLER_H

class LEDHandler {
private:
    int redPin;
    int bluePin;

public:
    LEDHandler(int redPin, int bluePin) : redPin(redPin), bluePin(bluePin) {
        pinMode(redPin, OUTPUT);
        pinMode(bluePin, OUTPUT);
    }

    void setRedLED(bool state) {
        digitalWrite(redPin, state ? HIGH : LOW);
    }

    void setBlueLED(bool state) {
        digitalWrite(bluePin, state ? HIGH : LOW);
    }
};

#endif