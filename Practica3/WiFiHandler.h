#ifndef WIFI_HANDLER_H
#define WIFI_HANDLER_H

#include <WiFi.h>

class WiFiHandler {
private:
    const char* ssid;
    const char* password;

public:
    WiFiHandler(const char* ssid, const char* password) : ssid(ssid), password(password) {}

    void connect() {
        Serial.print("Connecting to ");
        Serial.println(ssid);
        WiFi.begin(ssid, password);
        
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        Serial.println("Connected to WiFi.");
    }

    bool isConnected() {
        return WiFi.status() == WL_CONNECTED;
    }
};

#endif