#ifndef MQTT_HANDLER_H
#define MQTT_HANDLER_H

#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

class MQTTHandler {
private:
    const char* broker;
    const int port;
    const char* clientId;
    const char* updateTopic;
    const char* updateDeltaTopic;
    WiFiClientSecure& wifiClient;
    PubSubClient client;
    StaticJsonDocument<JSON_OBJECT_SIZE(64)> inputDoc;
    StaticJsonDocument<JSON_OBJECT_SIZE(4)> outputDoc;
    char outputBuffer[128];

public:
    MQTTHandler(const char* broker, int port, const char* clientId, 
                const char* updateTopic, const char* updateDeltaTopic,
                WiFiClientSecure& wifiClient)
        : broker(broker), port(port), clientId(clientId),
          updateTopic(updateTopic), updateDeltaTopic(updateDeltaTopic),
          wifiClient(wifiClient), client(wifiClient) {
    }

    void setup(const char* rootCA, const char* cert, const char* privateKey) {
        wifiClient.setCACert(rootCA);
        wifiClient.setCertificate(cert);
        wifiClient.setPrivateKey(privateKey);
        
        client.setServer(broker, port);
    }

    void setCallback(MQTT_CALLBACK_SIGNATURE) {
        client.setCallback(callback);
    }

    bool connect() {
        if (!client.connected()) {
            Serial.print("Attempting MQTT connection...");
            if (client.connect(clientId)) {
                Serial.println("connected");
                client.subscribe(updateDeltaTopic);
                return true;
            }
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" trying again in 5 seconds");
            return false;
        }
        return true;
    }

    void publish(const char* topic, const char* payload) {
        client.publish(topic, payload);
    }

    void loop() {
        client.loop();
    }

    bool isConnected() {
        return client.connected();
    }

    const char* getUpdateTopic() const { return updateTopic; }
    const char* getUpdateDeltaTopic() const { return updateDeltaTopic; }
};

#endif