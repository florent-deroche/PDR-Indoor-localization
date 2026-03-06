#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEScan.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h> 
#include <BLEAdvertisedDevice.h>

// We want fast publication of result, python ros node will filter abberant value for RSSI. 

// Wifi ID, MQTT, anchor Names Id
#include "shared_config.h"

WiFiClient espClient; 
PubSubClient mqttClient(espClient);
BLEScan* pBLEScan; 

// init rssi signal value to very very low (-100dBm)
int rssi_anchor_1 = -100;
int rssi_anchor_2 = -100;
int rssi_anchor_3 = -100; 

const int SCAN_TIME = 200; // in milliseconds

// automatically called function when a BLE device is detected (Callback function)

class DetectedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice detectedDevice) {
        if(detectedDevice.haveName()) {
            String deviceName = detectedDevice.getName().c_str();
            int rssi = detectedDevice.getRSSI();

            // we keep only the signals from our anchors, we filter other devices
            if (deviceName == ANCHOR_1_NAME) rssi_anchor_1 = rssi; // from shared_config.h
            else if (deviceName == ANCHOR_2_NAME) rssi_anchor_2 = rssi; 
            else if (deviceName == ANCHOR_3_NAME) rssi_anchor_3 = rssi;

        };

    };


};

void setup_wifi() {
    delay(10);
    Serial.println("Connecting to WiFi:");
    Serial.println(ssid); // defined in shared_config.h
    WiFi.begin(ssid, password);

    while(WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print("."); 
    }
    Serial.println("\nSuccesfully connected to WiFi ! IP : ");
    Serial.println(WiFi.localIP());
}

void reconnect_mqtt() {
    while(!mqttClient.connected()) {
        Serial.print("Connecting to MQTT Serv...");
        String clientId = "RobotESP32-1"; // change this if we want to localise various robots
        if (mqttClient.connect(clientId.c_str())) {
            Serial.println("Connected to mqtt broker ! ");

        } else {
            Serial.print("Connection failure, error : ");
            Serial.print(mqttClient.state());
            Serial.print("Retrying in 5 seconds");
            delay(5000);
        }

    }


}

void setup() {
    Serial.begin(115200);

    // wifi mqtt serv launch 
    setup_wifi(); 
    mqttClient.setServer(mqtt_server, mqtt_port); // see shared_config.h

    // BLE scan launch 
    Serial.println("Initializing BLE Scanner...");
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new DetectedDeviceCallbacks);
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(200); 
    pBLEScan->setWindow(199); 


}

void loop() {

    // mqtt connection verification 
    if (!mqttClient.connected()) {
        reconnect_mqtt();
    }
    mqttClient.loop();


    // BLE scan launching at 100 ms
    Serial.printf("Scanning BLE (%dms)...", SCAN_TIME);
    pBLEScan->start(0,nullptr,false);
    delay(SCAN_TIME); 
    pBLEScan->stop(); 
    pBLEScan->clearResults(); //clearing memory

    // creating JSON packet for RSSI data from the anchors
    StaticJsonDocument<200> document; 
    document["Anchor_1"] = rssi_anchor_1;
    document["Anchor_2"] = rssi_anchor_2;
    document["Anchor_3"] = rssi_anchor_3; 

    char jsonBuffer[512]; 
    serializeJson(document, jsonBuffer); 

    // sending to mqtt server 

    Serial.print("Publication MQTT: "); 
    Serial.println(jsonBuffer);
    mqttClient.publish("PDR/robot/rssi",jsonBuffer);

    //resetting anchor rssi to very very far 
    rssi_anchor_1 = -100;
    rssi_anchor_2 = -100;
    rssi_anchor_3 = -100; 

}