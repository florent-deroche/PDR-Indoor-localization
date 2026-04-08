#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEScan.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h> 
#include <BLEAdvertisedDevice.h>
#include <math.h> 

#include "shared_config.h"

WiFiClient espClient; 
PubSubClient mqttClient(espClient);
BLEScan* pBLEScan; 

// Listen during 500ms to catch multiple packets 
const int SCAN_TIME = 500; 

// --- ANCHOR BUFFERING SIZE FOR MEAN  ---

const int MAX_SAMPLES = 25; 

struct AnchorBuffer {
    int samples[MAX_SAMPLES];
    int count;

    void reset() { count = 0; }

    void add(int rssi) {
        // Short Filter 
        if (rssi <= -95) return; 

        if (count < MAX_SAMPLES) {
            samples[count++] = rssi;
        }
    }

    // Return Mean or -100 if no valid measurements
    int average() const {
        if (count == 0) return -100;
        long sum = 0;
        for (int i = 0; i < count; i++) sum += samples[i];
        
        
        return (int)round((float)sum / count);
    }
};


volatile bool scan_in_progress = false;
AnchorBuffer buf_anchor_1;
AnchorBuffer buf_anchor_2;
AnchorBuffer buf_anchor_3;

class DetectedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice detectedDevice) {
        if (!detectedDevice.haveName()) return;
        if (!scan_in_progress) return;

        String deviceName = detectedDevice.getName().c_str();
        int rssi = detectedDevice.getRSSI();

        // We store all multiple packets in buffers 
        if (deviceName == ANCHOR_1_NAME)      buf_anchor_1.add(rssi);
        else if (deviceName == ANCHOR_2_NAME) buf_anchor_2.add(rssi);
        else if (deviceName == ANCHOR_3_NAME) buf_anchor_3.add(rssi);
    }
};

void setup_wifi() {
    delay(10);
    Serial.println("Connexion WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connecté ! IP : ");
    Serial.println(WiFi.localIP());
}

void reconnect_mqtt() {
    while (!mqttClient.connected()) {
        Serial.print("Connexion MQTT...");
        String clientId = "RobotESP32-1";
        if (mqttClient.connect(clientId.c_str())) {
            Serial.println("Connecté au broker MQTT !");
        } else {
            Serial.print("Échec, erreur : ");
            Serial.print(mqttClient.state());
            Serial.println(" — Retry dans 5s");
            delay(5000);
        }
    }
}

void setup() {
    Serial.begin(115200);
    setup_wifi();
    mqttClient.setServer(mqtt_server, mqtt_port);

    Serial.println("Initialisation BLE Scanner...");
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new DetectedDeviceCallbacks);
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(200); 
    pBLEScan->setWindow(199);   // Robot is listening 99% of the time 
}

void loop() {
    if (!mqttClient.connected()) reconnect_mqtt();
    mqttClient.loop();

    // Buffer Reset before scanning 
    buf_anchor_1.reset();
    buf_anchor_2.reset();
    buf_anchor_3.reset();

    // BLE scan 
    Serial.printf("Scan BLE (%dms)...\n", SCAN_TIME);
    scan_in_progress = true;
    pBLEScan->start(0, nullptr, false);
    delay(SCAN_TIME);
    pBLEScan->stop();
    scan_in_progress = false;  
    pBLEScan->clearResults();

    // Mean calculation 
    int rssi_1 = buf_anchor_1.average();
    int rssi_2 = buf_anchor_2.average();
    int rssi_3 = buf_anchor_3.average();

    
    Serial.printf("Anchor_1: %d dBm (%d samples) | Anchor_2: %d dBm (%d samples) | Anchor_3: %d dBm (%d samples)\n",
                  rssi_1, buf_anchor_1.count,
                  rssi_2, buf_anchor_2.count,
                  rssi_3, buf_anchor_3.count);

    // JSON Publisher
    StaticJsonDocument<200> document;
    document["Anchor_1"] = rssi_1;
    document["Anchor_2"] = rssi_2;
    document["Anchor_3"] = rssi_3;

    char jsonBuffer[512];
    serializeJson(document, jsonBuffer);

    Serial.print("Publication MQTT : ");
    Serial.println(jsonBuffer);
    mqttClient.publish("PDR/robot/rssi", jsonBuffer);
}