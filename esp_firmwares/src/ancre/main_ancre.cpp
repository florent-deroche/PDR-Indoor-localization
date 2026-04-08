##include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "shared_config.h"
#include "esp_bt.h"

/*
Warning, change this ANCHOR_ID to 1, 2 or 3 depending on the anchor you are flashing the firmware in.
Each Anchor must have a different Anchor Id
*/
#define ANCHOR_ID 2

#define BLUE_LED 2

void setup() {
    Serial.begin(115200);
    Serial.println("BLE Anchor starting..");
    
    // blue led init for debug
    pinMode(BLUE_LED, OUTPUT);

    String deviceName = "ESP32_Anchor_ID:" + String(ANCHOR_ID);
    
    // init of the bluetooth peripheral 
    BLEDevice::init(deviceName.c_str());
    
    // emitter config 
    BLEServer *pServer = BLEDevice::createServer();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

    // UUID Generic service 
    pAdvertising->addServiceUUID(BLEUUID((uint16_t)0x1801));

    // signal full power and visibility 
    pAdvertising->setScanResponse(true);
 
    // Standart signal power so the BLE doesn't bounce a lot on walls 
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P3);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P3); // +3dbm
   
    // 100ms 
    pAdvertising->setMinInterval(0x90); 
    pAdvertising->setMaxInterval(0xB0);
    // BLE emition start 
    BLEDevice::startAdvertising(); 

    // serial prints 
    Serial.println("§§§-----------------------------§§§");
    Serial.println("Emitting BLE as :" + deviceName);
    Serial.println("§§§-----------------------------§§§");

}

void loop() { 
    // nothing to do here, BLE is running in background task
    // there is no blue integrated led on this esp model 
    digitalWrite(BLUE_LED,HIGH); 
    delay(1000);
    digitalWrite(BLUE_LED,LOW); 
    delay(1000);

}