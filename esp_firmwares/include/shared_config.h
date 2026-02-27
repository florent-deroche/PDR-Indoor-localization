#ifndef SHARED_CONFIG_H
#define SHARED_CONFIG_H


// we define here all the shared config off the esp32 like Names of the diff anchor etc... to make ble transmitting easier 
#define ANCHOR_1_NAME "ESP32_Anchor_ID:1"
#define ANCHOR_2_NAME "ESP32_Anchor_ID:2"  
#define ANCHOR_3_NAME "ESP32_Anchor_ID:3"

// wifi and mqtt config
#define ssid "Netsphere_Terminal"
#define password "goon1234"

// MQTT config
#define mqtt_port 1883
#define mqtt_server "10.36.126.210"

#endif  