#pragma once

#include <Adafruit_EMC2101.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_wifi.h"

void MQTT_setup();
void checkMQTTconnection();
void mqttMessageReceived(char *topic, byte *payload, unsigned int length);
void loopMQTT();

extern Adafruit_EMC2101  emc2101_1;
extern Adafruit_EMC2101 emc2101_2;

#define MQTT_SERVER "robotmqtt"
#define MQTT_PORT 1883 

#define MQTT_TOPIC_FAN_1 "cabinet/fans/1/rpm" 
#define MQTT_TOPIC_FAN_2 "cabinet/fans/2/rpm" 


#define MQTT_USER "public" 
#define MQTT_PASSWORD "public" 

