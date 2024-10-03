#pragma once

#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_wifi.h"

void MQTT_setup();
void checkMQTTconnection();
void mqttMessageReceived(char *topic, byte *payload, unsigned int length);
void loopMQTT();

#define MQTT_SERVER "robotmqtt" // Replace with your MQTT server address  
#define MQTT_PORT 1883 // Replace with your MQTT server port  

#define MQTT_TOPIC_3 "cabinet/fans/1/rpm" // Replace with your desired topic
#define MQTT_TOPIC_4 "cabinet/fans/2/rpm" // Replace with your desired topic
#define MQTT_TOPIC_5 "cabinet/temperature/1" // Replace with your desired topic
#define MQTT_TOPIC_6 "cabinet/temperature/2" // Replace with your desired topic

#define MQTT_USER "public" // Replace with your desired topic
#define MQTT_PASSWORD "public" // Replace with your desired topic

