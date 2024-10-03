#include <Arduino.h>
#include "IoT.h"

WiFiClient client;
PubSubClient mqttClient;

void MQTT_setup()
{
  // set this up as early as possible
  mqttClient.setClient(client);
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttMessageReceived);
}

void mqttMessageReceived(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.println("]");

  std::string receivedMsg;

  for (int i = 0; i < length; i++)
  {
    char c = payload[i];

    receivedMsg += c;
  }

  Serial.print("Message:");
  Serial.println(receivedMsg.c_str());
}

void checkMQTTconnection()
{
  if (mqttClient.connected() == false)
  {
    Serial.println("MQTTClient NOT Connected :(");
    u_long startTime = millis();

    do
    {
      if (mqttClient.connected() == true)
      {
        break;
      }

      // get the unique id into a variable
      // String wifiMacString = WiFi.macAddress();
      std::string mqtt_client = "CABINET";
      // mqtt_client = mqtt_client + "_" + wifiMacString.c_str();

      Serial.print(mqtt_client.c_str());
      Serial.print(":");
      Serial.print(MQTT_USER);
      Serial.print(":");
      Serial.println(MQTT_PASSWORD);

      mqttClient.connect(mqtt_client.c_str(), MQTT_USER, MQTT_PASSWORD);

      delay(500);

    } while (1);

    Serial.print("MQTTClient now connected in ");
    Serial.print(millis() - startTime);
    Serial.println("ms :)");

    // set to true to get the subscriptions setup again
    //mqttClient.subscribe(MQTT_TOPIC_1);
    //mqttClient.subscribe(MQTT_TOPIC_2);
    mqttClient.subscribe(MQTT_TOPIC_3);
    mqttClient.subscribe(MQTT_TOPIC_4);
    mqttClient.subscribe(MQTT_TOPIC_5);
    mqttClient.subscribe(MQTT_TOPIC_6);
  }
}

void loopMQTT()
{
  checkMQTTconnection();

  mqttClient.loop();
}