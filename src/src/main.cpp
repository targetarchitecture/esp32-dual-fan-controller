// Basic demo for readings from Adafruit EMC2101
#include <Arduino.h>
#include <Adafruit_EMC2101.h>
#include <Wire.h>
#include <PubSubClient.h>
#include "IoT.h"
#include "WifiMgr.h"

extern PubSubClient mqttClient;

#define MQTT_TOPIC_FAN_1_SPEED "cabinet/fans/1/speed"
#define MQTT_TOPIC_FAN_2_SPEED "cabinet/fans/2/speed"
#define MQTT_TOPIC_TEMP_1 "cabinet/temperature/1"
#define MQTT_TOPIC_TEMP_2 "cabinet/temperature/2"

Adafruit_EMC2101 emc2101_1;
Adafruit_EMC2101 emc2101_2;

#define SDA_1 27
#define SCL_1 26

#define SDA_2 33
#define SCL_2 32

void setup(void)
{
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Cabinet fans: EMC2101!");

  Wifi_setup();
  MQTT_setup();

  Wire.begin(SDA_1, SCL_1);
  Wire1.begin(SDA_2, SCL_2);

  // Try to initialize!
  if (!emc2101_1.begin(0x4C, &Wire))
  {
    Serial.println("Failed to find EMC2101 1 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("EMC2101 1 Found!");

  // Try to initialize!
  if (!emc2101_2.begin(0x4C, &Wire1))
  {
    Serial.println("Failed to find EMC2101 2 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("EMC2101 2 Found!");

  emc2101_2.setDataRate(EMC2101_RATE_1_HZ);
  emc2101_2.enableTachInput(true);
  emc2101_2.setPWMDivisor(0);
  emc2101_2.setDutyCycle(50);

  emc2101_1.setDataRate(EMC2101_RATE_1_HZ);
  emc2101_1.enableTachInput(true);
  emc2101_1.setPWMDivisor(0);
  emc2101_1.setDutyCycle(50);
}

void loop()
{
  loopMQTT();

  std::string FanRPM1 = std::to_string(emc2101_1.getFanRPM());
  std::string FanRPM2 = std::to_string(emc2101_2.getFanRPM());

  mqttClient.publish(MQTT_TOPIC_FAN_1_SPEED, FanRPM1.c_str());
  mqttClient.publish(MQTT_TOPIC_FAN_2_SPEED, FanRPM2.c_str());

  std::string temp1 = std::to_string(emc2101_1.getInternalTemperature());
  std::string temp2 = std::to_string(emc2101_2.getInternalTemperature());

  mqttClient.publish(MQTT_TOPIC_TEMP_1, temp1.c_str());
  mqttClient.publish(MQTT_TOPIC_TEMP_2, temp2.c_str());

  delay(1000);
}