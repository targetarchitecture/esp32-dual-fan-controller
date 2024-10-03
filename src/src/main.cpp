// Basic demo for readings from Adafruit EMC2101
#include <Arduino.h>
#include <Adafruit_EMC2101.h>
#include <Wire.h>
#include <PubSubClient.h>
#include "IoT.h"
#include "WifiMgr.h"

extern PubSubClient mqttClient;

#define MQTT_TOPIC_1 "cabinet/fans/1/speed" // Replace with your desired topic
#define MQTT_TOPIC_2 "cabinet/fans/2/speed" // Replace with your desired topic

// Adafruit_EMC2101  emc2101_1;
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

  Serial.println("Adafruit EMC2101 test!");

  Wifi_setup();
  MQTT_setup();

  Wire.begin(SDA_1, SCL_1);
  Wire1.begin(SDA_2, SCL_2);

  // Try to initialize!
  // if (!emc2101_1.begin(0x4C,&Wire)) {
  //   Serial.println("Failed to find EMC2101 1 chip");
  //   while (1) { delay(10); }
  // }
  // Serial.println("EMC2101 1 Found!");

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

  Serial.print("Data rate set to: ");
  switch (emc2101_2.getDataRate())
  {
  case EMC2101_RATE_1_16_HZ:
    Serial.println("1/16_HZ");
    break;
  case EMC2101_RATE_1_8_HZ:
    Serial.println("1/8_HZ");
    break;
  case EMC2101_RATE_1_4_HZ:
    Serial.println("1/4_HZ");
    break;
  case EMC2101_RATE_1_2_HZ:
    Serial.println("1/2_HZ");
    break;
  case EMC2101_RATE_1_HZ:
    Serial.println("1 HZ");
    break;
  case EMC2101_RATE_2_HZ:
    Serial.println("2 HZ");
    break;
  case EMC2101_RATE_4_HZ:
    Serial.println("4 HZ");
    break;
  case EMC2101_RATE_8_HZ:
    Serial.println("8 HZ");
    break;
  case EMC2101_RATE_16_HZ:
    Serial.println("16 HZ");
    break;
  case EMC2101_RATE_32_HZ:
    Serial.println("32 HZ");
    break;
  }

  emc2101_2.enableTachInput(true);
  emc2101_2.setPWMDivisor(0);
  emc2101_2.setDutyCycle(50);
}

void loop()
{
  Serial.print("Internal Temperature: ");
  Serial.print(emc2101_2.getInternalTemperature());
  Serial.println(" degrees C");

  Serial.print("Duty Cycle: ");
  Serial.print(emc2101_2.getDutyCycle());
  Serial.print("% / Fan RPM: ");
  Serial.print(emc2101_2.getFanRPM());
  Serial.println(" RPM");
  Serial.println("");



  loopMQTT();

    mqttClient.publish(MQTT_TOPIC_1, "50");
  mqttClient.publish(MQTT_TOPIC_2, "50");

  delay(1000);
}