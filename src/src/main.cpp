// Basic demo for readings from Adafruit EMC2101
#include <Arduino.h>
#include <Adafruit_EMC2101.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include "esp_wifi.h"

WiFiClient client;
PubSubClient mqttClient;

void Wifi_setup();
void WiFiEvent(WiFiEvent_t event);
void MQTT_setup();
void checkMQTTconnection();
void mqttMessageReceived(char *topic, byte *payload, unsigned int length);
void loopMQTT();

#define MQTT_SERVER "robotmqtt"
#define MQTT_PORT 1883 
#define MQTT_USER "public" 
#define MQTT_PASSWORD "public" 
#define MQTT_TOPIC_FAN_1 "cabinet/fans/1/rpm" 
#define MQTT_TOPIC_FAN_2 "cabinet/fans/2/rpm" 
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

  if (strcmp(topic, MQTT_TOPIC_FAN_1) == 0)
  {

    uint8_t pwm = std::stoi(receivedMsg);

    emc2101_1.setDutyCycle(pwm);
  }

    if (strcmp(topic, MQTT_TOPIC_FAN_2) == 0)
  {

    uint8_t pwm = std::stoi(receivedMsg);

    emc2101_2.setDutyCycle(pwm);
  }
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

      std::string mqtt_client = "CABINET";

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
    mqttClient.subscribe(MQTT_TOPIC_FAN_1);
    mqttClient.subscribe(MQTT_TOPIC_FAN_2);
  }
}

void loopMQTT()
{
  checkMQTTconnection();

  mqttClient.loop();
}


/*
  WiFi Events

  0  SYSTEM_EVENT_WIFI_READY               < ESP32 WiFi ready
  1  SYSTEM_EVENT_SCAN_DONE                < ESP32 finish scanning AP
  2  SYSTEM_EVENT_STA_START                < ESP32 station start
  3  SYSTEM_EVENT_STA_STOP                 < ESP32 station stop
  4  SYSTEM_EVENT_STA_CONNECTED            < ESP32 station connected to AP
  5  SYSTEM_EVENT_STA_DISCONNECTED         < ESP32 station disconnected from AP
  6  SYSTEM_EVENT_STA_AUTHMODE_CHANGE      < the auth mode of AP connected by ESP32 station changed
  7  SYSTEM_EVENT_STA_GOT_IP               < ESP32 station got IP from connected AP
  8  SYSTEM_EVENT_STA_LOST_IP              < ESP32 station lost IP and the IP is reset to 0
  9  SYSTEM_EVENT_STA_WPS_ER_SUCCESS       < ESP32 station wps succeeds in enrollee mode
  10 SYSTEM_EVENT_STA_WPS_ER_FAILED        < ESP32 station wps fails in enrollee mode
  11 SYSTEM_EVENT_STA_WPS_ER_TIMEOUT       < ESP32 station wps timeout in enrollee mode
  12 SYSTEM_EVENT_STA_WPS_ER_PIN           < ESP32 station wps pin code in enrollee mode
  13 SYSTEM_EVENT_AP_START                 < ESP32 soft-AP start
  14 SYSTEM_EVENT_AP_STOP                  < ESP32 soft-AP stop
  15 SYSTEM_EVENT_AP_STACONNECTED          < a station connected to ESP32 soft-AP
  16 SYSTEM_EVENT_AP_STADISCONNECTED       < a station disconnected from ESP32 soft-AP
  17 SYSTEM_EVENT_AP_STAIPASSIGNED         < ESP32 soft-AP assign an IP to a connected station
  18 SYSTEM_EVENT_AP_PROBEREQRECVED        < Receive probe request packet in soft-AP interface
  19 SYSTEM_EVENT_GOT_IP6                  < ESP32 station or ap or ethernet interface v6IP addr is preferred
  20 SYSTEM_EVENT_ETH_START                < ESP32 ethernet start
  21 SYSTEM_EVENT_ETH_STOP                 < ESP32 ethernet stop
  22 SYSTEM_EVENT_ETH_CONNECTED            < ESP32 ethernet phy link up
  23 SYSTEM_EVENT_ETH_DISCONNECTED         < ESP32 ethernet phy link down
  24 SYSTEM_EVENT_ETH_GOT_IP               < ESP32 ethernet got IP from connected AP
  25 SYSTEM_EVENT_MAX
*/

void WiFiEvent(WiFiEvent_t event)
{
    // Serial.printf("[WiFi-event] event: %d\n", event);

    switch (event)
    {
    case SYSTEM_EVENT_WIFI_READY:
        Serial.println("WiFi interface ready");
        break;
    case SYSTEM_EVENT_SCAN_DONE:
        Serial.println("Completed scan for access points");
        break;
    case SYSTEM_EVENT_STA_START:
        Serial.println("WiFi client started");
        break;
    case SYSTEM_EVENT_STA_STOP:
        Serial.println("WiFi clients stopped");
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        Serial.println("Connected to access point");
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("Disconnected from WiFi access point");
        Serial.println("Reconnecting...");

        WiFi.begin("the robot network", "isaacasimov");
        break;
    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
        Serial.println("Authentication mode of access point has changed");
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.print("Obtained IP address: ");
        Serial.print(WiFi.localIP());

        Serial.print(" in ");
        Serial.print(millis());
        Serial.println("ms");

        break;
    case SYSTEM_EVENT_STA_LOST_IP:
        Serial.println("Lost IP address and IP address is reset to 0");
        break;
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
        Serial.println("WiFi Protected Setup (WPS): succeeded in enrollee mode");
        break;
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
        Serial.println("WiFi Protected Setup (WPS): failed in enrollee mode");
        break;
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
        Serial.println("WiFi Protected Setup (WPS): timeout in enrollee mode");
        break;
    case SYSTEM_EVENT_STA_WPS_ER_PIN:
        Serial.println("WiFi Protected Setup (WPS): pin code in enrollee mode");
        break;
    case SYSTEM_EVENT_AP_START:
        Serial.println("WiFi access point started");
        break;
    case SYSTEM_EVENT_AP_STOP:
        Serial.println("WiFi access point  stopped");
        break;
    case SYSTEM_EVENT_AP_STACONNECTED:
        Serial.println("Client connected");
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        Serial.println("Client disconnected");
        break;
    case SYSTEM_EVENT_AP_STAIPASSIGNED:
        Serial.println("Assigned IP address to client");
        break;
    case SYSTEM_EVENT_AP_PROBEREQRECVED:
        Serial.println("Received probe request");
        break;
    case SYSTEM_EVENT_GOT_IP6:
        Serial.println("IPv6 is preferred");
        break;
    case SYSTEM_EVENT_ETH_START:
        Serial.println("Ethernet started");
        break;
    case SYSTEM_EVENT_ETH_STOP:
        Serial.println("Ethernet stopped");
        break;
    case SYSTEM_EVENT_ETH_CONNECTED:
        Serial.println("Ethernet connected");
        break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
        Serial.println("Ethernet disconnected");
        break;
    case SYSTEM_EVENT_ETH_GOT_IP:
        Serial.println("Obtained IP address");
        break;
    default:
        break;
    }
}

void Wifi_setup()
{
    // register wifi events
    WiFi.onEvent(WiFiEvent);

    WiFi.begin("the robot network", "isaacasimov");
}
