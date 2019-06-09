#include <Arduino.h>
#include <EEPROM.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>

/*------------------------------------------------------------------------------------*/
/* Constant Definitions                                                               */
/*------------------------------------------------------------------------------------*/
// Access point to configure Wi-Fi
const char* ACCESS_POINT_NAME = "ESP8266";
const char* ACCESS_POINT_PASS = "esp8266";

// MQTT constants
const char * MQTT_CLIENT_PREFIX = "MoistSensor-";
const char * MQTT_BROKER_ADDRESS = "192.168.1.215";
const char * MQTT_IN_TOPIC = "/home-assistant/moist/request";

// MQTT Commands
const char MQTT_CMD_KEEP_AWAKE = 'a';   // Keep away for OTA update
const char MQTT_CMD_DEEP_SLEEP = 's';   // Go back to deep sleep

// MQTT Events
const char * MQTT_REPORT_MOISTURE = "/home-assistant/moist/moist";

// Moisture sensor values normalization constants
const uint16_t MAX_SENSOR_VALUE = 900;
const uint16_t MIN_SENSOR_VALUE = 400;
const uint16_t NORM_SENSOR_RANGE = MAX_SENSOR_VALUE - MIN_SENSOR_VALUE;

const uint16_t DRYNESS_LOW = 600;
const uint16_t DRYNESS_HIGH = 700;

// Deep Sleep Period
const uint64_t DEEP_SLEEP_PERIOD = 20e6; // 20 seconds

/*------------------------------------------------------------------------------------*/
/* GPIO Definitions                                                                   */
/*------------------------------------------------------------------------------------*/
const uint8_t GPIO_ANALOG_00 = 0;          // ESP8266 NodeMCU A0
const uint8_t GPIO_UNUSED_00 = 0;          // ESP8266 NodeMCU D3
const uint8_t GPIO_UNUSED_01 = 2;          // ESP8266 NodeMUC D4 (UART)
const uint8_t GPIO_UNUSED_02 = 2;          // ESP8266 NodeMUC D4 (Boot mode. Do not user for INPUT)
const uint8_t GPIO_UNUSED_03 = 3;          // ESP8266 NodeMCU D9 (UART)
const uint8_t GPIO_DISPLAY_SDA = 4;        // ESP8266 NodeMCU D2 (SDA) 
const uint8_t GPIO_DISPLAY_SCL = 5;        // ESP8266 NodeMCU D1 (SCL)
const uint8_t GPIO_UNUSED_06 = 6;          // ESP8266 NodeMCU -+ F M
const uint8_t GPIO_UNUSED_07 = 7;          // ESP8266 NodeMCU  + L E
const uint8_t GPIO_UNUSED_08 = 8;          // ESP8266 NodeMCU  + A M
const uint8_t GPIO_UNUSED_09 = 9;          // ESP8266 NodeMCU  + S O
const uint8_t GPIO_UNUSED_10 = 10;         // ESP8266 NodeMCU  + H R
const uint8_t GPIO_UNUSED_11 = 11;         // ESP8266 NodeMCU -+   Y
const uint8_t GPIO_RGB_LED_RED = 12;       // ESP8266 NodeMCU D6
const uint8_t GPIO_RGB_LED_BLUE = 13;      // ESP8266 NodeMUC D7
const uint8_t GPIO_RGB_LED_GREEN = 14;     // ESP8266 NodeMCU D5
const uint8_t GPIO_UNUSED_15 = 15;         // ESP8266 NodeMCU D8 (Boot from SD Card)
const uint8_t GPIO_UNUSED_16 = 16;         // ESP8266 NodeMCU D0

const uint8_t GPIO_TOO_DRY = GPIO_RGB_LED_RED;
const uint8_t GPIO_MOIST = GPIO_RGB_LED_GREEN;
const uint8_t GPIO_TOO_WET = GPIO_RGB_LED_BLUE;
const uint8_t GPIO_MOIST_SENSOR = GPIO_ANALOG_00;

/*------------------------------------------------------------------------------------*/
/* Global Variables                                                                   */
/*------------------------------------------------------------------------------------*/
// WiFi Manager
WiFiManager wifiManager;

// MQTT
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Prevent Deep Sleep to Allow OTA updates
bool deepSleep = true;

/*------------------------------------------------------------------------------------*/
/* WiFi Manager Global Functions                                                      */
/*------------------------------------------------------------------------------------*/
// WiFiManager Configuration CallBack
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("[WIFI]: Entered config mode");
  Serial.print("[WIFI]:"); Serial.println(WiFi.softAPIP());
  Serial.printf("[WIFI]: %s", (myWiFiManager->getConfigPortalSSID()).c_str());
}

void checkOTA(void) {
  Serial.println("[OTA]: Checking OTA...");
  for (uint8_t i = 0; i < 20; i++) {
    mqttClient.loop();
    delay(100);
  }
  Serial.println("[OTA]: End checking OTA...");
}

uint8_t calculateMoistPercent(uint16_t dryness) {
  // Normalize
  dryness = (dryness > MAX_SENSOR_VALUE ? MAX_SENSOR_VALUE : dryness);
  dryness = (dryness < MIN_SENSOR_VALUE ? MIN_SENSOR_VALUE : dryness);
  uint8_t percent = 100 - (dryness - MIN_SENSOR_VALUE) / (NORM_SENSOR_RANGE / 100);
  return percent;
}

void processSensorRead(uint16_t dryness) {
  Serial.printf("[MOIST]: Dryness: %u\n", dryness);
  uint8_t moistPercent = calculateMoistPercent(dryness);
  // Report to MQTT broker
  char payload[20];
  sprintf(payload, "%u", moistPercent);
  Serial.printf("[MOIST]: Reporting moisture. Moisture: %s%%\n", payload);
  mqttClient.publish(MQTT_REPORT_MOISTURE, payload);
  if (dryness > DRYNESS_HIGH) {
    digitalWrite(GPIO_TOO_DRY, LOW);
  } else if (dryness < DRYNESS_LOW) {
    digitalWrite(GPIO_TOO_WET, LOW);
  } else {
    digitalWrite(GPIO_MOIST, LOW);
  }
}

// MQTT Subscribe Callback
void callback(char* topic, byte* payload, uint8_t length) {
  Serial.printf("[MQTT]: Message arrived [%s]\n", topic);
  Serial.print("[MQTT]: Payload (");
  for (uint8_t i=0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println(")");
  switch((char) payload[0]) {
    case MQTT_CMD_KEEP_AWAKE:
      if (deepSleep) {
        deepSleep = false;
      }
      break;
    case MQTT_CMD_DEEP_SLEEP:
      if (!deepSleep) {
        deepSleep = true;
        Serial.println("[MQTT]: Going to sleep now...");
        delay(1000);
        ESP.deepSleep(DEEP_SLEEP_PERIOD);
      }
      break;
    default: 
      Serial.printf("[MQTT]: Unknown MQTT Command: %c\n", (char) payload[0]);
      break;
  }
}

// MQTT Client reconnection
void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.println("[MQTT]: Attempting MQTT connection...");
    // Create a random client ID
    String clientId = MQTT_CLIENT_PREFIX;
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("[MQTT]: Connected");
      // ... and resubscribe
      mqttClient.subscribe(MQTT_IN_TOPIC);
    } else {
      Serial.printf("[MQTT]: Failed, rc= %d, try again in 5 seconds\n", mqttClient.state());
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Instantiate and setup WiFiManager
  // wifiManager.resetSettings(); Uncomment to reset wifi settings
  wifiManager.setAPCallback(configModeCallback);
  if (!wifiManager.autoConnect(ACCESS_POINT_NAME, ACCESS_POINT_PASS)) {
    Serial.println("Failed to connect and hit timeout");
    ESP.reset();
    delay(1000);  
  }

  // Setup MQTT client
  mqttClient.setServer(MQTT_BROKER_ADDRESS, 1883);
  mqttClient.setCallback(callback);

  // MQTT
  if (!mqttClient.connected()) {
    reconnect();
  }
  
  // Config time
  setenv("TZ", "EST5EDT,M3.2.0/02:00:00,M11.1.0/02:00:00", 1);
  configTime(0, 0, "pool.ntp.org");

  // Initialize OTA (Over the air) update
  ArduinoOTA.setHostname(ACCESS_POINT_NAME);
  ArduinoOTA.setPassword(ACCESS_POINT_PASS);

  ArduinoOTA.onStart([]() {
    Serial.println("[OTA]: Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("[OTA]: End");
  });
  ArduinoOTA.onProgress([](uint32_t progress, uint32_t total) {
    Serial.printf("[OTA]: Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA]: Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("[OTA]: Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("[OTA]: Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("[OTA]: Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("[OTA]: Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("[OTA]: End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("[OTA]: Ready");

  // GPIO Setup
  pinMode(GPIO_TOO_DRY, OUTPUT);
  pinMode(GPIO_TOO_WET, OUTPUT);
  pinMode(GPIO_MOIST, OUTPUT);
  digitalWrite(GPIO_TOO_DRY, HIGH);
  digitalWrite(GPIO_TOO_WET, HIGH);
  digitalWrite(GPIO_MOIST, HIGH);

  // Read moisture sensor and process data
  processSensorRead(analogRead(GPIO_MOIST_SENSOR));
  
  checkOTA();

  digitalWrite(GPIO_TOO_DRY, HIGH);
  digitalWrite(GPIO_TOO_WET, HIGH);
  digitalWrite(GPIO_MOIST, HIGH);

  // Deep Sleep
  if (deepSleep) {
    ESP.deepSleep(DEEP_SLEEP_PERIOD);
  }

}

void loop() {
  if (!deepSleep) {
    // OTA
    ArduinoOTA.handle();

    // MQTT
    if (!mqttClient.connected()) {
      reconnect();
    }
    mqttClient.loop();

    // Indicate ready for OTA update
    digitalWrite(GPIO_MOIST, LOW);
  }
}