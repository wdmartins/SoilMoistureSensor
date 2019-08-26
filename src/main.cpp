#include <Arduino.h>
#include <EEPROM.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include "secret.h"

/*------------------------------------------------------------------------------------*/
/* Constant Definitions                                                               */
/*------------------------------------------------------------------------------------*/
// Access point to configure Wi-Fi
const char* ACCESS_POINT_NAME = "ESP8266";
const char* ACCESS_POINT_PASS = "esp8266";

// MQTT constants
const char * MQTT_CLIENT_PREFIX = "MoistSensor-";
const char * MQTT_IN_TOPIC_RAW = "/home-assistant/moist/%c/request";

// MQTT Moist Client Id. Every Moisture Sensor in the MQTT network must have a different id
const char MQTT_MOIST_CLIENT_ID = 'A';

// MQTT Commands
const char MQTT_CMD_KEEP_AWAKE = 'a';   // Keep awake for OTA update
const char MQTT_CMD_DEEP_SLEEP = 's';   // Go back to deep sleep
const char MQTT_CMD_DEEP_TEST = 't';    // Run hardware test
const char MQTT_CMD_DEEP_RANGE = 'r';   // wet-dry Range set

// MQTT Events
const char * MQTT_REPORT_MOISTURE_RAW = "/home-assistant/moist/%c/moist";
const char * MQTT_REPORT_TEST_ENDED_RAW = "/home-assistant/moist/%c/testended";
const char * MQTT_REPORT_RANGE_RAW = "/home-assistant/moist/%c/range";
const char * MQTT_OTA_READY_RAW = "/home-assistant/moist/%c/otaready";

// Moisture sensor values normalization constants
const uint16_t MAX_SENSOR_VALUE = 600;
const uint16_t MIN_SENSOR_VALUE = 300;
const uint16_t NORM_SENSOR_RANGE = MAX_SENSOR_VALUE - MIN_SENSOR_VALUE;

const uint16_t DRYNESS_LOW = 400;
const uint16_t DRYNESS_HIGH = 500;

// Deep Sleep Period
const uint64_t DEEP_SLEEP_PERIOD_TEST = 20e6;   // 20 seconds
const uint64_t DEEP_SLEEP_PERIOD_PROD = 3600e6; // 1 hour
const uint64_t DEEP_SLEEP_PERIOD = DEEP_SLEEP_PERIOD_PROD;
const uint16_t MAX_SLEEP_PERIOD_WITHOUT_REPORTING = 3;
const uint16_t MAX_PERCENT_POINTS_WITHOUT_REPORTING = 5;

/*------------------------------------------------------------------------------------*/
/* GPIO Definitions                                                                   */
/*------------------------------------------------------------------------------------*/
const uint8_t GPIO_ANALOG_00 = 0;          // ESP8266 NodeMCU A0
const uint8_t GPIO_UNUSED_00 = 0;          // ESP8266 NodeMCU D3
const uint8_t GPIO_UNUSED_01 = 1;          // ESP8266 NodeMUC TX (UART)
const uint8_t GPIO_UNUSED_02 = 2;          // ESP8266 NodeMUC D4 (Boot mode. Do not user for INPUT)
const uint8_t GPIO_UNUSED_03 = 3;          // ESP8266 NodeMCU RX (UART)
const uint8_t GPIO_DISPLAY_SDA = 4;        // ESP8266 NodeMCU D2 (SDA) 
const uint8_t GPIO_DISPLAY_SCL = 5;        // ESP8266 NodeMCU D1 (SCL)
const uint8_t GPIO_UNUSED_06 = 6;          // ESP8266 NodeMCU -+ F M
const uint8_t GPIO_UNUSED_07 = 7;          // ESP8266 NodeMCU  + L E
const uint8_t GPIO_UNUSED_08 = 8;          // ESP8266 NodeMCU  + A M
const uint8_t GPIO_UNUSED_09 = 9;          // ESP8266 NodeMCU  + S O
const uint8_t GPIO_UNUSED_10 = 10;         // ESP8266 NodeMCU  + H R
const uint8_t GPIO_UNUSED_11 = 11;         // ESP8266 NodeMCU -+   Y
const uint8_t GPIO_RGB_LED_GREEN = 12;     // ESP8266 NodeMCU D6
const uint8_t GPIO_RGB_LED_BLUE = 13;      // ESP8266 NodeMCU D7
const uint8_t GPIO_RGB_LED_RED = 14;       // ESP8266 NodeMUC D5
const uint8_t GPIO_UNUSED_15 = 15;         // ESP8266 NodeMCU D8 (Boot from SD Card)
const uint8_t GPIO_UNUSED_16 = 16;         // ESP8266 NodeMCU D0

const uint8_t GPIO_TOO_DRY = GPIO_RGB_LED_RED;
const uint8_t GPIO_MOIST = GPIO_RGB_LED_GREEN;
const uint8_t GPIO_TOO_WET = GPIO_RGB_LED_BLUE;
const uint8_t GPIO_MOIST_SENSOR = GPIO_ANALOG_00;

/*------------------------------------------------------------------------------------*/
/* Forward Declarations                                                               */
/*------------------------------------------------------------------------------------*/
uint8_t calculateMoistPercent(uint16_t dryness);

/*------------------------------------------------------------------------------------*/
/* Global Variables                                                                   */
/*------------------------------------------------------------------------------------*/
// RTC Data Structure
struct {
  uint32_t crc32;
  struct {
    uint16_t lastReadValue;
    uint16_t periodCount;
  } data;
} rtcData;

// WiFi Manager
WiFiManager wifiManager;

// MQTT
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Prevent Deep Sleep to Allow OTA updates
bool deepSleep = true;

// Default Mositure Range
uint8_t tooWet = calculateMoistPercent(DRYNESS_LOW);
uint8_t tooDry = calculateMoistPercent(DRYNESS_HIGH);

// Prevent reporting in loop
bool rangeReported = false;

char MQTT_IN_TOPIC[64];
char MQTT_REPORT_MOISTURE[64];
char MQTT_REPORT_TEST_ENDED[64];
char MQTT_REPORT_RANGE[64];
char MQTT_OTA_READY[64];

void initMqttTopics(void) {
  sprintf(MQTT_IN_TOPIC, MQTT_IN_TOPIC_RAW, MQTT_MOIST_CLIENT_ID);
  sprintf(MQTT_REPORT_MOISTURE, MQTT_REPORT_MOISTURE_RAW, MQTT_MOIST_CLIENT_ID);
  sprintf(MQTT_REPORT_TEST_ENDED, MQTT_REPORT_TEST_ENDED_RAW, MQTT_MOIST_CLIENT_ID);
  sprintf(MQTT_REPORT_RANGE, MQTT_REPORT_RANGE_RAW, MQTT_MOIST_CLIENT_ID);
  sprintf(MQTT_OTA_READY, MQTT_OTA_READY_RAW, MQTT_MOIST_CLIENT_ID);
}
/*------------------------------------------------------------------------------------*/
/* Global Functions                                                                   */
/*------------------------------------------------------------------------------------*/
// WiFiManager Configuration CallBack
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("[WIFI]: Entered config mode");
  Serial.print("[WIFI]:"); Serial.println(WiFi.softAPIP());
  Serial.printf("[WIFI]: %s", (myWiFiManager->getConfigPortalSSID()).c_str());
}

// Calculate CRC32 to validate RTC stored data
uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}

// Save data to RTC storage (Deep Sleep surviving)
void saveRtcData(void) {
  // Update CRC32 of data
  rtcData.crc32 = calculateCRC32((uint8_t*) &rtcData.data, sizeof(rtcData.data));
  // Write struct to RTC memory
  ESP.rtcUserMemoryWrite(0, (uint32_t*) &rtcData, sizeof(rtcData));
}

// Report set moisture range (too wet/too dry) through MQTT
void reportRange(void) {
  char payload[64];
  sprintf(payload, "From: %02u%% to: %02u%%", tooDry, tooWet);
  Serial.printf("[MOIST] Reporting range [%s]", payload);
  mqttClient.publish(MQTT_REPORT_RANGE, payload);
}

// Check is system should stay awake for OTA update
void checkOTA(void) {
  Serial.println("[OTA]: Checking OTA...");
  for (uint8_t i = 0; i < 20; i++) {
    mqttClient.loop();
    delay(100);
  }
  Serial.println("[OTA]: End checking OTA...");
}

// Calculate moisture percentage base on dryness value provide by sensor
uint8_t calculateMoistPercent(uint16_t dryness) {
  // Normalize
  dryness = (dryness > MAX_SENSOR_VALUE ? MAX_SENSOR_VALUE : dryness);
  dryness = (dryness < MIN_SENSOR_VALUE ? MIN_SENSOR_VALUE : dryness);
  uint8_t percent = 100 - (dryness - MIN_SENSOR_VALUE) / (NORM_SENSOR_RANGE / 100);
  return percent;
}

// Run system test
void runTest(void) {
  digitalWrite(GPIO_TOO_DRY, LOW);
  digitalWrite(GPIO_TOO_WET, LOW);
  digitalWrite(GPIO_MOIST, LOW);

  digitalWrite(GPIO_TOO_DRY, HIGH);
  delay(1000);
  digitalWrite(GPIO_TOO_DRY, LOW);
  digitalWrite(GPIO_TOO_WET, HIGH);
  delay(1000);
  digitalWrite(GPIO_TOO_WET, LOW);
  digitalWrite(GPIO_MOIST, HIGH);
  delay(1000);
  digitalWrite(GPIO_MOIST, LOW);

  uint16_t value = analogRead(GPIO_MOIST_SENSOR);
  Serial.printf("[MOIST]: Dryness Read %u\n", value);
}

// Read moisture sensor value
uint16_t readSensor(void) {
  // Read three times and report average
  uint16_t read1 = analogRead(GPIO_MOIST_SENSOR);
  delay(500);
  uint16_t read2 = analogRead(GPIO_MOIST_SENSOR);
  delay(500);
  uint16_t read3 = analogRead(GPIO_MOIST_SENSOR);
  Serial.printf("[MOIST]: Dryness reads: (%u), (%u), (%u)\n", read1, read2, read3);
  return (read1 + read2 + read3) / 3;
}

// Process sensor value
void processSensorRead(uint16_t moistPercent) {
  if (moistPercent <= tooDry) {
    digitalWrite(GPIO_TOO_DRY, HIGH);
  } else if (moistPercent >= tooWet) {
    digitalWrite(GPIO_TOO_WET, HIGH);
  } else {
    digitalWrite(GPIO_MOIST, HIGH);
  }
}

// Report current moisture value through MQTT
void publishMoisture(uint16_t moistPercent) {
  // Report to MQTT broker
  char payload[20];
  sprintf(payload, "%u", moistPercent);
  Serial.printf("[MOIST]: Reporting moisture. Moisture: %s%%\n", payload);
  mqttClient.publish(MQTT_REPORT_MOISTURE, payload);
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
    case MQTT_CMD_DEEP_TEST:
      Serial.println("[MQTT]: Start test...");
      runTest();
      mqttClient.publish(MQTT_REPORT_TEST_ENDED, "");
      break;
    case MQTT_CMD_DEEP_RANGE:
      Serial.printf("[MQTT]: Received new Wet-Dry range.");
      char aux[16];
      sprintf(aux, "%c%c", payload[1], payload[2]);
      tooDry = atoi(aux);
      sprintf(aux, "%c%c", payload[3], payload[4]);
      tooWet = atoi(aux);
      reportRange();
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
    if (mqttClient.connect(clientId.c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
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
  delay(1000);
  Serial.println("");

  // Check RTC Memory
  if (ESP.rtcUserMemoryRead(0, (uint32_t*) &rtcData, sizeof(rtcData))) {
    uint32_t crcOfData = calculateCRC32((uint8_t*) &rtcData.data, sizeof(rtcData.data));
    Serial.printf("[RTC]: Calculate CRC32: %X\n", crcOfData);
    if (crcOfData != rtcData.crc32) {
      Serial.printf("[RTC]: Store CRC32 (%X) does not match calculated CRC32\n", rtcData.crc32);
      Serial.println("[RTC]: Powering up");
      rtcData.data.periodCount = MAX_SLEEP_PERIOD_WITHOUT_REPORTING;
      rtcData.data.lastReadValue = 0;
    } else {
      Serial.printf("[RTC]: Store CRC32 (%X) matches calculated CRC32\n", rtcData.crc32);
      Serial.println("[RTC]: Waiking up");
      Serial.printf("[RTC]: Stored values: Period = %u, LastRead = %u\n", rtcData.data.periodCount, rtcData.data.lastReadValue);
    }
  }

  // GPIO Setup
  pinMode(GPIO_TOO_DRY, OUTPUT);
  pinMode(GPIO_TOO_WET, OUTPUT);
  pinMode(GPIO_MOIST, OUTPUT);

  runTest();


  // Read moisture sensor and process data
  uint16_t moistPercent = calculateMoistPercent(readSensor());
  if (moistPercent > (rtcData.data.lastReadValue + MAX_PERCENT_POINTS_WITHOUT_REPORTING) ||
      moistPercent < (rtcData.data.lastReadValue - MAX_PERCENT_POINTS_WITHOUT_REPORTING) ||
      rtcData.data.periodCount ++ > MAX_SLEEP_PERIOD_WITHOUT_REPORTING) {
    rtcData.data.periodCount = 0;
    rtcData.data.lastReadValue = moistPercent;
    saveRtcData();
  } else {
    Serial.println("[MOIST]: No reporting needed now. Go back to sleep.");
    rtcData.data.lastReadValue = moistPercent;
    saveRtcData();
    delay(1000);
    ESP.deepSleep(DEEP_SLEEP_PERIOD);
  }

  digitalWrite(GPIO_TOO_DRY, LOW);
  digitalWrite(GPIO_TOO_WET, LOW);
  digitalWrite(GPIO_MOIST, LOW);

  processSensorRead(moistPercent);

  // Instantiate and setup WiFiManager
  // wifiManager.resetSettings(); Uncomment to reset wifi settings
  wifiManager.setAPCallback(configModeCallback);
  if (!wifiManager.autoConnect(ACCESS_POINT_NAME, ACCESS_POINT_PASS)) {
    Serial.println("Failed to connect and hit timeout");
    ESP.reset();
    delay(1000);  
  }

  // Setup MQTT client
  initMqttTopics();
  mqttClient.setServer(MQTT_BROKER_ADDRESS, 1883);
  mqttClient.setCallback(callback);

  // MQTT
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.publish(MQTT_OTA_READY, "No");

  checkOTA();

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

  Serial.printf("[MOIST]: Publishing mositure value (%u)", moistPercent);
  publishMoisture(moistPercent);
  Serial.println("[MOIST]: Going back to sleep");

  delay(1000);
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
    digitalWrite(GPIO_MOIST, HIGH);

    // Report current range
    if (!rangeReported) {
      // Report reay for OTA
      mqttClient.publish(MQTT_OTA_READY, "yes");
      reportRange();
      rangeReported = true;
    }
  }
}