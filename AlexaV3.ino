/*
 * Example for how to use SinricPro Temperaturesensor device:
 * - setup a temperature sensor device
 * - send temperature event to SinricPro server when temperature has changed
 *
 * DHT Sensor is connected to D5 on ESP8266 devices / GPIO5 on ESP32 devices
 *
 * DHT Library used in this example: https://github.com/markruys/arduino-DHT
 *
 * If you encounter any issues:
 * - check the readme.md at https://github.com/sinricpro/esp8266-esp32-sdk/blob/master/README.md
 * - ensure all dependent libraries are installed
 *   - see https://github.com/sinricpro/esp8266-esp32-sdk/blob/master/README.md#arduinoide
 *   - see https://github.com/sinricpro/esp8266-esp32-sdk/blob/master/README.md#dependencies
 * - open serial monitor and check whats happening
 * - check full user documentation at https://sinricpro.github.io/esp8266-esp32-sdk
 * - visit https://github.com/sinricpro/esp8266-esp32-sdk/issues and check for existing issues or open a new one
 */

// Uncomment the following line to enable serial debug output
//#define ENABLE_DEBUG

#ifdef ENABLE_DEBUG
       #define DEBUG_ESP_PORT Serial
       #define NODEBUG_WEBSOCKETS
       #define NDEBUG
#endif


#include <Arduino.h>
#ifdef ESP8266
       #include <ESP8266WiFi.h>
#endif
#ifdef ESP32  
       #include <WiFi.h>
#endif

#include "SinricPro.h"
#include "SinricProTemperaturesensor.h"
#include "SinricProLight.h"
#define WIFI_SSID         "juanc"
#define WIFI_PASS         "11111111"
#define APP_KEY           "bb2d282c-e628-4ca6-9f31-44f66e664a2e"      // Should look like "de0bxxxx-1x3x-4x3x-ax2x-5dabxxxxxxxx"
#define APP_SECRET        "24a81730-7d54-476e-96c1-ecb901a86946-53de2046-11bc-4624-a36d-eb85fed10bf3"   // Should look like "5f36xxxx-x3x7-4x3x-xexe-e86724a9xxxx-4c4axxxx-3x3x-x5xe-x9x3-333d65xxxxxx"
#define TEMP_SENSOR_ID    "6281917a1d6a67083b50a0d5"    // Should look like "5dc1564130xxxxxxxxxxxxxx"
#define MOTOR_ID          "62699e6bd0fd258c521bf3d5"    // Should look like "5dc1564130xxxxxxxxxxxxxx"
#define BAUD_RATE         9600                // Change baudrate to your need
#define EVENT_WAIT_TIME   60000               // send event every 60 seconds

#ifdef ESP8266
#endif
#ifdef ESP32
#endif


bool deviceIsOn;                              // Temeprature sensor on/off state
bool powerState;        
float temperature;                            // actual temperature
float humidity;                               // actual humidity
float lastTemperature;                        // last known temperature (for compare)
float lastHumidity;                           // last known humidity (for compare)
unsigned long lastEvent = (-EVENT_WAIT_TIME); // last time event has been sent

/* bool onPowerState(String deviceId, bool &state)
 *
 * Callback for setPowerState request
 * parameters
 *  String deviceId (r)
 *    contains deviceId (useful if this callback used by multiple devices)
 *  bool &state (r/w)
 *    contains the requested state (true:on / false:off)
 *    must return the new state
 *
 * return
 *  true if request should be marked as handled correctly / false if not
 */
bool onPowerState(String deviceId, bool &state) {
  
  if(deviceId == TEMP_SENSOR_ID){
  Serial.printf("Temperaturesensor turned %s (via SinricPro) \r\n", state?"on":"off");
  deviceIsOn = state; // turn on / off temperature sensor
  if (state) {
    SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];  // get temperaturesensor device
    mySensor.sendTemperatureEvent(13, 7);
  }
  return true; // request handled properly
  }
 
  if(deviceId == MOTOR_ID){
  powerState = state;
  if (state) {
    Serial.println("gg");
    digitalWrite(2,HIGH);
  } else {
    Serial.println("off");
    digitalWrite(2,LOW);
  }
  return true; // request handled properly
  }
}

/* handleTemperatatureSensor()
 * - Checks if Temperaturesensor is turned on
 * - Checks if time since last event > EVENT_WAIT_TIME to prevent sending too much events
 * - Get actual temperature and humidity and check if these values are valid
 * - Compares actual temperature and humidity to last known temperature and humidity
 * - Send event to SinricPro Server if temperature or humidity changed
 */


// setup function for WiFi connection
void setupWiFi() {
  Serial.printf("\r\n[Wifi]: Connecting");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.printf(".");
    delay(250);
  }
  IPAddress localIP = WiFi.localIP();
  Serial.printf("connected!\r\n[WiFi]: IP-Address is %d.%d.%d.%d\r\n", localIP[0], localIP[1], localIP[2], localIP[3]);
}

// setup function for SinricPro
void setupSinricPro() {
  // add device to SinricPro
  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];
  SinricProLight &myLight = SinricPro[MOTOR_ID];
  mySensor.onPowerState(onPowerState);
  myLight.onPowerState(onPowerState);

  // setup SinricPro
  SinricPro.onConnected([](){ Serial.printf("Connected to SinricPro\r\n"); });
  SinricPro.onDisconnected([](){ Serial.printf("Disconnected from SinricPro\r\n"); });
  //SinricPro.restoreDeviceStates(true); // Uncomment to restore the last known state from the server.
  SinricPro.begin(APP_KEY, APP_SECRET);  
}

// main setup function
void setup() {
  Serial.begin(BAUD_RATE); Serial.printf("\r\n\r\n");

  setupWiFi();
  setupSinricPro();
  pinMode(2,OUTPUT);
  digitalWrite(2,LOW);
}

void loop() {
  SinricPro.handle();
  return;
}
