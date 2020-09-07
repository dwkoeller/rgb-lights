//This can be used to output the date the code was compiled
const char compile_date[] = __DATE__ " " __TIME__;

/************ WIFI, OTA and MQTT INFORMATION (CHANGE THESE FOR YOUR SETUP) ******************/
//#define WIFI_SSID "" //enter your WIFI SSID
//#define WIFI_PASSWORD "" //enter your WIFI Password
//#define MQTT_SERVER "" // Enter your MQTT server address or IP.
//#define MQTT_USER "" //enter your MQTT username
//#define MQTT_PASSWORD "" //enter your password
#define MQTT_DEVICE "gamerooom-rgb-right" // Enter your MQTT device
#define MQTT_DEVICE_NAME "Gameroom TV Right"
#define MQTT_SSL_PORT 8883 // Enter your MQTT server port.
#define MQTT_SOCKET_TIMEOUT 120
#define FW_UPDATE_INTERVAL_SEC 24*3600
#define STATUS_UPDATE_INTERVAL_SEC 120
#define FIRMWARE_VERSION "-2.01"

#define MQTT_HEARTBEAT_SUB "heartbeat/#"
#define MQTT_HEARTBEAT_TOPIC "heartbeat"
#define MQTT_UPDATE_REQUEST "update"
#define MQTT_DISCOVERY_LIGHT_PREFIX  "homeassistant/light/"
#define MQTT_DISCOVERY_SENSOR_PREFIX  "homeassistant/sensor/"
#define HA_TELEMETRY "ha"

String LightCommandTopic = String(MQTT_DISCOVERY_LIGHT_PREFIX) + MQTT_DEVICE + "/command";
String LightBrightnessCommandTopic = String(MQTT_DISCOVERY_LIGHT_PREFIX) + MQTT_DEVICE + "/brightness_command";
String LightRGBCommandTopic = String(MQTT_DISCOVERY_LIGHT_PREFIX) + MQTT_DEVICE + "/rgb_command";

#define LIGHT_ON "ON"
#define LIGHT_OFF "OFF"

#define WATCHDOG_PIN 5  // D1
 
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include "credentials.h" // Place credentials for wifi and mqtt in this file
#include "certificates.h" // Place certificates for mqtt in this file

Ticker ticker_fw, ticker_status;

bool readyForFwUpdate = false;
bool poweredOn = false;
bool registered = false;

WiFiClientSecure espClient;

// variables used to store the state, the brightness and the color of the light
boolean m_rgb_state = false;
uint8_t m_rgb_brightness = 0;
uint8_t m_rgb_red = 255;
uint8_t m_rgb_green = 255;
uint8_t m_rgb_blue = 255;

// pins used for the rgb led (PWM)
const PROGMEM uint8_t RGB_LIGHT_RED_PIN = D7;  // D5
const PROGMEM uint8_t RGB_LIGHT_GREEN_PIN = D6; // D7
const PROGMEM uint8_t RGB_LIGHT_BLUE_PIN = D5; // D6

// buffer used to send/receive data with MQTT
const uint8_t MSG_BUFFER_SIZE = 20;
char m_msg_buffer[MSG_BUFFER_SIZE]; 

PubSubClient client(espClient);

#include "common.h"

// function called to adapt the brightness and the color of the led
void setColor(uint8_t p_red, uint8_t p_green, uint8_t p_blue) {
  // convert the brightness in % (0-100%) into a digital value (0-255)
  uint8_t brightness = map(m_rgb_brightness, 0, 100, 0, 255);

  Serial.println("setColor function " + String(p_red) + " " + String(p_green) + " " + String(p_blue) + " " + String(brightness));

  Serial.print ("map p_red: ");
  Serial.println (byte(~map(p_red, 0, 255, 0, brightness)));
  Serial.print ("map p_green: ");
  Serial.println (byte(~map(p_green, 0, 255, 0, brightness)));
  Serial.print ("map p_blue: ");
  Serial.println (byte(~map(p_blue, 0, 255, 0, brightness)));

  analogWrite(RGB_LIGHT_RED_PIN, byte(~map(p_red, 0, 255, 0, brightness)));
  analogWrite(RGB_LIGHT_GREEN_PIN, byte(~map(p_green, 0, 255, 0, brightness)));
  analogWrite(RGB_LIGHT_BLUE_PIN, byte(~map(p_blue, 0, 255, 0, brightness)));
}

// function called to publish the state of the led (on/off)
void publishRGBState() {
  if (m_rgb_state) {
    updateLightState(MQTT_DEVICE, LIGHT_ON);
  } else {
    updateLightState(MQTT_DEVICE, LIGHT_OFF);
  }
}

// function called to publish the brightness of the led (0-100)
void publishRGBBrightness() {
  snprintf(m_msg_buffer, MSG_BUFFER_SIZE, "%d", m_rgb_brightness);
  updateLightBrightnessState(MQTT_DEVICE, m_msg_buffer);
}

// function called to publish the colors of the led (xx(x),xx(x),xx(x))
void publishRGBColor() {
  snprintf(m_msg_buffer, MSG_BUFFER_SIZE, "%d,%d,%d", m_rgb_red, m_rgb_green, m_rgb_blue);
  updateLightRGBState(MQTT_DEVICE, m_msg_buffer);
}

// function called when a MQTT message arrived
void callback(char* p_topic, byte* p_payload, unsigned int p_length) {
  // concat the payload into a string
  String payload;
  for (uint8_t i = 0; i < p_length; i++) {
    payload.concat((char)p_payload[i]);
  }
  if (String(MQTT_HEARTBEAT_TOPIC).equals(p_topic)) {
    resetWatchdog();
    updateTelemetry(payload);
    if (payload.equals(String(MQTT_UPDATE_REQUEST))) {
      checkForUpdates();
    }    
    return;
  }        
  // handle message topic
  if (LightCommandTopic.equals(p_topic)) {
    // test if the payload is equal to "ON" or "OFF"
    if (payload.equals(String(LIGHT_ON))) {
      poweredOn = true;
      if (m_rgb_state != true) {
        m_rgb_state = true;
        setColor(m_rgb_red, m_rgb_green, m_rgb_blue);
        Serial.println("setColor Light ON " + String(m_rgb_red) + " " + String(m_rgb_green) + " " + String(m_rgb_blue));
        publishRGBState();
      }
    } else if (payload.equals(String(LIGHT_OFF))) {
      poweredOn = false;
      if (m_rgb_state != false) {
        m_rgb_state = false;
        setColor(0,0,0);
        Serial.println("setColor Light OFF " + String(m_rgb_red) + " " + String(m_rgb_green) + " " + String(m_rgb_blue));
        publishRGBState();
      }
    }
  } else if (LightBrightnessCommandTopic.equals(p_topic)) {
    uint8_t brightness = payload.toInt();
    if (brightness < 0 || brightness > 100) {
      // do nothing...
      return;
    } else {
      m_rgb_brightness = brightness;
      setColor(m_rgb_red, m_rgb_green, m_rgb_blue);
      Serial.println("setColor Light ON " + String(m_rgb_red) + " " + String(m_rgb_green) + " " + String(m_rgb_blue));
      publishRGBBrightness();
    }
  } else if (LightRGBCommandTopic.equals(p_topic)) {
    // get the position of the first and second commas
    uint8_t firstIndex = payload.indexOf(',');
    uint8_t lastIndex = payload.lastIndexOf(',');
    
    uint8_t rgb_red = payload.substring(0, firstIndex).toInt();
    if (rgb_red < 0 || rgb_red > 255) {
      return;
    } else {
      m_rgb_red = rgb_red;
    }
    
    uint8_t rgb_green = payload.substring(firstIndex + 1, lastIndex).toInt();
    if (rgb_green < 0 || rgb_green > 255) {
      return;
    } else {
      m_rgb_green = rgb_green;
    }
    
    uint8_t rgb_blue = payload.substring(lastIndex + 1).toInt();
    if (rgb_blue < 0 || rgb_blue > 255) {
      return;
    } else {
      m_rgb_blue = rgb_blue;
    }
    
    setColor(m_rgb_red, m_rgb_green, m_rgb_blue);
    Serial.println("setColor Light ON " + String(m_rgb_red) + " " + String(m_rgb_green) + " " + String(m_rgb_blue));
    publishRGBColor();
  }
}

void setup() {
  Serial.begin(115200);

  // init the RGB led
  pinMode(RGB_LIGHT_BLUE_PIN, OUTPUT);
  pinMode(RGB_LIGHT_RED_PIN, OUTPUT);
  pinMode(RGB_LIGHT_GREEN_PIN, OUTPUT);

  pinMode(WATCHDOG_PIN, OUTPUT);
   
  analogWriteRange(255);

  setup_wifi();

  IPAddress result;
  int err = WiFi.hostByName(MQTT_SERVER, result) ;
  if(err == 1){
        Serial.print("MQTT Server IP address: ");
        Serial.println(result);
        MQTTServerIP = result.toString();
  } else {
        Serial.print("Error code: ");
        Serial.println(err);
  }  

  client.setBufferSize(2048);  
  client.setServer(MQTT_SERVER, MQTT_SSL_PORT);
  client.setCallback(callback);

  ticker_fw.attach_ms(FW_UPDATE_INTERVAL_SEC * 1000, fwTicker);
  ticker_status.attach_ms(STATUS_UPDATE_INTERVAL_SEC * 1000, statusTicker);

  checkForUpdates();  
  setColor(0,0,0);
  resetWatchdog();
}

void loop() {
  client.loop();
  
  if (!client.connected()) {
    reconnect();
    // Once connected, publish an announcement...
    // publish the initial values
    publishRGBState();
    publishRGBBrightness();
    publishRGBColor();

    // ... and resubscribe
    client.subscribe(LightCommandTopic.c_str());
    client.subscribe(LightBrightnessCommandTopic.c_str());
    client.subscribe(LightRGBCommandTopic.c_str());
    client.subscribe(MQTT_HEARTBEAT_SUB);

  }

  if(readyForFwUpdate) {
    readyForFwUpdate = false;
    checkForUpdates();
  }

  if (! registered) {
    registerTelemetry();
    updateTelemetry("Unknown");
    createLight(MQTT_DEVICE, MQTT_DEVICE_NAME);
    registered = true;
  }
  
}

void statusTicker() {
  String status;
  if (poweredOn > 0) {
    status = "ON";
  }
  else {
    status = "OFF";
  }
  updateLightState(MQTT_DEVICE, status.c_str());
}

void createLight(String light, String light_name) {
  String topic = String(MQTT_DISCOVERY_LIGHT_PREFIX) + light + "/config";
  String message = String("{\"name\": \"") + light_name +
                   String("\", \"retain\": \"true") +
                   String("\", \"unique_id\": \"") + light + getUUID() +
                   String("\", \"optimistic\": \"false") +
                   String("\", \"rgb_state_topic\": \"") + String(MQTT_DISCOVERY_LIGHT_PREFIX) + light +
                   String("/rgb_state\", \"rgb_command_topic\": \"") + String(MQTT_DISCOVERY_LIGHT_PREFIX) + light +
                   String("/rgb_command\", \"brightness_state_topic\": \"") + String(MQTT_DISCOVERY_LIGHT_PREFIX) + light +
                   String("/brightness_state\", \"brightness_command_topic\": \"") + String(MQTT_DISCOVERY_LIGHT_PREFIX) + light +
                   String("/brightness_command\", \"command_topic\": \"") + String(MQTT_DISCOVERY_LIGHT_PREFIX) + light +
                   String("/command\", \"state_topic\": \"") + String(MQTT_DISCOVERY_LIGHT_PREFIX) + light +
                   String("/state\"}");
  Serial.print(F("MQTT - "));
  Serial.print(topic);
  Serial.print(F(" : "));
  Serial.println(message.c_str());

  client.publish(topic.c_str(), message.c_str(), true);

}

void updateLightState(String light, String state) {
  String topic = String(MQTT_DISCOVERY_LIGHT_PREFIX) + light + "/state";

  Serial.print(F("MQTT - "));
  Serial.print(topic);
  Serial.print(F(" : "));
  Serial.println(state);
  client.publish(topic.c_str(), state.c_str(), true);

}

void updateLightBrightnessState(String light, String state) {
  String topic = String(MQTT_DISCOVERY_LIGHT_PREFIX) + light + "/brightness_state";

  Serial.print(F("MQTT - "));
  Serial.print(topic);
  Serial.print(F(" : "));
  Serial.println(state);
  client.publish(topic.c_str(), state.c_str(), true);

}

void updateLightRGBState(String light, String state) {
  String topic = String(MQTT_DISCOVERY_LIGHT_PREFIX) + light + "/rgb_state";

  Serial.print(F("MQTT - "));
  Serial.print(topic);
  Serial.print(F(" : "));
  Serial.println(state);
  client.publish(topic.c_str(), state.c_str(), true);

}
