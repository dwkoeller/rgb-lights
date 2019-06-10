//This can be used to output the date the code was compiled
const char compile_date[] = __DATE__ " " __TIME__;

/************ WIFI, OTA and MQTT INFORMATION (CHANGE THESE FOR YOUR SETUP) ******************/
//#define WIFI_SSID "" //enter your WIFI SSID
//#define WIFI_PASSWORD "" //enter your WIFI Password
//#define MQTT_SERVER "" // Enter your MQTT server address or IP.
//#define MQTT_USER "" //enter your MQTT username
//#define MQTT_PASSWORD "" //enter your password
#define MQTT_DEVICE "gamerooom-rgb-left" // Enter your MQTT device
#define MQTT_SSL_PORT 8883 // Enter your MQTT server port.
#define MQTT_SOCKET_TIMEOUT 120
#define FW_UPDATE_INTERVAL_SEC 24*3600
#define WATCHDOG_UPDATE_INTERVAL_SEC 1
#define WATCHDOG_RESET_INTERVAL_SEC 120
#define STATUS_UPDATE_INTERVAL_SEC 120
#define UPDATE_SERVER "http://192.168.100.15/firmware/"
#define FIRMWARE_VERSION "-1.20"
#define MQTT_VERSION_PUB "gameroom/rgb_left/version"
#define MQTT_COMPILE_PUB "gameroom/rgb_left/compile"
#define MQTT_HEARTBEAT_SUB "heartbeat/#"
#define MQTT_HEARTBEAT_TOPIC "heartbeat"
#define MQTT_HEARTBEAT_PUB "gameroom/rgb_left/heartbeat"

// state
#define ROOM_LIGHT_STATE_TOPIC "gameroom/rgb_left/light/status"
#define ROOM_LIGHT_COMMAND_TOPIC "gameroom/rgb_left/light/switch"

// brightness
#define ROOM_LIGHT_BRIGHTNESS_STATE_TOPIC "gameroom/rgb_left/brightness/status"
#define ROOM_LIGHT_BRIGHTNESS_COMMAND_TOPIC "gameroom/rgb_left/brightness/set"

// colors (rgb)
#define ROOM_LIGHT_RGB_STATE_TOPIC "gameroom/rgb_left/rgb/status"
#define ROOM_LIGHT_RGB_COMMAND_TOPIC "gameroom/rgb_left/rgb/set"

// payloads by default (on/off)
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
    client.publish(ROOM_LIGHT_STATE_TOPIC, LIGHT_ON, true);
  } else {
    client.publish(ROOM_LIGHT_STATE_TOPIC, LIGHT_OFF, true);
  }
}

// function called to publish the brightness of the led (0-100)
void publishRGBBrightness() {
  snprintf(m_msg_buffer, MSG_BUFFER_SIZE, "%d", m_rgb_brightness);
  client.publish(ROOM_LIGHT_BRIGHTNESS_STATE_TOPIC, m_msg_buffer, true);
}

// function called to publish the colors of the led (xx(x),xx(x),xx(x))
void publishRGBColor() {
  snprintf(m_msg_buffer, MSG_BUFFER_SIZE, "%d,%d,%d", m_rgb_red, m_rgb_green, m_rgb_blue);
  client.publish(ROOM_LIGHT_RGB_STATE_TOPIC, m_msg_buffer, true);
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
    client.publish(MQTT_HEARTBEAT_PUB, "Heartbeat Received");  
    return;
  }        
  // handle message topic
  if (String(ROOM_LIGHT_COMMAND_TOPIC).equals(p_topic)) {
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
  } else if (String(ROOM_LIGHT_BRIGHTNESS_COMMAND_TOPIC).equals(p_topic)) {
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
  } else if (String(ROOM_LIGHT_RGB_COMMAND_TOPIC).equals(p_topic)) {
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
  // init the serial
  Serial.begin(115200);

  // init the RGB led
  pinMode(RGB_LIGHT_BLUE_PIN, OUTPUT);
  pinMode(RGB_LIGHT_RED_PIN, OUTPUT);
  pinMode(RGB_LIGHT_GREEN_PIN, OUTPUT);
   
  analogWriteRange(255);

  setup_wifi();

  // init the MQTT connection
  client.setServer(MQTT_SERVER, MQTT_SSL_PORT);
  client.setCallback(callback);

  ticker_fw.attach_ms(FW_UPDATE_INTERVAL_SEC * 1000, fwTicker);
  ticker_status.attach_ms(STATUS_UPDATE_INTERVAL_SEC * 1000, statusTicker);

  checkForUpdates();  
  setColor(0,0,0);
}

void loop() {
  if (!client.connected()) {
    reconnect();
    // Once connected, publish an announcement...
    // publish the initial values
    publishRGBState();
    publishRGBBrightness();
    publishRGBColor();

    // ... and resubscribe
    client.subscribe(ROOM_LIGHT_COMMAND_TOPIC);
    client.subscribe(ROOM_LIGHT_BRIGHTNESS_COMMAND_TOPIC);
    client.subscribe(ROOM_LIGHT_RGB_COMMAND_TOPIC);
    client.subscribe(MQTT_HEARTBEAT_SUB);

  }

  if(readyForFwUpdate) {
    readyForFwUpdate = false;
    checkForUpdates();
  }

  client.loop();
}

void statusTicker() {
  String status;
  if (poweredOn > 0) {
    status = "ON";
  }
  else {
    status = "OFF";
  }
  client.publish(ROOM_LIGHT_STATE_TOPIC, status.c_str());
}
