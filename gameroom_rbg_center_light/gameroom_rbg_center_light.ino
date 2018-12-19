#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include "credentials.h" // Place credentials for wifi and mqtt in this file

//This can be used to output the date the code was compiled
const char compile_date[] = __DATE__ " " __TIME__;

/************ WIFI, OTA and MQTT INFORMATION (CHANGE THESE FOR YOUR SETUP) ******************/
//#define WIFI_SSID "" //enter your WIFI SSID
//#define WIFI_PASSWORD "" //enter your WIFI Password
//#define MQTT_SERVER "" // Enter your MQTT server address or IP.
//#define MQTT_USER "" //enter your MQTT username
//#define MQTT_PASSWORD "" //enter your password
#define MQTT_DEVICE "gamerooom-rgb-center" // Enter your MQTT device
#define MQTT_PORT 1883 // Enter your MQTT server port.
#define MQTT_SOCKET_TIMEOUT 120
#define FW_UPDATE_INTERVAL_SEC 24*3600
#define WATCHDOG_UPDATE_INTERVAL_SEC 1
#define WATCHDOG_RESET_INTERVAL_SEC 120
#define STATUS_UPDATE_INTERVAL_SEC 120
#define UPDATE_SERVER "http://192.168.100.15/firmware/"
#define FIRMWARE_VERSION "-1.08"
#define MQTT_VERSION_PUB "gameroom/rgb_center/version"
#define MQTT_COMPILE_PUB "gameroom/rgb_center/compile"
#define MQTT_HEARTBEAT_SUB "heartbeat/#"
#define MQTT_HEARTBEAT_TOPIC "heartbeat"

// state
#define ROOM_LIGHT_STATE_TOPIC "gameroom/rgb_center/light/status"
#define ROOM_LIGHT_COMMAND_TOPIC "gameroom/rgb_center/light/switch"

// brightness
#define ROOM_LIGHT_BRIGHTNESS_STATE_TOPIC "gameroom/rgb_center/brightness/status"
#define ROOM_LIGHT_BRIGHTNESS_COMMAND_TOPIC "gameroom/rgb_center/brightness/set"

// colors (rgb)
#define ROOM_LIGHT_RGB_STATE_TOPIC "gameroom/rgb_center/rgb/status"
#define ROOM_LIGHT_RGB_COMMAND_TOPIC "gameroom/rgb_center/rgb/set"

// payloads by default (on/off)
#define LIGHT_ON "ON"
#define LIGHT_OFF "OFF"

volatile int watchDogCount = 0;

Ticker ticker_fw, ticker_watchdog, ticker_status;

bool readyForFwUpdate = false;
bool poweredOn = false;

WiFiClient espClient;

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

WiFiClient wifiClient;
PubSubClient client(wifiClient);

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
    watchDogCount = 0;  
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

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("INFO: Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MQTT_DEVICE, MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("INFO: connected");
      
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
      String firmwareVer = String("Firmware Version: ") + String(FIRMWARE_VERSION);
      String compileDate = String("Build Date: ") + String(compile_date);
      client.publish(MQTT_VERSION_PUB, firmwareVer.c_str(), true);
      client.publish(MQTT_COMPILE_PUB, compileDate.c_str(), true);
      
    } else {
      Serial.print("ERROR: failed, rc=");
      Serial.print(client.state());
      Serial.println("DEBUG: try again in 5 seconds");
      // Wait 5 seconds before retrying
      my_delay(5000);
    }
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
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);

  ticker_fw.attach_ms(FW_UPDATE_INTERVAL_SEC * 1000, fwTicker);
  ticker_watchdog.attach_ms(WATCHDOG_UPDATE_INTERVAL_SEC * 1000, watchdogTicker);
  ticker_status.attach_ms(STATUS_UPDATE_INTERVAL_SEC * 1000, statusTicker);

  checkForUpdates();  
  setColor(0,0,0);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }

  if(readyForFwUpdate) {
    readyForFwUpdate = false;
    checkForUpdates();
  }

  client.loop();
}

void setup_wifi() {
  int count = 0;
  my_delay(50);

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.hostname(MQTT_DEVICE);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    my_delay(250);
    Serial.print(".");
    count++;
    if(count > 50) {
      WiFiManager wifiManager;
      wifiManager.resetSettings();
      wifiManager.autoConnect();
    }
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
}

// FW update ticker
void fwTicker() {
  readyForFwUpdate = true;
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

// Watchdog update ticker
void watchdogTicker() {
  watchDogCount++;
  if(watchDogCount >= WATCHDOG_RESET_INTERVAL_SEC) {
    Serial.println("Reset system");
    ESP.restart();  
  }
}

String WiFi_macAddressOf(IPAddress aIp) {
  if (aIp == WiFi.localIP())
    return WiFi.macAddress();

  if (aIp == WiFi.softAPIP())
    return WiFi.softAPmacAddress();

  return String("00-00-00-00-00-00");
}

void checkForUpdates() {

  String clientMAC = WiFi_macAddressOf(espClient.localIP());

  Serial.print("MAC: ");
  Serial.println(clientMAC);
  clientMAC.replace(":", "-");
  String filename = clientMAC.substring(9);
  String firmware_URL = String(UPDATE_SERVER) + filename + String(FIRMWARE_VERSION);
  String current_firmware_version_URL = String(UPDATE_SERVER) + filename + String("-current_version");

  HTTPClient http;

  http.begin(current_firmware_version_URL);
  int httpCode = http.GET();
  
  if ( httpCode == 200 ) {

    String newFirmwareVersion = http.getString();
    newFirmwareVersion.trim();
    
    Serial.print( "Current firmware version: " );
    Serial.println( FIRMWARE_VERSION );
    Serial.print( "Available firmware version: " );
    Serial.println( newFirmwareVersion );
    
    if(newFirmwareVersion.substring(1).toFloat() > String(FIRMWARE_VERSION).substring(1).toFloat()) {
      Serial.println( "Preparing to update" );
      String new_firmware_URL = String(UPDATE_SERVER) + filename + newFirmwareVersion + ".bin";
      Serial.println(new_firmware_URL);
      t_httpUpdate_return ret = ESPhttpUpdate.update( new_firmware_URL );

      switch(ret) {
        case HTTP_UPDATE_FAILED:
          Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
          break;

        case HTTP_UPDATE_NO_UPDATES:
          Serial.println("HTTP_UPDATE_NO_UPDATES");
         break;
      }
    }
    else {
      Serial.println("Already on latest firmware");  
    }
  }
  else {
    Serial.print("GET RC: ");
    Serial.println(httpCode);
  }
}

void my_delay(unsigned long ms) {
  uint32_t start = micros();

  while (ms > 0) {
    yield();
    while ( ms > 0 && (micros() - start) >= 1000) {
      ms--;
      start += 1000;
    }
  }
}
