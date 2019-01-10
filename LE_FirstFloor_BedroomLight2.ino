//#define DEBUGLib_DEBUG
#include <DEBUGLib.h>
#include <OneButton.h>
#include <INTERVAL.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <ESPTools.h>
#include <MqttLight.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "BaseConfig.h"

const uint8_t ledPin = LED_BUILTIN; //D4
const uint8_t buttonPins[] = { D5, D6 };
const uint8_t sdaPin = D2;
const uint8_t sclPin = D1;
const uint8_t tempPin = D3;
const uint8_t relayPins[] = { D7, D8 }; // D7 needs pullup!

WiFiClient espClient;
PubSubClient mqttclient(espClient);
const uint32_t mqttReconnectInterval = 5000;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
OneButton left_button(buttonPins[0], true);
OneButton right_button(buttonPins[1], true);
OneWire oneWire(tempPin);
DallasTemperature sensors(&oneWire);

const char* mqtt_topic_tree                   = MQTT_TOPIC_BASE "light/#";
const char* mqtt_topic_pwm_freqency_cmd       = MQTT_TOPIC_BASE "light/pwm/frequency/cmd";
const char* mqtt_topic_pwm_freqency_state     = MQTT_TOPIC_BASE "light/pwm/frequency/state";
const char* mqtt_topic_mode_cmd               = MQTT_TOPIC_BASE "light/mode/cmd";
const char* mqtt_topic_mode_state             = MQTT_TOPIC_BASE "light/mode/state";
const char* mqtt_topic_button1                = MQTT_TOPIC_BASE "button/1";
const char* mqtt_topic_button2                = MQTT_TOPIC_BASE "button/2";
const char* mqtt_topic_temperature_psu        = MQTT_TOPIC_BASE "temperature_psu";

MqttPWMServoDriverLight light[] = {
  {pwm,  0, 0, mqttclient, MQTT_TOPIC_BASE "light/indirect/rgb0/b", 50, 1,   4095},
  {pwm,  1, 0, mqttclient, MQTT_TOPIC_BASE "light/indirect/rgb0/g", 50, 1,   4095},
  {pwm,  2, 0, mqttclient, MQTT_TOPIC_BASE "light/indirect/rgb0/r", 50, 1,   4095},
  {pwm,  3, 0, mqttclient, MQTT_TOPIC_BASE "light/direct/warm0",    50, 700, 4095},
  {pwm,  4, 0, mqttclient, MQTT_TOPIC_BASE "light/indirect/rgb1/g", 50, 1,   4095},
  {pwm,  5, 0, mqttclient, MQTT_TOPIC_BASE "light/indirect/rgb1/b", 50, 1,   4095},
  {pwm,  6, 0, mqttclient, MQTT_TOPIC_BASE "light/indirect/rgb1/r", 50, 1,   4095},
  {pwm,  7, 0, mqttclient, MQTT_TOPIC_BASE "light/direct/warm1",    50, 700, 4095},
  {pwm,  8, 0, mqttclient, MQTT_TOPIC_BASE "light/indirect/rgb2/b", 50, 1,   4095},
  {pwm,  9, 0, mqttclient, MQTT_TOPIC_BASE "light/indirect/rgb2/g", 50, 1,   4095},
  {pwm, 10, 0, mqttclient, MQTT_TOPIC_BASE "light/indirect/rgb2/r", 50, 1,   4095},
  {pwm, 11, 0, mqttclient, MQTT_TOPIC_BASE "light/direct/cold0",    50, 720, 4095},
  {pwm, 12, 0, mqttclient, MQTT_TOPIC_BASE "light/indirect/rgb3/b", 50, 1,   4095},
  {pwm, 13, 0, mqttclient, MQTT_TOPIC_BASE "light/indirect/rgb3/g", 50, 1,   4095},
  {pwm, 14, 0, mqttclient, MQTT_TOPIC_BASE "light/indirect/rgb3/r", 50, 1,   4095},
  {pwm, 15, 0, mqttclient, MQTT_TOPIC_BASE "light/direct/cold1",    50, 720, 4095}
};
uint8_t light_n = 16;

const uint32_t longclick_interval_offline = 333;
const uint32_t longclick_interval_mqtt = 333;
const uint8_t debounce_interval = 500;

const uint32_t tempInterval = 10000;

boolean onlineMode = false;

const uint16_t defaultcontrol = 1 << 3 | 1 << 7 | 1 << 11 | 1 << 15;
uint16_t control = defaultcontrol;
uint16_t changed = 0b0;

uint16_t pwm_freqency = 500;

void cycle_direct_controlled() {
  DEBUG_PRINT("Set control from ");
  DEBUG_PRINT2(control, BIN);
  DEBUG_PRINT(" to ");
  if (control      == (1 << 3 | 1 << 7 | 1 << 11 | 1 << 15))
  {
    DEBUG_PRINT(" 1: ");
    control         = 1 << 3 | 1 << 7 | 0 << 11 | 0 << 15;
    DEBUG_PRINTLN2(control, BIN);
  }
  else if (control == (1 << 3 | 1 << 7 | 0 << 11 | 0 << 15))
  {
    DEBUG_PRINT(" 2: ");
    control         = 0 << 3 | 0 << 7 | 1 << 11 | 1 << 15;
    DEBUG_PRINTLN2(control, BIN);
  }
  else if (control == (0 << 3 | 0 << 7 | 1 << 11 | 1 << 15))
  {
    DEBUG_PRINT(" 3: ");
    control         = 1 << 3 | 1 << 7 | 1 << 11 | 1 << 15;
    DEBUG_PRINTLN2(control, BIN);
  }
  else
  {
    DEBUG_PRINT(" D: ");
    control = defaultcontrol;
    DEBUG_PRINTLN2(control, BIN);
  }
}

void increase_brightness() {
  if (control == 0)
    return;

  for (int i = 0; i < light_n; i++) {
    if (control & 1 << i) {
      light[i].increaseBrightness();
    }
  }
  changed |= control;
}

void decrease_brightness() {
  if (control == 0)
    return;

  for (int i = 0; i < light_n; i++) {
    if (control & 1 << i) {
      light[i].decreaseBrightness();
    }
  }
  changed |= control;
}

void toggle_onoff() {
  DEBUG_PRINT("toggle_onoff ");
  DEBUG_PRINTLN2(control, BIN);
  if (control == 0)
    return;

  for (int i = 0; i < light_n; i++) {
    if (control & 1 << i) {
      DEBUG_PRINT("toggle_onoff ");
      DEBUG_PRINTLN(i);
      light[i].toggleOnOff();
    }
  }
  changed |= control;
}

void switch_off() {
  if (control == 0)
    return;

  for (int i = 0; i < light_n; i++) {
    if (control & 1 << i) {
      light[i].switchOff();
    }
  }
  changed |= control;
}

void switch_on() {
  if (control == 0)
    return;

  for (int i = 0; i < light_n; i++) {
    if (control & 1 << i) {
      light[i].switchOn();
    }
  }
  changed |= control;
}

void set_brightness_offline(uint8_t abs_brightness) {
  if (control == 0)
    return;

  for (int i = 0; i < light_n; i++) {
    if (control & 1 << i) {
      light[i].setBrightness(abs_brightness);
    }
  }
  changed |= control;
}

void setLight()
{
  if (changed == 0)
    return;

  for (int i = 0; i < light_n; i++) {
    if (changed & 1 << i) {
      light[i].setLevel();
    }
  }
  changed = 0;
}

void left_click() {
  if (!onlineMode) {
    toggle_onoff();
  }

  if (mqttclient.connected()) {
    if (mqttclient.publish(mqtt_topic_button1, "click", false)) {
      DEBUG_PRINTLN("Published left click");
    } else {
      DEBUG_PRINTLN("Publishing left click failed.");
    }
  } else {
    DEBUG_PRINTLN("MQTT not connected");
  }
}

void right_click() {
  if (!onlineMode) {
    cycle_direct_controlled();
  }

  if (mqttclient.connected()) {
    if (mqttclient.publish(mqtt_topic_button2, "click", false)) {
      DEBUG_PRINTLN("Published right click");
    } else {
      DEBUG_PRINTLN("Publishing right click failed.");
    }
  } else {
    DEBUG_PRINTLN("MQTT not connected");
  }
}

void left_doubleclick() {
  if (mqttclient.connected()) {
    if (mqttclient.publish(mqtt_topic_button1, "doubleclick", false)) {
      DEBUG_PRINTLN("Published left doubleclick");
    } else {
      DEBUG_PRINTLN("Publishing left doubleclick failed.");
    }
  } else {
    DEBUG_PRINTLN("MQTT not connected");
  }
}

void right_doubleclick() {
  if (!onlineMode) {
    control = defaultcontrol;
  }
  if (mqttclient.connected()) {
    if (mqttclient.publish(mqtt_topic_button2, "doubleclick", false)) {
      DEBUG_PRINTLN("Published right doubleclick");
    } else {
      DEBUG_PRINTLN("Publishing right doubleclick failed.");
    }
  } else {
    DEBUG_PRINTLN("MQTT not connected");
  }
}

void left_duringlongpress() {
  if (!onlineMode) {
    INTERVAL(longclick_interval_offline)
    {
      increase_brightness();
    }
  }
  INTERVAL(longclick_interval_mqtt)
  {
    if (mqttclient.connected()) {
      if (mqttclient.publish(mqtt_topic_button1, "longpress", false)) {
        DEBUG_PRINTLN("Published left longpress");
      } else {
        DEBUG_PRINTLN("Publishing left longpress failed.");
      }
    } else {
      DEBUG_PRINTLN("MQTT not connected");
    }
  }
}

void right_duringlongpress() {
  if (!onlineMode) {
    INTERVAL(longclick_interval_offline)
    {
      decrease_brightness();
    }
  }
  INTERVAL(longclick_interval_mqtt)
  {
    if (mqttclient.connected()) {
      if (mqttclient.publish(mqtt_topic_button2, "longpress", false)) {
        DEBUG_PRINTLN("Published right longpress");
      } else {
        DEBUG_PRINTLN("Publishing right longpress failed.");
      }
    } else {
      DEBUG_PRINTLN("MQTT not connected");
    }
  }
}

void left_longpressstart() {
  if (mqttclient.connected()) {
    if (mqttclient.publish(mqtt_topic_button1, "longpressstart", false)) {
      DEBUG_PRINTLN("Published left longpress start");
    } else {
      DEBUG_PRINTLN("Publishing left longpress start failed.");
    }
  } else {
    DEBUG_PRINTLN("MQTT not connected");
  }
}

void right_longpressstart() {
  if (mqttclient.connected()) {
    if (mqttclient.publish(mqtt_topic_button2, "longpressstart", false)) {
      DEBUG_PRINTLN("Published right longpress start");
    } else {
      DEBUG_PRINTLN("Publishing right longpress start failed.");
    }
  } else {
    DEBUG_PRINTLN("MQTT not connected");
  }
}

void left_longpressstop() {
  if (mqttclient.connected()) {
    if (mqttclient.publish(mqtt_topic_button1, "longpressstop", false)) {
      DEBUG_PRINTLN("Published left stop longpress");
    } else {
      DEBUG_PRINTLN("Publishing left stop longpress failed.");
    }
  } else {
    DEBUG_PRINTLN("MQTT not connected");
  }
}

void right_longpressstop() {
  if (mqttclient.connected()) {
    if (mqttclient.publish(mqtt_topic_button2, "longpressstop", false)) {
      DEBUG_PRINTLN("Published right stop longpress");
    } else {
      DEBUG_PRINTLN("Publishing right stop longpress failed.");
    }
  } else {
    DEBUG_PRINTLN("MQTT not connected");
  }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length)
{
  // Make const char* from byte*
  char message_buff[length + 1];
  int i;
  for (i = 0; i < length; i++)
  {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';
  const char *p_payload = message_buff;
  DEBUG_PRINT(topic);
  DEBUG_PRINT(": ");
  DEBUG_PRINTLN(p_payload);

  for (int i = 0; i < light_n; i++)
  {
    char topicstr[light[i].getMQTTCommandTopicLen() + 1];
    light[i].getMQTTCommandTopic(topicstr);
    if (!strcmp(topic, topicstr))
    {
      DEBUG_PRINT("Matched light ");
      DEBUG_PRINTLN(i);
      light[i].parsePayload(p_payload);
      return;
    } else {
      //DEBUG_PRINT("Did not match ");
      //DEBUG_PRINTLN(topicstr);
    }
  }

  if (!strcmp(topic, mqtt_topic_pwm_freqency_cmd))
  {
    DEBUG_PRINT("Setting pwm frequency to");
    DEBUG_PRINTLN(atoi(p_payload));
    pwm.setPWMFreq(atoi(p_payload));
    char pubchar[5];
    sprintf(pubchar, "%d", atoi(p_payload));
    mqttclient.publish(mqtt_topic_pwm_freqency_state, pubchar, true);
    return;
  }
  else if (!strcmp(topic, mqtt_topic_mode_cmd))
  {
    DEBUG_PRINT("Setting mode to");
    DEBUG_PRINTLN(p_payload);
    if (!strcmp(p_payload, "ONLINE"))
    {
      onlineMode = true;
    } else {
      onlineMode = false;
    }
    mqttclient.publish(mqtt_topic_mode_state, onlineMode ? "ONLINE" : "OFFLINE", true);
    return;
  }
}

void setup() {
  left_button.setClickTicks(debounce_interval);
  right_button.setClickTicks(debounce_interval);

  left_button.attachClick(left_click);
  right_button.attachClick(right_click);

  left_button.attachDoubleClick(left_doubleclick);
  right_button.attachDoubleClick(right_doubleclick);

  left_button.attachDuringLongPress(left_duringlongpress);
  right_button.attachDuringLongPress(right_duringlongpress);

  left_button.attachLongPressStart(left_longpressstart);
  right_button.attachLongPressStart(right_longpressstart);

  left_button.attachLongPressStop(left_longpressstop);
  right_button.attachLongPressStop(right_longpressstop);

  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(D3, OUTPUT);
  digitalWrite(D3, LOW);

  sensors.begin();

  DEBUG_PRINT("DEBUG ");
  Serial.println("Started ...");
  startWifi(ssid, password, device_name);
  Serial.println( "IP address: " );
  Serial.println( WiFi.localIP() );

  mqttclient.setServer(mqtt_server, 1883);
  mqttclient.setCallback(mqtt_callback);
  Serial.println("Initialization complete.");

  configArduinoOTA(device_name, ota_password);

  pwm.begin();
  pwm.setPWMFreq(pwm_freqency);

  char pubchar[5];
  sprintf(pubchar, "%d", pwm_freqency);
  mqttclient.publish(mqtt_topic_pwm_freqency_state, pubchar, true);
  mqttclient.publish(mqtt_topic_mode_state, onlineMode ? "ONLINE" : "OFFLINE", true);

  for (int i = 0; i < light_n; i++) {
    light[i].switchOff();
  }
}

void loop() {
  delay(2);
  ArduinoOTA.handle();

  if (!mqtt_check(mqttclient, mqttReconnectInterval, device_name, mqtt_user, mqtt_pw, mqtt_topic_tree)) {
    DEBUG_PRINT("MQTT reconnect failed, rc=");
    DEBUG_PRINT(mqttclient.state());
    DEBUG_PRINTLN(" try again in 5 seconds");
  }
  yield();
  left_button.tick();
  right_button.tick();
  yield();
  left_button.tick();
  right_button.tick();
  setLight();
  yield();

  INTERVAL(tempInterval)
  {
    sensors.requestTemperatures();
    char pubchar[10];
    sprintf(pubchar, "%d", sensors.getTempCByIndex(0));
    mqttclient.publish(mqtt_topic_temperature_psu, pubchar, true);
  }
}

