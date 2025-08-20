#include "DHT.h"
#include "WiFi.h"
#include "WebServer.h"
#include "ArduinoJson.h"
#include "Preferences.h"
#include "driver/ledc.h"

// Define the sensor type and the GPIO pin it's connected to.
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Define GPIO pins (ESP32-C3 safe mapping)
const int fanrelay = 6;            // Relay / Fan control
const int lightsensor = 2;         // LDR (analog input, ADC1_CH2)
const int nightled = 7;            // PWM brightness control (LDR LED)
const int mainledsw = 0;           // Touch sensor
const int proximitysw = 3;         // Proximity sensor
const int mainled = 1;             // Main LED PWM control

// Variables for sensor states and control
int currentTouchValue = 0;
int lastTouchValue = 0;
int currentProximityValue = 0;
int lastProximityValue = 0;

// Thresholds
int lightThreshold = 2350;

// PWM settings for LEDC using ESP-IDF functions
const ledc_mode_t ledcMode = LEDC_LOW_SPEED_MODE;
const ledc_timer_bit_t ledcResolution = LEDC_TIMER_8_BIT;

// LDR LED PWM settings
const ledc_timer_t ledcTimer_ldr = LEDC_TIMER_0;
const ledc_channel_t ledcChannel_ldr = LEDC_CHANNEL_0;
int ledcBaseFreq = 5000;
int currentBrightnessDutyCycle = 128; // Default 50% brightness for LDR LED

// Main LED PWM settings
const ledc_timer_t ledcTimer_mainLed = LEDC_TIMER_1;
const ledc_channel_t ledcChannel_mainLed = LEDC_CHANNEL_1;
int mainLedBaseFreq = 5000;
int mainLedBrightnessDutyCycle = 255; // Main LED full brightness

// Calibration offsets
const float tempOffset = -0.1;
const float humidityOffset = 6.0;

// Temperature thresholds
float tempOn = 29.0;
float tempOff = 28.5;

// WiFi Configuration
const char* ssid = "wifi_slow";
IPAddress staticIP(192, 168, 1, 4);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);
Preferences preferences;

// Timers and State variables
unsigned long previousSensorMillis = 0;
const long sensorInterval = 10000; // Sensor read interval (10 seconds)
unsigned long lastLDRChangeTime = 0;
const long debounceDelay = 50; // Reduced to make the night LED react faster

float lastHumidity = 0;
float lastTemperature = 0;
int lastLight = 0;
bool lastLDRState;
bool mainLedManualState = false;
bool proximityManualState = false; // This variable controls the master switch state

// Define the control modes and current mode variable
enum ControlMode { AUTOMATED, MANUAL_ON_PERMANENT, MANUAL_ON_TIMED };
ControlMode currentMode = AUTOMATED;
unsigned long manualTimerEnd = 0;

// === Sensor Reading Function ===
void readSensors() {
  lastHumidity = dht.readHumidity() + humidityOffset;
  lastTemperature = dht.readTemperature() + tempOffset;
  lastLight = analogRead(lightsensor);
}

// === Web Handlers ===

/**
 * @brief Sets the fan relay to a permanent ON state.
 */
void handleOn() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  currentMode = MANUAL_ON_PERMANENT;
  digitalWrite(fanrelay, HIGH);
  server.send(200, "text/plain", "GPIO 6 is now manually ON (permanent).");
}

/**
 * @brief Sets the fan relay to OFF and resumes automated control.
 */
void handleOff() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  currentMode = AUTOMATED;
  digitalWrite(fanrelay, LOW);
  server.send(200, "text/plain", "GPIO 6 is now OFF. Automated control resumed.");
}

/**
 * @brief Sets the fan relay ON for 1 hour.
 */
void handleOn1h() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  currentMode = MANUAL_ON_TIMED;
  digitalWrite(fanrelay, HIGH);
  manualTimerEnd = millis() + 3600000;
  server.send(200, "text/plain", "GPIO 6 is now manually ON for 1 hour.");
}

/**
 * @brief Sets the fan relay ON for 2 hours.
 */
void handleOn2h() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  currentMode = MANUAL_ON_TIMED;
  digitalWrite(fanrelay, HIGH);
  manualTimerEnd = millis() + 7200000;
  server.send(200, "text/plain", "GPIO 6 is now manually ON for 2 hours.");
}

/**
 * @brief Sets the temperature ON threshold.
 */
void handleSetTempOn() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (server.hasArg("value")) {
    tempOn = server.arg("value").toFloat();
    preferences.putFloat("tempOn", tempOn);
    server.send(200, "text/plain", "Temperature ON threshold set to " + String(tempOn) + "C.");
  } else {
    server.send(400, "text/plain", "Missing 'value' parameter.");
  }
}

/**
 * @brief Sets the temperature OFF threshold.
 */
void handleSetTempOff() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (server.hasArg("value")) {
    tempOff = server.arg("value").toFloat();
    preferences.putFloat("tempOff", tempOff);
    server.send(200, "text/plain", "Temperature OFF threshold set to " + String(tempOff) + "C.");
  } else {
    server.send(400, "text/plain", "Missing 'value' parameter.");
  }
}

/**
 * @brief Sets the night LED brightness.
 */
void handleSetBrightness() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (server.hasArg("value")) {
    currentBrightnessDutyCycle = server.arg("value").toInt();
    if (currentBrightnessDutyCycle > 255) currentBrightnessDutyCycle = 255;
    if (currentBrightnessDutyCycle < 0) currentBrightnessDutyCycle = 0;
    preferences.putUInt("brightness", currentBrightnessDutyCycle);

    ledc_set_duty(ledcMode, ledcChannel_ldr, currentBrightnessDutyCycle);
    ledc_update_duty(ledcMode, ledcChannel_ldr);

    server.send(200, "text/plain", "LED brightness level set to " + String((int)(currentBrightnessDutyCycle / 2.55)) + "%..");
  } else {
    server.send(400, "text/plain", "Missing 'value' parameter.");
  }
}

/**
 * @brief Sets the light sensor threshold.
 */
void handleSetLightThreshold() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (server.hasArg("value")) {
    lightThreshold = server.arg("value").toInt();
    preferences.putUInt("lightThreshold", lightThreshold);
    server.send(200, "text/plain", "Light sensor threshold set to " + String(lightThreshold) + ".");
  } else {
    server.send(400, "text/plain", "Missing 'value' parameter.");
  }
}

/**
 * @brief Sets the main LED brightness.
 */
void handleSetMainLedBrightness() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (server.hasArg("value")) {
    mainLedBrightnessDutyCycle = server.arg("value").toInt();
    if (mainLedBrightnessDutyCycle > 255) mainLedBrightnessDutyCycle = 255;
    if (mainLedBrightnessDutyCycle < 0) mainLedBrightnessDutyCycle = 0;
    preferences.putUInt("mainLedBrightness", mainLedBrightnessDutyCycle);

    server.send(200, "text/plain", "Main LED brightness level set to " + String((int)(mainLedBrightnessDutyCycle / 2.55)) + "%..");
  } else {
    server.send(400, "text/plain", "Missing 'value' parameter.");
  }
}

/**
 * @brief Toggles the main LED state and saves it.
 */
void handleToggleMainLed() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (proximityManualState) {
    mainLedManualState = !mainLedManualState;
    preferences.putBool("mainLedManualState", mainLedManualState);
    server.send(200, "text/plain", "Main LED state toggled to " + String(mainLedManualState ? "ON" : "OFF") + ".");
  } else {
    server.send(200, "text/plain", "Master switch is OFF. Cannot toggle main LED.");
  }
}

/**
 * @brief Toggles the master switch state and saves it.
 */
void handleToggleMasterSwitch() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  proximityManualState = !proximityManualState;
  preferences.putBool("proximityManualState", proximityManualState);
  server.send(200, "text/plain", "Master switch state toggled to " + String(proximityManualState ? "ON" : "OFF") + ".");
}

/**
 * @brief Provides sensor data and current status in JSON format.
 */
void handleData() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  StaticJsonDocument<512> jsonDoc;
  char tempStr[6];
  sprintf(tempStr, "%.1f", lastTemperature);

  jsonDoc["temperature"] = tempStr;
  jsonDoc["humidity"] = (int)lastHumidity;
  jsonDoc["light"] = lastLight;
  jsonDoc["gpio_status"] = digitalRead(fanrelay) == HIGH ? "on" : "off";
  jsonDoc["master_switch_state"] = proximityManualState ? "on" : "off";
  jsonDoc["main_led_status"] = mainLedManualState ? "on" : "off";
  jsonDoc["control_mode"] = currentMode == AUTOMATED ? "Automated" : (currentMode == MANUAL_ON_PERMANENT ? "Manual (Permanent)" : "Manual (Timed)");
  jsonDoc["ldr_brightness_level"] = (int)(currentBrightnessDutyCycle / 2.55);
  jsonDoc["main_led_brightness_level"] = (int)(mainLedBrightnessDutyCycle / 2.55);
  jsonDoc["temp_on_threshold"] = tempOn;
  jsonDoc["temp_off_threshold"] = tempOff;
  jsonDoc["light_threshold"] = lightThreshold;

  String response;
  serializeJson(jsonDoc, response);
  server.send(200, "application/json", response);
}

// === Setup ===
void setup() {
  setCpuFrequencyMhz(80);
  Serial.begin(9600);
  dht.begin();
  preferences.begin("my-app", false);

  tempOn = preferences.getFloat("tempOn", 29.0);
  tempOff = preferences.getFloat("tempOff", 28.5);
  currentBrightnessDutyCycle = preferences.getUInt("brightness", 128);
  lightThreshold = preferences.getUInt("lightThreshold", 2350);
  mainLedBrightnessDutyCycle = preferences.getUInt("mainLedBrightness", 255);
  ledcBaseFreq = preferences.getUInt("pwmFrequency", 5000);
  mainLedBaseFreq = preferences.getUInt("mainLedPwmFrequency", 5000);
  proximityManualState = preferences.getBool("proximityManualState", false);
  mainLedManualState = preferences.getBool("mainLedManualState", false);

  pinMode(fanrelay, OUTPUT);
  pinMode(mainledsw, INPUT);
  pinMode(proximitysw, INPUT);
  pinMode(mainled, OUTPUT);

  // Configure PWM timer for LDR LED
  ledc_timer_config_t ledcTimerConfig_ldr = {
    .speed_mode = ledcMode,
    .duty_resolution = ledcResolution,
    .timer_num = ledcTimer_ldr,
    .freq_hz = ledcBaseFreq,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledcTimerConfig_ldr);

  // Configure PWM timer for main LED
  ledc_timer_config_t ledcTimerConfig_mainLed = {
    .speed_mode = ledcMode,
    .duty_resolution = ledcResolution,
    .timer_num = ledcTimer_mainLed,
    .freq_hz = mainLedBaseFreq,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledcTimerConfig_mainLed);

  // Configure PWM channel for LDR LED
  ledc_channel_config_t ledcChannelConfig_ldr = {
    .gpio_num = nightled,
    .speed_mode = ledcMode,
    .channel = ledcChannel_ldr,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = ledcTimer_ldr,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&ledcChannelConfig_ldr);

  // Configure PWM channel for main LED
  ledc_channel_config_t ledcChannelConfig_mainLed = {
    .gpio_num = mainled,
    .speed_mode = ledcMode,
    .channel = ledcChannel_mainLed,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = ledcTimer_mainLed,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&ledcChannelConfig_mainLed);

  // Set initial duty cycle for LDR light
  ledc_set_duty(ledcMode, ledcChannel_ldr, currentBrightnessDutyCycle);
  ledc_update_duty(ledcMode, ledcChannel_ldr);

  WiFi.config(staticIP, gateway, subnet);
  WiFi.begin(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  server.on("/on-perm", handleOn);
  server.on("/off", handleOff);
  server.on("/on-1h", handleOn1h);
  server.on("/on-2h", handleOn2h);
  server.on("/set-temp-on", handleSetTempOn);
  server.on("/set-temp-off", handleSetTempOff);
  server.on("/set-brightness", handleSetBrightness);
  server.on("/set-light-threshold", handleSetLightThreshold);
  server.on("/set-main-led-brightness", handleSetMainLedBrightness);
  server.on("/toggle-main-led", handleToggleMainLed);
  server.on("/toggle-master-switch", handleToggleMasterSwitch);
  server.on("/data", handleData);
  server.begin();

  readSensors();
  lastLDRState = lastLight < lightThreshold;
  if (lastLDRState) {
    ledc_set_duty(ledcMode, ledcChannel_ldr, currentBrightnessDutyCycle);
    ledc_update_duty(ledcMode, ledcChannel_ldr);
  } else {
    ledc_set_duty(ledcMode, ledcChannel_ldr, 0);
    ledc_update_duty(ledcMode, ledcChannel_ldr);
  }
}

// === Loop ===
void loop() {
  server.handleClient();
  unsigned long currentMillis = millis();

  // Handle sensor reading on a timed interval
  if (currentMillis - previousSensorMillis >= sensorInterval) {
    previousSensorMillis = currentMillis;
    readSensors();
  }

  // Handle latching for proximitysw (master switch toggle)
  currentProximityValue = digitalRead(proximitysw);
  if (currentProximityValue == HIGH && lastProximityValue == LOW) {
    proximityManualState = !proximityManualState;
    preferences.putBool("proximityManualState", proximityManualState);
  }
  lastProximityValue = currentProximityValue;

  // Master Control: proximityManualState dictates main operations
  if (proximityManualState) {
    // Handle latching for mainledsw (main LED toggle)
    currentTouchValue = digitalRead(mainledsw);
    if (currentTouchValue == HIGH && lastTouchValue == LOW) {
      mainLedManualState = !mainLedManualState;
      preferences.putBool("mainLedManualState", mainLedManualState);
    }
    lastTouchValue = currentTouchValue;

    // Control Main LED
    if (mainLedManualState) {
      ledc_set_duty(ledcMode, ledcChannel_mainLed, mainLedBrightnessDutyCycle);
    } else {
      ledc_set_duty(ledcMode, ledcChannel_mainLed, 0);
    }
    ledc_update_duty(ledcMode, ledcChannel_mainLed);

    // Control Fan
    // Check for manual timed mode timeout first
    if (currentMode == MANUAL_ON_TIMED && currentMillis >= manualTimerEnd) {
      currentMode = AUTOMATED;
      digitalWrite(fanrelay, LOW);
    }
    
    if (currentMode == AUTOMATED) {
      if (lastTemperature >= tempOn) {
        digitalWrite(fanrelay, HIGH);
      } else if (lastTemperature <= tempOff) {
        digitalWrite(fanrelay, LOW);
      }
    }
  } else {
    // If master switch is OFF, turn off both fan and main LED
    digitalWrite(fanrelay, LOW);
    ledc_set_duty(ledcMode, ledcChannel_mainLed, 0);
    ledc_update_duty(ledcMode, ledcChannel_mainLed);
  }

  // === LDR and Brightness Control Logic ===
  int currentLight = analogRead(lightsensor);
  bool currentLDRState = currentLight < lightThreshold;

  // Use a simple debounce logic for LDR
  if (currentLDRState != lastLDRState) {
    lastLDRChangeTime = currentMillis;
  }

  if ((currentMillis - lastLDRChangeTime) >= debounceDelay) {
    if (currentLDRState) {
      ledc_set_duty(ledcMode, ledcChannel_ldr, currentBrightnessDutyCycle);
    } else {
      ledc_set_duty(ledcMode, ledcChannel_ldr, 0);
    }
    ledc_update_duty(ledcMode, ledcChannel_ldr);
  }
  lastLDRState = currentLDRState;
}
