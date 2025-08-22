#include "DHT.h"
#include "WiFi.h"
#include "WebServer.h"
#include "ArduinoJson.h"
#include "Preferences.h"
#include "driver/ledc.h"
#include "HTTPClient.h"

// DHT Sensor Pin and Type
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Pin Definitions
const int fanrelay = 6;
const int lightsensor = 2;
const int nightled = 7;
const int mainledsw = 0;
const int proximitysw = 3;
const int mainled = 1;
const int reverseled = 8; // Added new pin for the reverse LED

// Debounce and Sensor Reading Variables
int currentTouchValue = 0;
int lastTouchValue = 0;
int currentProximityValue = 0;
int lastProximityValue = 0;

int lightThreshold = 2350;

// Software Averaging for Light Sensor
const int NUM_LDR_READINGS = 100;
int ldrReadings[NUM_LDR_READINGS];
int ldrReadIndex = 0;
long ldrTotal = 0;
int ldrAverage = 0;
int ldrBlinkCounter = 0; // Counter for LDR blink

// LEDC (PWM) Configuration for Night LED
const ledc_mode_t ledcMode = LEDC_LOW_SPEED_MODE;
const ledc_timer_bit_t ledcResolution = LEDC_TIMER_13_BIT;
const int ledcMaxValue = 8191;
const int ledcMinValue = 20;

const ledc_timer_t ledcTimer_ldr = LEDC_TIMER_0;
const ledc_channel_t ledcChannel_ldr = LEDC_CHANNEL_0;
int ledcBaseFreq = 5000;
int currentBrightnessDutyCycle = 4096;

// LEDC (PWM) Configuration for Main LED
const ledc_timer_t ledcTimer_mainLed = LEDC_TIMER_1;
const ledc_channel_t ledcChannel_mainLed = LEDC_CHANNEL_1;
int mainLedBaseFreq = 5000;
int mainLedBrightnessDutyCycle = 8191;

// Temperature and Humidity Offsets
const float tempOffset = -0.1;
const float humidityOffset = 6.0;

// Fan Control Thresholds
float tempOn = 29.0;
float tempOff = 28.5;

// WiFi Configuration
const char* ssid = "wifi_slow";
IPAddress staticIP(192, 168, 1, 4);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);
Preferences preferences;

// Timers for non-blocking operations
unsigned long previousDHTMillis = 0;
const long dhtInterval = 5000;
unsigned long previousLDRMillis = 0;
const long ldrInterval = 40; // Read every 40ms for a 4s average
unsigned long lastLDRChangeTime = 0;
long debounceDelay = 50;

// Sensor Data Storage
float lastHumidity = 0;
float lastTemperature = 0;
int lastLight = 0;
bool lastLDRState;
bool mainLedManualState = false;
bool proximityManualState = false;
bool lastFanState = false;

// Fan Control Mode
enum ControlMode { AUTOMATED, MANUAL_ON_PERMANENT, MANUAL_ON_TIMED };
ControlMode currentMode = AUTOMATED;
unsigned long manualTimerEnd = 0;

void readDHT() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    return;
  }
  lastHumidity = h + humidityOffset;
  lastTemperature = t + tempOffset;
}

void updateLdrAverage() {
  // Read and average light sensor
  ldrTotal = ldrTotal - ldrReadings[ldrReadIndex];
  ldrReadings[ldrReadIndex] = analogRead(lightsensor);
  ldrTotal = ldrTotal + ldrReadings[ldrReadIndex];
  ldrReadIndex = ldrReadIndex + 1;

  if (ldrReadIndex >= NUM_LDR_READINGS) {
    ldrReadIndex = 0;
  }
  
  ldrAverage = ldrTotal / NUM_LDR_READINGS;
  lastLight = ldrAverage;
}

// Function to blink the reverse LED for a short duration
void blinkReverseLed() {
  digitalWrite(reverseled, LOW);  // Turn on the LED (inverse logic)
  delay(50);                      // Wait for 50 milliseconds
  digitalWrite(reverseled, HIGH); // Turn off the LED (inverse logic)
}

void handleOn() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  currentMode = MANUAL_ON_PERMANENT;
  digitalWrite(fanrelay, HIGH);
  blinkReverseLed();
  server.send(200, "text/plain", "OK");
}

void handleOff() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  currentMode = AUTOMATED;
  digitalWrite(fanrelay, LOW);
  blinkReverseLed();
  server.send(200, "text/plain", "OK");
}

void handleOn1h() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  currentMode = MANUAL_ON_TIMED;
  digitalWrite(fanrelay, HIGH);
  manualTimerEnd = millis() + 3600000;
  blinkReverseLed();
  server.send(200, "text/plain", "OK");
}

void handleOn30m() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  currentMode = MANUAL_ON_TIMED;
  digitalWrite(fanrelay, HIGH);
  manualTimerEnd = millis() + 1800000;
  blinkReverseLed();
  server.send(200, "text/plain", "OK");
}

void handleSetTempOn() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (server.hasArg("value")) {
    tempOn = server.arg("value").toFloat();
    preferences.putFloat("tempOn", tempOn);
    blinkReverseLed();
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Error");
  }
}

void handleSetTempOff() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (server.hasArg("value")) {
    tempOff = server.arg("value").toFloat();
    preferences.putFloat("tempOff", tempOff);
    blinkReverseLed();
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Error");
  }
}

void handleSetBrightness() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (server.hasArg("value")) {
    int requestedValue = server.arg("value").toInt();
    if (requestedValue >= 0 && requestedValue <= ledcMaxValue) {
      currentBrightnessDutyCycle = requestedValue;
      preferences.putUInt("brightness", currentBrightnessDutyCycle);
      // Immediately set the new brightness if the LED is currently on
      if (ledc_get_duty(ledcMode, ledcChannel_ldr) > 0) {
        ledc_set_duty(ledcMode, ledcChannel_ldr, currentBrightnessDutyCycle);
        ledc_update_duty(ledcMode, ledcChannel_ldr);
      }
      blinkReverseLed();
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Error");
    }
  } else {
    server.send(400, "text/plain", "Error");
  }
}

void handleSetLightThreshold() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (server.hasArg("value")) {
    lightThreshold = server.arg("value").toInt();
    preferences.putUInt("lightThreshold", lightThreshold);
    blinkReverseLed();
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Error");
  }
}

void handleSetMainLedBrightness() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (server.hasArg("value")) {
    int requestedValue = server.arg("value").toInt();
    if (requestedValue >= 0 && requestedValue <= ledcMaxValue) {
      mainLedBrightnessDutyCycle = requestedValue;
      preferences.putUInt("mainLedBrightness", mainLedBrightnessDutyCycle);
      blinkReverseLed();
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Error");
    }
  } else {
    server.send(400, "text/plain", "Error");
  }
}

void handleSetDebounceDelay() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (server.hasArg("value")) {
    debounceDelay = server.arg("value").toInt();
    preferences.putUInt("debounceDelay", debounceDelay);
    blinkReverseLed();
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Error");
  }
}

void handleToggleMainLed() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (proximityManualState) {
    mainLedManualState = !mainLedManualState;
    preferences.putBool("mainLedManualState", mainLedManualState);
    blinkReverseLed();
    server.send(200, "text/plain", "OK");
  } else {
    blinkReverseLed();
    server.send(200, "text/plain", "OK");
  }
}

void handleToggleMasterSwitch() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  proximityManualState = !proximityManualState;
  preferences.putBool("proximityManualState", proximityManualState);
  blinkReverseLed();
  server.send(200, "text/plain", "OK");
}

void handleData() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  StaticJsonDocument<512> jsonDoc;
  char tempStr[6];
  sprintf(tempStr, "%.1f", lastTemperature);

  unsigned long uptimeSeconds = millis() / 1000;

  jsonDoc["temperature"] = tempStr;
  jsonDoc["humidity"] = (int)lastHumidity;
  jsonDoc["light"] = lastLight;
  jsonDoc["gpio_status"] = digitalRead(fanrelay) == HIGH ? "on" : "off";
  jsonDoc["master_switch_state"] = proximityManualState ? "on" : "off";
  jsonDoc["main_led_status"] = mainLedManualState ? "on" : "off";
  jsonDoc["control_mode"] = currentMode == AUTOMATED ? "Automated" : (currentMode == MANUAL_ON_PERMANENT ? "Manual (Permanent)" : "Manual (Timed)");
  float nightLedPercent = (float)ledc_get_duty(ledcMode, ledcChannel_ldr) / ledcMaxValue * 100.0;
  float mainLedPercent = (float)ledc_get_duty(ledcMode, ledcChannel_mainLed) / ledcMaxValue * 100.0;
  jsonDoc["ldr_brightness_level"] = nightLedPercent;
  jsonDoc["main_led_brightness_level"] = mainLedPercent;
  jsonDoc["temp_on_threshold"] = tempOn;
  jsonDoc["temp_off_threshold"] = tempOff;
  jsonDoc["light_threshold"] = lightThreshold;
  jsonDoc["debounce_delay_ms"] = debounceDelay;
  jsonDoc["uptime_seconds"] = uptimeSeconds;

  String response;
  serializeJson(jsonDoc, response);
  blinkReverseLed();
  server.send(200, "application/json", response);
}

void setup() {
  setCpuFrequencyMhz(80);
  
  dht.begin();
  preferences.begin("my-app", false);

  tempOn = preferences.getFloat("tempOn", 29.0);
  tempOff = preferences.getFloat("tempOff", 28.5);
  currentBrightnessDutyCycle = preferences.getUInt("brightness", 4096);
  if (currentBrightnessDutyCycle == 0) {
    currentBrightnessDutyCycle = ledcMinValue;
  }
  lightThreshold = preferences.getUInt("lightThreshold", 2350);
  mainLedBrightnessDutyCycle = preferences.getUInt("mainLedBrightness", 8191);
  ledcBaseFreq = preferences.getUInt("pwmFrequency", 5000);
  mainLedBaseFreq = preferences.getUInt("mainLedPwmFrequency", 5000);
  proximityManualState = preferences.getBool("proximityManualState", false);
  mainLedManualState = preferences.getBool("mainLedManualState", false);
  debounceDelay = preferences.getUInt("debounceDelay", 50);

  pinMode(fanrelay, OUTPUT);
  pinMode(mainledsw, INPUT);
  pinMode(proximitysw, INPUT);
  pinMode(mainled, OUTPUT);
  pinMode(reverseled, OUTPUT); // Configure the new reverse LED pin
  digitalWrite(fanrelay, LOW);
  digitalWrite(reverseled, HIGH); // Start with the reverse LED off (inverse logic)

  ledc_timer_config_t ledcTimerConfig_ldr = {
    .speed_mode = ledcMode,
    .duty_resolution = ledcResolution,
    .timer_num = ledcTimer_ldr,
    .freq_hz = ledcBaseFreq,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledcTimerConfig_ldr);

  ledc_timer_config_t ledcTimerConfig_mainLed = {
    .speed_mode = ledcMode,
    .duty_resolution = ledcResolution,
    .timer_num = ledcTimer_mainLed,
    .freq_hz = mainLedBaseFreq,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledcTimerConfig_mainLed);

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

  ledc_set_duty(ledcMode, ledcChannel_mainLed, 0);
  ledc_update_duty(ledcMode, ledcChannel_mainLed);
  ledc_set_duty(ledcMode, ledcChannel_ldr, currentBrightnessDutyCycle);
  ledc_update_duty(ledcMode, ledcChannel_ldr);

  // Initialize all LDR readings to 0
  for (int thisReading = 0; thisReading < NUM_LDR_READINGS; thisReading++) {
    ldrReadings[thisReading] = 0;
  }

  WiFi.config(staticIP, gateway, subnet);
  WiFi.begin(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  server.on("/on-perm", handleOn);
  server.on("/off", handleOff);
  server.on("/on-1h", handleOn1h);
  server.on("/on-30m", handleOn30m);
  server.on("/set-temp-on", handleSetTempOn);
  server.on("/set-temp-off", handleSetTempOff);
  server.on("/set-brightness", handleSetBrightness);
  server.on("/set-light-threshold", handleSetLightThreshold);
  server.on("/set-main-led-brightness", handleSetMainLedBrightness);
  server.on("/set-debounce-delay", handleSetDebounceDelay);
  server.on("/toggle-main-led", handleToggleMainLed);
  server.on("/toggle-master-switch", handleToggleMasterSwitch);
  server.on("/data", handleData);
  server.begin();

  readDHT();
  updateLdrAverage();
  lastLDRState = lastLight < lightThreshold;
  if (lastLDRState) {
    ledc_set_duty(ledcMode, ledcChannel_ldr, currentBrightnessDutyCycle);
    ledc_update_duty(ledcMode, ledcChannel_ldr);
  } else {
    ledc_set_duty(ledcMode, ledcChannel_ldr, 0);
    ledc_update_duty(ledcMode, ledcChannel_ldr);
  }

  lastProximityValue = digitalRead(proximitysw);
  lastFanState = digitalRead(fanrelay);
}

void loop() {
  yield();
  server.handleClient();
  unsigned long currentMillis = millis();

  // Read DHT sensor every 5 seconds
  if (currentMillis - previousDHTMillis >= dhtInterval) {
    previousDHTMillis = currentMillis;
    readDHT();
    blinkReverseLed(); // Blink the LED after the DHT read
  }

  // Update LDR average every 40ms for a smoother 4s average
  if (currentMillis - previousLDRMillis >= ldrInterval) {
    previousLDRMillis = currentMillis;
    updateLdrAverage();
    // Check if a full averaging cycle is complete
    ldrBlinkCounter++;
    if (ldrBlinkCounter >= NUM_LDR_READINGS) {
      ldrBlinkCounter = 0;
      blinkReverseLed(); // Blink the LED after a full LDR average is complete
    }
  }

  currentProximityValue = digitalRead(proximitysw);
  if (currentProximityValue == HIGH && lastProximityValue == LOW) {
    proximityManualState = !proximityManualState;
    preferences.putBool("proximityManualState", proximityManualState);
  }
  lastProximityValue = currentProximityValue;

  if (proximityManualState) {
    currentTouchValue = digitalRead(mainledsw);
    if (currentTouchValue == HIGH && lastTouchValue == LOW) {
      mainLedManualState = !mainLedManualState;
      preferences.putBool("mainLedManualState", mainLedManualState);
    }
    lastTouchValue = currentTouchValue;

    if (mainLedManualState) {
      ledc_set_duty(ledcMode, ledcChannel_mainLed, mainLedBrightnessDutyCycle);
    } else {
      ledc_set_duty(ledcMode, ledcChannel_mainLed, 0);
    }
    ledc_update_duty(ledcMode, ledcChannel_mainLed);

    if (currentMode == MANUAL_ON_TIMED && currentMillis >= manualTimerEnd) {
      currentMode = AUTOMATED;
      digitalWrite(fanrelay, LOW);
    }
    
    if (currentMode == AUTOMATED) {
      if (lastTemperature >= tempOn && digitalRead(fanrelay) == LOW) {
        digitalWrite(fanrelay, HIGH);
      } else if (lastTemperature <= tempOff && digitalRead(fanrelay) == HIGH) {
        digitalWrite(fanrelay, LOW);
      }
    }
  } else {
    if (digitalRead(fanrelay) == HIGH) {
      digitalWrite(fanrelay, LOW);
    }
    if (ledc_get_duty(ledcMode, ledcChannel_mainLed) > 0) {
      ledc_set_duty(ledcMode, ledcChannel_mainLed, 0);
      ledc_update_duty(ledcMode, ledcChannel_mainLed);
    }
  }

  // The LDR state is now based on the averaged reading from updateLdrAverage()
  bool currentLDRState = lastLight < lightThreshold;

  if (currentLDRState != lastLDRState) {
    lastLDRChangeTime = currentMillis;
  }

  if ((currentMillis - lastLDRChangeTime) >= debounceDelay) {
    if (currentLDRState) {
      // It's dark, so the night LED turns on
      if (ledc_get_duty(ledcMode, ledcChannel_ldr) != currentBrightnessDutyCycle) {
        ledc_set_duty(ledcMode, ledcChannel_ldr, currentBrightnessDutyCycle);
        ledc_update_duty(ledcMode, ledcChannel_ldr);
      }
    } else {
      // It's light, so the night LED turns off
      if (ledc_get_duty(ledcMode, ledcChannel_ldr) > 0) {
        ledc_set_duty(ledcMode, ledcChannel_ldr, 0);
        ledc_update_duty(ledcMode, ledcChannel_ldr);
      }
    }
  }
  lastLDRState = currentLDRState;
}
