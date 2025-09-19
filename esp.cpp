#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "time.h"
#include <driver/ledc.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HTTPClient.h>

#define MAINLED_PIN 1
#define NIGHTLED_PIN 7
#define MAINLEDSW_PIN 2
#define MASTERSW_PIN 3
#define DS18B20_PIN 4
#define FAN_PIN 6
#define ACTIVITY_LED_PIN 0

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_SDA 8
#define OLED_SCL 9
#define OLED_ADDR 0x3C

#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 8
#define LED_CHANNEL 0

const char* ssid = "wifi_slow2";

IPAddress staticIP(192, 168, 1, 4);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

const char* ntpServer = "192.168.1.2";
const long gmtOffset_sec = 8 * 3600;
const int daylightOffset_sec = 0;

int mainledswState = 0;
int lastMainledswReading = 0;
int masterswState = 0;
int lastMasterswReading = 0;
int nightledPWMValue = 0;

OneWire oneWireBus(DS18B20_PIN);
DallasTemperature sensors(&oneWireBus);
DeviceAddress tempSensorAddress;
float tempThresholdOn = 28.9;
float tempThresholdOff = 28.7;
float currentTemperature = 0.0;
unsigned long lastSensorReadTime = 0;
const unsigned long sensorReadInterval = 4000;

bool fanOverride = false;
unsigned long fanOverrideStartTime = 0;
const unsigned long fanOverrideDuration = 30 * 60 * 1000;
unsigned long lastFanStateChange = 0;
const unsigned long fanCooldownDelay = 5000;
bool fanIsOnAutomatic = true;
bool serverStarted = false;
bool isAutoToggleDone = false;
bool isOledOn = true;

Preferences preferences;
WebServer server(80);

const int maxFailedReads = 5;
int consecutiveFailedReads = 0;
bool sensorIsFaulty = false;

bool isBlinking = false;
unsigned long blinkStartTime = 0;
const unsigned long blinkDuration = 10;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

String solarStatus = "N/A";
float outdoorTemp = 0.0;
float currentPower = 0.0;
unsigned long lastExternalDataFetch = 0;
const unsigned long externalDataFetchInterval = 10000;

void triggerBlink() {
  digitalWrite(ACTIVITY_LED_PIN, HIGH);
  blinkStartTime = millis();
  isBlinking = true;
}

void onWiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      WiFi.begin(ssid);
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      break;
    default:
      break;
  }
}

bool isTimeInRange(int startHour, int startMin, int endHour, int endMin) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return false;
  }
  
  int nowInMinutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;
  int startInMinutes = startHour * 60 + startMin;
  int endInMinutes = endHour * 60 + endMin;

  if (startInMinutes > endInMinutes) {
    return (nowInMinutes >= startInMinutes || nowInMinutes < endInMinutes);
  } else {
    return (nowInMinutes >= startInMinutes && nowInMinutes < endInMinutes);
  }
}

void handleMainToggle() {
  triggerBlink();
  if (masterswState == 1) {
    mainledswState = 1 - mainledswState;
  }
  String response = "Main LED Switch state is now: " + String(mainledswState);
  server.send(200, "text/plain", response);
}

void handleMasterToggle() {
  triggerBlink();
  masterswState = 1 - masterswState;
  String response = "Master Switch state is now: " + String(masterswState);
  server.send(200, "text/plain", response);
}

void handleSetTempOn() {
  triggerBlink();
  if (!server.hasArg("tempOn")) {
    server.send(400, "text/plain", "Missing 'tempOn' parameter.");
    return;
  }
  
  float newTempOn = server.arg("tempOn").toFloat();
  if (newTempOn > tempThresholdOff) {
    tempThresholdOn = newTempOn;
    preferences.begin("bilik-config", false);
    preferences.putFloat("tempOn", tempThresholdOn);
    preferences.end();
    server.send(200, "text/plain", "Fan 'tempOn' threshold set successfully.");
  } else {
    server.send(400, "text/plain", "New 'tempOn' must be greater than current 'tempOff'.");
  }
}

void handleSetTempOff() {
  triggerBlink();
  if (!server.hasArg("tempOff")) {
    server.send(400, "text/plain", "Missing 'tempOff' parameter.");
    return;
  }
  
  float newTempOff = server.arg("tempOff").toFloat();
  if (tempThresholdOn > newTempOff) {
    tempThresholdOff = newTempOff;
    preferences.begin("bilik-config", false);
    preferences.putFloat("tempOff", tempThresholdOff);
    preferences.end();
    server.send(200, "text/plain", "Fan 'tempOff' threshold set successfully.");
  } else {
    server.send(400, "text/plain", "New 'tempOff' must be less than current 'tempOn'.");
  }
}

void handleFanOn30m() {
  triggerBlink();
  if (masterswState == 1 && fanIsOnAutomatic == true && digitalRead(FAN_PIN) == LOW) {
    fanOverride = true;
    fanOverrideStartTime = millis();
    fanIsOnAutomatic = false;
    digitalWrite(FAN_PIN, HIGH);
    server.send(200, "text/plain", "Fan turned on for 30 minutes. Automatic mode is suspended.");
  } else {
    server.send(400, "text/plain", "Cannot turn fan on. Master switch is off, fan is not in automatic mode, or fan is already on.");
  }
}

void handleFanOff() {
  triggerBlink();
  if (fanIsOnAutomatic && digitalRead(FAN_PIN) == HIGH) {
    server.send(400, "text/plain", "Cannot turn fan off: Fan is currently running in automatic mode.");
  } else {
    digitalWrite(FAN_PIN, LOW);
    fanOverride = false;
    fanIsOnAutomatic = true;
    server.send(200, "text/plain", "Fan turned off. Automatic mode is restored.");
  }
}

void handleSetNightledPWM() {
  triggerBlink();
  if (!server.hasArg("pwmValue")) {
    server.send(400, "text/plain", "Missing 'pwmValue' parameter.");
    return;
  }
  
  int newPWM = server.arg("pwmValue").toInt();
  if (newPWM >= 0 && newPWM <= 255) {
    nightledPWMValue = newPWM;
    preferences.begin("bilik-config", false);
    preferences.putInt("nightledPWM", nightledPWMValue);
    preferences.end();
    server.send(200, "text/plain", "Night LED PWM value set successfully.");
  } else {
    server.send(400, "text/plain", "PWM value must be between 0 and 255.");
  }
}

bool shouldNightLedBeOn() {
  return isTimeInRange(19, 15, 7, 15) && (mainledswState == 0 || masterswState == 0);
}

void handleITemp() {
  triggerBlink();
  if (sensorIsFaulty) {
    server.send(404, "text/plain", "Sensor is faulty.");
  } else {
    server.send(200, "text/plain", String(currentTemperature));
  }
}

void handleState() {
  triggerBlink();
  StaticJsonDocument<300> doc;
  
  struct tm timeinfo;
  char timeString[64];
  getLocalTime(&timeinfo);
  strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo);
  doc["currentTime"] = timeString;

  long uptimeSeconds = millis() / 1000;
  doc["uptimeSeconds"] = uptimeSeconds;

  doc["mainledswState"] = mainledswState;
  doc["masterswState"] = masterswState;
  doc["mainledState"] = digitalRead(MAINLED_PIN);
  
  doc["nightledState"] = shouldNightLedBeOn();
  doc["nightledPWMValue"] = nightledPWMValue;

  doc["temperature"] = currentTemperature;
  doc["fanState"] = digitalRead(FAN_PIN);
  doc["tempThresholdOn"] = tempThresholdOn;
  doc["tempThresholdOff"] = tempThresholdOff;
  doc["fanOverride"] = fanOverride;
  doc["sensorIsFaulty"] = sensorIsFaulty;
  doc["fanIsOnAutomatic"] = fanIsOnAutomatic;

  String jsonResponse;
  serializeJson(doc, jsonResponse);

  server.send(200, "application/json", jsonResponse);
}

void controlNightLED() {
  if (shouldNightLedBeOn()) {
    ledcWrite(NIGHTLED_PIN, nightledPWMValue);
  } else {
    ledcWrite(NIGHTLED_PIN, 0);
  }
}

void fetchExternalData() {
  triggerBlink();
  HTTPClient http;

  http.begin("http://192.168.1.3/api/esp");
  int httpCode = http.GET();
  if (httpCode > 0) {
    String payload = http.getString();
    StaticJsonDocument<100> doc;
    deserializeJson(doc, payload);
    
    if (doc.containsKey("relay_state")) {
      solarStatus = doc["relay_state"].as<String>();
    }
    if (doc.containsKey("outdoor_temp_C")) {
      outdoorTemp = doc["outdoor_temp_C"].as<float>();
    }
    if (doc.containsKey("power_mW")) {
      currentPower = doc["power_mW"].as<float>() / 1000.0;
    }
  }
  http.end();
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.setTextSize(1);
  display.print("WiFi: [");
  long rssi = WiFi.RSSI();
  int bars = map(rssi, -100, -30, 0, 12);
  for (int i = 0; i < 12; i++) {
    if (i < bars) {
      display.print("=");
    } else {
      display.print(" ");
    }
  }
  display.println("]");
  
  display.setCursor(0, 16);
  display.setTextSize(1);
  display.print("I");
  display.setCursor(18, 16);
  display.setTextSize(2);
  display.print(String(currentTemperature, 1));
  
  display.setTextSize(2);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(solarStatus, 0, 0, &x1, &y1, &w, &h);
  display.setCursor(SCREEN_WIDTH - w, 16);
  display.print(solarStatus);

  display.setCursor(0, 40);
  display.setTextSize(1);
  display.print("O");
  display.setCursor(18, 40);
  display.setTextSize(2);
  display.print(String(outdoorTemp, 1));
  
  String powerString = String(currentPower, 2);
  display.getTextBounds(powerString, 0, 0, &x1, &y1, &w, &h);
  display.setCursor(SCREEN_WIDTH - w, 40);
  display.setTextSize(2);
  display.print(powerString);
  
  display.display();
}

void setup() {
  setCpuFrequencyMhz(80);
  
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Booting...");
  display.display();
  delay(1000);

  WiFi.onEvent(onWiFiEvent);
  WiFi.config(staticIP, gateway, subnet);
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Connecting to WiFi...");
  display.display();
  WiFi.begin(ssid);

  int attempt = 0;
  while (WiFi.status() != WL_CONNECTED && attempt < 20) {
    delay(500);
    attempt++;
  }
  
  display.clearDisplay();
  display.setCursor(0, 0);
  if (WiFi.status() == WL_CONNECTED) {
    display.println("WiFi Connected!");
    display.print("IP: ");
    display.println(WiFi.localIP());
  } else {
    display.println("WiFi Failed!");
    display.println("Using static IP");
  }
  display.display();
  delay(2000);

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  preferences.begin("bilik-config", false);
  tempThresholdOn = preferences.getFloat("tempOn", 28.9);
  tempThresholdOff = preferences.getFloat("tempOff", 28.7);
  nightledPWMValue = preferences.getInt("nightledPWM", 0);
  preferences.end();

  pinMode(MAINLED_PIN, OUTPUT);
  pinMode(NIGHTLED_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(ACTIVITY_LED_PIN, OUTPUT);
  pinMode(MAINLEDSW_PIN, INPUT_PULLUP);
  pinMode(MASTERSW_PIN, INPUT_PULLUP);

  sensors.begin();
  sensors.setResolution(tempSensorAddress, 12);

  tempSensorAddress[0] = 0x28;
  tempSensorAddress[1] = 0x07;
  tempSensorAddress[2] = 0xBB;
  tempSensorAddress[3] = 0x83;
  tempSensorAddress[4] = 0x00;
  tempSensorAddress[5] = 0x00;
  tempSensorAddress[6] = 0x00;
  tempSensorAddress[7] = 0xF5;
  
  ledcAttach(NIGHTLED_PIN, PWM_FREQUENCY, PWM_RESOLUTION);

  server.on("/mainledswState", handleMainToggle);
  server.on("/masterswState", handleMasterToggle);
  server.on("/set_temp_on", handleSetTempOn);
  server.on("/set_temp_off", handleSetTempOff);
  server.on("/on-30m", handleFanOn30m);
  server.on("/off", handleFanOff);
  server.on("/set_nightled_pwm", handleSetNightledPWM);
  server.on("/state", handleState);
  server.on("/i_temp", handleITemp);
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Setup Complete!");
  display.display();
  delay(1000);
}

void loop() {
  if (isBlinking && (millis() - blinkStartTime) >= blinkDuration) {
    digitalWrite(ACTIVITY_LED_PIN, LOW);
    isBlinking = false;
  }
  if (WiFi.status() == WL_CONNECTED && !serverStarted) {
    server.begin();
    serverStarted = true;
  }
  
  server.handleClient();

  if (millis() - lastSensorReadTime >= sensorReadInterval && masterswState == 1 && fanIsOnAutomatic) {
    triggerBlink();
    sensors.requestTemperatures();
    float temp = sensors.getTempC(tempSensorAddress);

    if (temp != DEVICE_DISCONNECTED_C) {
      currentTemperature = temp;
      consecutiveFailedReads = 0;
      sensorIsFaulty = false;
    } else {
      consecutiveFailedReads++;
      if (consecutiveFailedReads >= maxFailedReads) {
        sensorIsFaulty = true;
      }
    }
    lastSensorReadTime = millis();
  }

  if (masterswState == 1 && millis() - lastExternalDataFetch >= externalDataFetchInterval) {
    fetchExternalData();
    lastExternalDataFetch = millis();
  }

  if (fanOverride && (millis() - fanOverrideStartTime) >= fanOverrideDuration) {
    fanOverride = false;
    fanIsOnAutomatic = true;
  }

  static unsigned long lastScheduledFanToggle = 0;
  static bool scheduledFanActive = false;
  const unsigned long fanScheduleInterval = 15 * 60 * 1000;
  const unsigned long fanScheduleDuration = 1 * 60 * 1000;

  if (masterswState == 1 && fanIsOnAutomatic && !fanOverride) {
    if (scheduledFanActive) {
      if (millis() - lastScheduledFanToggle >= fanScheduleDuration) {
        digitalWrite(FAN_PIN, LOW);
        scheduledFanActive = false;
      }
    } else {
      if (digitalRead(FAN_PIN) == LOW && millis() - lastScheduledFanToggle >= fanScheduleInterval) {
        digitalWrite(FAN_PIN, HIGH);
        lastScheduledFanToggle = millis();
        scheduledFanActive = true;
      }
    }
  } else {
    scheduledFanActive = false;
  }
  
  if (fanOverride) {
    digitalWrite(FAN_PIN, HIGH);
  } else if (scheduledFanActive) {
  } else if (fanIsOnAutomatic && !sensorIsFaulty) {
    if (masterswState == 1) {
      if (millis() - lastFanStateChange < fanCooldownDelay) {
      }
      else if (digitalRead(FAN_PIN) == LOW && currentTemperature >= tempThresholdOn) {
        digitalWrite(FAN_PIN, HIGH);
        lastFanStateChange = millis();
      }
      else if (digitalRead(FAN_PIN) == HIGH && currentTemperature <= tempThresholdOff) {
        digitalWrite(FAN_PIN, LOW);
        lastFanStateChange = millis();
      }
    } else {
      digitalWrite(FAN_PIN, LOW);
    }
  } else {
    digitalWrite(FAN_PIN, LOW);
  }

  int currentMainledswReading = digitalRead(MAINLEDSW_PIN);
  if (currentMainledswReading != lastMainledswReading && currentMainledswReading == LOW) {
    if (masterswState == 1) {
      mainledswState = 1 - mainledswState;
    }
  }
  lastMainledswReading = currentMainledswReading;

  int currentMasterswReading = digitalRead(MASTERSW_PIN);
  if (currentMasterswReading != lastMasterswReading && currentMasterswReading == LOW) {
    masterswState = 1 - masterswState;
  }
  lastMasterswReading = currentMasterswReading;

  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    if (timeinfo.tm_hour == 19 && timeinfo.tm_min == 0 && !isAutoToggleDone) {
      if (mainledswState == 0) {
        mainledswState = 1;
        isAutoToggleDone = true;
      }
    }
    if (timeinfo.tm_hour < 19) {
      isAutoToggleDone = false;
    }
  }

  if (mainledswState == 1 && masterswState == 1) {
    digitalWrite(MAINLED_PIN, HIGH);
  } else {
    digitalWrite(MAINLED_PIN, LOW);
  }

  controlNightLED();
  
  static unsigned long lastDisplayUpdate = 0;
  const unsigned long displayUpdateInterval = 1000;
  
  if (shouldNightLedBeOn()) {
    if (isOledOn) {
      display.clearDisplay();
      display.display();
      isOledOn = false;
    }
  } else {
    if (!isOledOn) {
      display.display();
      isOledOn = true;
    }
    if (millis() - lastDisplayUpdate >= displayUpdateInterval) {
      if (masterswState == 1) {
        updateDisplay();
      } else {
        display.clearDisplay();
        display.display();
      }
      lastDisplayUpdate = millis();
    }
  }
  
  yield();
}
