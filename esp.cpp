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
#define OVERRIDE_SW_PIN 5
#define FAN_PIN 6
#define ACTIVITY_LED_PIN 0
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_SDA 8
#define OLED_SCL 9
#define OLED_ADDR 0x3C
#define TOUCH_SENSOR_PIN 10
#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 8
#define LED_CHANNEL 0

const unsigned long DEBOUNCE_DELAY = 50;
unsigned long mainledswLastDebounceTime = 0;
unsigned long masterswLastDebounceTime = 0;
unsigned long overrideSwLastDebounceTime = 0;

const char* ssid = "wifi_slow2";
IPAddress staticIP(192, 168, 1, 4);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 1, 1);

const char* ntpServer = "time.google.com";
const long gmtOffset_sec = 8 * 3600;
const int daylightOffset_sec = 0;

int mainledswState = 0;
int lastMainledswReading = HIGH;
int masterswState = 0;
int lastMasterswReading = HIGH;
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
bool isEveningToggleDone = false;
bool isMorningToggleDone = false;
bool isNtpSynced = false;

bool isOledActive = false;
unsigned long oledOnTime = 0;
const unsigned long oledActiveDuration = 10000;
bool isTempStatusDisplay = false;
unsigned long masterSwOledDisplayStartTime = 0;
const unsigned long masterSwOledDisplayDuration = 5000;
unsigned long oledErrorDisplayStartTime = 0;
const unsigned long oledErrorDisplayDuration = 5000;

Preferences preferences;
WebServer server(80);

const int maxFailedReads = 5;
int consecutiveFailedReads = 0;
bool sensorIsFaulty = false;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

static unsigned long lastScheduledFanToggle = 0;
static bool scheduledFanActive = false;
const unsigned long fanScheduleInterval = 15 * 60 * 1000;
const unsigned long fanScheduleDuration = 1 * 60 * 1000;

void onWiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            isNtpSynced = false;
            WiFi.begin(ssid);
            break;
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            break;
        default:
            break;
    }
}

void addCorsHeaders() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "*");
}

void handleOptions() {
    addCorsHeaders();
    server.send(204);
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
    if (masterswState == 1) {
        mainledswState = 1 - mainledswState;
    }
    String response = "Main LED Switch state is now: " + String(mainledswState);
    addCorsHeaders();
    server.send(200, "text/plain", response);
}

void handleMasterToggle() {
    int oldMasterswState = masterswState;
    masterswState = 1 - masterswState;
    if (oldMasterswState != masterswState) {
        masterSwOledDisplayStartTime = millis();
        if (!isOledActive) {
            isOledActive = true;
            display.ssd1306_command(SSD1306_DISPLAYON);
        }
    }
    String response = "Master Switch state is now: " + String(masterswState);
    addCorsHeaders();
    server.send(200, "text/plain", response);
}

void handleSetTempOn() {
    if (!server.hasArg("tempOn")) {
        addCorsHeaders();
        server.send(400, "text/plain", "Missing 'tempOn' parameter.");
        return;
    }
    float newTempOn = server.arg("tempOn").toFloat();
    if (newTempOn > tempThresholdOff) {
        tempThresholdOn = newTempOn;
        preferences.begin("bilik-config", false);
        preferences.putFloat("tempOn", tempThresholdOn);
        preferences.end();
        addCorsHeaders();
        server.send(200, "text/plain", "Fan 'tempOn' threshold set successfully.");
    } else {
        addCorsHeaders();
        server.send(400, "text/plain", "New 'tempOn' must be greater than current 'tempOff'.");
    }
}

void handleSetTempOff() {
    if (!server.hasArg("tempOff")) {
        addCorsHeaders();
        server.send(400, "text/plain", "Missing 'tempOff' parameter.");
        return;
    }
    float newTempOff = server.arg("tempOff").toFloat();
    if (tempThresholdOn > newTempOff) {
        tempThresholdOff = newTempOff;
        preferences.begin("bilik-config", false);
        preferences.putFloat("tempOff", tempThresholdOff);
        preferences.end();
        addCorsHeaders();
        server.send(200, "text/plain", "Fan 'tempOff' threshold set successfully.");
    } else {
        addCorsHeaders();
        server.send(400, "text/plain", "New 'tempOff' must be less than current 'tempOn'.");
    }
}

void handleFanOn30m() {
    if (masterswState == 1 && fanIsOnAutomatic == true && digitalRead(FAN_PIN) == LOW) {
        fanOverride = true;
        fanOverrideStartTime = millis();
        fanIsOnAutomatic = false;
        digitalWrite(FAN_PIN, HIGH);
        if (!isOledActive) {
            isOledActive = true;
            display.ssd1306_command(SSD1306_DISPLAYON);
        }
        oledOnTime = millis();
        isTempStatusDisplay = false;
        masterSwOledDisplayStartTime = 0;
        addCorsHeaders();
        server.send(200, "text/plain", "Fan turned on for 30 minutes. Automatic mode is suspended.");
    } else {
        oledErrorDisplayStartTime = millis();
        if (!isOledActive) {
            isOledActive = true;
            display.ssd1306_command(SSD1306_DISPLAYON);
        }
        oledOnTime = millis();
        addCorsHeaders();
        server.send(400, "text/plain", "Cannot turn fan on.");
    }
}

void handleFanOff() {
    if (fanIsOnAutomatic && digitalRead(FAN_PIN) == HIGH) {
        addCorsHeaders();
        server.send(400, "text/plain", "Cannot turn fan off: Fan is running in automatic mode.");
    } else {
        digitalWrite(FAN_PIN, LOW);
        fanOverride = false;
        fanIsOnAutomatic = true;
        if (isOledActive && masterSwOledDisplayStartTime == 0) {
            display.clearDisplay();
            display.display();
            display.ssd1306_command(SSD1306_DISPLAYOFF);
            isOledActive = false;
            isTempStatusDisplay = false;
        }
        addCorsHeaders();
        server.send(200, "text/plain", "Fan turned off. Automatic mode is restored.");
    }
}

void handleSetNightledPWM() {
    if (!server.hasArg("pwmValue")) {
        addCorsHeaders();
        server.send(400, "text/plain", "Missing 'pwmValue' parameter.");
        return;
    }
    int newPWM = server.arg("pwmValue").toInt();
    if (newPWM >= 0 && newPWM <= 255) {
        nightledPWMValue = newPWM;
        preferences.begin("bilik-config", false);
        preferences.putInt("nightledPWM", nightledPWMValue);
        preferences.end();
        addCorsHeaders();
        server.send(200, "text/plain", "Night LED PWM value set successfully.");
    } else {
        addCorsHeaders();
        server.send(400, "text/plain", "PWM value must be between 0 and 255.");
    }
}

bool shouldNightLedBeOn() {
    return isTimeInRange(19, 15, 7, 15) && (mainledswState == 0 || masterswState == 0);
}

void handleITemp() {
    sensors.requestTemperatures();
    float temp = sensors.getTempC(tempSensorAddress);
    addCorsHeaders();
    if (temp != DEVICE_DISCONNECTED_C) {
        currentTemperature = temp;
        consecutiveFailedReads = 0;
        sensorIsFaulty = false;
        server.send(200, "text/plain", String(currentTemperature));
    } else {
        consecutiveFailedReads++;
        if (consecutiveFailedReads >= maxFailedReads) {
            sensorIsFaulty = true;
        }
        if (sensorIsFaulty) {
            server.send(404, "text/plain", "Sensor is faulty.");
        } else {
            server.send(404, "text/plain", "Sensor is temporarily unavailable.");
        }
    }
}

void handleState() {
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
    addCorsHeaders();
    server.send(200, "application/json", jsonResponse);
}

void controlNightLED() {
    if (shouldNightLedBeOn()) {
        ledcWrite(NIGHTLED_PIN, nightledPWMValue);
    } else {
        ledcWrite(NIGHTLED_PIN, 0);
    }
}

void updateDisplay() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    if (oledErrorDisplayStartTime != 0 && (millis() - oledErrorDisplayStartTime) < oledErrorDisplayDuration) {
        display.setTextSize(2);
        display.setCursor(0, 0);
        display.print("Can't turn");
        display.setCursor(0, 16);
        display.print("on fan 30m");
    }
    else if (masterSwOledDisplayStartTime != 0 && (millis() - masterSwOledDisplayStartTime) < masterSwOledDisplayDuration) {
        String masterStateStr = masterswState == 1 ? "SW ON" : "SW OFF";
        display.setTextSize(3);
        int16_t x1, y1;
        uint16_t w, h;
        display.getTextBounds(masterStateStr, 0, 0, &x1, &y1, &w, &h);
        display.setCursor((SCREEN_WIDTH - w) / 2, (SCREEN_HEIGHT - h) / 2);
        display.print(masterStateStr);
    }
    else if (fanOverride && !isTempStatusDisplay) {
        unsigned long elapsed = millis() - fanOverrideStartTime;
        unsigned long remainingTimeMs = (elapsed < fanOverrideDuration) ? (fanOverrideDuration - elapsed) : 0;
        long remainingSeconds = remainingTimeMs / 1000;
        int minutes = remainingSeconds / 60;
        int seconds = remainingSeconds % 60;
        display.setCursor(0, 0);
        display.setTextSize(1);
        String masterStateStr = masterswState == 1 ? "(MS ON)" : "(MS OFF)";
        display.print("FAN OVERRIDE ");
        display.println(masterStateStr);
        display.setCursor(0, 16);
        display.setTextSize(2);
        if (minutes < 10) display.print("0");
        display.print(minutes);
        display.print(":");
        if (seconds < 10) display.print("0");
        display.print(seconds);
    }
    else {
        display.setCursor(0, 0);
        display.setTextSize(1);
        display.print("WiFi: [");
        long rssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : -100;
        int bars = map(rssi, -100, -30, 0, 13);
        for (int i = 0; i < 13; i++) {
            if (i < bars) display.print("=");
            else display.print(" ");
        }
        display.println("]");
        display.setCursor(0, 16);
        display.setTextSize(1);
        display.print("TEMP:");
        display.setCursor(40, 16);
        display.setTextSize(2);
        display.print(String(currentTemperature, 1));
        display.print("C");
    }
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
    display.println("System Booting...");
    display.display();

    WiFi.onEvent(onWiFiEvent);
    WiFi.config(staticIP, gateway, subnet, dns);
    WiFi.begin(ssid);

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
    pinMode(TOUCH_SENSOR_PIN, INPUT);
    pinMode(OVERRIDE_SW_PIN, INPUT_PULLUP);

    sensors.begin();
    sensors.setResolution(tempSensorAddress, 12);
    tempSensorAddress[0] = 0x28; tempSensorAddress[1] = 0x07; tempSensorAddress[2] = 0xBB; tempSensorAddress[3] = 0x83;
    tempSensorAddress[4] = 0x00; tempSensorAddress[5] = 0x00; tempSensorAddress[6] = 0x00; tempSensorAddress[7] = 0xF5;

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

    server.on("/mainledswState", HTTP_OPTIONS, handleOptions);
    server.on("/masterswState", HTTP_OPTIONS, handleOptions);
    server.on("/set_temp_on", HTTP_OPTIONS, handleOptions);
    server.on("/set_temp_off", HTTP_OPTIONS, handleOptions);
    server.on("/on-30m", HTTP_OPTIONS, handleOptions);
    server.on("/off", HTTP_OPTIONS, handleOptions);
    server.on("/set_nightled_pwm", HTTP_OPTIONS, handleOptions);
    server.on("/state", HTTP_OPTIONS, handleOptions);
    server.on("/i_temp", HTTP_OPTIONS, handleOptions);

    delay(1000);
    display.ssd1306_command(SSD1306_DISPLAYOFF);
}

void loop() {
    static unsigned long lastWifiAttempt = 0;
    if (WiFi.status() != WL_CONNECTED) {
        serverStarted = false;
        if (millis() - lastWifiAttempt >= 20) {
            WiFi.begin(ssid);
            lastWifiAttempt = millis();
        }
    } else {
        if (!serverStarted) {
            server.begin();
            serverStarted = true;
        }
        server.handleClient();

        if (!isNtpSynced) {
            struct tm timeinfo;
            if (getLocalTime(&timeinfo)) {
                isNtpSynced = true;
            }
        }
    }

    bool isMasterSwDisplayActive = (masterSwOledDisplayStartTime != 0 && (millis() - masterSwOledDisplayStartTime) < masterSwOledDisplayDuration);
    bool isOverrideDisplayActive = fanOverride;
    bool isErrorDisplayActive = (oledErrorDisplayStartTime != 0 && (millis() - oledErrorDisplayStartTime) < oledErrorDisplayDuration);

    if (digitalRead(TOUCH_SENSOR_PIN) == HIGH) {
        if (!isOledActive) {
            isOledActive = true;
            display.ssd1306_command(SSD1306_DISPLAYON);
        }
        oledOnTime = millis();
        masterSwOledDisplayStartTime = 0;
        oledErrorDisplayStartTime = 0;
        if (fanOverride) isTempStatusDisplay = !isTempStatusDisplay;
        else isTempStatusDisplay = true;
    }

    if (isMasterSwDisplayActive) {
        if (!isOledActive) {
            isOledActive = true;
            display.ssd1306_command(SSD1306_DISPLAYON);
        }
        if (fanOverride) isTempStatusDisplay = false;
        oledErrorDisplayStartTime = 0;
    } else if (masterSwOledDisplayStartTime != 0) {
        masterSwOledDisplayStartTime = 0;
    }

    if (isOledActive && !isMasterSwDisplayActive && !isOverrideDisplayActive && !isErrorDisplayActive) {
        if (millis() - oledOnTime >= oledActiveDuration) {
            display.clearDisplay();
            display.display();
            display.ssd1306_command(SSD1306_DISPLAYOFF);
            isOledActive = false;
            isTempStatusDisplay = false;
        }
    }

    if (oledErrorDisplayStartTime != 0 && !isErrorDisplayActive) oledErrorDisplayStartTime = 0;

    if (fanOverride && isTempStatusDisplay && (millis() - oledOnTime >= oledActiveDuration)) isTempStatusDisplay = false;

    static unsigned long lastDisplayUpdate = 0;
    if ((isOledActive || isMasterSwDisplayActive || isErrorDisplayActive) && (millis() - lastDisplayUpdate >= 1000)) {
        updateDisplay();
        lastDisplayUpdate = millis();
    }

    if (millis() - lastSensorReadTime >= sensorReadInterval && masterswState == 1 && fanIsOnAutomatic) {
        sensors.requestTemperatures();
        float temp = sensors.getTempC(tempSensorAddress);
        if (temp != DEVICE_DISCONNECTED_C) {
            currentTemperature = temp;
            consecutiveFailedReads = 0;
            sensorIsFaulty = false;
        } else {
            consecutiveFailedReads++;
            if (consecutiveFailedReads >= maxFailedReads) sensorIsFaulty = true;
        }
        lastSensorReadTime = millis();
    }

    if (fanOverride && (millis() - fanOverrideStartTime) >= fanOverrideDuration) {
        fanOverride = false;
        fanIsOnAutomatic = true;
        lastScheduledFanToggle = millis();
        scheduledFanActive = false;
        if (isOledActive && masterSwOledDisplayStartTime == 0) {
            display.clearDisplay();
            display.display();
            display.ssd1306_command(SSD1306_DISPLAYOFF);
            isOledActive = false;
            isTempStatusDisplay = false;
        }
    }

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
    } else if (!scheduledFanActive && fanIsOnAutomatic && !sensorIsFaulty) {
        if (masterswState == 1) {
            if (millis() - lastFanStateChange >= fanCooldownDelay) {
                if (digitalRead(FAN_PIN) == LOW && currentTemperature >= tempThresholdOn) {
                    digitalWrite(FAN_PIN, HIGH);
                    lastFanStateChange = millis();
                }
                else if (digitalRead(FAN_PIN) == HIGH && currentTemperature <= tempThresholdOff) {
                    digitalWrite(FAN_PIN, LOW);
                    lastFanStateChange = millis();
                }
            }
        } else {
            digitalWrite(FAN_PIN, LOW);
        }
    } else if (!scheduledFanActive) {
        digitalWrite(FAN_PIN, LOW);
    }

    static int lastRawMain = HIGH, lastStableMain = HIGH;
    int curRawMain = digitalRead(MAINLEDSW_PIN);
    if (curRawMain != lastRawMain) { mainledswLastDebounceTime = millis(); lastRawMain = curRawMain; }
    if ((millis() - mainledswLastDebounceTime) > DEBOUNCE_DELAY) {
        if (curRawMain == LOW && lastStableMain == HIGH) {
            if (masterswState == 1) mainledswState = 1 - mainledswState;
            lastStableMain = LOW;
        } else if (curRawMain == HIGH) lastStableMain = HIGH;
    }

    static int lastRawMast = HIGH, lastStableMast = HIGH;
    int curRawMast = digitalRead(MASTERSW_PIN);
    if (curRawMast != lastRawMast) { masterswLastDebounceTime = millis(); lastRawMast = curRawMast; }
    if ((millis() - masterswLastDebounceTime) > DEBOUNCE_DELAY) {
        if (curRawMast == LOW && lastStableMast == HIGH) {
            masterswState = 1 - masterswState;
            masterSwOledDisplayStartTime = millis();
            if (!isOledActive) { isOledActive = true; display.ssd1306_command(SSD1306_DISPLAYON); }
            lastStableMast = LOW;
        } else if (curRawMast == HIGH) lastStableMast = HIGH;
    }

    static int lastRawOver = HIGH, lastStableOver = HIGH;
    int curRawOver = digitalRead(OVERRIDE_SW_PIN);
    if (curRawOver != lastRawOver) { overrideSwLastDebounceTime = millis(); lastRawOver = curRawOver; }
    if ((millis() - overrideSwLastDebounceTime) > DEBOUNCE_DELAY) {
        if (curRawOver == LOW && lastStableOver == HIGH) {
            if (fanOverride) {
                digitalWrite(FAN_PIN, LOW);
                fanOverride = false;
                fanIsOnAutomatic = true;
                if (isOledActive && masterSwOledDisplayStartTime == 0) {
                    display.clearDisplay(); display.display(); display.ssd1306_command(SSD1306_DISPLAYOFF);
                    isOledActive = false; isTempStatusDisplay = false;
                }
            } else {
                if (masterswState == 1 && fanIsOnAutomatic == true && digitalRead(FAN_PIN) == LOW) {
                    fanOverride = true; fanOverrideStartTime = millis(); fanIsOnAutomatic = false;
                    digitalWrite(FAN_PIN, HIGH);
                    if (!isOledActive) { isOledActive = true; display.ssd1306_command(SSD1306_DISPLAYON); }
                    oledOnTime = millis(); isTempStatusDisplay = false; masterSwOledDisplayStartTime = 0;
                } else {
                    oledErrorDisplayStartTime = millis();
                    if (!isOledActive) { isOledActive = true; display.ssd1306_command(SSD1306_DISPLAYON); }
                    oledOnTime = millis();
                }
            }
            lastStableOver = LOW;
        } else if (curRawOver == HIGH) lastStableOver = HIGH;
    }

    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
        if (timeinfo.tm_hour == 19 && timeinfo.tm_min == 15 && !isEveningToggleDone) {
            if (mainledswState == 0) { mainledswState = 1; isEveningToggleDone = true; }
        }
        if (timeinfo.tm_hour == 7 && timeinfo.tm_min == 15 && !isMorningToggleDone) {
            if (mainledswState == 1) { mainledswState = 0; isMorningToggleDone = true; }
        }
        if (!isTimeInRange(19, 15, 7, 15)) { isEveningToggleDone = false; isMorningToggleDone = false; }
    }

    digitalWrite(MAINLED_PIN, (mainledswState == 1 && masterswState == 1) ? HIGH : LOW);
    controlNightLED();
    digitalWrite(ACTIVITY_LED_PIN, masterswState == 1 ? HIGH : LOW);

    yield();
}
