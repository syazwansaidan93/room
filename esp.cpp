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
#include <Update.h>

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

const unsigned long DEBOUNCE_DELAY = 50;
unsigned long mainledswLastDebounceTime = 0;
unsigned long masterswLastDebounceTime = 0;
unsigned long overrideSwLastDebounceTime = 0;

const char* ssid = "wifi_slow2";
IPAddress staticIP(192, 168, 1, 4);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

const char* ntpServer = "192.168.1.1";
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

int consecutiveFailedReads = 0;
bool sensorIsFaulty = false;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

unsigned long lastScheduledFanToggle = 0;
bool scheduledFanActive = false;
const unsigned long fanScheduleInterval = 15 * 60 * 1000;
const unsigned long fanScheduleDuration = 1 * 60 * 1000;

unsigned long lastWifiCheck = 0;
unsigned long wifiReconnectInterval = 10000;
bool wifiConnecting = false;

unsigned long lastNetworkProcessTime = 0;
const unsigned long networkProcessInterval = 20; 

const char* index_html = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP32C3 Control</title><style>body { font-family: sans-serif; background: #121212; color: white; text-align: center; padding: 20px; }
.card { background: #1e1e1e; padding: 20px; border-radius: 10px; margin-bottom: 20px; }
.btn { padding: 12px 24px; font-size: 16px; border: none; border-radius: 5px; cursor: pointer; color: white; margin: 5px; }
.btn-blue { background: #007bff; } .btn-red { background: #dc3545; } .btn-green { background: #28a745; }</style></head>
<body><div class="card"><h2>Temp: <span id="temp">--</span>&deg;C</h2><p>Fan: <span id="fan">--</span> | LED: <span id="led">--</span></p></div>
<div class="card"><button class="btn btn-blue" onclick="api('/mainledswState')">Toggle LED</button>
<button class="btn btn-blue" onclick="api('/masterswState')">Toggle Master</button>
<button class="btn btn-green" onclick="api('/on-30m')">Fan 30M</button>
<button class="btn btn-red" onclick="api('/off')">Fan Auto</button></div>
<script>function api(p){fetch(p);} function update(){fetch('/state').then(r=>r.json()).then(s=>{
document.getElementById('temp').innerText=s.temperature.toFixed(1);
document.getElementById('fan').innerText=s.fanState?"ON":"OFF";
document.getElementById('led').innerText=s.mainledState?"ON":"OFF";});}
setInterval(update,2000);update();</script></body></html>)rawliteral";

void addCorsHeaders() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
}

bool isTimeInRange(int startH, int startM, int endH, int endM) {
    struct tm ti;
    if (!getLocalTime(&ti)) return false;
    int now = ti.tm_hour * 60 + ti.tm_min;
    int start = startH * 60 + startM;
    int end = endH * 60 + endM;
    return (start > end) ? (now >= start || now < end) : (now >= start && now < end);
}

void handleMainToggle() { if (masterswState == 1) mainledswState = 1 - mainledswState; addCorsHeaders(); server.send(200); }
void handleMasterToggle() { masterswState = 1 - masterswState; masterSwOledDisplayStartTime = millis(); if (!isOledActive) { isOledActive = true; display.ssd1306_command(SSD1306_DISPLAYON); } addCorsHeaders(); server.send(200); }
void handleSetTempOn() { if (!server.hasArg("tempOn")) return; tempThresholdOn = server.arg("tempOn").toFloat(); preferences.begin("bilik-config", false); preferences.putFloat("tempOn", tempThresholdOn); preferences.end(); addCorsHeaders(); server.send(200); }
void handleSetTempOff() { if (!server.hasArg("tempOff")) return; tempThresholdOff = server.arg("tempOff").toFloat(); preferences.begin("bilik-config", false); preferences.putFloat("tempOff", tempThresholdOff); preferences.end(); addCorsHeaders(); server.send(200); }
void handleFanOn30m() { if (masterswState == 1 && fanIsOnAutomatic && digitalRead(FAN_PIN) == LOW) { fanOverride = true; fanOverrideStartTime = millis(); fanIsOnAutomatic = false; digitalWrite(FAN_PIN, HIGH); oledOnTime = millis(); addCorsHeaders(); server.send(200); } else { oledErrorDisplayStartTime = millis(); addCorsHeaders(); server.send(400); } }
void handleFanOff() { digitalWrite(FAN_PIN, LOW); fanOverride = false; fanIsOnAutomatic = true; addCorsHeaders(); server.send(200); }
void handleSetNightledPWM() { if (!server.hasArg("pwmValue")) return; nightledPWMValue = server.arg("pwmValue").toInt(); preferences.begin("bilik-config", false); preferences.putInt("nightledPWM", nightledPWMValue); preferences.end(); addCorsHeaders(); server.send(200); }

void handleState() {
    StaticJsonDocument<400> doc;
    doc["mainledswState"] = mainledswState;
    doc["masterswState"] = masterswState;
    doc["mainledState"] = digitalRead(MAINLED_PIN);
    doc["temperature"] = currentTemperature;
    doc["fanState"] = digitalRead(FAN_PIN);
    String res; serializeJson(doc, res); addCorsHeaders(); server.send(200, "application/json", res);
}

void updateDisplay() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    if (oledErrorDisplayStartTime != 0 && (millis() - oledErrorDisplayStartTime) < oledErrorDisplayDuration) {
        display.setCursor(0, 0); display.print("Can't turn"); display.setCursor(0, 16); display.print("on fan 30m");
    } else if (masterSwOledDisplayStartTime != 0 && (millis() - masterSwOledDisplayStartTime) < masterSwOledDisplayDuration) {
        display.setCursor(20, 8); display.setTextSize(2); display.print(masterswState == 1 ? "SW ON" : "SW OFF");
    } else {
        display.setCursor(0, 0); display.print("TEMP:"); display.setCursor(40, 10); display.setTextSize(2);
        display.print(currentTemperature, 1); display.print("C");
    }
    display.display();
}

void processNetwork() {
    if (WiFi.status() != WL_CONNECTED) {
        if (!wifiConnecting && (millis() - lastWifiCheck >= wifiReconnectInterval)) {
            lastWifiCheck = millis(); wifiConnecting = true; WiFi.disconnect(); WiFi.begin(ssid);
            wifiReconnectInterval = (wifiReconnectInterval < 60000) ? wifiReconnectInterval + 5000 : 60000;
        }
    } else {
        wifiConnecting = false;
        wifiReconnectInterval = 10000;
        if (!serverStarted) { server.begin(); serverStarted = true; }
        server.handleClient();
        if (!isNtpSynced) { struct tm ti; if (getLocalTime(&ti)) isNtpSynced = true; }
    }
}

void setup() {
    Wire.begin(OLED_SDA, OLED_SCL);
    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    display.clearDisplay(); display.display();

    WiFi.config(staticIP, gateway, subnet);
    WiFi.begin(ssid);
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    preferences.begin("bilik-config", false);
    tempThresholdOn = preferences.getFloat("tempOn", 28.9);
    tempThresholdOff = preferences.getFloat("tempOff", 28.7);
    nightledPWMValue = preferences.getInt("nightledPWM", 0);
    preferences.end();

    pinMode(MAINLED_PIN, OUTPUT); pinMode(NIGHTLED_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT); pinMode(ACTIVITY_LED_PIN, OUTPUT);
    pinMode(MAINLEDSW_PIN, INPUT_PULLUP); pinMode(MASTERSW_PIN, INPUT_PULLUP);
    pinMode(TOUCH_SENSOR_PIN, INPUT); pinMode(OVERRIDE_SW_PIN, INPUT_PULLUP);

    sensors.begin();
    tempSensorAddress[0] = 0x28; tempSensorAddress[1] = 0x07; tempSensorAddress[2] = 0xBB; tempSensorAddress[3] = 0x83;
    tempSensorAddress[4] = 0x00; tempSensorAddress[5] = 0x00; tempSensorAddress[6] = 0x00; tempSensorAddress[7] = 0xF5;

    ledcAttach(NIGHTLED_PIN, PWM_FREQUENCY, PWM_RESOLUTION);

    server.on("/", []() { server.send(200, "text/html", index_html); });
    server.on("/mainledswState", handleMainToggle);
    server.on("/masterswState", handleMasterToggle);
    server.on("/state", handleState);
    server.on("/on-30m", handleFanOn30m);
    server.on("/off", handleFanOff);

    server.on("/update", HTTP_POST, []() {
        server.send(200); delay(100); ESP.restart();
    }, []() {
        HTTPUpload& u = server.upload();
        if (u.status == UPLOAD_FILE_START) Update.begin(UPDATE_SIZE_UNKNOWN);
        else if (u.status == UPLOAD_FILE_WRITE) Update.write(u.buf, u.currentSize);
        else if (u.status == UPLOAD_FILE_END) Update.end(true);
    });

    display.ssd1306_command(SSD1306_DISPLAYOFF);
}

void loop() {
    if (millis() - lastNetworkProcessTime >= networkProcessInterval) {
        processNetwork();
        lastNetworkProcessTime = millis();
    }

    if (digitalRead(TOUCH_SENSOR_PIN) == HIGH) {
        if (!isOledActive) { isOledActive = true; display.ssd1306_command(SSD1306_DISPLAYON); }
        oledOnTime = millis();
    }

    if (isOledActive && (millis() - oledOnTime >= oledActiveDuration)) {
        display.clearDisplay(); display.display(); display.ssd1306_command(SSD1306_DISPLAYOFF);
        isOledActive = false;
    }

    if (isOledActive) updateDisplay();

    if (millis() - lastSensorReadTime >= sensorReadInterval && masterswState == 1) {
        sensors.requestTemperatures();
        float t = sensors.getTempC(tempSensorAddress);
        if (t != DEVICE_DISCONNECTED_C) { currentTemperature = t; sensorIsFaulty = false; }
        else sensorIsFaulty = true;
        lastSensorReadTime = millis();
    }

    if (fanOverride && (millis() - fanOverrideStartTime) >= fanOverrideDuration) {
        fanOverride = false; fanIsOnAutomatic = true;
    }

    if (fanOverride) digitalWrite(FAN_PIN, HIGH);
    else if (fanIsOnAutomatic && masterswState == 1 && !sensorIsFaulty) {
        if (currentTemperature >= tempThresholdOn) digitalWrite(FAN_PIN, HIGH);
        else if (currentTemperature <= tempThresholdOff) digitalWrite(FAN_PIN, LOW);
    } else digitalWrite(FAN_PIN, LOW);

    static int lRM = HIGH, lSM = HIGH;
    int cRM = digitalRead(MAINLEDSW_PIN);
    if (cRM != lRM) { mainledswLastDebounceTime = millis(); lRM = cRM; }
    if ((millis() - mainledswLastDebounceTime) > DEBOUNCE_DELAY) {
        if (cRM == LOW && lSM == HIGH) { if (masterswState == 1) mainledswState = 1 - mainledswState; lSM = LOW; }
        else if (cRM == HIGH) lSM = HIGH;
    }

    static int lRMS = HIGH, lSMS = HIGH;
    int cRMS = digitalRead(MASTERSW_PIN);
    if (cRMS != lRMS) { masterswLastDebounceTime = millis(); lRMS = cRMS; }
    if ((millis() - masterswLastDebounceTime) > DEBOUNCE_DELAY) {
        if (cRMS == LOW && lSMS == HIGH) { masterswState = 1 - masterswState; lSMS = LOW; }
        else if (cRMS == HIGH) lSMS = HIGH;
    }

    digitalWrite(MAINLED_PIN, (mainledswState == 1 && masterswState == 1) ? HIGH : LOW);
    ledcWrite(NIGHTLED_PIN, (isTimeInRange(19, 15, 7, 15) && mainledswState == 0) ? nightledPWMValue : 0);
    digitalWrite(ACTIVITY_LED_PIN, masterswState);
}
