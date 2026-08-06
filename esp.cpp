#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "time.h"
#include <driver/ledc.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
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

Preferences preferences;
WebServer server(80);

const int maxFailedReads = 5;
int consecutiveFailedReads = 0;
bool sensorIsFaulty = false;

static unsigned long lastScheduledFanToggle = 0;
static bool scheduledFanActive = false;
const unsigned long fanScheduleInterval = 15 * 60 * 1000;
const unsigned long fanScheduleDuration = 1 * 60 * 1000;

unsigned long lastWifiCheck = 0;
unsigned long wifiReconnectInterval = 10000; 
bool wifiConnecting = false;

const char* index_html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32C3 Control</title>
    <style>
        body { font-family: sans-serif; background: #121212; color: white; text-align: center; padding: 20px; }
        .card { background: #1e1e1e; padding: 20px; border-radius: 10px; margin-bottom: 20px; box-shadow: 0 4px 8px rgba(0,0,0,0.5); }
        .btn { padding: 12px 24px; font-size: 16px; border: none; border-radius: 5px; cursor: pointer; color: white; margin: 5px; }
        .btn-blue { background: #007bff; }
        .btn-red { background: #dc3545; }
        .btn-green { background: #28a745; }
        .status { font-size: 1.2em; color: #00ff00; }
        input[type=number], input[type=range] { padding: 8px; border-radius: 5px; border: 1px solid #444; background: #333; color: white; width: 80px; }
        #update-form { margin-top: 20px; border-top: 1px solid #444; padding-top: 20px; }
    </style>
</head>
<body>
    <div class="card">
        <h2>Environment</h2>
        <p>Temp: <span id="temp">--</span>&deg;C</p>
        <p>Fan: <span id="fan">--</span> | Main LED: <span id="led">--</span></p>
    </div>
    <div class="card">
        <h2>Controls</h2>
        <button class="btn btn-blue" onclick="api('/mainledswState')">Toggle Main LED</button>
        <button class="btn btn-blue" onclick="api('/masterswState')">Toggle Master SW</button>
        <br>
        <button class="btn btn-green" onclick="api('/on-30m')">Fan 30M On</button>
        <button class="btn btn-red" onclick="api('/off')">Fan Auto/Off</button>
    </div>
    <div class="card">
        <h2>Settings</h2>
        <p>On Threshold: <input type="number" step="0.1" id="tOn" onchange="setVal('/set_temp_on?tempOn=', this.value)"> &deg;C</p>
        <p>Off Threshold: <input type="number" step="0.1" id="tOff" onchange="setVal('/set_temp_off?tempOff=', this.value)"> &deg;C</p>
        <p>Night LED PWM: <input type="range" min="0" max="255" id="nPwm" oninput="setVal('/set_nightled_pwm?pwmValue=', this.value)"> <span id="pwmVal"></span></p>
    </div>
    <div class="card" id="update-form">
        <h2>Firmware Update</h2>
        <form method='POST' action='/update' enctype='multipart/form-data'>
            <input type='file' name='update'>
            <input type='submit' value='Update' class="btn btn-red">
        </form>
    </div>
    <script>
        function api(path) { fetch(path).then(r => r.text()).then(t => console.log(t)); }
        function setVal(path, val) { fetch(path + val); if(path.includes('pwm')) document.getElementById('pwmVal').innerText = val; }
        function updateState() {
            fetch('/state').then(r => r.json()).then(s => {
                document.getElementById('temp').innerText = s.temperature.toFixed(1);
                document.getElementById('fan').innerText = s.fanState ? "ON" : "OFF";
                document.getElementById('led').innerText = s.mainledState ? "ON" : "OFF";
                document.getElementById('tOn').value = s.tempThresholdOn;
                document.getElementById('tOff').value = s.tempThresholdOff;
                document.getElementById('nPwm').value = s.nightledPWMValue;
                document.getElementById('pwmVal').innerText = s.nightledPWMValue;
            }).catch(e => console.log("State fetch failed"));
        }
        setInterval(updateState, 2000);
        updateState();
    </script>
</body>
</html>
)rawliteral";

void onWiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            isNtpSynced = false;
            serverStarted = false;
            wifiConnecting = false;
            break;
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            wifiConnecting = false;
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
    if (!getLocalTime(&timeinfo)) return false;
    int nowInMinutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;
    int startInMinutes = startHour * 60 + startMin;
    int endInMinutes = endHour * 60 + endMin;
    if (startInMinutes > endInMinutes) return (nowInMinutes >= startInMinutes || nowInMinutes < endInMinutes);
    else return (nowInMinutes >= startInMinutes && nowInMinutes < endInMinutes);
}

void handleMainToggle() {
    if (masterswState == 1) mainledswState = 1 - mainledswState;
    addCorsHeaders();
    server.send(200, "text/plain", "Main LED: " + String(mainledswState));
}

void handleMasterToggle() {
    masterswState = 1 - masterswState;
    addCorsHeaders();
    server.send(200, "text/plain", "Master: " + String(masterswState));
}

void handleSetTempOn() {
    if (!server.hasArg("tempOn")) { addCorsHeaders(); server.send(400); return; }
    float newTempOn = server.arg("tempOn").toFloat();
    if (newTempOn > tempThresholdOff) {
        tempThresholdOn = newTempOn;
        preferences.begin("bilik-config", false);
        preferences.putFloat("tempOn", tempThresholdOn);
        preferences.end();
        addCorsHeaders(); server.send(200);
    } else { addCorsHeaders(); server.send(400); }
}

void handleSetTempOff() {
    if (!server.hasArg("tempOff")) { addCorsHeaders(); server.send(400); return; }
    float newTempOff = server.arg("tempOff").toFloat();
    if (tempThresholdOn > newTempOff) {
        tempThresholdOff = newTempOff;
        preferences.begin("bilik-config", false);
        preferences.putFloat("tempOff", tempThresholdOff);
        preferences.end();
        addCorsHeaders(); server.send(200);
    } else { addCorsHeaders(); server.send(400); }
}

void handleFanOn30m() {
    if (masterswState == 1 && fanIsOnAutomatic && digitalRead(FAN_PIN) == LOW) {
        fanOverride = true;
        fanOverrideStartTime = millis();
        fanIsOnAutomatic = false;
        digitalWrite(FAN_PIN, HIGH);
        addCorsHeaders(); server.send(200);
    } else {
        addCorsHeaders(); server.send(400);
    }
}

void handleFanOff() {
    if (fanIsOnAutomatic && digitalRead(FAN_PIN) == HIGH) {
        addCorsHeaders(); server.send(400);
    } else {
        digitalWrite(FAN_PIN, LOW);
        fanOverride = false;
        fanIsOnAutomatic = true;
        addCorsHeaders(); server.send(200);
    }
}

void handleSetNightledPWM() {
    if (!server.hasArg("pwmValue")) { addCorsHeaders(); server.send(400); return; }
    int newPWM = server.arg("pwmValue").toInt();
    if (newPWM >= 0 && newPWM <= 255) {
        nightledPWMValue = newPWM;
        preferences.begin("bilik-config", false);
        preferences.putInt("nightledPWM", nightledPWMValue);
        preferences.end();
        addCorsHeaders(); server.send(200);
    } else { addCorsHeaders(); server.send(400); }
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
        if (consecutiveFailedReads >= maxFailedReads) sensorIsFaulty = true;
        server.send(404);
    }
}

void handleState() {
    StaticJsonDocument<400> doc;
    struct tm timeinfo;
    char timeString[64] = "N/A";
    if(getLocalTime(&timeinfo)) strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo);
    doc["currentTime"] = timeString;
    doc["uptimeSeconds"] = millis() / 1000;
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
    if (shouldNightLedBeOn()) ledcWrite(NIGHTLED_PIN, nightledPWMValue);
    else ledcWrite(NIGHTLED_PIN, 0);
}

void setup() {
    WiFi.onEvent(onWiFiEvent);
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
    pinMode(OVERRIDE_SW_PIN, INPUT_PULLUP);

    sensors.begin();
    tempSensorAddress[0] = 0x28; tempSensorAddress[1] = 0x07; tempSensorAddress[2] = 0xBB; tempSensorAddress[3] = 0x83;
    tempSensorAddress[4] = 0x00; tempSensorAddress[5] = 0x00; tempSensorAddress[6] = 0x00; tempSensorAddress[7] = 0xF5;
    sensors.setResolution(tempSensorAddress, 12);

    ledcAttach(NIGHTLED_PIN, PWM_FREQUENCY, PWM_RESOLUTION);

    server.on("/", []() { server.send(200, "text/html", index_html); });
    server.on("/mainledswState", handleMainToggle);
    server.on("/masterswState", handleMasterToggle);
    server.on("/set_temp_on", handleSetTempOn);
    server.on("/set_temp_off", handleSetTempOff);
    server.on("/on-30m", handleFanOn30m);
    server.on("/off", handleFanOff);
    server.on("/set_nightled_pwm", handleSetNightledPWM);
    server.on("/state", handleState);
    server.on("/i_temp", handleITemp);

    server.on("/update", HTTP_POST, []() {
        server.sendHeader("Connection", "close");
        server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
        ESP.restart();
    }, []() {
        HTTPUpload& upload = server.upload();
        if (upload.status == UPLOAD_FILE_START) {
            if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
        } else if (upload.status == UPLOAD_FILE_WRITE) {
            if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(Serial);
        } else if (upload.status == UPLOAD_FILE_END) {
            if (!Update.end(true)) Update.printError(Serial);
        }
    });

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
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        if (!wifiConnecting && (millis() - lastWifiCheck >= wifiReconnectInterval)) {
            lastWifiCheck = millis();
            wifiConnecting = true;
            WiFi.disconnect();
            WiFi.begin(ssid);
            wifiReconnectInterval = (wifiReconnectInterval < 60000) ? wifiReconnectInterval + 5000 : 60000;
        }
    } else {
        wifiConnecting = false;
        wifiReconnectInterval = 10000;
        if (!serverStarted) {
            server.begin();
            serverStarted = true;
        }
        server.handleClient();
        if (!isNtpSynced) {
            struct tm timeinfo;
            if (getLocalTime(&timeinfo)) isNtpSynced = true;
        }
    }

    if (millis() - lastSensorReadTime >= sensorReadInterval && masterswState == 1 && fanIsOnAutomatic) {
        sensors.requestTemperatures();
        float temp = sensors.getTempC(tempSensorAddress);
        if (temp != DEVICE_DISCONNECTED_C) {
            currentTemperature = temp;
            consecutiveFailedReads = 0;
            sensorIsFaulty = false;
        } else {
            if (++consecutiveFailedReads >= maxFailedReads) sensorIsFaulty = true;
        }
        lastSensorReadTime = millis();
    }

    if (fanOverride && (millis() - fanOverrideStartTime) >= fanOverrideDuration) {
        fanOverride = false; 
        fanIsOnAutomatic = true; 
        lastScheduledFanToggle = millis(); 
        scheduledFanActive = false;
    }

    if (masterswState == 1 && fanIsOnAutomatic && !fanOverride) {
        if (scheduledFanActive) {
            if (millis() - lastScheduledFanToggle >= fanScheduleDuration) {
                digitalWrite(FAN_PIN, LOW); scheduledFanActive = false;
            }
        } else {
            if (digitalRead(FAN_PIN) == LOW && millis() - lastScheduledFanToggle >= fanScheduleInterval) {
                digitalWrite(FAN_PIN, HIGH); lastScheduledFanToggle = millis(); scheduledFanActive = true;
            }
        }
    } else scheduledFanActive = false;

    if (fanOverride) digitalWrite(FAN_PIN, HIGH);
    else if (!scheduledFanActive && fanIsOnAutomatic && !sensorIsFaulty) {
        if (masterswState == 1 && millis() - lastFanStateChange >= fanCooldownDelay) {
            if (digitalRead(FAN_PIN) == LOW && currentTemperature >= tempThresholdOn) { digitalWrite(FAN_PIN, HIGH); lastFanStateChange = millis(); }
            else if (digitalRead(FAN_PIN) == HIGH && currentTemperature <= tempThresholdOff) { digitalWrite(FAN_PIN, LOW); lastFanStateChange = millis(); }
        } else if (masterswState == 0) digitalWrite(FAN_PIN, LOW);
    } else if (!scheduledFanActive) digitalWrite(FAN_PIN, LOW);

    static int lastRawMain = HIGH, lastStableMain = HIGH;
    int curRawMain = digitalRead(MAINLEDSW_PIN);
    if (curRawMain != lastRawMain) { mainledswLastDebounceTime = millis(); lastRawMain = curRawMain; }
    if ((millis() - mainledswLastDebounceTime) > DEBOUNCE_DELAY) {
        if (curRawMain == LOW && lastStableMain == HIGH) { if (masterswState == 1) mainledswState = 1 - mainledswState; lastStableMain = LOW; }
        else if (curRawMain == HIGH) lastStableMain = HIGH;
    }

    static int lastRawMast = HIGH, lastStableMast = HIGH;
    int curRawMast = digitalRead(MASTERSW_PIN);
    if (curRawMast != lastRawMast) { masterswLastDebounceTime = millis(); lastRawMast = curRawMast; }
    if ((millis() - masterswLastDebounceTime) > DEBOUNCE_DELAY) {
        if (curRawMast == LOW && lastStableMast == HIGH) {
            masterswState = 1 - masterswState;
            lastStableMast = LOW;
        } else if (curRawMast == HIGH) lastStableMast = HIGH;
    }

    static int lastRawOver = HIGH, lastStableOver = HIGH;
    int curRawOver = digitalRead(OVERRIDE_SW_PIN);
    if (curRawOver != lastRawOver) { overrideSwLastDebounceTime = millis(); lastRawOver = curRawOver; }
    if ((millis() - overrideSwLastDebounceTime) > DEBOUNCE_DELAY) {
        if (curRawOver == LOW && lastStableOver == HIGH) {
            if (fanOverride) {
                digitalWrite(FAN_PIN, LOW); fanOverride = false; fanIsOnAutomatic = true;
            } else if (masterswState == 1 && fanIsOnAutomatic && digitalRead(FAN_PIN) == LOW) {
                fanOverride = true; fanOverrideStartTime = millis(); fanIsOnAutomatic = false; digitalWrite(FAN_PIN, HIGH);
            }
            lastStableOver = LOW;
        } else if (curRawOver == HIGH) lastStableOver = HIGH;
    }

    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
        if (timeinfo.tm_hour == 19 && timeinfo.tm_min == 15 && !isEveningToggleDone) { if (mainledswState == 0) mainledswState = 1; isEveningToggleDone = true; }
        if (timeinfo.tm_hour == 7 && timeinfo.tm_min == 15 && !isMorningToggleDone) { if (mainledswState == 1) mainledswState = 0; isMorningToggleDone = true; }
        if (!isTimeInRange(19, 15, 7, 15)) { isEveningToggleDone = false; isMorningToggleDone = false; }
    }

    digitalWrite(MAINLED_PIN, (mainledswState == 1 && masterswState == 1) ? HIGH : LOW);
    controlNightLED();
    digitalWrite(ACTIVITY_LED_PIN, masterswState == 1 ? HIGH : LOW);
    yield();
}
