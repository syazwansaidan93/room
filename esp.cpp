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
int lastStableOver = HIGH;
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

const char* index_html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32C3 Control</title>
    <style>
        body { font-family: sans-serif; background: #121212; color: white; text-align: center; padding: 20px; }
        .card { background: #1e1e1e; padding: 20px; border-radius: 10px; margin-bottom: 20px; box-shadow: 0 4px 8px rgba(0,0,0,0.5); }
        .btn { padding: 12px 24px; font-size: 16px; border: none; border-radius: 5px; cursor: pointer; color: white; margin: 5px; }
        .btn-blue { background: #007bff; }
        .btn-red { background: #dc3545; }
        .btn-green { background: #28a745; }
        input[type=number], input[type=range] { padding: 8px; border-radius: 5px; border: 1px solid #444; background: #333; color: white; width: 80px; }
    </style>
</head>
<body>
    <div class="card">
        <h2>Environment</h2>
        <p>Temp: <span id="temp">--</span>°C</p>
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
        <p>On Threshold: <input type="number" step="0.1" id="tOn" onchange="setVal('/set_temp_on?tempOn=', this.value)"> °C</p>
        <p>Off Threshold: <input type="number" step="0.1" id="tOff" onchange="setVal('/set_temp_off?tempOff=', this.value)"> °C</p>
        <p>Night LED PWM: <input type="range" min="0" max="255" id="nPwm" oninput="setVal('/set_nightled_pwm?pwmValue=', this.value)"> <span id="pwmVal"></span></p>
    </div>
    <div class="card">
        <h2>Update</h2>
        <form method='POST' action='/update' enctype='multipart/form-data'>
            <input type='file' name='update'>
            <input type='submit' value='Update' class="btn btn-red">
        </form>
    </div>
    <script>
        function api(path) { fetch(path); }
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
            });
        }
        setInterval(updateState, 2000);
        updateState();
    </script>
</body>
</html>
)rawliteral";

void onWiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            server.begin();
            isNtpSynced = false;
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            isNtpSynced = false;
            WiFi.begin(ssid);
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
    if (startInMinutes > endInMinutes) {
        return (nowInMinutes >= startInMinutes || nowInMinutes < endInMinutes);
    } else {
        return (nowInMinutes >= startInMinutes && nowInMinutes < endInMinutes);
    }
}

void handleMainToggle() {
    if (masterswState == 1) mainledswState = 1 - mainledswState;
    addCorsHeaders();
    server.send(200, "text/plain", String(mainledswState));
}

void handleMasterToggle() {
    masterswState = 1 - masterswState;
    masterSwOledDisplayStartTime = millis();
    if (!isOledActive) {
        isOledActive = true;
        display.ssd1306_command(SSD1306_DISPLAYON);
    }
    addCorsHeaders();
    server.send(200, "text/plain", String(masterswState));
}

void handleSetTempOn() {
    if (!server.hasArg("tempOn")) {
        addCorsHeaders();
        server.send(400);
        return;
    }
    tempThresholdOn = server.arg("tempOn").toFloat();
    preferences.begin("bilik-config", false);
    preferences.putFloat("tempOn", tempThresholdOn);
    preferences.end();
    addCorsHeaders();
    server.send(200);
}

void handleSetTempOff() {
    if (!server.hasArg("tempOff")) {
        addCorsHeaders();
        server.send(400);
        return;
    }
    tempThresholdOff = server.arg("tempOff").toFloat();
    preferences.begin("bilik-config", false);
    preferences.putFloat("tempOff", tempThresholdOff);
    preferences.end();
    addCorsHeaders();
    server.send(200);
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
        addCorsHeaders();
        server.send(200);
    } else {
        oledErrorDisplayStartTime = millis();
        if (!isOledActive) {
            isOledActive = true;
            display.ssd1306_command(SSD1306_DISPLAYON);
        }
        addCorsHeaders();
        server.send(400);
    }
}

void handleFanOff() {
    if (fanIsOnAutomatic && digitalRead(FAN_PIN) == HIGH) {
        addCorsHeaders();
        server.send(400);
    } else {
        digitalWrite(FAN_PIN, LOW);
        fanOverride = false;
        fanIsOnAutomatic = true;
        addCorsHeaders();
        server.send(200);
    }
}

void handleSetNightledPWM() {
    if (!server.hasArg("pwmValue")) {
        addCorsHeaders();
        server.send(400);
        return;
    }
    nightledPWMValue = server.arg("pwmValue").toInt();
    preferences.begin("bilik-config", false);
    preferences.putInt("nightledPWM", nightledPWMValue);
    preferences.end();
    addCorsHeaders();
    server.send(200);
}

void handleState() {
    StaticJsonDocument<350> doc;
    struct tm ti;
    char ts[32];
    getLocalTime(&ti);
    strftime(ts, sizeof(ts), "%H:%M:%S", &ti);
    doc["currentTime"] = ts;
    doc["mainledswState"] = mainledswState;
    doc["masterswState"] = masterswState;
    doc["mainledState"] = digitalRead(MAINLED_PIN);
    doc["nightledPWMValue"] = nightledPWMValue;
    doc["temperature"] = currentTemperature;
    doc["fanState"] = digitalRead(FAN_PIN);
    doc["tempThresholdOn"] = tempThresholdOn;
    doc["tempThresholdOff"] = tempThresholdOff;
    doc["fanIsOnAutomatic"] = fanIsOnAutomatic;
    String res;
    serializeJson(doc, res);
    addCorsHeaders();
    server.send(200, "application/json", res);
}

void controlNightLED() {
    if (isTimeInRange(19, 15, 7, 15) && (mainledswState == 0 || masterswState == 0)) 
        ledcWrite(NIGHTLED_PIN, nightledPWMValue);
    else 
        ledcWrite(NIGHTLED_PIN, 0);
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
    } else if (masterSwOledDisplayStartTime != 0 && (millis() - masterSwOledDisplayStartTime) < masterSwOledDisplayDuration) {
        String masterStateStr = masterswState == 1 ? "SW ON" : "SW OFF";
        display.setTextSize(3);
        int16_t x1, y1; uint16_t w, h;
        display.getTextBounds(masterStateStr, 0, 0, &x1, &y1, &w, &h);
        display.setCursor((SCREEN_WIDTH - w) / 2, (SCREEN_HEIGHT - h) / 2);
        display.print(masterStateStr);
    } else if (fanOverride && !isTempStatusDisplay) {
        unsigned long elap = millis() - fanOverrideStartTime;
        long rem = (fanOverrideDuration - elap) / 1000;
        display.setCursor(0, 0);
        display.setTextSize(1);
        display.print("FAN OVERRIDE ");
        display.println(masterswState == 1 ? "MS ON" : "MS OFF");
        display.setCursor(0, 16);
        display.setTextSize(2);
        display.printf("%02ld:%02ld", rem / 60, rem % 60);
    } else {
        display.setCursor(0, 0);
        display.setTextSize(1);
        display.print("WiFi: [");
        int bars = map(constrain(WiFi.RSSI(), -100, -30), -100, -30, 0, 13);
        for (int i = 0; i < 13; i++) display.print(i < bars ? "=" : " ");
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
    Wire.begin(OLED_SDA, OLED_SCL);
    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    display.clearDisplay();
    display.display();

    WiFi.onEvent(onWiFiEvent);
    WiFi.config(staticIP, gateway, subnet);
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
    sensors.setResolution(12);
    tempSensorAddress[0] = 0x28; tempSensorAddress[1] = 0x07; tempSensorAddress[2] = 0xBB; tempSensorAddress[3] = 0x83;
    tempSensorAddress[4] = 0x00; tempSensorAddress[5] = 0x00; tempSensorAddress[6] = 0x00; tempSensorAddress[7] = 0xF5;

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

    server.on("/update", HTTP_POST, []() {
        server.sendHeader("Connection", "close");
        server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
        ESP.restart();
    }, []() {
        HTTPUpload& upload = server.upload();
        if (upload.status == UPLOAD_FILE_START) {
            Update.begin(UPDATE_SIZE_UNKNOWN);
        } else if (upload.status == UPLOAD_FILE_WRITE) {
            Update.write(upload.buf, upload.currentSize);
        } else if (upload.status == UPLOAD_FILE_END) {
            Update.end(true);
        }
    });

    delay(1000);
    display.ssd1306_command(SSD1306_DISPLAYOFF);
}

void loop() {
    if (WiFi.status() == WL_CONNECTED) {
        server.handleClient();
        if (!isNtpSynced) {
            struct tm ti;
            if (getLocalTime(&ti)) isNtpSynced = true;
        }
    }

    if (digitalRead(TOUCH_SENSOR_PIN) == HIGH) {
        if (!isOledActive) { isOledActive = true; display.ssd1306_command(SSD1306_DISPLAYON); }
        oledOnTime = millis();
        isTempStatusDisplay = fanOverride ? !isTempStatusDisplay : true;
    }

    if (isOledActive && (millis() - oledOnTime >= oledActiveDuration) && masterSwOledDisplayStartTime == 0 && !fanOverride && oledErrorDisplayStartTime == 0) {
        display.clearDisplay(); display.display(); display.ssd1306_command(SSD1306_DISPLAYOFF);
        isOledActive = false; isTempStatusDisplay = false;
    }

    static unsigned long lastDisp = 0;
    if (isOledActive && (millis() - lastDisp >= 1000)) { updateDisplay(); lastDisp = millis(); }

    if (masterswState == 1 && (millis() - lastSensorReadTime >= sensorReadInterval)) {
        sensors.requestTemperatures();
        float temp = sensors.getTempC(tempSensorAddress);
        if (temp != DEVICE_DISCONNECTED_C) { currentTemperature = temp; sensorIsFaulty = false; consecutiveFailedReads = 0; }
        else if (++consecutiveFailedReads >= maxFailedReads) sensorIsFaulty = true;
        lastSensorReadTime = millis();
    }

    if (fanOverride && (millis() - fanOverrideStartTime) >= fanOverrideDuration) {
        fanOverride = false; fanIsOnAutomatic = true;
    }

    if (masterswState == 1 && fanIsOnAutomatic && !fanOverride) {
        if (scheduledFanActive) {
            if (millis() - lastScheduledFanToggle >= fanScheduleDuration) { digitalWrite(FAN_PIN, LOW); scheduledFanActive = false; }
        } else if (digitalRead(FAN_PIN) == LOW && millis() - lastScheduledFanToggle >= fanScheduleInterval) {
            digitalWrite(FAN_PIN, HIGH); lastScheduledFanToggle = millis(); scheduledFanActive = true;
        }
    }

    if (!scheduledFanActive && !fanOverride && fanIsOnAutomatic && !sensorIsFaulty && masterswState == 1) {
        if (millis() - lastFanStateChange >= fanCooldownDelay) {
            if (digitalRead(FAN_PIN) == LOW && currentTemperature >= tempThresholdOn) { digitalWrite(FAN_PIN, HIGH); lastFanStateChange = millis(); }
            else if (digitalRead(FAN_PIN) == HIGH && currentTemperature <= tempThresholdOff) { digitalWrite(FAN_PIN, LOW); lastFanStateChange = millis(); }
        }
    } else if (!scheduledFanActive && !fanOverride && (masterswState == 0 || !fanIsOnAutomatic)) {
        digitalWrite(FAN_PIN, LOW);
    } else if (fanOverride) {
        digitalWrite(FAN_PIN, HIGH);
    }

    static int lastS1 = HIGH; int r1 = digitalRead(MAINLEDSW_PIN);
    if (r1 != lastS1) { mainledswLastDebounceTime = millis(); lastS1 = r1; }
    if ((millis() - mainledswLastDebounceTime) > DEBOUNCE_DELAY && r1 == LOW && lastMainledswReading == HIGH) {
        if (masterswState == 1) mainledswState = 1 - mainledswState; lastMainledswReading = LOW;
    } else if (r1 == HIGH) lastMainledswReading = HIGH;

    static int lastS2 = HIGH; int r2 = digitalRead(MASTERSW_PIN);
    if (r2 != lastS2) { masterswLastDebounceTime = millis(); lastS2 = r2; }
    if ((millis() - masterswLastDebounceTime) > DEBOUNCE_DELAY && r2 == LOW && lastMasterswReading == HIGH) {
        masterswState = 1 - masterswState; masterSwOledDisplayStartTime = millis();
        if (!isOledActive) { isOledActive = true; display.ssd1306_command(SSD1306_DISPLAYON); }
        lastMasterswReading = LOW;
    } else if (r2 == HIGH) lastMasterswReading = HIGH;

    static int lastS3 = HIGH; int r3 = digitalRead(OVERRIDE_SW_PIN);
    if (r3 != lastS3) { overrideSwLastDebounceTime = millis(); lastS3 = r3; }
    if ((millis() - overrideSwLastDebounceTime) > DEBOUNCE_DELAY && r3 == LOW && lastStableOver == HIGH) {
        if (fanOverride) { fanOverride = false; fanIsOnAutomatic = true; }
        else handleFanOn30m();
        lastStableOver = LOW;
    } else if (r3 == HIGH) lastStableOver = HIGH;

    struct tm ti;
    if (getLocalTime(&ti)) {
        if (ti.tm_hour == 19 && ti.tm_min == 15 && !isEveningToggleDone) { mainledswState = 1; isEveningToggleDone = true; }
        if (ti.tm_hour == 7 && ti.tm_min == 15 && !isMorningToggleDone) { mainledswState = 0; isMorningToggleDone = true; }
        if (ti.tm_hour != 19 && ti.tm_hour != 7) { isEveningToggleDone = false; isMorningToggleDone = false; }
    }

    digitalWrite(MAINLED_PIN, (mainledswState == 1 && masterswState == 1) ? HIGH : LOW);
    controlNightLED();
    digitalWrite(ACTIVITY_LED_PIN, masterswState == 1 ? HIGH : LOW);
    yield();
}
