#include "DHT.h"
#include "WiFi.h"
#include "WebServer.h"
#include "ArduinoJson.h"
#include "Preferences.h"

// Define the sensor type and the GPIO pin it's connected to.
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Define GPIO pins (ESP32-C3 safe mapping)
const int ledPin = 8;        // Status LED
const int controlPin = 6;    // Relay / Fan control
const int ldrPin = 2;        // LDR (analog input, ADC1_CH2)
const int optocouplerPin = 1;// Optocoupler input (ADC1_CH1)
const int controlPin2 = 7;   // PWM brightness control

// Thresholds
const int optocouplerThreshold = 2000;
int lightThreshold = 2350; // This is now user-configurable

// PWM duty cycle values (0–255)
int currentBrightnessDutyCycle = 128; // Default 50% brightness

// Calibration offsets
const float tempOffset = -0.1;
const float humidityOffset = 6.0;

// Temperature thresholds
float tempOn = 29.0;
float tempOff = 28.5;

// WiFi Configuration
const char* ssid = "wifi_slow";
// const char* password = "your_password"; // kalau ada password

IPAddress staticIP(192, 168, 1, 4);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);
Preferences preferences;

// Timers
unsigned long previousMillis = 0;
const long interval = 30000;
unsigned long previousLightMillis = 0;
const long lightInterval = 1000;
unsigned long lastLDRChangeTime = 0;
const long debounceDelay = 200;

// Global sensor values
float lastHumidity = 0;
float lastTemperature = 0;
int lastLight = 0;
String optocouplerStatus = "off";
String previousOptocouplerStatus = "off";
bool lastLDRState;

// Control modes
enum ControlMode { AUTOMATED, MANUAL_ON_PERMANENT, MANUAL_ON_TIMED };
ControlMode currentMode = AUTOMATED;
unsigned long manualTimerEnd = 0;

// === Web Handlers ===
void handleOn() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  currentMode = MANUAL_ON_PERMANENT;
  digitalWrite(controlPin, HIGH);
  server.send(200, "text/plain", "GPIO 6 is now manually ON (permanent).");
}
void handleOff() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  currentMode = AUTOMATED;
  digitalWrite(controlPin, LOW);
  server.send(200, "text/plain", "GPIO 6 is now OFF. Automated control resumed.");
}
void handleOn1h() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  currentMode = MANUAL_ON_TIMED;
  digitalWrite(controlPin, HIGH);
  manualTimerEnd = millis() + 3600000;
  server.send(200, "text/plain", "GPIO 6 is now manually ON for 1 hour.");
}
void handleOn2h() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  currentMode = MANUAL_ON_TIMED;
  digitalWrite(controlPin, HIGH);
  manualTimerEnd = millis() + 7200000;
  server.send(200, "text/plain", "GPIO 6 is now manually ON for 2 hours.");
}

void handleSetTempOn() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (server.hasArg("value")) {
    tempOn = server.arg("value").toFloat();
    preferences.putFloat("tempOn", tempOn); // Save to flash
    server.send(200, "text/plain", "Temperature ON threshold set to " + String(tempOn) + "C.");
  } else {
    server.send(400, "text/plain", "Missing 'value' parameter.");
  }
}

void handleSetTempOff() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (server.hasArg("value")) {
    tempOff = server.arg("value").toFloat();
    preferences.putFloat("tempOff", tempOff); // Save to flash
    server.send(200, "text/plain", "Temperature OFF threshold set to " + String(tempOff) + "C.");
  } else {
    server.send(400, "text/plain", "Missing 'value' parameter.");
  }
}

void handleSetBrightness() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (server.hasArg("value")) {
    currentBrightnessDutyCycle = server.arg("value").toInt();
    preferences.putUInt("brightness", currentBrightnessDutyCycle); // Save to flash
    // We no longer call analogWrite() here; the LDR logic in loop() will handle it.
    server.send(200, "text/plain", "LED brightness level set to " + String((int)(currentBrightnessDutyCycle / 2.55)) + "%.");
  } else {
    server.send(400, "text/plain", "Missing 'value' parameter.");
  }
}

void handleSetLightThreshold() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (server.hasArg("value")) {
    lightThreshold = server.arg("value").toInt();
    preferences.putUInt("lightThreshold", lightThreshold); // Save to flash
    server.send(200, "text/plain", "Light sensor threshold set to " + String(lightThreshold) + ".");
  } else {
    server.send(400, "text/plain", "Missing 'value' parameter.");
  }
}

void readAndStoreSensors() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  int light = analogRead(ldrPin);
  if (!isnan(humidity) && !isnan(temperature)) {
    lastHumidity = humidity + humidityOffset;
    lastTemperature = temperature + tempOffset;
  }
  lastLight = light;
}

void handleData() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  readAndStoreSensors();
  StaticJsonDocument<512> jsonDoc;
  char tempStr[6];
  sprintf(tempStr, "%.1f", lastTemperature);

  jsonDoc["temperature"] = tempStr;
  jsonDoc["humidity"] = (int)lastHumidity;
  jsonDoc["light"] = lastLight;
  jsonDoc["gpio_status"] = digitalRead(controlPin) == HIGH ? "on" : "off";
  jsonDoc["optocoupler_status"] = optocouplerStatus;
  jsonDoc["control_mode"] = currentMode == AUTOMATED ? "Automated" : (currentMode == MANUAL_ON_PERMANENT ? "Manual (Permanent)" : "Manual (Timed)");
  jsonDoc["brightness_level"] = (int)(currentBrightnessDutyCycle / 2.55);
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
  preferences.begin("my-app", false); // Open Preferences with a namespace

  // Load saved values from flash, or use defaults
  tempOn = preferences.getFloat("tempOn", 29.0);
  tempOff = preferences.getFloat("tempOff", 28.5);
  currentBrightnessDutyCycle = preferences.getUInt("brightness", 128);
  lightThreshold = preferences.getUInt("lightThreshold", 2500);

  pinMode(ledPin, OUTPUT);
  pinMode(controlPin, OUTPUT);
  pinMode(controlPin2, OUTPUT);

  // Initialize the LDR state based on the initial reading
  readAndStoreSensors();
  lastLDRState = lastLight < lightThreshold;
  if (lastLDRState) analogWrite(controlPin2, currentBrightnessDutyCycle);
  else analogWrite(controlPin2, 0);

  digitalWrite(ledPin, HIGH);

  WiFi.config(staticIP, gateway, subnet);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid); // kalau ada password, guna WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/on-perm", handleOn);
  server.on("/off", handleOff);
  server.on("/on-1h", handleOn1h);
  server.on("/on-2h", handleOn2h);
  server.on("/set-temp-on", handleSetTempOn);
  server.on("/set-temp-off", handleSetTempOff);
  server.on("/set-brightness", handleSetBrightness);
  server.on("/set-light-threshold", handleSetLightThreshold);
  server.on("/data", handleData);
  server.begin();
  Serial.println("Web server started.");
}

// === Loop ===
void loop() {
  // Always handle web client requests
  server.handleClient();

  // Read the optocoupler and update its status
  int optocouplerValue = analogRead(optocouplerPin);
  previousOptocouplerStatus = optocouplerStatus;
  optocouplerStatus = (optocouplerValue < optocouplerThreshold) ? "on" : "off";

  // === Main Control Logic ===
  // Only execute control logic if the optocoupler is ON.
  // This is a mandatory condition.
  if (optocouplerStatus == "on") {

    // Check for the manual timed mode timeout
    unsigned long currentMillis = millis();
    if (currentMode == MANUAL_ON_TIMED && currentMillis >= manualTimerEnd) {
      currentMode = AUTOMATED;
      digitalWrite(controlPin, LOW); // Turn off the relay when the timer expires
      Serial.println("Manual timer expired. Returning to automated mode.");
    }

    // Handle the control based on the current mode
    if (currentMode == AUTOMATED) {
      // Check if it's time to read sensors and apply temperature logic
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        readAndStoreSensors(); // Read temperature and humidity
        Serial.print("Temperature: ");
        Serial.print(lastTemperature);
        Serial.print(" C, Humidity: ");
        Serial.println(lastHumidity);

        if (lastTemperature >= tempOn) {
          digitalWrite(controlPin, HIGH);
          Serial.println("Temperature is above threshold. GPIO 6 ON.");
        } else if (lastTemperature <= tempOff) {
          digitalWrite(controlPin, LOW);
          Serial.println("Temperature is below threshold. GPIO 6 OFF.");
        }
      }
    } else {
      // We are in a MANUAL mode. The relay state is managed by the web handlers.
      // This is a safety check; the handlers already set the state.
      digitalWrite(controlPin, HIGH);
    }
  } else {
    // === Mandatory Condition: Optocoupler is OFF ===
    // If the optocoupler is off, turn the fan OFF, regardless of temperature.
    digitalWrite(controlPin, LOW);
    if (optocouplerStatus != previousOptocouplerStatus) {
      Serial.println("Optocoupler is now OFF. GPIO 6 is turned OFF.");
    }
  }

  // === LED and Brightness Control Logic ===
  // This logic is independent and always runs
  if (optocouplerStatus == "on") {
    // Blink LED to show activity
    digitalWrite(ledPin, LOW);
    delay(50);
    digitalWrite(ledPin, HIGH);
  }

  unsigned long currentLightMillis = millis();
  if (currentLightMillis - previousLightMillis >= lightInterval) {
    previousLightMillis = currentLightMillis;
    readAndStoreSensors();
    bool currentLDRState = lastLight < lightThreshold;

    if (currentLDRState != lastLDRState) {
      lastLDRChangeTime = currentLightMillis;
    }

    if ((currentLightMillis - lastLDRChangeTime) >= debounceDelay) {
      if (currentLDRState) {
        analogWrite(controlPin2, currentBrightnessDutyCycle);
      } else {
        analogWrite(controlPin2, 0);
      }
    }
    lastLDRState = currentLDRState;
  }
}
