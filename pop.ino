#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include "max6675.h"

// --- Hardware Pins ---
const int thermo_SO = 16;  
const int thermo_CS = 17;  
const int thermo_SCK = 18; 
const int ONE_WIRE_BUS = 7;   
const int chamberRelay = 4;   // SINGLE RELAY for the Chamber 
const int buzzerPin = 6;      
const int i2c_sda = 21;
const int i2c_scl = 22;

// --- Network & MQTT ---
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

// --- System Parameters ---
const float manualTargetTemp = 190.0; 

// --- PID Algorithm Variables ---
double pidInput, pidOutput, pidSetpoint;
double Kp = 5.0, Ki = 1.0, Kd = 1.0; // Tuning parameters
PID chamberPID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);
int WindowSize = 2000; // 2-second time-proportional window
unsigned long windowStartTime;

// --- Profiling Variables ---
const int MAX_PROFILE_POINTS = 100;           
const unsigned long RECORD_INTERVAL = 5000;   
float tempProfile[MAX_PROFILE_POINTS];        
int savedProfileLength = 0;                   
int currentProfileIndex = 0;                  
unsigned long lastIntervalTime = 0;           
bool hasProfile = false;                      

// --- State Variables ---
enum MachineState { IDLE, LEARN_MODE, SMART_MODE, DONE };
MachineState currentState = IDLE;
unsigned long lastTempReadTime = 0;
float chamberTemp = 0.0;
float exhaustTemp = 0.0;

// --- Objects ---
MAX6675 thermocouple(thermo_SCK, thermo_CS, thermo_SO);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);
LiquidCrystal_I2C lcd(0x27, 16, 2); // 16x2 Display
Preferences preferences;

// --- Wi-Fi & MQTT Setup ---
void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) message += (char)payload[i];
  
  if (message == "PLAY" && hasProfile && (currentState == IDLE || currentState == DONE)) {
    currentProfileIndex = 0;
    changeState(SMART_MODE);
  } else if (message == "LEARN" && (currentState == IDLE || currentState == DONE)) {
    currentProfileIndex = 0;
    savedProfileLength = 0;
    hasProfile = false;
    changeState(LEARN_MODE);
  } else if (message == "STOP") {
    if (currentState == LEARN_MODE) {
      saveProfileToPreferences();
      hasProfile = true;
    }
    changeState(DONE);
  }
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32PopcornMaker_S3")) {
      client.subscribe("popcorn/command");
    } else {
      delay(5000);
    }
  }
}

void setup() {
  Wire.begin(i2c_sda, i2c_scl); 
  pinMode(chamberRelay, OUTPUT);
  pinMode(buzzerPin, OUTPUT); 
  digitalWrite(chamberRelay, LOW);

  ds18b20.begin();
  loadProfileFromPreferences();
  
  // Initialize PID
  windowStartTime = millis();
  chamberPID.SetOutputLimits(0, WindowSize);
  chamberPID.SetMode(MANUAL); // Keep off until active

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("ESP32 POPCORN");
  lcd.setCursor(0, 1); lcd.print("Connecting WiFi.");
  
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);

  lcd.clear();
  updateDisplay();
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  unsigned long currentMillis = millis();

  // --- 1. Read Sensors & Publish Telemetry ---
  if (currentMillis - lastTempReadTime >= 500) {
    chamberTemp = thermocouple.readCelsius();
    ds18b20.requestTemperatures();
    exhaustTemp = ds18b20.getTempCByIndex(0);
    lastTempReadTime = currentMillis;
    pidInput = chamberTemp;
    
    char payload[150];
    snprintf(payload, sizeof(payload), "{\"chamber\":%.1f, \"exhaust\":%.1f, \"state\":%d}", chamberTemp, exhaustTemp, currentState);
    client.publish("popcorn/telemetry", payload);
    
    updateDisplay();
  }

  // --- 2. Execute State & PID Control Logic ---
  switch (currentState) {
    case IDLE:
    case DONE:
      chamberPID.SetMode(MANUAL);
      digitalWrite(chamberRelay, LOW);
      break;

    case LEARN_MODE:
      chamberPID.SetMode(AUTOMATIC);
      pidSetpoint = manualTargetTemp;
      chamberPID.Compute();
      
      // Time-Proportional Relay Output
      if (currentMillis - windowStartTime > WindowSize) windowStartTime += WindowSize;
      if (pidOutput > (currentMillis - windowStartTime)) digitalWrite(chamberRelay, HIGH);
      else digitalWrite(chamberRelay, LOW);

      // Record Profile
      if (currentMillis - lastIntervalTime >= RECORD_INTERVAL) {
        if (currentProfileIndex < MAX_PROFILE_POINTS) {
          tempProfile[currentProfileIndex] = chamberTemp;
          savedProfileLength = ++currentProfileIndex; 
        }
        lastIntervalTime = currentMillis;
      }
      break;

    case SMART_MODE: 
      chamberPID.SetMode(AUTOMATIC);
      pidSetpoint = tempProfile[currentProfileIndex];
      chamberPID.Compute();

      // Time-Proportional Relay Output
      if (currentMillis - windowStartTime > WindowSize) windowStartTime += WindowSize;
      if (pidOutput > (currentMillis - windowStartTime)) digitalWrite(chamberRelay, HIGH);
      else digitalWrite(chamberRelay, LOW);

      // Playback Progression
      if (currentMillis - lastIntervalTime >= RECORD_INTERVAL) {
        lastIntervalTime = currentMillis;
        if (++currentProfileIndex >= savedProfileLength) changeState(DONE);
      }
      break;
  }
}

// --- Memory & Display Helpers ---
void saveProfileToPreferences() {
  preferences.begin("popcorn", false); 
  preferences.putInt("len", savedProfileLength);
  preferences.putBytes("prof", tempProfile, sizeof(tempProfile));
  preferences.end();
}

void loadProfileFromPreferences() {
  preferences.begin("popcorn", true); 
  savedProfileLength = preferences.getInt("len", 0);
  if (savedProfileLength > 0 && savedProfileLength <= MAX_PROFILE_POINTS) {
    preferences.getBytes("prof", tempProfile, sizeof(tempProfile));
    hasProfile = true;
  } else {
    hasProfile = false;
  }
  preferences.end();
}

void changeState(MachineState newState) {
  if (newState == DONE) {
    beep(500); delay(200); beep(500); delay(200); beep(500); 
  }
  currentState = newState;
  lastIntervalTime = millis(); 
  lcd.clear(); 
  updateDisplay();
}

void beep(int duration) {
  digitalWrite(buzzerPin, HIGH);
  delay(duration);
  digitalWrite(buzzerPin, LOW);
}

void updateDisplay() {
  // Row 0: "Cham:150.0 R:1" (Fits 16 chars)
  lcd.setCursor(0, 0);
  lcd.print("C:");
  if (chamberTemp < 100) lcd.print(" ");
  lcd.print(chamberTemp, 1);
  lcd.print(" R:");
  lcd.print(digitalRead(chamberRelay) ? "ON " : "OFF");

  // Row 1: Mode specific text
  lcd.setCursor(0, 1);
  switch (currentState) {
    case IDLE:    
      lcd.print("IDLE|Prof:");
      lcd.print(hasProfile ? "YES" : "NO ");
      break;
    case LEARN_MODE: 
      lcd.print("LEARN | Pts:");
      if(currentProfileIndex < 10) lcd.print(" "); 
      lcd.print(currentProfileIndex);
      break;
    case SMART_MODE: 
      lcd.print("Tgt:");
      lcd.print(tempProfile[currentProfileIndex], 0); 
      lcd.print("C "); 
      lcd.print((savedProfileLength - currentProfileIndex) * (RECORD_INTERVAL/1000));
      lcd.print("s ");
      break;
    case DONE:    
      lcd.print("DONE! ENJOY!    "); 
      break;
  }
}
