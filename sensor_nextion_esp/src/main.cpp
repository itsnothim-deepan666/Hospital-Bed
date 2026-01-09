#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ----------------------- Pins & Config -----------------------
#define NEXTION_RX_PIN 16
#define NEXTION_TX_PIN 17
#define NEXTION_BAUD 9600
#define ECG_PIN 34
#define ECG_LO_MINUS 35
#define ECG_LO_PLUS 25
#define MOTOR_PWM_VALUE 200
#define MAX_MOVE_TIME 10000
#define TEMP_SENSOR_ADDR 0x5A
#define PULSE_SENSOR_ADDR 0x57
#define IMU_SENSOR_ADDR 0x68
#define MUX_SENSOR_ADDR 0x70
#define MUX_CHANNEL_TOP_IMU 0
#define MUX_CHANNEL_BOTTOM_IMU 2
#define MUX_CHANNEL_SIDE_IMU 3
#define MUX_CHANNEL_TEMP_SENSOR 6
#define MUX_CHANNEL_PULSE_SENSOR 7
#define MAX_TOP_ANGLE 60
#define MAX_BOTTOM_ANGLE 30
#define MAX_SIDE_ANGLE 30
#define ESP32_LED_PIN 2
#define LOCAL_UDP_PORT 12345
#define BLINK_COOLDOWN 500
#define HEALTH_UPDATE_INTERVAL 1000
#define TEMP_READ_INTERVAL 2000
#define PULSE_READ_INTERVAL 100
#define ECG_READ_INTERVAL 10
#define BP_UPDATE_INTERVAL 5000
#define NEXTION_BUFFER_SIZE 64
#define UDP_PACKET_SIZE 255
#define SENSOR_RETRY_ATTEMPTS 3
#define SENSOR_RETRY_DELAY 500
#define WIFI_RECONNECT_INTERVAL 30000
#define DATA_STALE_TIMEOUT 5000
#define IMU_READ_TIMEOUT 100

// Motor pin configuration
const uint8_t pwmMotor1 = 32, dirMotor1 = 33;
const uint8_t pwmMotor2 = 18, dirMotor2 = 19;
const uint8_t pwmMotor3 = 4,  dirMotor3 = 5;

// Network credentials (const char* uses less RAM than String)
const char* ssid = "deepan";
const char* password = "hehehehe";

// ----------------------- UI mapping -----------------------
const char* const OPTION_BUTTONS[4] PROGMEM = {"b0", "b1", "b3", "b2"};
const char* const ANGLE_BUTTONS[5] PROGMEM  = {"b1", "b2", "b3", "b4", "b0"};
const char* const IMU_NAMES[3] PROGMEM = {"TOP", "BOTTOM", "SIDE"};
const char* const BODY_NAMES[4] PROGMEM = {"TOP", "RIGHT", "BOTTOM", "LEFT"};
const char* const ANGLE_NAMES[5] PROGMEM = {"0°", "30°", "45°", "60°", "BACK"};

// ----------------------- Globals -----------------------
enum SystemState : uint8_t { STATE_IDLE, STATE_SELECTING_BODY, STATE_SELECTING_ANGLE, STATE_MOVING, STATE_CONFIRMING };
SystemState currentState = STATE_SELECTING_BODY;
int8_t selectedBody = -1;
uint8_t currentOptionIndex = 0;
uint8_t currentAngleIndex = 0;
MAX30105 particleSensor;
Adafruit_MLX90614 mlx;
HardwareSerial SerialNextion(2);

#define NUM_IMUS 3
Adafruit_MPU6050 imu_arr[NUM_IMUS];
bool imu_initialized[NUM_IMUS] = {false, false, false};
const uint8_t imu_channels[NUM_IMUS] = {MUX_CHANNEL_TOP_IMU, MUX_CHANNEL_BOTTOM_IMU, MUX_CHANNEL_SIDE_IMU};

// Sensor status flags
struct {
  bool mlx : 1;
  bool pulse : 1;
  bool ecg : 1;
  bool wifi : 1;
} sensor_status = {false, false, true, false};

WiFiUDP udp;
char incomingPacket[UDP_PACKET_SIZE];

// Health data
float spo2_value = 98.6;
float temperature_value = 36.5;
float object_temp = 0;
float skin_temp = 0;
uint8_t systolic_bp = 120;
uint8_t diastolic_bp = 80;
uint8_t heart_rate = 75;
uint16_t ecg_value = 512;
bool alert_active = false;
char alert_message[64] = "";

// Pulse detection
const uint8_t RATE_SIZE = 4;
uint8_t rates[RATE_SIZE];
uint8_t rateSpot = 0;
unsigned long lastBeat = 0;
float beatsPerMinute = 0;
uint8_t beatAvg = 0;

// ECG filtering
const float ALPHA = 0.075;
int16_t ecgFiltered = 0;
int16_t ecgBaseline = 512;

// IMU data
float currentAngles[NUM_IMUS] = {0, 0, 0};

// Timing
unsigned long lastBlinkTime = 0;
unsigned long lastTempUpdate = 0;
unsigned long lastPulseUpdate = 0;
unsigned long lastECGUpdate = 0;
unsigned long lastWiFiCheck = 0;

// ----------------------- Forward Declarations -----------------------
void sendAlert(const char* msg);
void stopMotors();

// ----------------------- Helpers -----------------------
inline bool selectMUXChannel(uint8_t ch) {
  if(ch > 7) return false;
  Wire.beginTransmission(MUX_SENSOR_ADDR);
  Wire.write(1 << ch);
  uint8_t error = Wire.endTransmission();
  if(error != 0) {
    Serial.printf("[ERROR] MUX channel %d switch failed: %d\n", ch, error);
    return false;
  }
  delayMicroseconds(100);
  return true;
}
inline void nextionEndCmd() {
  SerialNextion.write(0xFF);
  SerialNextion.write(0xFF);
  SerialNextion.write(0xFF);
}

void nextionCommand(const char* cmd) {
  SerialNextion.print(cmd);
  nextionEndCmd();
}

void nextionCommand(const String& cmd) {
  SerialNextion.print(cmd);
  nextionEndCmd();
}
// Calculate horizontal angle from IMU accelerometer data
float calculateAngle(const sensors_event_t& a, uint8_t idx) {
  static const float offsets[3][3] = {
    {0.05, -0.01, 0.03},
    {0.02, -0.02, 0.01},
    {0, 0, 0}
  };
  float ax = a.acceleration.x - offsets[idx][0];
  float ay = a.acceleration.y - offsets[idx][1];
  float az = a.acceleration.z - offsets[idx][2];
  return atan2(-ax, sqrtf(ay*ay + az*az)) * 180.0 / PI;
}

// ----------------------- Display -----------------------
void updateHealthDisplay() {
  static unsigned long last = 0;
  if(millis() - last < HEALTH_UPDATE_INTERVAL) return;
  last = millis();
  char buf[32];
  bool tempStale = (millis() - lastTempUpdate > DATA_STALE_TIMEOUT && lastTempUpdate > 0);
  bool pulseStale = (millis() - lastPulseUpdate > DATA_STALE_TIMEOUT && lastPulseUpdate > 0);
  bool ecgStale = (millis() - lastECGUpdate > DATA_STALE_TIMEOUT && lastECGUpdate > 0);
  snprintf(buf, sizeof(buf), "vaSpO.txt=\"%.1f%%%s\"", spo2_value, pulseStale ? " !" : ""); nextionCommand(buf);
  snprintf(buf, sizeof(buf), "vaTen.txt=\"%.1f°C%s\"", temperature_value, tempStale ? " !" : ""); nextionCommand(buf);
  snprintf(buf, sizeof(buf), "vaBHF.txt=\"%d/%d\"", systolic_bp, diastolic_bp); nextionCommand(buf);
  snprintf(buf, sizeof(buf), "vaHR.txt=\"%d BPM%s\"", heart_rate, pulseStale ? " !" : ""); nextionCommand(buf);
  if(alert_active) {
    snprintf(buf, sizeof(buf), "alertBox.txt=\"%s\"", alert_message); nextionCommand(buf);
    nextionCommand("alertBox.bco=63488");
  } else { nextionCommand("alertBox.txt=\"System Ready\""); nextionCommand("alertBox.bco=1024"); }
}
void updateAngleDisplay() {
  char buf[64];
  snprintf(buf, sizeof(buf), "t1.txt=\"T:%.0f° B:%.0f° S:%.0f°\"",
           currentAngles[0], currentAngles[1], currentAngles[2]);
  nextionCommand(buf);
}
void updateNextionDisplay() {
  char buf[80];
  if(currentState == STATE_SELECTING_BODY) {
    nextionCommand("page 0"); delay(50);
    for(uint8_t i = 0; i < 4; i++) {
      snprintf(buf, sizeof(buf), "%s.bco=%d", OPTION_BUTTONS[i], i == currentOptionIndex ? 65504 : 8200);
      nextionCommand(buf);
    }
    snprintf(buf, sizeof(buf), "t0.txt=\"Select: %s\"", BODY_NAMES[currentOptionIndex]);
    nextionCommand(buf);
    nextionCommand("t1.txt=\"Blink to cycle, Touch to select\"");
  } else if(currentState == STATE_SELECTING_ANGLE) {
    nextionCommand("page 1"); delay(50);
    for(uint8_t i = 0; i < 5; i++) {
      snprintf(buf, sizeof(buf), "%s.bco=%d", ANGLE_BUTTONS[i], i == currentAngleIndex ? 65504 : 8200);
      nextionCommand(buf);
    }
    snprintf(buf, sizeof(buf), "t0.txt=\"Body: %s\\rAngle: %s\"", BODY_NAMES[selectedBody], ANGLE_NAMES[currentAngleIndex]);
    nextionCommand(buf);
  }
}
void sendECGData(uint8_t v) {
  static uint8_t cnt = 0;
  char buf[16];
  snprintf(buf, sizeof(buf), "add 1,0,%d", v);
  nextionCommand(buf);
  if(++cnt > 200) {
    nextionCommand("clr 1,0");
    cnt = 0;
  }
}

// ----------------------- Motors -----------------------
void setupPWM() {
  const uint8_t pins[] = {pwmMotor1, pwmMotor2, pwmMotor3, dirMotor1, dirMotor2, dirMotor3};
  for(uint8_t pin : pins) pinMode(pin, OUTPUT);
  analogWrite(pwmMotor1, 0); analogWrite(pwmMotor2, 0); analogWrite(pwmMotor3, 0);
}

void stopMotors() {
  analogWrite(pwmMotor1, 0); analogWrite(pwmMotor2, 0); analogWrite(pwmMotor3, 0);
  nextionCommand("t1.txt=\"Motors Stopped\"");
}

inline void setMotor(uint8_t pwmPin, uint8_t dirPin, bool forward, uint8_t speed) {
  digitalWrite(dirPin, forward ? HIGH : LOW);
  analogWrite(pwmPin, speed);
}

// Unified motor movement function - reduces 6 functions to 1
void moveMotor(uint8_t imuIdx, uint8_t muxCh, uint8_t pwmPin, uint8_t dirPin, float targetAngle, bool forward) {
  if(!imu_initialized[imuIdx]) return;
  const unsigned long startTime = millis();
  bool isUp = forward ^ (imuIdx == 1);
  uint8_t readFailures = 0;
  
  while(millis() - startTime < MAX_MOVE_TIME && !alert_active) {
    if(!selectMUXChannel(muxCh)) {
      delay(100); continue;
    }
    unsigned long readStart = millis();
    sensors_event_t a, g, t;
    if(imu_arr[imuIdx].getEvent(&a, &g, &t)) {
      if(millis() - readStart > IMU_READ_TIMEOUT) {
        Serial.println("[WARNING] IMU read timeout");
        readFailures++;
        if(readFailures > 5) { sendAlert("IMU Read Timeout"); break; }
      } else {
        readFailures = 0;
        float curr = calculateAngle(a, imuIdx);
        if(!isnan(curr) && (isUp ? curr >= targetAngle : curr <= targetAngle)) break;
      }
    } else {
      readFailures++;
      if(readFailures > 5) { sendAlert("IMU Communication Lost"); break; }
    }
    setMotor(pwmPin, dirPin, forward, MOTOR_PWM_VALUE);
    delay(50);
  }
  stopMotors();
}

// Simplified wrapper functions
void TopUp(float a) { moveMotor(0, MUX_CHANNEL_TOP_IMU, pwmMotor3, dirMotor3, a, false); }
void TopDown(float a) { moveMotor(0, MUX_CHANNEL_TOP_IMU, pwmMotor3, dirMotor3, a, true); }
void BottomUp(float a) { moveMotor(1, MUX_CHANNEL_BOTTOM_IMU, pwmMotor2, dirMotor2, a, true); }
void BottomDown(float a) { moveMotor(1, MUX_CHANNEL_BOTTOM_IMU, pwmMotor2, dirMotor2, a, false); }
void SideLeft(float a) { moveMotor(2, MUX_CHANNEL_SIDE_IMU, pwmMotor1, dirMotor1, a, false); }
void SideRight(float a) { moveMotor(2, MUX_CHANNEL_SIDE_IMU, pwmMotor1, dirMotor1, a, true); }

// ----------------------- Sensors -----------------------
void initSensors() {
  selectMUXChannel(MUX_CHANNEL_TEMP_SENSOR); delay(100);
  sensor_status.mlx = mlx.begin(TEMP_SENSOR_ADDR, &Wire);
  Serial.println(sensor_status.mlx ? "[OK] MLX90614 initialized" : "[WARNING] MLX90614 not found");
  selectMUXChannel(MUX_CHANNEL_PULSE_SENSOR); delay(100);
  sensor_status.pulse = particleSensor.begin(Wire, I2C_SPEED_STANDARD);
  if(sensor_status.pulse) {
    particleSensor.setup(60, 4, 2, 100, 411, 4096);
    particleSensor.setPulseAmplitudeRed(0x0A); particleSensor.setPulseAmplitudeIR(0x0A);
    particleSensor.enableDIETEMPRDY(); memset(rates, 0, RATE_SIZE);
    Serial.println("[OK] MAX30102 initialized");
  } else Serial.println("[WARNING] MAX30102 not found");
  pinMode(ECG_PIN, INPUT); pinMode(ECG_LO_MINUS, INPUT_PULLUP); pinMode(ECG_LO_PLUS, INPUT_PULLUP);
}
void readMLX90614() {
  static unsigned long last = 0;
  if(millis() - last < TEMP_READ_INTERVAL) return;
  last = millis();
  if(!sensor_status.mlx) {
    // Attempt to reinitialize sensor
    if(selectMUXChannel(MUX_CHANNEL_TEMP_SENSOR)) {
      delay(100);
      if(mlx.begin(TEMP_SENSOR_ADDR, &Wire)) {
        sensor_status.mlx = true;
        Serial.println("[RECOVERY] MLX90614 reinitialized");
      }
    }
    return;
  }
  for(uint8_t attempt = 0; attempt < SENSOR_RETRY_ATTEMPTS; attempt++) {
    if(!selectMUXChannel(MUX_CHANNEL_TEMP_SENSOR)) continue;
    const float amb = mlx.readAmbientTempC(), obj = mlx.readObjectTempC();
    if(!isnan(amb) && !isnan(obj) && amb > 0 && amb < 100 && obj > 0 && obj < 100) {
      object_temp = amb; skin_temp = obj; temperature_value = skin_temp;
      lastTempUpdate = millis();
      return;
    }
    if(attempt < SENSOR_RETRY_ATTEMPTS - 1) delay(50);
  }
  Serial.println("[WARNING] MLX90614 read failed after retries");
}
void readMAX30102() {
  static unsigned long last = 0;
  if(millis() - last < PULSE_READ_INTERVAL) return;
  last = millis();
  if(!sensor_status.pulse) {
    // Attempt to reinitialize sensor
    if(selectMUXChannel(MUX_CHANNEL_PULSE_SENSOR)) {
      delay(100);
      if(particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
        particleSensor.setup(60, 4, 2, 100, 411, 4096);
        particleSensor.setPulseAmplitudeRed(0x0A); particleSensor.setPulseAmplitudeIR(0x0A);
        particleSensor.enableDIETEMPRDY();
        sensor_status.pulse = true;
        Serial.println("[RECOVERY] MAX30102 reinitialized");
      }
    }
    return;
  }
  if(!selectMUXChannel(MUX_CHANNEL_PULSE_SENSOR)) return;
  if(particleSensor.available()) {
    const long ir = particleSensor.getIR();
    if(ir > 10000) {
      const long red = particleSensor.getRed();
      if(checkForBeat(red)) {
        const long delta = millis() - lastBeat; lastBeat = millis();
        beatsPerMinute = 60000.0 / delta;
        if(beatsPerMinute > 20 && beatsPerMinute < 255) {
          rates[rateSpot++] = (uint8_t)beatsPerMinute; rateSpot %= RATE_SIZE;
          uint16_t sum = 0;
          for(uint8_t i = 0; i < RATE_SIZE; i++) sum += rates[i];
          beatAvg = sum / RATE_SIZE; heart_rate = beatAvg;
          lastPulseUpdate = millis();
          static unsigned long lastSpO2 = 0;
          if(millis() - lastSpO2 > 3000) { spo2_value = constrain(98.0 - (heart_rate - 72) * 0.05, 90.0, 100.0); lastSpO2 = millis(); }
        }
      }
    }
    particleSensor.nextSample();
  }
}
void readECG() {
  static unsigned long last = 0; static int16_t prev = 512;
  if(millis() - last < ECG_READ_INTERVAL || !sensor_status.ecg) return;
  last = millis();
  const int16_t raw = analogRead(ECG_PIN);
  if(raw < 0 || raw > 4095) return;
  const bool leadsOff = (digitalRead(ECG_LO_MINUS) == HIGH) || (digitalRead(ECG_LO_PLUS) == HIGH);
  if(leadsOff) { ecg_value = 512; sendECGData(50); }
  else {
    ecgFiltered = ALPHA * raw + (1 - ALPHA) * prev; prev = ecgFiltered;
    const int16_t ecgAC = ecgFiltered - ecgBaseline;
    ecgBaseline = 0.999 * ecgBaseline + 0.001 * ecgFiltered;
    const uint8_t scaled = map(constrain(ecgAC, -300, 300), -300, 300, 0, 100);
    sendECGData(scaled); ecg_value = raw;
    lastECGUpdate = millis();
  }
}
void simulateBloodPressure() {
  static unsigned long last = 0;
  if(millis() - last < BP_UPDATE_INTERVAL) return;
  last = millis();
  systolic_bp = constrain(systolic_bp + random(-3, 4), 110, 130);
  diastolic_bp = constrain(diastolic_bp + random(-2, 3), 70, 85);
  if(diastolic_bp > systolic_bp - 20) diastolic_bp = systolic_bp - 20 - random(0, 5);
}
void calibrateECGBaseline() {
  long sum = 0;
  for(uint8_t i = 0; i < 100; i++, delay(10)) sum += analogRead(ECG_PIN);
  ecgBaseline = sum / 100;
}

void calibrateTemperature() {
  if(!sensor_status.mlx) { Serial.println("[WARNING] Skipping temp calibration - sensor unavailable"); return; }
  float sum = 0; uint8_t count = 0;
  for(uint8_t i = 0; i < 5; i++) {
    selectMUXChannel(MUX_CHANNEL_TEMP_SENSOR); delay(100);
    const float temp = mlx.readObjectTempC();
    if(!isnan(temp) && temp > 20 && temp < 50) sum += temp, count++;
    delay(500);
  }
  if(count > 0) temperature_value = sum / count;
  else Serial.println("[WARNING] Temperature calibration failed");
}
void readAllIMUs() {
  for(uint8_t i = 0; i < NUM_IMUS; i++) {
    if(!imu_initialized[i]) {
      // Attempt to reinitialize IMU
      if(selectMUXChannel(imu_channels[i])) {
        delay(50);
        if(imu_arr[i].begin(IMU_SENSOR_ADDR, &Wire)) {
          imu_arr[i].setAccelerometerRange(MPU6050_RANGE_8_G);
          imu_arr[i].setGyroRange(MPU6050_RANGE_500_DEG);
          imu_arr[i].setFilterBandwidth(MPU6050_BAND_21_HZ);
          imu_initialized[i] = true;
          Serial.printf("[RECOVERY] %s IMU reinitialized\n", IMU_NAMES[i]);
        }
      }
      continue;
    }
    if(!selectMUXChannel(imu_channels[i])) continue;
    sensors_event_t a, g, t;
    if(imu_arr[i].getEvent(&a, &g, &t)) {
      const float angle = calculateAngle(a, i);
      if(!isnan(angle) && angle >= -90 && angle <= 90) currentAngles[i] = angle;
    }
  }
}
void sendAlert(const char* msg) {
  alert_active = true;
  strncpy(alert_message, msg, sizeof(alert_message) - 1);
  alert_message[sizeof(alert_message) - 1] = '\0';
  Serial.print("[ALERT] "); Serial.println(msg);
  for(uint8_t i = 0; i < 3; i++) { digitalWrite(ESP32_LED_PIN, HIGH); delay(200); digitalWrite(ESP32_LED_PIN, LOW); delay(200); }
  digitalWrite(ESP32_LED_PIN, HIGH);
}
void checkAngleSafety() {
  struct { float angle; float limit; const char* name; } checks[] = {
    {currentAngles[0], MAX_TOP_ANGLE, "TOP"},
    {abs(currentAngles[2]), MAX_SIDE_ANGLE, "SIDE"},
    {currentAngles[1], MAX_BOTTOM_ANGLE, "BOTTOM"}
  };
  for(auto& c : checks) {
    if(c.angle > c.limit) {
      char msg[32];
      snprintf(msg, sizeof(msg), "%s ANGLE LIMIT!", c.name);
      sendAlert(msg);
      stopMotors();
      currentState = STATE_IDLE;
      return;
    }
  }
}

void checkHealthAlerts() {
  char msg[64] = "";
  if(spo2_value < 95.0 && spo2_value > 1.0) strcat(msg, "Low SpO2 ");
  if(temperature_value > 1.0) {
    if(temperature_value < 36.0) strcat(msg, "Low Temp ");
    else if(temperature_value > 37.5) strcat(msg, "High Temp ");
  }
  if(heart_rate > 20) {
    if(heart_rate < 60) strcat(msg, "Low HR ");
    else if(heart_rate > 100) strcat(msg, "High HR ");
  }
  if(digitalRead(ECG_LO_MINUS) == HIGH || digitalRead(ECG_LO_PLUS) == HIGH) strcat(msg, "ECG Leads Off ");
  
  if(msg[0] && (!alert_active || strcmp(alert_message, msg) != 0)) sendAlert(msg);
  else if(!msg[0] && alert_active) { alert_active = false; alert_message[0] = '\0'; }
}
void readAllSensors() {
  readMLX90614();
  readMAX30102();
  readECG();
  simulateBloodPressure();
  readAllIMUs();
  checkAngleSafety();
  checkHealthAlerts();
}

// ----------------------- Movement -----------------------
void executeMovement() {
  // Handle back button
  if(currentAngleIndex == 4) {
    currentState = STATE_SELECTING_BODY;
    selectedBody = -1;
    updateNextionDisplay();
    return;
  }
  
  // Validate body selection
  if(selectedBody < 0 || selectedBody > 3) {
    nextionCommand("t1.txt=\"Error: No body selected\"");
    return;
  }
  
  // Get target angle
  const float angleVal = (currentAngleIndex == 0) ? 0 :
                         (currentAngleIndex == 1) ? 30 :
                         (currentAngleIndex == 2) ? 45 : 60;
  
  // Map body selection to IMU index
  const uint8_t imuIndex = (selectedBody == 0) ? 0 :
                           (selectedBody == 2) ? 1 : 2;
  
  // Check if required IMU is available
  if(!imu_initialized[imuIndex]) {
    nextionCommand("t1.txt=\"Error: IMU unavailable\"");
    currentState = STATE_SELECTING_BODY;
    return;
  }
  
  currentState = STATE_MOVING;
  
  // Read current angle
  selectMUXChannel(imu_channels[imuIndex]);
  sensors_event_t a, g, t;
  
  if(!imu_arr[imuIndex].getEvent(&a, &g, &t)) {
    nextionCommand("t1.txt=\"Error: Cannot read IMU\"");
    currentState = STATE_SELECTING_BODY;
    return;
  }
  
  const float pitch = calculateAngle(a, imuIndex);
  if(isnan(pitch)) {
    nextionCommand("t1.txt=\"Error: Invalid IMU data\"");
    currentState = STATE_SELECTING_BODY;
    return;
  }
  
  // Execute movement based on angle and body part
  auto moveBody = [&](float target, bool up) {
    if(imuIndex == 0) up ? TopUp(target) : TopDown(target);
    else if(imuIndex == 1) up ? BottomUp(target) : BottomDown(target);
    else (selectedBody == 1) ^ up ? SideRight(target) : SideLeft(target);
  };
  
  if(angleVal == 0) {
    if(pitch > 1) moveBody(0, false);
    else if(pitch < -1 && selectedBody == 3) SideRight(0);
  } else {
    bool needsUp = pitch < angleVal - 5;
    moveBody(selectedBody == 3 ? -angleVal : angleVal, needsUp);
  }
  
  // Reset state
  currentState = STATE_SELECTING_BODY;
  selectedBody = -1;
  currentOptionIndex = 0;
  nextionCommand("t1.txt=\"Movement Complete\"");
  updateNextionDisplay();
}

// ----------------------- Input Handlers -----------------------
void handleBlinkCommand(const String& cmd) {
  const unsigned long now = millis();
  if(now - lastBlinkTime < BLINK_COOLDOWN || currentState == STATE_MOVING || alert_active) return;
  lastBlinkTime = now;
  
  if(cmd == "0") {
    if(currentState == STATE_SELECTING_BODY) currentOptionIndex = (currentOptionIndex + 1) % 4;
    else if(currentState == STATE_SELECTING_ANGLE) currentAngleIndex = (currentAngleIndex + 1) % 5;
    updateNextionDisplay();
  } else if(cmd == "1") {
    if(currentState == STATE_SELECTING_BODY) {
      selectedBody = currentOptionIndex;
      currentState = STATE_SELECTING_ANGLE;
      currentAngleIndex = 0;
      updateNextionDisplay();
    } else if(currentState == STATE_SELECTING_ANGLE) {
      if(currentAngleIndex == 4) {
        currentState = STATE_SELECTING_BODY;
        selectedBody = -1;
        updateNextionDisplay();
      } else executeMovement();
    }
  }
}
void handleNextionTouch(const String& cmd) {
  if(currentState == STATE_SELECTING_BODY) {
    const char* bodyBtns[] = {"b0", "b1", "b3", "b2"};
    for(uint8_t i = 0; i < 4; i++) {
      if(cmd == bodyBtns[i]) {
        selectedBody = i;
        currentState = STATE_SELECTING_ANGLE;
        currentAngleIndex = 0;
        updateNextionDisplay();
        return;
      }
    }
  } else if(currentState == STATE_SELECTING_ANGLE) {
    const char* angleBtns[] = {"b1", "b2", "b3", "b4", "b0"};
    for(uint8_t i = 0; i < 5; i++) {
      if(cmd == angleBtns[i]) {
        currentAngleIndex = i;
        executeMovement();
        return;
      }
    }
  }
}

// ----------------------- Network & Init -----------------------
void connectToWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  for(uint8_t i = 0; WiFi.status() != WL_CONNECTED && i < 20; i++, delay(500)) Serial.print(".");
  if((sensor_status.wifi = (WiFi.status() == WL_CONNECTED))) {
    udp.begin(LOCAL_UDP_PORT);
    Serial.println("\n[OK] WiFi connected");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
    lastWiFiCheck = millis();
  } else Serial.println("\n[WARNING] WiFi connection failed");
}

void checkWiFiConnection() {
  if(millis() - lastWiFiCheck < WIFI_RECONNECT_INTERVAL) return;
  lastWiFiCheck = millis();
  if(WiFi.status() != WL_CONNECTED) {
    Serial.println("[WARNING] WiFi disconnected, attempting reconnect...");
    sensor_status.wifi = false;
    WiFi.disconnect();
    delay(100);
    connectToWiFi();
  }
}
void initIMUs() {
  for(uint8_t i = 0; i < NUM_IMUS; i++) {
    selectMUXChannel(imu_channels[i]); delay(100);
    if((imu_initialized[i] = imu_arr[i].begin(IMU_SENSOR_ADDR, &Wire))) {
      imu_arr[i].setAccelerometerRange(MPU6050_RANGE_8_G);
      imu_arr[i].setGyroRange(MPU6050_RANGE_500_DEG);
      imu_arr[i].setFilterBandwidth(MPU6050_BAND_21_HZ);
      Serial.printf("[OK] %s IMU initialized\n", IMU_NAMES[i]);
    } else Serial.printf("[WARNING] %s IMU not found\n", IMU_NAMES[i]);
  }
}
void initNextion() {
  nextionCommand("rest");
  delay(500);
  nextionCommand("page 0");
  currentState = STATE_SELECTING_BODY;
  updateNextionDisplay();
  nextionCommand("t1.txt=\"System Ready - Touch or Blink\"");
}
void scanI2CDevices() {
  Serial.println("\n[INFO] Scanning I2C devices...");
  uint8_t deviceCount = 0;
  
  for(uint8_t ch = 0; ch < 8; ch++) {
    selectMUXChannel(ch);
    delay(50);
    
    for(uint8_t addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if(Wire.endTransmission() == 0) {
        Serial.printf("Ch%d: 0x%02X\n", ch, addr);
        deviceCount++;
      }
    }
  }
  Serial.printf("Found %d device(s)\n\n", deviceCount);
}

// ----------------------- Setup & Loop -----------------------
void setup() {
  Serial.begin(115200); delay(1000);
  Serial.println("\n=== System Initialization ===");
  pinMode(ESP32_LED_PIN, OUTPUT); digitalWrite(ESP32_LED_PIN, LOW);
  Wire.begin(21, 22); Wire.setClock(100000);
  scanI2CDevices();
  SerialNextion.begin(NEXTION_BAUD, SERIAL_8N1, NEXTION_RX_PIN, NEXTION_TX_PIN);
  initSensors(); calibrateECGBaseline(); calibrateTemperature();
  setupPWM(); connectToWiFi(); initIMUs(); initNextion();
  Serial.println("=== Initialization Complete ===\n");
}
void loop() {
  readAllSensors();
  static unsigned long lastDisp = 0;
  if(millis() - lastDisp >= HEALTH_UPDATE_INTERVAL) { updateHealthDisplay(); updateAngleDisplay(); lastDisp = millis(); }
  
  static char nbuf[NEXTION_BUFFER_SIZE]; static uint8_t nbufIdx = 0;
  while(SerialNextion.available()) {
    const char c = SerialNextion.read();
    if(c == 0xFF) { if(nbufIdx > 0) { nbuf[nbufIdx] = '\0'; handleNextionTouch(String(nbuf)); nbufIdx = 0; } }
    else if(c >= 32 && c <= 126) { if(nbufIdx < NEXTION_BUFFER_SIZE - 1) nbuf[nbufIdx++] = c; else nbufIdx = 0; }
  }
  
  if(sensor_status.wifi) {
    const int packetSize = udp.parsePacket();
    if(packetSize > 0) {
      if(packetSize >= sizeof(incomingPacket)) {
        Serial.printf("[WARNING] UDP packet too large: %d bytes, discarding\n", packetSize);
        udp.flush();
      } else {
        const int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
        if(len > 0 && len < sizeof(incomingPacket)) {
          incomingPacket[len] = '\0';
          handleBlinkCommand(String(incomingPacket));
        }
      }
    }
  }
  checkWiFiConnection();
  
  if(Serial.available()) {
    String cmd = Serial.readStringUntil('\n'); cmd.trim();
    if(cmd == "0" || cmd == "1") handleBlinkCommand(cmd);
    else if(cmd == "status") {
      Serial.printf("State:%d Body:%d Opt:%d Ang:%d\n", currentState, selectedBody, currentOptionIndex, currentAngleIndex);
      Serial.printf("Angles: T:%.1f B:%.1f S:%.1f\n", currentAngles[0], currentAngles[1], currentAngles[2]);
      Serial.printf("WiFi:%d MLX:%d Pulse:%d ECG:%d\n", sensor_status.wifi, sensor_status.mlx, sensor_status.pulse, sensor_status.ecg);
    } else if(cmd == "reset") {
      currentState = STATE_SELECTING_BODY; selectedBody = -1; currentOptionIndex = 0; currentAngleIndex = 0;
      updateNextionDisplay(); Serial.println("System reset");
    }
  }
  delay(10);
}