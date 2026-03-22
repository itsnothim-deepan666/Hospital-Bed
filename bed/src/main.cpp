// Cleaned and ESP32-adapted version
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFiUdp.h>
#include <functional>

WiFiUDP udp;

const int pwmMotor1 = 32; // Motor 1 (Top) pin D32
const int dirMotor1 = 33;
const int pwmMotor2 = 18; // Motor 2 (Bottom) pin D18
const int dirMotor2 = 19;
const int pwmMotor3 = 5; // Motor 3 (Side) 
const int dirMotor3 = 4;

// LEDC (PWM) configuration for ESP32
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8; // 8-bit (0-255)
const int PWM_CHANNEL_MOTOR1 = 0;
const int PWM_CHANNEL_MOTOR2 = 1;
const int PWM_CHANNEL_MOTOR3 = 2;


// Nextion Serial (Serial2) pins & baud
const int NEXTION_RX_PIN = 16; // ESP32 pin connected to Nextion TX
const int NEXTION_TX_PIN = 17; // ESP32 pin connected to Nextion RX
const uint32_t NEXTION_BAUD = 9600;

// Send a raw command to Nextion (keeps only this helper as requested)
void nextionCommand(const char *cmd) {
  Serial2.print(cmd);
  Serial2.write((uint8_t)0xFF);
  Serial2.write((uint8_t)0xFF);
  Serial2.write((uint8_t)0xFF);
  Serial2.flush();
}

// WiFi credentials
String ssid = "deepan";
String password = "hehehehe";

// Options and angles
const char* optionNames[4] = {"TOP", "RIGHT", "BOTTOM", "LEFT"};
const float angleValues[4] = {0.0f, 30.0f, 45.0f, 60.0f};
int currentoption = 0;
int currentangle = 0;
// state for Nextion pages and selections
int currentPage = 0; // 0 = page0, 1 = page1
int selPage0 = 0;    // selected index on page0 (0..3)
int selPage1 = 0;    // selected index on page1 (0..4)

// Nextion component names - update these to match your HMI
const char* PAGE0_BTN_NAMES[4] = {"b0", "b1", "b3", "b2"};
const char* PAGE1_BTN_NAMES[5] = {"b1", "b2", "b3", "b4", "b0"};
#define LED_PIN 2
// Hardware & networking
#define TCAADDR 0x70
#define LED_PIN 2
Adafruit_MPU6050 imu;
bool imuTop_ok = false;
bool imuBottom_ok = false;
bool imuSides_ok = false;
const unsigned int localUdpPort = 12345;
char incomingPacket[255];

// Helper index wrapper
int Index(int idx, int size) {
  if (idx < 0) return 0;
  return idx % size;
}

float hAngle(sensors_event_t a) {
  // Simple pitch calculation from accelerometer
  float accel_offsets[3] = {0.05f, -0.01f, 0.03f};
  float ax = a.acceleration.x - accel_offsets[0];
  float ay = a.acceleration.y - accel_offsets[1];
  float az = a.acceleration.z - accel_offsets[2];
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  return pitch;
}

// Select TCA9548A multiplexer channel (0..7)
void selectMuxChannel(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
  // small delay to allow bus to settle
  delay(1);
}

// single IMU instance will be used after selecting the appropriate mux channel

void stopMotors() {
  ledcWrite(pwmMotor1, 0);
  ledcWrite(pwmMotor2, 0);
  ledcWrite(pwmMotor3, 0);
  Serial.println("All Motors Stopped");
}

// keep only nextionCommand (defined earlier)

// Add simple timeout safety to motor movements
bool waitUntilPitchCondition(uint32_t timeoutMs, std::function<bool(float)> condition, uint8_t mux_channel) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    selectMuxChannel(mux_channel);
    sensors_event_t a, g, t;
    imu.getEvent(&a, &g, &t);
    float pitch = hAngle(a);
    if (condition(pitch)) return true;
    delay(20);
  }
  return false;
}

void TopUp(float target, int unused) {
  digitalWrite(dirMotor1, HIGH);
  ledcWrite(PWM_CHANNEL_MOTOR1, 255);
  bool ok = waitUntilPitchCondition(10000, [&](float p){ return p >= target; }, 0);
  if (!ok) Serial.println("TopUp timeout");
  stopMotors();
}

void TopDown(float target, int unused) {
  digitalWrite(dirMotor1, LOW);
  ledcWrite(PWM_CHANNEL_MOTOR1, 255);
  bool ok = waitUntilPitchCondition(10000, [&](float p){ return p <= target; }, 0);
  if (!ok) Serial.println("TopDown timeout");
  stopMotors();
}

void BottomUp(float target, int unused) {
  digitalWrite(dirMotor2, LOW);
  ledcWrite(PWM_CHANNEL_MOTOR2, 255);
  bool ok = waitUntilPitchCondition(10000, [&](float p){ return p >= target; }, 2);
  if (!ok) Serial.println("BottomUp timeout");
  stopMotors();
}

void BottomDown(float target, int unused) {
  digitalWrite(dirMotor2, HIGH);
  ledcWrite(PWM_CHANNEL_MOTOR2, 255);
  bool ok = waitUntilPitchCondition(10000, [&](float p){ return p <= target; }, 2);
  if (!ok) Serial.println("BottomDown timeout");
  stopMotors();
}

void SideLeft(float target) {
  digitalWrite(dirMotor3, LOW);
  ledcWrite(PWM_CHANNEL_MOTOR3, 255);
  bool ok = waitUntilPitchCondition(10000, [&](float p){ return p >= target; }, 3);
  if (!ok) Serial.println("SideLeft timeout");
  stopMotors();
}

void SideRight(float target) {
  digitalWrite(dirMotor3, HIGH);
  ledcWrite(PWM_CHANNEL_MOTOR3, 255);
  bool ok = waitUntilPitchCondition(10000, [&](float p){ return p <= target; }, 3);
  if (!ok) Serial.println("SideRight timeout");
  stopMotors();
}

void handling(String cmd) {
  // cmd is expected to be "0" for cycle, "1" for select/confirm
  if (cmd == "0") {
    if (currentPage == 0) {
      selPage0 = (selPage0 + 1) % 4;
      Serial.print("Page0 selection -> "); Serial.println(selPage0);
      // update colors: highlighted = 65504, others = 8200
      for (int i = 0; i < 4; ++i) {
        String c = String(PAGE0_BTN_NAMES[i]) + ".bco=" + String((i == selPage0) ? 65504 : 8200);
        nextionCommand(c.c_str());
      }
    } else if (currentPage == 1) {
      selPage1 = (selPage1 + 1) % 5;
      Serial.print("Page1 selection -> "); Serial.println(selPage1);
      for (int i = 0; i < 5; ++i) {
        String c = String(PAGE1_BTN_NAMES[i]) + ".bco=" + String((i == selPage1) ? 65504 : 8200);
        nextionCommand(c.c_str());
      }
    }
    return;
  }

  if (cmd == "1") {
    if (currentPage == 0) {
      // confirm selection on page0 -> go to page1
      int opt = selPage0;
      Serial.print("Confirmed page0 opt: "); Serial.println(opt);
      // move to page 1 on the Nextion
      nextionCommand("page 1");
      delay(50);
      currentPage = 1;
      selPage1 = 0;
      // initialize page1 buttons colors
      for (int i = 0; i < 5; ++i) {
        String c = String(PAGE1_BTN_NAMES[i]) + ".bco=" + String((i == selPage1) ? 65504 : 8200);
        nextionCommand(c.c_str());
      }
      return;
    } else if (currentPage == 1) {
      // confirm selection on page1
      Serial.print("Confirmed page1 opt: "); Serial.println(selPage1);
      if (selPage1 == 4) {
        // back selected -> return to page0
        nextionCommand("page 0");
        delay(50);
        currentPage = 0;
        selPage0 = 0;
        for (int i = 0; i < 4; ++i) {
          String c = String(PAGE0_BTN_NAMES[i]) + ".bco=" + String((i == selPage0) ? 65504 : 8200);
          nextionCommand(c.c_str());
        }
        return;
      }

      // else: a valid angle was selected (0,30,45,60)
      float target = angleValues[selPage1];
      int axis = selPage0; // axis selected previously on page0
      sensors_event_t a,g,t;
      // choose the correct IMU based on axis: 0=TOP,2=BOTTOM,1/3=SIDES
      switch(axis) {
        case 0:
          selectMuxChannel(0);
          imu.getEvent(&a,&g,&t);
          break;
        case 2:
          selectMuxChannel(2);
          imu.getEvent(&a,&g,&t);
          break;
        case 1:
        case 3:
        default:
          selectMuxChannel(3);
          imu.getEvent(&a,&g,&t);
          break;
      }
      float pitch = hAngle(a);
      Serial.print("Moving axis "); Serial.print(axis); Serial.print(" to "); Serial.println(target);
      switch(axis) {
        case 0: // TOP
          if (pitch < target) TopUp(target,0); else TopDown(target,0);
          Serial.println("TOP movement implemented virtually");
          break;
        case 2: // BOTTOM
          if (pitch < target) BottomUp(target,0); else BottomDown(target,0);
          Serial.println("BOTTOM movement implemented virtually");
          break;
        case 1: // RIGHT
          if (pitch < target) SideRight(target); else SideLeft(-target);
          Serial.println("RIGHT movement implemented virtually");
          break;
        case 3: // LEFT
          if (pitch < target) SideRight(target); else SideLeft(-target);
          Serial.println("LEFT movement implemented virtually");
          break;
        default:
          Serial.println("Invalid axis");
      }
      // Optionally notify Nextion that movement is done
      String doneCmd = String("t1.txt=\"Done\"");
      nextionCommand(doneCmd.c_str());
      return;
    }
  }
}

void setup() {
  Serial.begin(115200);
  // I2C pins for ESP32
  const int SDA_PIN = 21;
  const int SCL_PIN = 22;
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.begin(21, 22);

  pinMode(pwmMotor1, OUTPUT);
  pinMode(dirMotor1, OUTPUT);
  pinMode(pwmMotor2, OUTPUT);
  pinMode(dirMotor2, OUTPUT);
  pinMode(pwmMotor3, OUTPUT);
  pinMode(dirMotor3, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Setup PWM channels for ESP32
  ledcSetup(PWM_CHANNEL_MOTOR1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(pwmMotor1, PWM_CHANNEL_MOTOR1);
  ledcSetup(PWM_CHANNEL_MOTOR2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(pwmMotor2, PWM_CHANNEL_MOTOR2);
  ledcSetup(PWM_CHANNEL_MOTOR3, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(pwmMotor3, PWM_CHANNEL_MOTOR3);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());
  udp.begin(localUdpPort);
  Serial.print("Listening on UDP port "); Serial.println(localUdpPort);
  
  // Initialize IMUs behind TCA9548A multiplexer
  selectMuxChannel(0);
  if (!imu.begin(0x68)) {
    Serial.println("MPU6050 (TOP) NOT found on mux channel 0!");
    imuTop_ok = false;
  } else {
    Serial.println("MPU6050 (TOP) initialized on mux channel 0");
    imu.setAccelerometerRange(MPU6050_RANGE_8_G);
    imuTop_ok = true;
  }

  selectMuxChannel(2);
  if (!imu.begin(0x68)) {
    Serial.println("MPU6050 (BOTTOM) NOT found on mux channel 2!");
    imuBottom_ok = false;
  } else {
    Serial.println("MPU6050 (BOTTOM) initialized on mux channel 2");
    imu.setAccelerometerRange(MPU6050_RANGE_8_G);
    imuBottom_ok = true;
  }

  selectMuxChannel(3);
  if (!imu.begin(0x68)) {
    Serial.println("MPU6050 (SIDES) NOT found on mux channel 3!");
    imuSides_ok = false;
  } else {
    Serial.println("MPU6050 (SIDES) initialized on mux channel 3");
    imu.setAccelerometerRange(MPU6050_RANGE_8_G);
    imuSides_ok = true;
  }
  // Initialize Serial2 for Nextion display and go to page 0
  Serial2.begin(NEXTION_BAUD, SERIAL_8N1, NEXTION_RX_PIN, NEXTION_TX_PIN);
  delay(100);
  Serial.println("Initializing Nextion to page 0");
  nextionCommand("page 0");
  delay(50);
  Serial.println("Initialized Nextion to page 0");
  currentPage = 0;
  selPage0 = 0;
  // initialize page0 buttons: highlight first, others default
  for (int i = 0; i < 4; ++i) {
    String c = String(PAGE0_BTN_NAMES[i]) + ".bco=" + String((i == selPage0) ? 65504 : 8200);
    nextionCommand(c.c_str());
  }
  // set a small idle text on t1
  //nextionCommand("t1.txt=\"Idle\"");
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      incomingPacket[len] = 0;
      Serial.print("Received from UDP: ");
      Serial.println(incomingPacket);
      if (strcmp(incomingPacket, "1") == 0) {
        digitalWrite(LED_PIN, HIGH);
        delay(200);
        digitalWrite(LED_PIN, LOW);
      } else if (strcmp(incomingPacket, "0") == 0) {
        digitalWrite(LED_PIN, HIGH);
        delay(800);
        digitalWrite(LED_PIN, LOW);
      }
      handling(String(incomingPacket));
    }
  }
  delay(50);
}
