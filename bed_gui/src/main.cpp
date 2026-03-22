//channel for bottom is changed

#include <Adafruit_MPU6050.h>
#include <WiFiUdp.h>
#include <WiFi.h>
#include <Wire.h>
#include <HardwareSerial.h>

String ssid = "deepan";
String password = "hehehehe";

// Built-in LED pin for ESP32 (usually GPIO 2)
#define ESP32_LED_PIN 2


WiFiUDP udp;
Adafruit_MPU6050 mpu;

const unsigned int localUdpPort = 12346;
char incomingPacket[255];
#define MOTOR_PWM_VALUE 255 // Full speed (0-255)
char packet2[255];

// Motor Pins
const int pwmMotor1 = 18; // Motor 1 (Top)
const int dirMotor1 = 19;
const int pwmMotor2 = 32; // Motor 2 (Bottom)
const int dirMotor2 = 33;
const int pwmMotor3 = 4; // Motor 3 (Side)
const int dirMotor3 = 5;

const float alpha = 0.98;                // complementary filter constant

// Example selectIMU function
#define MPU_ADDR 0x68  // I2C address of MPU6050

void stopMotors() {
  analogWrite(pwmMotor1, 0);
  analogWrite(pwmMotor2, 0);
  analogWrite(pwmMotor3, 0);
  Serial.println("All Motors Stopped");
}

float hAngle(sensors_event_t a) {
  float x_offset = 0.05;
  float y_offset = -0.01;
  float z_offset = 0.03;

  float ax = a.acceleration.x - x_offset;
  float ay = a.acceleration.y - y_offset;
  float az = a.acceleration.z - z_offset;

  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  return pitch;
}

void TopUp(float angle,  int i) {
  while (true) {
    Serial.println("Motor1up");
    sensors_event_t a,g,t;
    mpu.getEvent(&a,&g,&t);
    float pitch = hAngle(a);
    Serial.print("Pitch: ");
    Serial.println(pitch);
    if (pitch >= angle){
      i++;
    }
    if (i==3) break;
    digitalWrite(dirMotor1, LOW);
    analogWrite(pwmMotor1, MOTOR_PWM_VALUE);
    Serial.println("Motor1up");
  }
  stopMotors();
}

void TopDown(float angle, int i) {
  while (true) {
    sensors_event_t a,g,t;
    mpu.getEvent(&a,&g,&t);
    float pitch = hAngle(a);
    if (pitch <= angle){
      i++;
    }
    if (i==3) break;
    Serial.print("Pitch: ");
    Serial.println(pitch);
    digitalWrite(dirMotor1, HIGH);
    analogWrite(pwmMotor1, MOTOR_PWM_VALUE);
    Serial.print("Motor1down");
  }
  stopMotors();
}

void BottomUp(float angle, int i) {
  while (true) {
    sensors_event_t a,g,t;
    mpu.getEvent(&a,&g,&t);
    float pitch = hAngle(a);
    Serial.print("Pitch: ");
    Serial.println(pitch);
    if (pitch >= angle){
      i++;
    }
    if (i==10) break;
    digitalWrite(dirMotor2, LOW);
    analogWrite(pwmMotor2, MOTOR_PWM_VALUE);
    Serial.print("Motor2up");
  }
  stopMotors();
}

void BottomDown(float angle, int i) {
  while (true) {
    sensors_event_t a,g,t;
    mpu.getEvent(&a,&g,&t);
    float pitch = hAngle(a);
    Serial.print("Pitch: ");
    Serial.println(pitch);
    if (pitch <= angle){
      i++;
    }
    if (i==3) break;
    digitalWrite(dirMotor2, HIGH);
    analogWrite(pwmMotor2, MOTOR_PWM_VALUE);
    Serial.print("Motor1down");
  }
  stopMotors();
}

void SideLeft(float angle, int i) {
  while (true) {
    sensors_event_t a,g,t;
    mpu.getEvent(&a,&g,&t);
    float pitch = hAngle(a);
    Serial.print("Pitch: ");
    Serial.println(pitch);
    if (pitch >= angle){
      i++;
    }
    if (i==3) break;
    digitalWrite(dirMotor3, LOW);
    digitalWrite(pwmMotor3, HIGH);
    Serial.print("Motor3left");
  }
  stopMotors();
}

void SideRight(float angle, int i) {
  while (true) {
    sensors_event_t a,g,t;
    mpu.getEvent(&a,&g,&t);
    float pitch = hAngle(a);
    Serial.print("Pitch: ");
    Serial.println(pitch);
    if (pitch >= angle){
      i++;
    }
    if (i==3) break;
    digitalWrite(dirMotor3, HIGH);
    digitalWrite(pwmMotor3, HIGH);
    Serial.print("Motor3right");
  }
  stopMotors();
}

void handling(String option,float angle){
  switch(option[0]) {
    case 'T': {
        sensors_event_t a,g,t;
        mpu.getEvent(&a,&g,&t);
        float pitch = hAngle(a);
        if (pitch < angle)
          TopUp(angle, 0);
        else if (pitch > angle)
          TopDown(angle, 0);
        else if (pitch == angle)
          Serial.println("Motor is already at the desired angle"); 
        break;
    }
    case 'B': {
        sensors_event_t a,g,t;
        mpu.getEvent(&a,&g,&t);
        float pitch = hAngle(a);
        if (pitch < angle)
          BottomUp(angle, 0);
        else if (pitch > angle)
          BottomDown(angle, 0);
        else if (pitch == angle)
          Serial.println("Motor is already at the desired angle"); 
        break;
    }
    case 'L': {
        sensors_event_t a,g,t;
        mpu.getEvent(&a,&g,&t);
        float pitch = hAngle(a);
        if (pitch < angle)
          SideLeft(angle, 0);
        else if (pitch > angle)
          SideRight(angle, 0);
        else if (pitch == angle)
          Serial.println("Motor is already at the desired angle");
        break;
    }
    case 'R': {
        sensors_event_t a,g,t;
        mpu.getEvent(&a,&g,&t);
        float pitch = hAngle(a);
        if (pitch < angle)
          SideRight(angle, 0);
        else if (pitch > angle)
          SideLeft(angle, 0);
        else if (pitch == angle)
          Serial.println("Motor is already at the desired angle");
        break;
    }
    default:
      Serial.println("Invalid option. Error in parsing the command.");
      break;
  }
}



void setup() {
  Serial.begin(115200);  // Set I2C clock speed to 400kHz
  // Setup motor pins
  pinMode(pwmMotor1, OUTPUT);
  pinMode(dirMotor1, OUTPUT);
  pinMode(pwmMotor2, OUTPUT);
  pinMode(dirMotor2, OUTPUT);
  pinMode(pwmMotor3, OUTPUT);
  pinMode(dirMotor3, OUTPUT);
  delay(1000);  // Wait for serial monitor to open    // Higher baud rate for ESP32
  Wire.begin();             // SDA and SCL default to GPIO21 and GPIO22
  delay(1000);

  pinMode(2, OUTPUT);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password); // <-- Set your WiFi credentials
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());
  udp.begin(localUdpPort);
  Serial.printf("Listening on UDP port %d\n", localUdpPort);
  mpu.begin(MPU_ADDR);
  

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 initialized");
}


// Set your laptop's hotspot IP and port here
#define LAPTOP_IP_1 192
#define LAPTOP_IP_2 168
#define LAPTOP_IP_3 137
#define LAPTOP_IP_4 1  // Default for Windows hotspot, change if needed
#define LAPTOP_PORT 12346  // Must match your Python script

void loop() {
  // Receive data from UDP and blink LED on each packet
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      incomingPacket[len] = 0;
      Serial.print("Received from UDP: ");
      Serial.println(incomingPacket);
      // Blink built-in LED if packet is '1' or '0'
      if (incomingPacket){
        digitalWrite(ESP32_LED_PIN, HIGH);
        delay(200); // Short blink for '1'
        digitalWrite(ESP32_LED_PIN, LOW);
        char* option = strtok(incomingPacket, " ");
        char* angle = strtok(NULL, " ");
        float ang = atof(angle);
        handling(option, ang);
      }
    }
  }
  delay(50);
}