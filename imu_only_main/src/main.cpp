#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <WiFiUdp.h>
#include <WiFi.h>
#include <Wire.h>
#include <HardwareSerial.h>

Adafruit_MPU6050 mpu;

String ssid = "deepan";
String password = "hehehehe";
String option[4] = {"TOP", "RIGHT", "BOTTOM", "LEFT"};
int currentoption = -1;
String angle[4] = {"0", "30", "45", "60"}; // Angles for each motor
int currentangle = -1;
int cir = 0;

int ind;
// Built-in LED pin for ESP32 (usually GPIO 2)
#define ESP32_LED_PIN 2

WiFiUDP udp;
const unsigned int localUdpPort = 12345;
char incomingPacket[255];
#define MOTOR_PWM_VALUE 255 // Full speed (0-255)
char packet2[255];

const int pwmMotor1 = 18; // Motor 1 (Top)
const int dirMotor1 = 14;
const int pwmMotor2 = 18; // Motor 2 (Bottom)
const int dirMotor2 = 19;
const int pwmMotor3 = 4; // Motor 3 (Side)
const int dirMotor3 = 5;

#define MPU_ADDR 0x68  // I2C address of MPU6050

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

void stopMotors() {
  analogWrite(pwmMotor1, 0);
  analogWrite(pwmMotor2, 0);
  analogWrite(pwmMotor3, 0);
  Serial.println("All Motors Stopped");
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

void handling(String cmd,int go){
  if (cmd=="0"){
    (go == 0) ? currentoption++ : currentangle++ ;
    int curr = (go == 0) ? currentoption : currentangle;
    curr = curr % 4;
    Serial.print("Current Option: ");
    Serial.println((go == 0) ? option[curr] : angle[curr]);
    return;
  }
  else if(cmd=="1"){
    int curr = (go == 0) ? currentoption : currentangle;
    curr = curr % 4;
    Serial.println("Your choice is: " + ((cir == 0 && currentangle==-1) ? option[curr] : angle[curr]+" degrees"));
    if(go == 0) {
      cir = 1;
      switch(currentoption % 4) {
        case 0: { // TOP
          ind = 0;
          break;
            }
        case 1: { // RIGHT
          ind = 2;
          break;
        }
        case 2: { // BOTTOM
          ind = 1;
          break;
        }
        case 3: { // LEFT
          ind = 2;
          break;
        }
        default:
          Serial.println("Invalid choice");
          break;
      }
    }
    else if (cir == 1) {
      Serial.println("You selected angle: " + angle[currentangle % 4]);
      switch(currentangle % 4) {
        case 0: {
          Serial.println("Setting angle to 0 degrees");
          sensors_event_t a,g,t;
          Serial.println("Collecting data...");
          mpu.getEvent(&a,&g,&t);
          Serial.println("Data collected calling hangle");
          float pitch = hAngle(a);
          Serial.println("Data collected");
          if (pitch > 0.0) {
            if (ind == 0) {
              TopDown(0.0,  0);
            } else if (ind == 1) {
              BottomDown(0.0, 0);
            } else if (ind == 2) {
              switch(curr){
                case 1:
                  SideLeft(0.0, 0);
                  break;
                default:
                  Serial.println("Motor is already at 0 degrees");
                  break;
              }
            }
          }
          else if (pitch < 0.0) {
            switch(ind) {
              case 3:
                SideRight(0.0, 0);
                break;
              default:
                Serial.println("Error: Pitch is negative, cannot set to 0 degrees");
                break;
            }
          }
          else {
            Serial.println("Motor is already at 0 degrees");
          }
          break;
        }
        case 1: {
          sensors_event_t a,g,t;
          mpu.getEvent(&a,&g,&t);
          float pitch = hAngle(a);
          if (pitch < 30.0 && pitch > -30.0) {
            if (ind == 0) {
              TopUp(30.0, 0);
            } else if (ind == 1) {
              BottomUp(30.0, 0);
            } else if(ind == 2){
              switch(curr){
                case 1:
                  SideRight(30.0, 0);
                  break;
                case 3:
                  SideLeft(-30.0, 0);
                  break;
              }
              break;
            }
          } else if(pitch > 30.0 && pitch < -30.0) {
              if (ind == 0) {
                TopDown(30.0, 0);
              } else if (ind == 1) {
                BottomDown(30.0, 0);
              } else if(ind == 2){
                switch(curr){
                  case 1:
                    SideLeft(30.0, 0);
                    break;
                  case 3:
                    SideRight(-30.0, 0);
                    break;
                }
              }
              break;
          }
          else {
            Serial.println("Motor is already at 30 degrees");
          }
          break;
        }
        case 2: {
          sensors_event_t a,g,t;
          mpu.getEvent(&a,&g,&t);
          float pitch = hAngle(a);
          if (pitch < 45.0 && pitch > -45.0) {
            if (ind == 0) {
              TopUp(45.0, 0);
            } else if (ind == 1) {
              BottomUp(45.0, 0);
            } else if(ind == 2){
              switch(curr){
                case 1:
                  SideRight(45.0, 0);
                  break;
                case 3:
                  SideLeft(-45.0, 0);
                  break;
              }
              break;
            }
          } else if(pitch > 45.0 && pitch < -45.0) {
              if (ind == 0) {
                TopDown(45.0, 0);
              } else if (ind == 1) {
                BottomDown(45.0, 0);
              } else if(ind == 2){
                switch(curr){
                  case 1:
                    SideLeft(45.0, 0);
                    break;
                  case 3:
                    SideRight(-45.0, 0);
                    break;
                }
                break;
              }
          }
          else {
            Serial.println("Motor is already at 45 degrees");
          }
          break;
        }
        case 3: {
          sensors_event_t a,g,t;
          mpu.getEvent(&a,&g,&t);
          float pitch = hAngle(a);
          if (pitch < 60.0 && pitch > -60.0) {
            if (ind == 0) {
              TopUp(60.0, 0);
            } else if (ind == 1) {
              BottomUp(60.0, 0);
            } else if(ind == 2){
              switch(curr){
                case 1:
                  SideRight(60.0, 0);
                  break;
                case 3:
                  SideLeft(-60.0, 0);
                  break;
              }
              break;
            }
          } else if(pitch > 60.0 && pitch < -60.0) {
              if (ind == 0) {
                TopDown(60.0,  0);
              } else if (ind == 1) {
                BottomDown(60.0, 0);
              } else if(ind == 2){
                switch(curr){                  case 1:
                    SideLeft(60.0, 0);
                    break;
                  case 3:
                    SideRight(-60.0, 0);
                    break;
                }
                break;
              }
          }
          else {
            Serial.println("Motor is already at 60 degrees");
          }
          break;
        }
        default:
          Serial.println("Invalid angle choice");
          break;
        }
      }
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
      if (strcmp(incomingPacket, "1") == 0) {
        digitalWrite(ESP32_LED_PIN, HIGH);
        delay(200); // Short blink for '1'
        digitalWrite(ESP32_LED_PIN, LOW);
      } else if (strcmp(incomingPacket, "0") == 0) {
        digitalWrite(ESP32_LED_PIN, HIGH);
        delay(800); // Long blink for '0'
        digitalWrite(ESP32_LED_PIN, LOW);
      }
      handling(String(incomingPacket), cir);
    }
  }
  delay(50);
}