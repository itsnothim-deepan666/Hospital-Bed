#include <WiFiUdp.h>
#include <WiFi.h>
#include <Wire.h>
#include <HardwareSerial.h>

const int pwmMotor1 = 32; // Motor 1 (Top)
const int dirMotor1 = 33;
const int pwmMotor2 = 18; // Motor 2 (Bottom)
const int dirMotor2 = 19;
const int pwmMotor3 = 5; // Motor 3 (Side)
const int dirMotor3 = 4;

// put function declarations here:
int myFunction(int, int);

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);        // Start I2C
  // Setup motor pins
  pinMode(pwmMotor1, OUTPUT);
  pinMode(dirMotor1, OUTPUT);
  pinMode(pwmMotor2, OUTPUT);
  pinMode(dirMotor2, OUTPUT);
  pinMode(pwmMotor3, OUTPUT);
  pinMode(dirMotor3, OUTPUT);
  // Setup onboard LED pin (GPIO 2)
  pinMode(2, OUTPUT);
  
  // Initialize WiFi and UDP
  WiFi.begin("deepan", "hehehehe");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  
  Serial.println("Connected to WiFi");
}

void loop() {
  Serial.println("Running motor 1 high");
  digitalWrite(dirMotor3, LOW);
  analogWrite(pwmMotor3, 255);
  delay(10000); // Run motor 1 for 1 second
  Serial.println("Running motor 1 low");
  digitalWrite(dirMotor3, HIGH);
  analogWrite(pwmMotor3, 255);
  delay(10000);
  Serial.println("All motors stopped");
}