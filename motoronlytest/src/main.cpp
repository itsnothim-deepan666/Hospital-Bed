#include "Arduino.h"


const int pwmPin = 25;     // Connected to EN or PWM pin on motor driver
const int dirPin = 32;    // Direction control

const int pwmChannel = 0;
const int freq = 1000;    // 1kHz PWM frequency
const int resolution = 8; // 8-bit resolution (0–255)

int duty = 255
; // Initial duty cycle

void setup() {
  Serial.begin(115200);
  pinMode(dirPin, OUTPUT);

  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(pwmPin, pwmChannel);

  Serial.println("Commands:");
  Serial.println("  F = forward");
  Serial.println("  R = reverse");
  Serial.println("  S = stop");
  Serial.println("  D<number> = set duty cycle (0–255)");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "F") {
      digitalWrite(dirPin, HIGH);
      ledcWrite(pwmChannel, duty);
      Serial.println("Direction: Forward");
    } else if (input == "R") {
      digitalWrite(dirPin, LOW);
      ledcWrite(pwmChannel, duty);
      Serial.println("Direction: Reverse");
    } else if (input == "S") {
      ledcWrite(pwmChannel, 0);
      Serial.println("Motor stopped");
    } else if (input.startsWith("D")) {
      int newDuty = input.substring(1).toInt();
      duty = constrain(newDuty, 0, 255);
      Serial.print("New Duty Cycle: ");
      Serial.println(duty);
    } else {
      Serial.println("Unknown command. Try F, R, S, or D<number>");
    }
  }
}

