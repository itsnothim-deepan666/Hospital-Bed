#include <Wire.h>
#include <Arduino.h>
#include <TCA9548.h>        // 🔄 Changed library include
#include "MPU6050.h"        // keep this as-is
#define PI 3.14159265358979323846

#define DT_MS 10
const uint8_t activeChannels[] = {0, 4, 5};
const uint8_t NUM_IMUS = sizeof(activeChannels) / sizeof(activeChannels[0]);

MPU6050 imu;
TCA9548 mux(0x70);           // 🔄 Changed constructor

float accel_offsets[8][3] = {0};  // offsets per channel
float pitch_estimates[8] = {0};  // pitch per channel
const float alpha = 0.98;         // complementary filter constant

void calibrateIMU(uint8_t channel) {
  mux.enableChannel(channel);      // 🔄
  delay(10);
  imu.initialize();
  float axSum = 0, aySum = 0, azSum = 0;
  const int samples = 100;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az;
    imu.getAcceleration(&ax, &ay, &az);
    axSum += ax / 16384.0;
    aySum += ay / 16384.0;
    azSum += az / 16384.0;
    delay(2);
  }

  accel_offsets[channel][0] = axSum / samples;
  accel_offsets[channel][1] = aySum / samples;
  accel_offsets[channel][2] = (azSum / samples) - 1.0;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  for (uint8_t i = 0; i < NUM_IMUS; i++) {
    uint8_t ch = activeChannels[i];
    mux.enableChannel(ch);        // 🔄 Changed function name
    delay(50);
    imu.initialize();
    if (imu.testConnection()) {
      Serial.printf("IMU on channel %d connected. Calibrating...\n", ch);
      calibrateIMU(ch);
    } else {
      Serial.printf("IMU on channel %d NOT detected!\n", ch);
    }
  }
}

float getPitchComplementary(uint8_t channel, float dt, uint8_t idx) {
  mux.enableChannel(channel);      // 🔄
  delay(5);
  imu.initialize();
  if (!imu.testConnection()) return NAN;

  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float aX = ax / 16384.0f - accel_offsets[channel][0];
  float aY = ay / 16384.0f - accel_offsets[channel][1];
  float aZ = az / 16384.0f - accel_offsets[channel][2];
  float gY = gy / 131.0f;

  float pitch_acc = atan2(-aX, sqrt(aY * aY + aZ * aZ)) * 180.0f / PI;

  pitch_estimates[idx] = alpha * (pitch_estimates[idx] + gY * dt)
                       + (1.0f - alpha) * pitch_acc;

  return pitch_estimates[idx];
}


void loop() {
  float dt = DT_MS / 1000.0f;

  for (uint8_t i = 0; i < NUM_IMUS; i++) {
    uint8_t ch = activeChannels[i];
    float pitch = getPitchComplementary(ch, dt, i);
    if (!isnan(pitch)) {
      Serial.printf("IMU[%d] Pitch: %.2f°\n", ch, pitch);
    } else {
      Serial.printf("IMU[%d] Pitch: N/A\n", ch);
    }
  }

  Serial.println("---");
  delay(1000);
}

