#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

#define TCA_ADDR 0x70

Adafruit_MPU6050 mpu[3];  // 3 working sensors
const uint8_t active_channels[3] = {0, 1, 2};  // Skip faulty Channel 1

// Optional: Calibration offsets for each IMU
const float accel_offsets[3][3] = {
  {0.0, 0.0, 0.0},  // Channel 0
  {0.0, 0.0, 0.0},  // Channel 2
  {0.0, 0.0, 0.0}   // Channel 3
};

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(5);  // Let TCA settle
}

float getPitchAngle(sensors_event_t a, uint8_t index) {
  float ax = a.acceleration.x - accel_offsets[index][0];
  float ay = a.acceleration.y - accel_offsets[index][1];
  float az = a.acceleration.z - accel_offsets[index][2];
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  return pitch;
}

void readPitchFromIMU(uint8_t index) {
  uint8_t channel = active_channels[index];
  tcaSelect(channel);

  sensors_event_t a, g, temp;
  if (!mpu[index].getEvent(&a, &g, &temp)) {
    Serial.print("Channel ");
    Serial.print(channel);
    Serial.println(" | Failed to read IMU!");
    return;
  }

  float pitch = getPitchAngle(a, index);

  Serial.print("Channel ");
  Serial.print(channel);
  Serial.print(" | Pitch Angle: ");
  Serial.print(pitch, 2);
  Serial.println(" deg");
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Starting...");

  Wire.begin(21, 22);  // ESP8266 I2C pins

  for (uint8_t i = 0; i < 3; i++) {
    uint8_t channel = active_channels[i];
    tcaSelect(channel);
    delay(50);

    if (!mpu[i].begin(0x68)) {
      Serial.print("MPU6050 NOT found on channel ");
      Serial.println(channel);
    } else {
      Serial.print("MPU6050 initialized on channel ");
      Serial.println(channel);
      mpu[i].setAccelerometerRange(MPU6050_RANGE_8_G);
    }
    delay(100);
  }
}

void loop() {
  Serial.println("------");
  for (uint8_t i = 0; i < 3; i++) {
    readPitchFromIMU(i);
    delay(200);
  }
  delay(1000);
}

