
#include <Adafruit_MPU6050.h>
#include <Wire.h>

// Motor pins (change as needed)
#define MOTOR1_PWM 32
#define MOTOR1_DIR 33
#define MOTOR2_PWM 18
#define MOTOR2_DIR 19
#define MOTOR3_PWM 4
#define MOTOR3_DIR 5

#define NUM_MOTORS 3
#define NUM_IMUS 3

Adafruit_MPU6050 imus[NUM_IMUS];
uint8_t imu_channels[NUM_IMUS] = {0, 2, 3}; // multiplexer channels for IMUs
float target_angles[NUM_MOTORS] = {30.0, 45.0, 60.0}; // target angles for each motor

void selectIMU(uint8_t channel) {
  Wire.beginTransmission(0x70); // PCA9548A address
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Motor pin setup
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_DIR, OUTPUT);

  // IMU initialization
  for (uint8_t i = 0; i < NUM_IMUS; i++) {
    selectIMU(imu_channels[i]);
    delay(50);
    if (!imus[i].begin()) {
      Serial.printf("IMU %d not found on channel %d\n", i, imu_channels[i]);
    } else {
      Serial.printf("IMU %d initialized on channel %d\n", i, imu_channels[i]);
      imus[i].setAccelerometerRange(MPU6050_RANGE_8_G);
    }
  }
}

void moveMotorToAngle(uint8_t motor_idx, float target_angle) {
  uint8_t imu_idx = motor_idx; // 1-to-1 mapping
  selectIMU(imu_channels[imu_idx]);
  sensors_event_t a, g, t;
  imus[imu_idx].getEvent(&a, &g, &t);
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

  Serial.printf("Motor %d IMU pitch: %.2f Target: %.2f\n", motor_idx + 1, pitch, target_angle);

  // Simple control: move until pitch >= target_angle
  while (pitch < target_angle) {
    // Set direction and PWM for upward movement
    switch (motor_idx) {
      case 0:
        digitalWrite(MOTOR1_DIR, HIGH);
        analogWrite(MOTOR1_PWM, 255);
        break;
      case 1:
        digitalWrite(MOTOR2_DIR, HIGH);
        analogWrite(MOTOR2_PWM, 255);
        break;
      case 2:
        digitalWrite(MOTOR3_DIR, HIGH);
        analogWrite(MOTOR3_PWM, 255);
        break;
    }
    delay(50);
    selectIMU(imu_channels[imu_idx]);
    imus[imu_idx].getEvent(&a, &g, &t);
    pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
    Serial.printf("Motor %d IMU pitch: %.2f\n", motor_idx + 1, pitch);
  }
  // Stop motor
  switch (motor_idx) {
    case 0:
      analogWrite(MOTOR1_PWM, 0);
      break;
    case 1:
      analogWrite(MOTOR2_PWM, 0);
      break;
    case 2:
      analogWrite(MOTOR3_PWM, 0);
      break;
  }
  Serial.printf("Motor %d reached target angle %.2f\n", motor_idx + 1, target_angle);
}

void loop() {
  // Test each motor one by one
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    moveMotorToAngle(i, target_angles[i]);
    delay(1000); // Wait before next motor
  }
  Serial.println("All motors tested. Restarting...");
  delay(5000);
}