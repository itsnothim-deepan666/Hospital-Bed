// MediTilt Pro - Standalone Hardware Remote Controller (ESP32)
// Priority: HARDWARE REMOTE (kill switch > homing > manual jog)
// Pins: Wire 4/5 (MPU6050), encoder ADC 36, torso motor 12/13, 
//       leg motor 25/26, remote shift register 27/14/19
//
// FIX (this version): torso homing was landing off true zero because of
// mechanical backlash in the torso leadscrew/gearbox. Two changes fix it:
//   1. Backlash compensation - whenever the torso motor needs to reverse
//      direction, extra "slack take-up" rotations are added to the target
//      so the real bed position (not just the encoder count) reaches zero.
//   2. Creep speed near target - the motor slows down inside
//      HOMING_SLOWDOWN_ZONE rotations of the target instead of running at
//      full PWM until the exact count, which used to cause overshoot.

#include <Wire.h>
#include <Arduino.h>
#include "calibration_table.h"

// Set to true to test the priority logic without a motor power supply connected.
#define SIMULATE_MOTOR false

// ===================================================================
// HARDWARE REMOTE (74HC165 shift register) - pins & state
// ===================================================================
// Bit layout (UPDATED FOR NEW WIRING):
//   0 Homing     1 RightUp    2 LegUp    3 HeadUp
//   4 HeadDown   5 LegDown    6 LeftUp   7 Kill
const int REMOTE_LATCH_PIN = 27;
const int REMOTE_CLOCK_PIN = 14;
const int REMOTE_DATA_PIN  = 19; // Changed from 34! (GPIO 34 lacks internal pull-downs)

// Second (leg/foot) motor driver - open-loop only, no encoder feedback.
#define LEG_MOTOR_PWM_PIN 25
#define LEG_MOTOR_DIR_PIN 26
const int LEG_MOTOR_SPEED = 255;

#ifndef LED_BUILTIN
#define LED_BUILTIN 2 
#endif

// Emergency stop - highest priority. While true, no motor is allowed to move.
volatile bool emergencyStopActive = false;

// Set while the physical remote is actively jogging the torso motor open-loop.
volatile bool headManualJogActive = false;

// Homing state variables
bool isHomingActive = false;
int homingStage = 0; // 0 = Idle, 1 = Tilt (Sim), 2 = Leg (Sim), 3 = Head (PHYSICAL)
unsigned long homingTimer = 0;

// Previous state tracking to avoid spamming the Serial Monitor.
int lastHeadState = 0;
int lastLegState = 0;
int lastTiltState = 0;
bool lastKillState = false;

// Remote LED blink timer
unsigned long lastRemoteBlinkTime = 0;
bool remoteLedState = HIGH; 

// Forward declarations
long rollToRotations(float targetRoll);
void updateMotorControl();
void setupMotor();
void readEncoder();
void setupMPU();
void readMPU();
void calibrateMPU();
byte read74HC165();
void updateRemoteLED(bool isCommandActive);
void setupLegMotor();
void legMotorStop();
void legMotorDriveUp();
void legMotorDriveDown();
void commandLegManual(int dir);
void commandHeadManual(int dir);
void commandTiltSim(int dir);
void haltAllMotorsHard();
void pollRemote();
void updateHomingSimulation();
long applyBacklashCompensation(long rawTarget);

// MPU6050 (IMU) variables
#define MPU6050_ADDR 0x68
int16_t AcX, AcY, AcZ;
int16_t GyX, GyY, GyZ;

const float ACC_SCALE  = 16384.0;
const float GYRO_SCALE = 131.0;

float accOffsetX = 0, accOffsetY = 0, accOffsetZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

float roll = 0;
float pitch = 0;
unsigned long lastMicros = 0;

class ComplementaryFilter {
  public:
    float alpha = 0.98;
    float angle = 0.0;
    float update(float gyroRate, float accAngle, float dt) {
      float gyroAngle = angle + gyroRate * dt;
      angle = alpha * gyroAngle + (1.0 - alpha) * accAngle;
      return angle;
    }
};

ComplementaryFilter cFilterRoll;
ComplementaryFilter cFilterPitch;

// AS5600 encoder variables
#define ENCODER_PIN 36
const unsigned long ENCODER_SAMPLE_INTERVAL_MS = 4;
unsigned long lastEncoderMillis = 0;

volatile long rotations = 0;
float lastAngle = 0;

void readEncoder() {
  int raw = analogRead(ENCODER_PIN);
  float angle = (raw / 4095.0) * 360.0;
  float diff = angle - lastAngle;

  if (diff > 200.0) {
    rotations++;
  }
  if (diff < -200.0) {
    rotations--;
    if (rotations < 0) rotations = 0;
  }
  lastAngle = angle;
}

// Main Torso Cytron motor driver pins
#define MOTOR_PWM_PIN 12
#define MOTOR_DIR_PIN 13

const int MOTOR_SPEED = 255;
long targetRotations = 0;
bool motorMoving = false;

// ---------------------------------------------------------------------
// BACKLASH / CREEP TUNING
// ---------------------------------------------------------------------
// BACKLASH_COMPENSATION: how many extra encoder "rotations" of motor-shaft
// travel are needed to take up the mechanical slack whenever the torso
// motor reverses direction. Start with a small value (e.g. 4-8) and
// increase on the bench: command a small up/down/up jog and see how many
// rotations pass before the bed visibly starts moving again after a
// reversal - that count is your BACKLASH_COMPENSATION.
const long BACKLASH_COMPENSATION = 6;

// Inside this many rotations of the target, the torso motor drops to
// MOTOR_SPEED_SLOW so it doesn't blow past the exact count between
// encoder polls (which was the main cause of homing overshoot).
const long HOMING_SLOWDOWN_ZONE = 15;
const int  MOTOR_SPEED_SLOW     = 90;

// Tracks the last direction the torso motor actually drove in:
// +1 = increasing, -1 = decreasing, 0 = unknown / not yet moved.
// This persists across stops (including manual jog stops) so that the
// NEXT move - whether manual or homing - knows whether it needs to pay
// the backlash toll.
int8_t torsoLastDirection = 0;

void motorStop() { analogWrite(MOTOR_PWM_PIN, 0); }
void motorDriveIncreasing(int spd = MOTOR_SPEED) { digitalWrite(MOTOR_DIR_PIN, LOW); analogWrite(MOTOR_PWM_PIN, spd); }
void motorDriveDecreasing(int spd = MOTOR_SPEED) { digitalWrite(MOTOR_DIR_PIN, HIGH); analogWrite(MOTOR_PWM_PIN, spd); }

void setupMotor() {
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  motorStop();
}

// Adds a backlash "toll" to a raw target if reaching it requires the torso
// motor to reverse direction from whatever it last actually drove.
// This is what makes homing (and any programmatic move) land on the true
// physical position instead of stopping short/long by the slack amount.
long applyBacklashCompensation(long rawTarget) {
  int8_t neededDir = (rawTarget > rotations) ? 1 : (rawTarget < rotations ? -1 : 0);

  if (neededDir != 0 && torsoLastDirection != 0 && neededDir != torsoLastDirection) {
    rawTarget += (long)neededDir * BACKLASH_COMPENSATION;
    Serial.print("[BACKLASH COMP] Direction reversal -> adding ");
    Serial.print(BACKLASH_COMPENSATION);
    Serial.println(" rotations of slack take-up to target.");
  }
  return rawTarget;
}

// ===================================================================
// LEG/FOOT MOTOR
// ===================================================================
void legMotorStop() { analogWrite(LEG_MOTOR_PWM_PIN, 0); }
void legMotorDriveUp()   { digitalWrite(LEG_MOTOR_DIR_PIN, LOW); analogWrite(LEG_MOTOR_PWM_PIN, LEG_MOTOR_SPEED); }
void legMotorDriveDown() { digitalWrite(LEG_MOTOR_DIR_PIN, HIGH);  analogWrite(LEG_MOTOR_PWM_PIN, LEG_MOTOR_SPEED); }

void setupLegMotor() {
  pinMode(LEG_MOTOR_PWM_PIN, OUTPUT);
  pinMode(LEG_MOTOR_DIR_PIN, OUTPUT);
  legMotorStop();
}

void commandLegManual(int dir) {
  if (dir == lastLegState) return;
  lastLegState = dir;

  if (emergencyStopActive) { legMotorStop(); return; }

  if (dir == 1) { Serial.println("CMD: LEG RAISING (real motor, open-loop)..."); legMotorDriveUp(); }
  else if (dir == -1) { Serial.println("CMD: LEG LOWERING (real motor, open-loop)..."); legMotorDriveDown(); }
  else { Serial.println("CMD: LEG STOPPED."); legMotorStop(); }
}

void commandHeadManual(int dir) {
  if (dir == lastHeadState) return;
  lastHeadState = dir;

  if (emergencyStopActive) { motorStop(); headManualJogActive = false; return; }

  if (dir == 1) {
    Serial.println("CMD: HEAD/TORSO LIFTING (real motor, manual jog)...");
    headManualJogActive = true;
    motorMoving = false; 
    motorDriveIncreasing();
    torsoLastDirection = 1; // remember for next backlash calc
  } else if (dir == -1) {
    Serial.println("CMD: HEAD/TORSO LOWERING (real motor, manual jog)...");
    headManualJogActive = true;
    motorMoving = false;
    motorDriveDecreasing();
    torsoLastDirection = -1; // remember for next backlash calc
  } else {
    Serial.println("CMD: HEAD/TORSO STOPPED.");
    motorStop();
    headManualJogActive = false;
    targetRotations = rotations; // Snap target to current spot to prevent drifting back
  }
}

void commandTiltSim(int dir) {
  if (dir == lastTiltState) return;
  lastTiltState = dir;

  if (dir == 1) Serial.println("CMD: SIDE TILT -> RAISING LEFT [SIMULATED - no tilt motor installed]");
  else if (dir == 2) Serial.println("CMD: SIDE TILT -> RAISING RIGHT (pushing Left to 0) [SIMULATED]");
  else Serial.println("CMD: SIDE TILT STOPPED. [SIMULATED]");
}

void haltAllMotorsHard() {
  motorStop();
  motorMoving = false;
  headManualJogActive = false;
  targetRotations = rotations;
  legMotorStop();
  lastHeadState = 0;
  lastLegState = 0;
  commandTiltSim(0);
  // NOTE: torsoLastDirection is intentionally NOT reset here - a hard
  // stop/kill doesn't erase the mechanical slack state, so the next move
  // still needs to know which way the motor last actually drove.
}

// Handles sequential homing routines across components
void updateHomingSimulation() {
  if (!isHomingActive) return;

  if (emergencyStopActive) {
    Serial.println("[ HOMING ABORTED ] Kill switch engaged.");
    isHomingActive = false;
    homingStage = 0;
    return;
  }

  unsigned long currentMillis = millis();

  // STAGE 1: Lateral Tilt Zeroing (Simulated)
  if (homingStage == 1) {
    if (currentMillis - homingTimer > 2000) {
      Serial.println("HOMING Step 1 Complete [SIMULATED]. Now zeroing leg axis...");
      homingStage = 2;
      homingTimer = currentMillis;
    }
  } 
  // STAGE 2: Leg Section Zeroing (Simulated/Open-Loop)
  else if (homingStage == 2) {
    if (currentMillis - homingTimer > 2000) {
      Serial.println("HOMING Step 2 Complete [SIMULATED]. Activating physical closed-loop homing for Head/Torso (Target: 0 deg)...");
      
      // Initialize closed-loop control sequence to physical 0 degrees.
      // FIX: run the raw calibration target through backlash compensation
      // so the motor pays the mechanical slack "toll" if it needs to
      // reverse direction from whatever it last did (usually true, since
      // homing is normally requested after manual jogging).
      long rawTarget = rollToRotations(0.0);
      targetRotations = applyBacklashCompensation(rawTarget);
      motorMoving = true;
      headManualJogActive = false; // Override manual jog controls
      
      homingStage = 3;
      homingTimer = currentMillis;
    }
  } 
  // STAGE 3: Physical Head/Torso Zeroing via MPU6050 & Calibration Lookup
  else if (homingStage == 3) {
    // Monitor the automated motor control sequence until tracking targets match position thresholds
    if (!motorMoving) {
      Serial.println("[ HOMING COMPLETE ] Head/Torso zeroed via closed-loop encoder tracking. System safe.");
      isHomingActive = false;
      homingStage = 0;
    }
  }
}

void updateRemoteLED(bool isCommandActive) {
  if (isCommandActive) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastRemoteBlinkTime >= 100) {
      lastRemoteBlinkTime = currentMillis;
      remoteLedState = !remoteLedState;
      digitalWrite(LED_BUILTIN, remoteLedState);
    }
  } else if (remoteLedState != HIGH) {
    digitalWrite(LED_BUILTIN, HIGH);
    remoteLedState = HIGH;
  }
}

byte read74HC165() {
  byte value = 0;
  for (int i = 0; i < 8; ++i) {
    int bitValue = digitalRead(REMOTE_DATA_PIN);
    value |= (bitValue << (7 - i));
    digitalWrite(REMOTE_CLOCK_PIN, HIGH);
    
    // DELAY INCREASED TO 5us for 1-Meter cable stability
    delayMicroseconds(5); 
    
    digitalWrite(REMOTE_CLOCK_PIN, LOW);
  }
  return value;
}

void pollRemote() {
  static unsigned long lastRemotePoll = 0;
  unsigned long now = millis();
  if (now - lastRemotePoll < 15) return; 
  lastRemotePoll = now;

  digitalWrite(REMOTE_LATCH_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(REMOTE_LATCH_PIN, HIGH);
  byte state = read74HC165();

  // --- UPDATED BIT MAPPING BASED ON NEW WIRING ---
  bool btnHoming    = bitRead(state, 0); // Pin 11 (D0)
  bool btnRightUp   = bitRead(state, 1); // Pin 12 (D1)
  bool btnLegUp     = bitRead(state, 2); // Pin 13 (D2)
  bool btnHeadUp    = bitRead(state, 3); // Pin 14 (D3)
  
  bool btnHeadDown  = bitRead(state, 4); // Pin 3  (D4)
  bool btnLegDown   = bitRead(state, 5); // Pin 4  (D5)
  bool btnLeftUp    = bitRead(state, 6); // Pin 5  (D6)
  bool btnKill      = bitRead(state, 7); // Pin 6  (D7)

  bool isMoving = (btnHeadUp || btnHeadDown || btnLegUp || btnLegDown ||
                    btnLeftUp || btnRightUp || isHomingActive);

  // --- PRIORITY 1: GLOBAL KILL SWITCH ---
  if (btnKill) {
    if (!lastKillState) {
      Serial.println("\n[ !!! EMERGENCY KILL SWITCH ACTIVATED !!! ]");
      lastKillState = true;
    }
    emergencyStopActive = true;
    isHomingActive = false;
    homingStage = 0;
    haltAllMotorsHard();
    digitalWrite(LED_BUILTIN, LOW); // Solid ON = Emergency hold state
    return;
  } else if (lastKillState) {
    Serial.println("[ KILL SWITCH RELEASED - System Ready ]");
    lastKillState = false;
    emergencyStopActive = false;
    digitalWrite(LED_BUILTIN, HIGH);
  }

  updateRemoteLED(isMoving);

  // --- PRIORITY 2: AUTOMATED SEQUENCE RUNNER ---
  if (btnHoming && !isHomingActive) {
    Serial.println("\n[ HOMING SEQUENCE INITIATED ] Executing multi-axis systemic reset...");
    isHomingActive = true;
    homingStage = 1;
    homingTimer = millis();
    haltAllMotorsHard(); // Clear existing target configurations
  }
  
  if (isHomingActive) {
    updateHomingSimulation();
    return; // Hard lock manual inputs to preserve alignment profile parameters
  }

  // --- PRIORITY 3: MANUAL AXIS INTERACTION LOOP ---
  if (btnHeadUp) commandHeadManual(1);
  else if (btnHeadDown) commandHeadManual(-1);
  else commandHeadManual(0);

  if (btnLegUp) commandLegManual(1);
  else if (btnLegDown) commandLegManual(-1);
  else commandLegManual(0);

  if (btnLeftUp) commandTiltSim(1);
  else if (btnRightUp) commandTiltSim(2);
  else commandTiltSim(0);
}

long rollToRotations(float targetRoll) {
  float rollFirst = pgm_read_float(&calRoll[0]);
  float rollLast  = pgm_read_float(&calRoll[CAL_TABLE_SIZE - 1]);
  bool increasing = (rollLast >= rollFirst);

  if (increasing) {
    if (targetRoll <= rollFirst) return pgm_read_dword(&calRotations[0]);
    if (targetRoll >= rollLast)  return pgm_read_dword(&calRotations[CAL_TABLE_SIZE - 1]);
  } else {
    if (targetRoll >= rollFirst) return pgm_read_dword(&calRotations[0]);
    if (targetRoll <= rollLast)  return pgm_read_dword(&calRotations[CAL_TABLE_SIZE - 1]);
  }

  for (int i = 0; i < CAL_TABLE_SIZE - 1; i++) {
    float roll0 = pgm_read_float(&calRoll[i]);
    float roll1 = pgm_read_float(&calRoll[i + 1]);
    bool inRange = increasing ? (targetRoll >= roll0 && targetRoll <= roll1) : (targetRoll <= roll0 && targetRoll >= roll1);

    if (inRange) {
      int32_t rot0 = pgm_read_dword(&calRotations[i]);
      int32_t rot1 = pgm_read_dword(&calRotations[i + 1]);
      float t = (roll1 == roll0) ? 0.0f : (targetRoll - roll0) / (roll1 - roll0);
      return rot0 + (long)lround(t * (rot1 - rot0));
    }
  }
  return pgm_read_dword(&calRotations[CAL_TABLE_SIZE - 1]);
}

void updateMotorControl() {
#if SIMULATE_MOTOR
  return; 
#endif
  if (emergencyStopActive) { motorStop(); motorMoving = false; return; }
  if (headManualJogActive) return; 
  if (!motorMoving) return;

  long error = targetRotations - rotations;

  if (error == 0) {
    motorStop();
    motorMoving = false;
    Serial.print("\n[SUCCESS] Position reached. Rotations: "); Serial.print(rotations);
    Serial.print(" | Active Roll: "); Serial.println(roll, 2);
    return;
  }

  // FIX: creep at reduced speed near the target instead of running full
  // PWM all the way in, which used to overshoot past the exact count
  // between 4ms encoder polls.
  int speed = (labs(error) <= HOMING_SLOWDOWN_ZONE) ? MOTOR_SPEED_SLOW : MOTOR_SPEED;

  if (error > 0) {
    motorDriveIncreasing(speed);
    torsoLastDirection = 1;
  } else {
    motorDriveDecreasing(speed);
    torsoLastDirection = -1;
  }
}

void setupMPU() {
  Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true);
  Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(true);
  Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(true);
}

void readMPU() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)MPU6050_ADDR, (uint8_t)14, true);

  AcX = (Wire.read() << 8) | Wire.read();
  AcY = (Wire.read() << 8) | Wire.read();
  AcZ = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  GyX = (Wire.read() << 8) | Wire.read();
  GyY = (Wire.read() << 8) | Wire.read();
  GyZ = (Wire.read() << 8) | Wire.read();
}

void calibrateMPU() {
  Serial.println("Calibrating MPU6050 Gyro...");
  long gx = 0, gy = 0, gz = 0;
  unsigned long startTime = millis();
  int samples = 0;

  while (millis() - startTime < 3000) {
    readMPU();
    gx += GyX; gy += GyY; gz += GyZ;
    samples++;
    delay(5);
  }

  gyroOffsetX = (float)gx / samples;
  gyroOffsetY = (float)gy / samples;
  gyroOffsetZ = (float)gz / samples;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nInitializing MediTilt Pro System (Standalone Remote Edition)...");

#if SIMULATE_MOTOR
  Serial.println("[SIMULATION] Skipping MPU6050 calibration - checking endpoints only.");
  rotations = rollToRotations(0.0);
#else
  Wire.begin(4, 5);
  setupMPU();
  delay(200);
  calibrateMPU();
  
  readMPU();
  float ax = AcX - accOffsetX;
  float ay = AcY - accOffsetY;
  float az = AcZ - accOffsetZ;
  cFilterRoll.angle  = atan2(ay, az) * 180.0 / PI;
  cFilterPitch.angle = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  lastMicros = micros();

  rotations = rollToRotations(cFilterRoll.angle);
#endif

  setupMotor();
  setupLegMotor();

  // Shift register hardware setup
  pinMode(REMOTE_LATCH_PIN, OUTPUT);
  pinMode(REMOTE_CLOCK_PIN, OUTPUT);
  digitalWrite(REMOTE_LATCH_PIN, HIGH);
  
  // CHANGED TO INPUT_PULLDOWN FOR SAFETY (Defaults to 0 if tether breaks)
  pinMode(REMOTE_DATA_PIN, INPUT_PULLDOWN); 
  
  digitalWrite(REMOTE_CLOCK_PIN, LOW);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 

  int raw = analogRead(ENCODER_PIN);
  lastAngle = (raw / 4095.0) * 360.0;

  Serial.println("Physical remote subsystem online and armed.");
}

void loop() {
  unsigned long currentMillis = millis();

  // Unconditional Encoder Polling
  if (currentMillis - lastEncoderMillis >= ENCODER_SAMPLE_INTERVAL_MS) {
    lastEncoderMillis = currentMillis;
    readEncoder();
  }

  // Handle shift register logic 
  pollRemote();

  // IMU update loop (20ms interval)
#if !SIMULATE_MOTOR
  static unsigned long lastSampleMillis = 0;
  if (currentMillis - lastSampleMillis >= 20) {
    lastSampleMillis = currentMillis;

    readMPU();
    unsigned long now = micros();
    float dt = (now - lastMicros) / 1000000.0;
    lastMicros = now;

    float ax = AcX - accOffsetX; float ay = AcY - accOffsetY; float az = AcZ - accOffsetZ;
    float gxRate = (GyX - gyroOffsetX) / GYRO_SCALE;
    float gyRate = (GyY - gyroOffsetY) / GYRO_SCALE;

    float rollAcc  = atan2(ay, az) * 180.0 / PI;
    float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    roll  = cFilterRoll.update(gxRate, rollAcc, dt);
    pitch = cFilterPitch.update(gyRate, pitchAcc, dt);
  }
#endif

  // Process updates to closed-loop motors if not actively jogged
  updateMotorControl();
}