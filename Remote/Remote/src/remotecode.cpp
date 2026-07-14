#include <Arduino.h>

// ESP8266 SPI Pins
const int LATCH_PIN = D1;       
const int CLOCK_PIN = D5;      
const int DATA_PIN  = D6;      

// ===================================================================
// PLATFORMIO FUNCTION PROTOTYPES (Required by Standard C++)
// ===================================================================
byte read74HC165();
void updateIndicatorLED(bool isCommandActive);
void commandHead(int dir);
void commandLeg(int dir);
void commandTilt(int dir);
void stopAllMotors();

// System States
bool isHomingActive = false;
int homingStage = 0; // 0 = Idle, 1 = Tilt, 2 = Leg, 3 = Head
unsigned long homingTimer = 0; // Tracks time spent in each homing stage

// Previous state tracking (Strictly for keeping the Serial Monitor clean)
int lastHeadState = 0;
int lastLegState = 0;
int lastTiltState = 0;
bool lastKillState = false;

// LED Blink Timer Variables
unsigned long lastBlinkTime = 0;
bool ledState = HIGH; // HIGH is OFF for the ESP8266 built-in LED

// Custom shift function to fix the Arduino shiftIn() clock bug
byte read74HC165() {
  byte value = 0;
  for(int i = 0; i < 8; ++i) {
    int bitValue = digitalRead(DATA_PIN);
    value |= (bitValue << (7 - i)); 
    digitalWrite(CLOCK_PIN, HIGH);
    delayMicroseconds(1); 
    digitalWrite(CLOCK_PIN, LOW);
  }
  return value;
}

// Non-blocking LED Blinker
void updateIndicatorLED(bool isCommandActive) {
  if (isCommandActive) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastBlinkTime >= 100) { // Fast blink rate (100ms)
      lastBlinkTime = currentMillis;
      ledState = !ledState; // Toggle state
      digitalWrite(LED_BUILTIN, ledState);
    }
  } else {
    // Ensure the LED is completely OFF when no commands are active
    if (ledState != HIGH) {
      digitalWrite(LED_BUILTIN, HIGH);
      ledState = HIGH;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n==============================================");
  Serial.println("  BED MASTER CONTROL: NON-BLOCKING HOMING V3  ");
  Serial.println("==============================================");
  
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  digitalWrite(LATCH_PIN, HIGH);
  pinMode(DATA_PIN, INPUT);
  digitalWrite(CLOCK_PIN, LOW); 

  // Initialize the onboard LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Turn it off initially
}

// ===================================================================
// MOTOR COMMAND PLACEHOLDERS (Add your motor driver logic here later)
// ===================================================================

void commandHead(int dir) { 
  if (dir != lastHeadState) {
    if (dir == 1) Serial.println("CMD: HEAD LIFTING...");
    else if (dir == -1) Serial.println("CMD: HEAD LOWERING...");
    else Serial.println("CMD: HEAD STOPPED.");
    lastHeadState = dir;
  }
}

void commandLeg(int dir) { 
  if (dir != lastLegState) {
    if (dir == 1) Serial.println("CMD: LEG LIFTING...");
    else if (dir == -1) Serial.println("CMD: LEG LOWERING...");
    else Serial.println("CMD: LEG STOPPED.");
    lastLegState = dir;
  }
}

void commandTilt(int dir) { 
  if (dir != lastTiltState) {
    if (dir == 1) Serial.println("CMD: SIDE TILT -> RAISING LEFT");
    else if (dir == 2) Serial.println("CMD: SIDE TILT -> RAISING RIGHT (Pushing Left to 0)");
    else Serial.println("CMD: SIDE TILT STOPPED.");
    lastTiltState = dir;
  }
}

void stopAllMotors() {
  commandHead(0);
  commandLeg(0);
  commandTilt(0);
}

// ===================================================================
// MAIN CONTROL LOOP
// ===================================================================

void loop() {
  // 1. Latch and Read Buttons
  digitalWrite(LATCH_PIN, LOW);
  delayMicroseconds(5); 
  digitalWrite(LATCH_PIN, HIGH);
  byte state = read74HC165();

  // 2. Parse Button States
  bool btnHeadUp   = bitRead(state, 0); 
  bool btnLegUp    = bitRead(state, 1); 
  bool btnLeftUp   = bitRead(state, 2); 
  bool btnRightUp  = bitRead(state, 3); 
  
  bool btnHeadDown = bitRead(state, 4); 
  bool btnLegDown  = bitRead(state, 5); 
  bool btnKill     = bitRead(state, 6); 
  bool btnHoming   = bitRead(state, 7); 

  // 3. Track if ANY movement command is currently active for the LED
  bool isMoving = (btnHeadUp || btnHeadDown || btnLegUp || btnLegDown || btnLeftUp || btnRightUp || isHomingActive);

  // ---------------------------------------------------------
  // PRIORITY 1: KILL SWITCH (Absolute Override)
  // ---------------------------------------------------------
  if (btnKill) {
    if (!lastKillState) {
      Serial.println("\n[ !!! EMERGENCY KILL SWITCH ACTIVATED !!! ]");
      lastKillState = true;
    }
    isHomingActive = false;
    homingStage = 0;
    stopAllMotors();
    
    // Solid LED to indicate an emergency halt state
    digitalWrite(LED_BUILTIN, LOW); 
    return; // Halt the loop here. Nothing else runs.
  } else {
    if (lastKillState) {
      Serial.println("\n[ KILL SWITCH RELEASED - System Ready ]");
      lastKillState = false;
      digitalWrite(LED_BUILTIN, HIGH); // Turn off emergency LED
    }
  }

  // Update the blinking LED (Blinks if moving, turns off if not)
  updateIndicatorLED(isMoving);

  // ---------------------------------------------------------
  // PRIORITY 2: HOMING SEQUENCE (Non-Blocking Single Tap Macro)
  // ---------------------------------------------------------
  if (btnHoming && !isHomingActive) {
    Serial.println("\n[ HOMING INITIATED ] Automating return to 0...");
    isHomingActive = true;
    homingStage = 1;
    homingTimer = millis(); // Start the timer for step 1
  }

  if (isHomingActive) {
    unsigned long currentMillis = millis();
    
    if (homingStage == 1) {
      if (currentMillis - homingTimer > 2000) {
        Serial.println("HOMING Step 1 Complete. Now Lowering Legs...");
        homingStage = 2; 
        homingTimer = currentMillis; // Reset timer for step 2
      }
    } 
    else if (homingStage == 2) {
      if (currentMillis - homingTimer > 2000) {
        Serial.println("HOMING Step 2 Complete. Now Lowering Head...");
        homingStage = 3;
        homingTimer = currentMillis; // Reset timer for step 3
      }
    }
    else if (homingStage == 3) {
      if (currentMillis - homingTimer > 2000) {
        Serial.println("[ HOMING COMPLETE ] System at 0.");
        isHomingActive = false;
        homingStage = 0;
      }
    }
    
    return; // Lock out manual controls while homing
  }

  // ---------------------------------------------------------
  // PRIORITY 3: MANUAL CONTINUOUS CONTROL
  // ---------------------------------------------------------
  
  // Head Control
  if (btnHeadUp) commandHead(1);
  else if (btnHeadDown) commandHead(-1);
  else commandHead(0);

  // Leg Control
  if (btnLegUp) commandLeg(1);
  else if (btnLegDown) commandLeg(-1);
  else commandLeg(0);

  // Side Tilt Control (Half-Gear Logic)
  if (btnLeftUp) commandTilt(1);
  else if (btnRightUp) commandTilt(2);
  else commandTilt(0);

  delay(20); // Small loop stability delay
}