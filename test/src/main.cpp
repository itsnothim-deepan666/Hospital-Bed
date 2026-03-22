#include <Arduino.h>
#include <Wire.h>

#define MUX_ADDR 0x70  // 7SEMI TCA9548A I2C MUX address

bool isMuxConnected() {
  Wire.beginTransmission(MUX_ADDR);
  uint8_t error = Wire.endTransmission();
  return (error == 0);
}

void selectChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void disableAllChannels() {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(0);
  Wire.endTransmission();
}

void scanChannel(uint8_t channel) {
  Serial.printf("\n[Checking Channel %d] Selecting MUX channel %d...\n", channel, channel);
  selectChannel(channel);
  delay(10);  // small delay for channel switch

  uint8_t deviceCount = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    if (addr == MUX_ADDR) continue;  // skip the MUX itself
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("  [CH%d] Device found at 0x%02X\n", channel, addr);
      deviceCount++;
    }
  }

  if (deviceCount == 0) {
    Serial.printf("  [CH%d] No devices found.\n", channel);
  } else {
    Serial.printf("  [CH%d] %d device(s) found.\n", channel, deviceCount);
  }
}

void scanAllChannels() {
  Serial.println("\n========================================");
  Serial.println("  Scanning all 8 MUX channels (0-7)");
  Serial.println("========================================");

  uint8_t totalDevices = 0;

  for (uint8_t ch = 0; ch < 8; ch++) {
    scanChannel(ch);
  }

  disableAllChannels();  // clean up after scan

  Serial.println("\n========================================");
  Serial.println("  Scan complete");
  Serial.println("========================================");
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Wire.begin();

  Serial.println("\n========================================");
  Serial.println("  7SEMI TCA9548A I2C MUX Scanner");
  Serial.println("========================================");
  Serial.printf("  MUX address: 0x%02X\n", MUX_ADDR);

  // Check if MUX is connected
  Serial.println("\n[Init] Checking MUX connection...");
  if (isMuxConnected()) {
    Serial.println("[Init] 7SEMI MUX detected on I2C bus. OK!");
    scanAllChannels();
  } else {
    Serial.println("[ERROR] 7SEMI MUX NOT found at 0x70!");
    Serial.println("[ERROR] Please check wiring:");
    Serial.println("  - SDA connected?");
    Serial.println("  - SCL connected?");
    Serial.println("  - VCC (3.3V) connected?");
    Serial.println("  - GND connected?");
    Serial.println("  - Address pins (A0-A2) set correctly?");
  }
}

void loop() {
  delay(10000);

  Serial.println("\n\n--- Rescanning ---");
  if (isMuxConnected()) {
    Serial.println("[OK] MUX still connected.");
    scanAllChannels();
  } else {
    Serial.println("[ERROR] MUX connection lost!");
  }
}