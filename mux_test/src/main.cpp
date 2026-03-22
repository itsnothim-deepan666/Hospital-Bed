#include <Wire.h>
#include <Arduino.h>

#define MUX_ADDR 0x70  // Default I2C address of PCA9548A

void selectMuxChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);  // Select channel
  Wire.endTransmission();
}

void scanI2CDevices() {
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(address, HEX);
    }
  }
}

void setup() {
  delay(3000);
  Serial.begin(9600);
  Serial.println("Setup started");
  delay(1000);
  Wire.begin();// Arduino Mega uses pins 20(SDA) and 21(SCL)
  Serial.println("Starting PCA9548A multiplexer scan...");

  for (uint8_t channel = 0; channel < 8; ++channel) {
    Serial.print("Scanning channel ");
    Serial.println(channel);
    selectMuxChannel(channel);
    delay(100);  // Give time to switch
    scanI2CDevices();
    Serial.println("--------------------------");
  }
}


void loop() {
  Serial.println("Reaching loop");
  // Nothing here — all done in setup!
}
