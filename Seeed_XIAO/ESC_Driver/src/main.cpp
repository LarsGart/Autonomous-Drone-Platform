#include <Arduino.h>
#include <Servo.h>

// Define ESCs
Servo m[4];

// Define speeds
int speed[4] = {1000, 1000, 1000, 1000};

// Define speed byte array
char speedBytes[8];
int bytesRead = 0;
bool readingData = false;
bool speedUpdated = false;

void readSerialData() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '<') {
      readingData = true;
      bytesRead = 0;
      memset(speedBytes, 0, 8);
    }
    else if (c == '>' && readingData) {
      readingData = false;
      speedUpdated = true;
    }
    else {
      if (readingData && bytesRead < 8) {
        speedBytes[bytesRead] = c;
        bytesRead++;
      }
    }
  }
}

void setup() {
  // Attach ESCs to the pins
  m[0].attach(4);
  m[1].attach(8);
  m[2].attach(5);
  m[3].attach(9);

  // Initialize serial port
  Serial.begin(115200);
}

void loop() {
  // Read speed input if available
  readSerialData();

  // Convert byte array to motor speeds
  if (speedUpdated) {
    speedUpdated = false;
    for (int i = 0; i < 4; ++i) {
      int mSpeed = (speedBytes[2 * i] << 8 | speedBytes[2 * i + 1]);

      // Only update speed if it's in the 1000-2000 range
      if (mSpeed >= 1000 && mSpeed <= 2000) {
        speed[i] = mSpeed;
      }
    }
  }

  // Write motor speeds to respective speed controllers
  for (int i = 0; i < 4; ++i) {
    m[i].writeMicroseconds(speed[i]);
  }
}