#include <Arduino.h>
#include <Servo.h>

// Define ESCs
Servo m[4];

// Define speeds
int speed[4] = {1000, 1000, 1000, 1000};

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
  if (Serial.available() > 0) {
    char buffer[16];
    size_t bytesRead = Serial.readBytesUntil('\n', buffer, 16);
    if (bytesRead >= 8) {
      for (int i = 0; i < 4; ++i) {
        // Convert byte stream to motor speeds
        speed[i] = (buffer[2 * i] << 8 | buffer[2 * i + 1]);

        // Constrain speed values to 1000-2000 range
        if (speed[i] < 1000) {
          speed[i] = 1000;
        }
        else if (speed[i] > 2000) {
          speed[i] = 2000;
        }
      }
    }
  }

  // Write motor speeds to respective speed controllers
  for (int i = 0; i < 4; ++i) {
    m[i].writeMicroseconds(speed[i]);
  }
}