#include <Arduino.h>
#include <Servo.h>

// Define ESCs
Servo m[4];

// Define speeds
int speed[4] = {1000, 1000, 1000, 1000};

// Define controller state
enum CTRL_STATES {waitForConnection, testConnection, receiveSpeed};
enum CTRL_STATES ctrlSm;

// Define speed byte array
char speedBytes[8];
int bytesRead = 0;
bool readingData = false;
bool speedUpdated = false;

// Define time
unsigned long lastTx = 0;

void resetSpeeds() {
   for (int i = 0; i < 4; i++) {
      speed[i] = 1000;
      m[i].writeMicroseconds(1000);
   }
}

void outputSpeeds() {
   for (int i = 0; i < 4; ++i) {
      m[i].writeMicroseconds(speed[i]);
   }
}

void decodeSpeeds() {
   for (int i = 0; i < 4; ++i) {
      int mSpeed = (speedBytes[2 * i] << 8 | speedBytes[2 * i + 1]);

      // Only update speed if it's in the 1000-2000 range
      if (mSpeed >= 1000 && mSpeed <= 2000) {
         speed[i] = mSpeed;
      }
   }
}

void getSpeedBytes(char c) {
   // '<' character initiates start of motor speed byte stream
   if (c == '<' && bytesRead == 0) {
      readingData = true;
      memset(speedBytes, 0, 8);
   }
   else if (bytesRead == 8) {
      bytesRead = 0;
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

void setup() {
   // Attach ESCs to the pins
   m[0].attach(4);
   m[1].attach(8);
   m[2].attach(5);
   m[3].attach(9);

   // Initialize USB Serial port
   Serial.begin(115200);

   // Initialize Jetson Serial port
   Serial1.begin(115200);

   // Set state machine to default state
   ctrlSm = waitForConnection;

   // Get current time
   lastTx = millis();

   // Output 0 speed to the motors
   resetSpeeds();
}

void loop() {
   if (ctrlSm == waitForConnection) {
      // Keep sending out ENQ every 100 ms
      unsigned long currentTime = millis();
      if (currentTime - lastTx > 100) {
         lastTx = currentTime;
         Serial1.write(5);
      }

      // If an ACK is received, advance to the testConnection state
      if (Serial1.available() > 0) {
         char c = Serial1.read();
         if (c == 6) {
            ctrlSm = testConnection;
         }
      }
   }
   else if (ctrlSm == testConnection) {
      if (Serial1.available() > 0) {
         char c = Serial1.read();
         // If an EOT is received, advance to the receiveSpeed state
         if (c == 4 && readingData == false) {
            ctrlSm = receiveSpeed;
         }
         // If an ACK is received, we know the calculated speeds and connection are good
         else if (c == 6 && readingData == false) {
         }
         // If a NACK is received, the Jetson received the wrong speeds back
         // Jump to the waitForConnection state
         else if (c == 21 && readingData == false) {
            ctrlSm = waitForConnection;
            resetSpeeds();
         }
         // If it's another character, then treat it as part of the speed bytes
         else {
            getSpeedBytes(c);
            if (speedUpdated) {
               speedUpdated = false;
               // Send decoded speeds back to Jetson as 4 comma-separated numbers
               Serial1.printf("%d,%d,%d,%d\n",
                  speedBytes[0] << 8 | speedBytes[1],
                  speedBytes[2] << 8 | speedBytes[3],
                  speedBytes[4] << 8 | speedBytes[5],
                  speedBytes[6] << 8 | speedBytes[7]
               );
            }
         }
      }
   }
   else if (ctrlSm == receiveSpeed) {
      if (Serial1.available() > 0) {
         char c = Serial1.read();
         // If an ESC is received, close connection and return to waitForConnection state
         if (c == 27 && readingData == false) {
            ctrlSm = waitForConnection;
            resetSpeeds();
         }
         // If a CAN is received, emergency stop the motors
         else if (c == 24 && readingData == false) {
            resetSpeeds();
         }
         // If it's another character, treat it as part of the speed bytes
         else {
            getSpeedBytes(c);
            if (speedUpdated) {
               speedUpdated = false;
               decodeSpeeds();
            }
         }
      }
      outputSpeeds();
   }
}