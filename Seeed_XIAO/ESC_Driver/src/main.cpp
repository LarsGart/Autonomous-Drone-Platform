#include <Arduino.h>
#include <Servo.h>

#define USB Serial
#define Jetson Serial1

// Define ESCs
Servo m[4];

// Define speeds
int speed[4] = {1000, 1000, 1000, 1000};

// Define controller state
enum CTRL_STATES {waitForConnection, testConnection, receiveSpeed};
enum CTRL_STATES ctrlSm;

// Enumerate ASCII values
enum ASCII {STX = 2, EOT = 4, ENQ = 5, ACK = 6, NACK = 21, CAN = 24, ESC = 27};

// Define speed byte array
char speedBytes[8];
int bytesRead = 0;
bool readingData = false;
bool speedUpdated = false;

// Define LED state
bool ledState = false;

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
   // STX character initiates start of motor speed byte stream
   if (c == STX && !readingData) {
      bytesRead = 0;
      readingData = true;
      memset(speedBytes, 0, 8);
   }
   else if (readingData) {
      speedBytes[bytesRead] = c;
      bytesRead++;
      if (bytesRead == 8) {
         readingData = false;
         speedUpdated = true;
      }
   }
}

void setup() {
   // Attach ESCs to the pins
   m[0].attach(4);
   m[1].attach(8);
   m[2].attach(5);
   m[3].attach(9);

   // Set the onboard LED as an output
   pinMode(PIN_LED2, OUTPUT);

   // Initialize USB Serial port
   USB.begin(115200);

   // Initialize Jetson Serial port
   Jetson.begin(115200);

   // Set state machine to default state
   ctrlSm = waitForConnection;

   // Get current time
   lastTx = millis();

   // Output 0 speed to the motors
   resetSpeeds();

   // Wait some time before sending out an ENQ message to prevent booting errors
   delay(5000);
}

void loop() {
   // Set the LED
   digitalWrite(PIN_LED2, (ledState ? LOW : HIGH));

   // Control State Machine
   if (ctrlSm == waitForConnection) {
      // Keep sending out ENQ every 250 ms
      unsigned long currentTime = millis();
      if (currentTime - lastTx > 250) {
         lastTx = currentTime;
         Jetson.write(ENQ);

         ledState = !ledState;
      }

      if (Jetson.available() > 0) {
         char c = Jetson.read();
         // If an ACK is received, advance to the testConnection state
         if (c == ACK) {
            ctrlSm = testConnection;
            ledState = true;
         }
         // If an ENQ is received, report back the waitForConnection state
         else if (c == ENQ) {
            Jetson.write(waitForConnection);
         }
      }
   }
   else if (ctrlSm == testConnection) {
      if (Jetson.available() > 0) {
         char c = Jetson.read();
         // If an EOT is received, advance to the receiveSpeed state
         if (c == EOT && !readingData) {
            ctrlSm = receiveSpeed;
            ledState = false;
         }
         // If an ACK is received, we know the calculated speeds and connection are good
         else if (c == ACK && !readingData) {
         }
         // If a NACK is received, the Jetson received the wrong speeds back
         // Jump to the waitForConnection state
         else if (c == NACK && !readingData) {
            ctrlSm = waitForConnection;
            resetSpeeds();
         }
         // If an ENQ is received, report back the testConnection state
         else if (c == ENQ && !readingData) {
            Jetson.write(testConnection);
         }
         // If it's another character, then treat it as part of the speed bytes
         else {
            getSpeedBytes(c);
            if (speedUpdated) {
               speedUpdated = false;
               // Send decoded speeds back to Jetson as 4 comma-separated numbers
               Jetson.printf("%d,%d,%d,%d\n",
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
      if (Jetson.available() > 0) {
         char c = Jetson.read();
         // If an ESC is received, close connection and return to waitForConnection state
         if (c == ESC && !readingData) {
            ctrlSm = waitForConnection;
            resetSpeeds();
         }
         // If a CAN is received, emergency stop the motors
         else if (c == CAN && !readingData) {
            resetSpeeds();
         }
         // If an ENQ is received, report back the receiveSpeed state
         else if (c == ENQ && !readingData) {
            Jetson.write(receiveSpeed);
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