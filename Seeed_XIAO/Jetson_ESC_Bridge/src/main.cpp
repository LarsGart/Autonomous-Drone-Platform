#include <Arduino.h>
#include <Jetson_SPI.h>
/*
  Debug
*/
#define ENABLE_SERIAL_DEBUG // Uncomment to enable serial debugging
/*
  Constants
*/
const uint16_t FIRMWARE_VERSION = 0x0100; // Firmware version 1.0
/*
  Variables
*/
uint16_t motor_speeds[4]; // Array to store motor speeds
bool motor_speeds_updated = false; // Flag to indicate if motor speeds have been updated
bool esc_armed = false; // Flag to indicate if ESC is armed
bool crc_enabled = false; // Flag to indicate if CRC is enabled

/*
  @name process_spi_rx_data
  @brief Processes the received SPI data and updates motor speeds or other settings based on the command
  This function checks the received command and updates the motor speeds or other settings accordingly.
  It is called whenever new data is received from the SPI slave.
*/
void process_spi_rx_data() {
  switch (jetson_spi.received_cmd) {
    case 0: // FIRMWARE_VERSION
      jetson_spi.wr_data[0] = (FIRMWARE_VERSION >> 8) & 0xFF; // High byte
      jetson_spi.wr_data[1] = FIRMWARE_VERSION & 0xFF;        // Low byte
      jetson_spi.wr_data_ready = true;
      break;

    case 1: // ESC_ARM_DISARM
      esc_armed = (jetson_spi.rd_data[0] == 0x01); // 1 to arm, 0 to disarm
      break;

    case 2: // CRC_ENABLE_DISABLE
      crc_enabled = (jetson_spi.rd_data[0] == 0x01); // 1 to enable, 0 to disable
      break;

    case 3: // MOTOR_SPEEDS
      for (uint8_t i = 0; i < 4; i++) {
        motor_speeds[i] = (jetson_spi.rd_data[i*2] << 8) | jetson_spi.rd_data[i*2 + 1];
      }
      motor_speeds_updated = true;
      break;

    case 4: // MOTOR_STOP
      for (uint8_t i = 0; i < 4; i++) {
        motor_speeds[i] = 1000; // Set all motor speeds to 1000 (stop)
      }
      motor_speeds_updated = true;
      break;
    
    default:
      break;
  }
}

void setup() {
  #ifdef ENABLE_SERIAL_DEBUG
    Serial.begin(115200);
    unsigned long serial_timeout = millis() + 5000; // 5 second timeout
    while (!Serial && (millis() < serial_timeout));
    Serial.println("Setup complete");
  #endif

  // Initialize Jetson SPI
  jetson_spi_init();
  #ifdef ENABLE_SERIAL_DEBUG
    Serial.println("SPI initialized successfully");
  #endif
}

void loop() {
  // Check if new SPI data is ready to be processed
  if (jetson_spi.rd_data_ready) {
    process_spi_rx_data();
    jetson_spi.rd_data_ready = false;
  }

  #ifdef ENABLE_SERIAL_DEBUG
    if (motor_speeds_updated) { // Check if speeds are received
      Serial.print("Motor speeds: ");
      for (int i = 0; i < 4; i++) {
        Serial.print(motor_speeds[i]);
        Serial.print(" ");
      }
      Serial.println();
      motor_speeds_updated = false; // Reset the flag after processing
    }
  #endif
}