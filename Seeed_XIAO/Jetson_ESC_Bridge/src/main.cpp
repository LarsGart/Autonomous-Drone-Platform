#include <Arduino.h>
#include <Jetson_SPI.h>
#include <Motors.h>
/*
  Debug
*/
// #define ENABLE_SERIAL_DEBUG // Uncomment to enable serial debugging
/*
  Constants
*/
const uint16_t FIRMWARE_VERSION = 0x0100; // Firmware version 1.0
/*
  Variables
*/
bool serial_initialized = false; // Flag to indicate if serial is initialized

uint16_t motor_speeds[4]; // Array to store motor speeds
bool motor_speeds_updated = false; // Flag to indicate if motor speeds have been updated

/*
  @name process_spi_rx_data
  @brief Processes the received SPI data and updates motor speeds or other settings based on the command
  This function checks the received command and updates the motor speeds or other settings accordingly.
  It is called whenever new data is received from the SPI slave.
*/
void process_spi_rx_data() {
  switch (jetson_spi.received_cmd) {
    case FIRMWARE_VERSION_CMD:
      jetson_spi.wr_data[0] = (FIRMWARE_VERSION >> 8) & 0xFF; // High byte
      jetson_spi.wr_data[1] = FIRMWARE_VERSION & 0xFF;        // Low byte
      jetson_spi.wr_data_ready = true;
      break;

    case ESC_ARM_DISARM_CMD:
      motors.armed = (jetson_spi.rd_data[0] == 0x01); // 1 to arm, 0 to disarm
      break;

    case MOTOR_SPEEDS_CMD:
      for (uint8_t i = 0; i < 4; i++) {
        motor_speeds[i] = (jetson_spi.rd_data[i*2] << 8) | jetson_spi.rd_data[i*2 + 1];
      }
      motor_speeds_updated = true;
      break;

    case MOTOR_STOP_CMD:
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
    if (!Serial) {
      // Serial not initialized within timeout
      // Handle error (e.g., blink an LED)
    } else {
      serial_initialized = true;
      Serial.println("Serial initialized");
    }
  #endif

  // Initialize peripherals
  uint8_t spi_init_status = jetson_spi_init();
  uint8_t motor_init_status = motors_init();
  #ifdef ENABLE_SERIAL_DEBUG
    if (serial_initialized) {
      if (spi_init_status != 0) {
        Serial.print("SPI initialization failed with error code: ");
        Serial.println(spi_init_status);
      }
      else {
        Serial.println("SPI initialized successfully");
      }

      if (motor_init_status != 0) {
        Serial.print("Motor initialization failed with error code: ");
        Serial.println(motor_init_status);
      }
      else {
        Serial.println("Motors initialized successfully");
      }
    }
  #endif


}

void loop() {
  // Process received SPI data if available
  if (jetson_spi.rd_data_ready) {
    process_spi_rx_data();
    jetson_spi.rd_data_ready = false;
  }

  // Update motor speeds if they have been changed
  if (motor_speeds_updated) {
    for (int i = 0; i < 4; i++) {
      set_motor_pulse_width(i, motor_speeds[i]);
    }
    motor_speeds_updated = false; // Reset the flag after processing

    #ifdef ENABLE_SERIAL_DEBUG
      if (serial_initialized) {
        Serial.print("Motor speeds: ");
        for (int i = 0; i < 4; i++) {
          Serial.print(motor_speeds[i]);
          Serial.print(" ");
        }
        Serial.println();
      }
    #endif
  }

  #ifdef ENABLE_SERIAL_DEBUG
    if (jetson_spi.crc_check_done) {
      if (serial_initialized) {
        Serial.print("CRC Check: ");
        Serial.println(jetson_spi.calculated_crc, HEX);
      }
      jetson_spi.crc_check_done = false; // Reset the flag after processing
    }
  #endif
}