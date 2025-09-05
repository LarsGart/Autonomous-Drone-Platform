#include <Arduino.h>
#include <Jetson_SPI.h>
#include <Motors.h>

/*
  Debug
*/
// #define ENABLE_SERIAL_DEBUG // Uncomment to enable serial debugging
/*
  Read-only register addresses
*/
#define FIRMWARE_VERSION_REG  0x00
#define BATTERY_VOLTAGE_REG   0x01
#define CURRENT_DRAW_REG      0x02
#define TELEMETRY_DATA_REG    0x03
#define ARM_STATUS_REG        0x04
#define MOTOR_SPEEDS_REG      0x05
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
  @name process_spi_cmd
  @brief Processes the received SPI command and takes appropriate action
*/
void process_spi_cmd() {
  switch (jetson_spi.received_cmd) {
    case ESC_ARM_DISARM_CMD: {
      motors.armed = (jetson_spi.rd_data[0] == 0x01); // 1 to arm, 0 to disarm
      break;
    }

    case MOTOR_SPEEDS_CMD: {
      // Each motor speed is 2 bytes (high byte first)
      for (uint8_t i = 0; i < 4; i++) {
        motor_speeds[i] = (jetson_spi.rd_data[i*2] << 8) | jetson_spi.rd_data[i*2 + 1];
      }
      motor_speeds_updated = true;
      break;
    }

    case MOTOR_STOP_CMD: {
      for (uint8_t i = 0; i < 4; i++) {
        motor_speeds[i] = 1000;
      }
      motor_speeds_updated = true;
      break;
    }

    case READ_REGISTER_CMD: {
      uint8_t reg_addr = jetson_spi.rd_data[0];
      uint8_t reg_value[8];
      uint8_t reg_value_len;
      switch (reg_addr) {
        case FIRMWARE_VERSION_REG: {
          reg_value[0] = (FIRMWARE_VERSION >> 8) & 0xFF;
          reg_value[1] = FIRMWARE_VERSION & 0xFF;
          reg_value_len = 2;
          break;
        }

        case BATTERY_VOLTAGE_REG: {
          // Placeholder: Replace with actual battery voltage reading
          uint16_t battery_voltage = 12000; // Example: 12.0V represented as 12000mV
          reg_value[0] = (battery_voltage >> 8) & 0xFF;
          reg_value[1] = battery_voltage & 0xFF;
          reg_value_len = 2;
          break;
        }

        case CURRENT_DRAW_REG: {
          // Placeholder: Replace with actual current draw reading
          uint16_t current_draw = 1500; // Example: 1.5A represented as 1500mA
          reg_value[0] = (current_draw >> 8) & 0xFF;
          reg_value[1] = current_draw & 0xFF;
          reg_value_len = 2;
          break;
        }

        case TELEMETRY_DATA_REG: {
          // Placeholder: Replace with actual telemetry data
          for (uint8_t i = 0; i < 8; i++) {
            reg_value[i] = 0; // Example: Fill with zeros
          }
          reg_value_len = 8;
          break;
        }

        case ARM_STATUS_REG: {
          reg_value[0] = motors.armed ? 1 : 0;
          reg_value_len = 1;
          break;
        }

        case MOTOR_SPEEDS_REG: {
          for (uint8_t i = 0; i < 4; i++) {
            reg_value[i*2] = (motor_speeds[i] >> 8) & 0xFF;
            reg_value[i*2 + 1] = motor_speeds[i] & 0xFF;
          }
          reg_value_len = 8;
          break;
        }

        default: {
          // Invalid register address, return zero length
          reg_value_len = 0;
          break;
        }
      }
      write_spi_data(reg_value, reg_value_len);
      break;
    }
    
    default: {
      break;
    }
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
  // Continuously check for and process SPI data
  process_spi_rx_data();

  // If a new command has been received, process it
  if (jetson_spi.rd_data_ready) {
    process_spi_cmd();
    jetson_spi.rd_data_ready = false;
  }

  // Update motor speeds if they have been changed
  if (motor_speeds_updated) {
    for (int i = 0; i < 4; i++) {
      set_motor_pulse_width(i, motor_speeds[i]);
    }
    motor_speeds_updated = false;

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
}