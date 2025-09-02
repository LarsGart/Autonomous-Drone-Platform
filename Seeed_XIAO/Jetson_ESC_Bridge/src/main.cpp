#include <Arduino.h>
#include <Sercom0_SPI_Slave.h>
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
  Objects
*/
SERCOM0_SPI_Slave spi_slave; // SPI slave object

// Process received SPI data
void process_spi_rx_data() {
  switch (spi_slave.received_cmd) {
    case 0: // FIRMWARE_VERSION
      spi_slave.wr_data[0] = (FIRMWARE_VERSION >> 8) & 0xFF; // High byte
      spi_slave.wr_data[1] = FIRMWARE_VERSION & 0xFF;        // Low byte
      spi_slave.wr_data_ready = true;
      break;

    case 1: // ESC_ARM_DISARM
      esc_armed = (spi_slave.rd_data[0] == 0x01); // 1 to arm, 0 to disarm
      break;

    case 2: // CRC_ENABLE_DISABLE
      crc_enabled = (spi_slave.rd_data[0] == 0x01); // 1 to enable, 0 to disable
      break;

    case 3: // MOTOR_SPEEDS
      for (uint8_t i = 0; i < 4; i++) {
        motor_speeds[i] = (spi_slave.rd_data[i*2] << 8) | spi_slave.rd_data[i*2 + 1];
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
  // Enable Serial if ENABLE_SERIAL_DEBUG is defined
  #ifdef ENABLE_SERIAL_DEBUG
    Serial.begin(115200); // Initialize serial communication at 115200 baud rate
    unsigned long serial_timeout = millis() + 5000; // 5 second timeout
    while (!Serial && (millis() < serial_timeout)); // Wait for serial port to connect or timeout
    Serial.println("Setup complete");
  #endif

  // Initialize SPI slave
  spi_slave.init(); // Pass true to enable debug output
  #ifdef ENABLE_SERIAL_DEBUG
    Serial.println("SPI initialized successfully");
  #endif
}

void loop() {
  // Check if new SPI data is ready to be processed
  if (spi_slave.rd_data_ready) {
    process_spi_rx_data();
    spi_slave.rd_data_ready = false;
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

void SERCOM0_Handler() {
  if (SERCOM0->SPI.INTFLAG.bit.RXC) {
    uint8_t data = SERCOM0->SPI.DATA.reg; // Read data from the RX data buffer to clear the interrupt
    spi_slave.handle_receive(data);
  }
}

// void SERCOM0_Handler() {
//   // if (SERCOM0->SPI.INTFLAG.bit.TXC) {
//   //   SERCOM0->SPI.INTFLAG.bit.TXC = 1; // Clear Transmit Complete interrupt
//   //   rx_done = true; // Set the reception flag to true when transmission is complete
//   //   num_bytes = i; // Store the number of bytes received
//   //   i = 0; // Reset i for the next reception
//   // }

//   // if (SERCOM0->SPI.INTFLAG.bit.DRE) {
//   //   // Data Register Empty interrupt: this is where the data is sent to the master
//   //   // Write data to be sent to the master
//   //   SERCOM0->SPI.DATA.reg = tx_buffer[j++]; // Increment j for next data to be sent
//   //   if (j == 4) { // Reset j to 0 after sending 4 bytes
//   //     j = 0;
//   //   }
//   // }

//   // Interrupt if RX data buffer has a new word in it
//   if (SERCOM0->SPI.INTFLAG.bit.RXC) {
//     rx_data = SERCOM0->SPI.DATA.reg; // Read data from the RX data buffer to clear the interrupt
//     rx_buffer[active_buffer][rx_data_i++] = rx_data; // Store received data in the active buffer
//   }
// }