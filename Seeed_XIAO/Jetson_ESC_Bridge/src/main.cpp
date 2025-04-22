#include <Arduino.h>
#include <SPISlave.h>

/*
  Debug
*/
#define SERIAL_DEBUG // Uncomment to enable serial debugging
/*
  Interrupt handler name replacements
*/
#define SPI_SLAVE_HANDLER SERCOM0_Handler // Define the SPI slave handler function
/*
  SPI interrupt variables
*/
volatile bool start_of_transaction = false; // Flag to indicate start of SPI transaction
volatile uint8_t rx_data = 0; // Variable to store received data
volatile uint8_t cmd_code = 0; // Variable to store command code
volatile uint8_t cmd_rd_wr = 0; // Variable to store read/write command
volatile uint8_t data_size = 0; // Variable to store data size

volatile uint8_t rx_buffer[2][8]; // Ping-pong buffer to store received data
volatile uint8_t active_buffer = 0; // Index for active buffer
volatile uint8_t rx_data_i = 0; // Index for received data
volatile uint8_t rx_data_size = 0; // Size of received data

volatile uint8_t rx_motor_data_buffer[2][9]; // Ping-pong buffer to store received motor data
volatile uint8_t motor_data_active_buffer = 0; // Index for active motor data buffer
volatile uint8_t motor_data_inactive_buffer = 1; // Index for inactive motor data buffer
volatile uint8_t rx_motor_data_i = 0; // Index for received motor data
/*
  System variables
*/
volatile bool esc_armed = false; // Flag to indicate if ESC is armed
volatile bool crc_enabled = false; // Flag to indicate if CRC is enabled
volatile bool speeds_received = false; // Flag to indicate if speeds are received
volatile bool data_received = false; // Flag to indicate if data is received

uint8_t motor_data_buffer_size = 8; // Tail index for motor data buffer
uint16_t motor_speeds[4]; // Array to store motor speeds
/*
  Objects
*/
SPISlave spi_slave; // SPI slave object

void setup() {
  // Initialize SPI slave
  spi_slave.init();

  // Enable Serial if SERIAL_DEBUG is defined
  #ifdef SERIAL_DEBUG
    Serial.begin(115200); // Initialize serial communication at 115200 baud rate
    while (!Serial); // Wait for serial port to connect
    Serial.println("Setup complete");
  #endif
}

void loop() {
  #ifdef SERIAL_DEBUG
    if (speeds_received) { // Check if speeds are received
      Serial.print("Motor speeds: ");
      for (int i = 0; i < 8; i++) {
        Serial.print(rx_motor_data_buffer[motor_data_inactive_buffer][i], HEX); // Print motor speeds in hexadecimal format
        Serial.print(" ");
      }
      Serial.println();
      speeds_received = false; // Reset the flag after processing
    }
  #endif
}

void SPI_SLAVE_HANDLER() {
  // if (SERCOM0->SPI.INTFLAG.bit.TXC) {
  //   SERCOM0->SPI.INTFLAG.bit.TXC = 1; // Clear Transmit Complete interrupt
  //   rx_done = true; // Set the reception flag to true when transmission is complete
  //   num_bytes = i; // Store the number of bytes received
  //   i = 0; // Reset i for the next reception
  // }

  // if (SERCOM0->SPI.INTFLAG.bit.DRE) {
  //   // Data Register Empty interrupt: this is where the data is sent to the master
  //   // Write data to be sent to the master
  //   SERCOM0->SPI.DATA.reg = tx_buffer[j++]; // Increment j for next data to be sent
  //   if (j == 4) { // Reset j to 0 after sending 4 bytes
  //     j = 0;
  //   }
  // }

  // Interrupt if RX data buffer has a new word in it
  if (SERCOM0->SPI.INTFLAG.bit.RXC) {
    rx_data = SERCOM0->SPI.DATA.reg; // Read data from the RX data buffer to clear the interrupt

    // Check if the start of a transaction is detected
    if (start_of_transaction && rx_data < NUM_CMDS) {
      cmd_code = rx_data; // Store the command code
      cmd_rd_wr = spi_slave.spi_cmd_info[cmd_code][0]; // Get the read/write command
      data_size = spi_slave.spi_cmd_info[cmd_code][1]; // Get the data size for the command

      if (cmd_code == CMD_MOTOR_SPEEDS) { // Check if the command is for motor speeds
        start_of_transaction = false; // Reset the start of transaction flag
        rx_motor_data_i = 0; // Reset index for received motor data
      } else if (cmd_rd_wr == CMD_WR) { // Check if it's a write command
        start_of_transaction = false; // Reset the start of transaction flag
        rx_data_i = 1; // Reset index for received data
        rx_data_size = data_size; // Set the data size for the command
        rx_buffer[active_buffer][0] = cmd_code; // Store the command in the active buffer
      } else {
        // Handle read command if needed
      }
    } else {
      if (cmd_code == CMD_MOTOR_SPEEDS) {
        rx_motor_data_buffer[motor_data_active_buffer][rx_motor_data_i++] = rx_data; // Store the received motor data in the active buffer
        if (rx_motor_data_i == MOTOR_SPEEDS_BYTE_LENGTH) { // Check if all motor bytes are received
          speeds_received = true; // Set flag to indicate speeds are received
          start_of_transaction = true; // Reset start of transaction flag
          motor_data_active_buffer = (motor_data_active_buffer == 0) ? 1 : 0; // Switch to the other buffer
          motor_data_inactive_buffer = (motor_data_active_buffer == 0) ? 1 : 0; // Switch to the other buffer
        }
      } else if (cmd_rd_wr == CMD_WR) { // Check if it's a write command
        rx_buffer[active_buffer][rx_data_i++] = rx_data; // Store the received data in the active buffer
        if (rx_data_i == rx_data_size) { // Check if the buffer is full
          data_received = true; // Set flag to indicate data is received
          start_of_transaction = true; // Reset start of transaction flag
          active_buffer = (active_buffer == 0) ? 1 : 0; // Switch to the other buffer
        }
      }
    }
  }
}