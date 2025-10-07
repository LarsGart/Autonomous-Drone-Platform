/*!
  \file Jetson_SPI.h
  \brief Jetson SPI communication interface

  This library defines the interface for SPI communication between the Jetson Nano
  and the microcontroller
  
  It uses SERCOM0 in SPI slave mode with DMA for efficient data transfer up to ~4 Mbps
  
  The SPI receive packet structure is illustrated below:
  _____________________________________________________________________
  | Command byte | Data bytes 0...n | CRC upper byte | CRC lower byte |
  ---------------------------------------------------------------------

  Where n is the amount of data bytes the Jetson will send with a command which varies
  depending on the command and is defined in the SPI_DATA_LENGTH array

  If the command is a "read register" command, the master (Jetson) should send 4 padding
  bytes after the packet and then send m bytes where m is the number of bytes the master
  wants to read
    - The padding bytes give the xiao enough time to process and load the write buffer
      with data to send to the master
    - The padding bytes and the m bytes the master sends after the packet can be any value

  An example read transaction would look like this:
        ________________________________________________________________________________
  MOSI: | Packet | 0x00 | 0x00 | 0x00 | 0x00 | 0xFF | 0xFF | 0xFF | 0xFF | 0xFF | 0xFF |
        --------------------------------------------------------------------------------
                 \___________________________/\________________________________________/
                               |                                  |
                          4 pad bytes                     6 read data bytes
                                                                  |
                                              /----------------------------------------\
        ________________________________________________________________________________
  MISO: |   ...  |  ... |  ... |  ... |  ... | 0x12 | 0x34 | 0xAB | 0xCD | 0xEF | 0xFF |
        --------------------------------------------------------------------------------
*/
#ifndef Jetson_SPI_H
#define Jetson_SPI_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

// Constants
#define NUM_CMDS            6
#define SPI_BUFFER_SIZE     8

// Command definitions
#define DEBUG_MODE_CMD      0x00
#define ESC_ARM_DISARM_CMD  0x01
#define MOTOR_SPEEDS_CMD    0x02
#define MOTOR_STOP_CMD      0x03
#define SET_LED_STATE_CMD   0x04
#define READ_REGISTER_CMD   0x05

// Constants
const uint16_t SPI_START_BYTES = 0xBC9E; // Start bytes for SPI communication
const uint8_t SPI_DATA_LENGTH[NUM_CMDS] = {
  1, // DEBUG_MODE
  1, // ESC_ARM_DISARM
  8, // MOTOR_SPEEDS
  1, // MOTOR_STOP
  1, // SET_LED_STATE
  1  // READ_REGISTER
};

// Struct to hold Jetson SPI data and state
typedef struct {
  uint16_t received_crc;
  uint16_t computed_crc;
  uint8_t received_cmd;
  uint8_t rd_data[SPI_BUFFER_SIZE];
  bool rd_data_ready;
} Jetson_SPI_t;

// Global instance of Jetson_SPI
extern Jetson_SPI_t jetson_spi;

// Functions
void jetson_spi_init(void);
void process_spi_rx_data(void);
void write_spi_data(const uint8_t* data, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif