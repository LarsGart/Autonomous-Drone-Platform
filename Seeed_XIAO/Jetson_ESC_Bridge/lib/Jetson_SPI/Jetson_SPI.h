/*
  Sercom0_SPI_Slave.h - Header file for SERCOM0 SPI slave interface
  This file defines the structure and functions for initializing and managing
  the SERCOM0 SPI slave communication.
  
  Revision: 1.0
  Date: 09/10/2025
*/

#ifndef Jetson_SPI_H
#define Jetson_SPI_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_CMDS  5     // Number of commands
#define CMD_WR    0x00  // Command write code
#define CMD_RD    0x01  // Command read code

static const uint16_t SPI_START_BYTES = 0xBC9E; // Start bytes for SPI communication

/*
  SPI table format:
  {rd_wr_command, data_size}
  - rd_wr_command: 0x00 for write, 0x01 for read
  - data_size: number of bytes this transaction will transfer
*/
const uint8_t spi_cmd_info[NUM_CMDS][2] = {
  {CMD_RD, 2},  // FIRMWARE_VERSION
  {CMD_WR, 1},  // ESC_ARM_DISARM
  {CMD_WR, 1},  // CRC_ENABLE_DISABLE
  {CMD_WR, 8},  // MOTOR_SPEEDS
  {CMD_WR, 1}   // MOTOR_STOP
};

// Struct to hold Jetson SPI slave variables
typedef struct {
  volatile bool rd_data_ready;
  volatile uint8_t received_cmd;
  volatile uint8_t rd_data[16];
  bool wr_data_ready;
  uint8_t wr_data[8];
} Jetson_SPI_t;

// Global instance of Jetson_SPI
extern Jetson_SPI_t jetson_spi;

// Initialize the Jetson SPI
void jetson_spi_init(void);

#ifdef __cplusplus
}
#endif

#endif