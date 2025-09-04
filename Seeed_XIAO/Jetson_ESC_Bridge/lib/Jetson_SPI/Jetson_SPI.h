/*
  Jetson_SPI.h - Header file for Jetson SPI communication
*/
#ifndef Jetson_SPI_H
#define Jetson_SPI_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_CMDS 4 // Number of commands

// Command definitions
#define FIRMWARE_VERSION_CMD  0x00
#define ESC_ARM_DISARM_CMD    0x01
#define MOTOR_SPEEDS_CMD      0x02
#define MOTOR_STOP_CMD        0x03

static const uint16_t SPI_START_BYTES = 0xBC9E; // Start bytes for SPI communication
static const uint8_t SPI_DATA_LENGTH[NUM_CMDS] = {
  2, // FIRMWARE_VERSION
  1, // ESC_ARM_DISARM
  8, // MOTOR_SPEEDS
  1  // MOTOR_STOP
};

// Struct to hold Jetson SPI data and state
typedef struct {
  volatile bool crc_check_done;
  volatile uint16_t calculated_crc;
  volatile bool rd_data_ready;
  volatile uint8_t received_cmd;
  volatile uint8_t rd_data[16];
  bool wr_data_ready;
  uint8_t wr_data[8];
} Jetson_SPI_t;

// Global instance of Jetson_SPI
extern Jetson_SPI_t jetson_spi;

uint8_t jetson_spi_init(void);

#ifdef __cplusplus
}
#endif

#endif