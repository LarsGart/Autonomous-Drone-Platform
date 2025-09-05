/*
  Jetson_SPI.h - Header file for Jetson SPI communication
*/
#ifndef Jetson_SPI_H
#define Jetson_SPI_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

// Constants
#define NUM_CMDS            4
#define SPI_BUFFER_SIZE     8

// Command definitions
#define ESC_ARM_DISARM_CMD  0x00
#define MOTOR_SPEEDS_CMD    0x01
#define MOTOR_STOP_CMD      0x02
#define READ_REGISTER_CMD   0x03

// Struct to hold Jetson SPI data and state
typedef struct {
  volatile uint8_t received_cmd;
  volatile uint8_t rd_data[SPI_BUFFER_SIZE];
  volatile bool rd_data_ready;
} Jetson_SPI_t;

// Global instance of Jetson_SPI
extern Jetson_SPI_t jetson_spi;

uint8_t jetson_spi_init(void);
void process_spi_rx_data(void);
void write_spi_data(uint8_t* data, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif