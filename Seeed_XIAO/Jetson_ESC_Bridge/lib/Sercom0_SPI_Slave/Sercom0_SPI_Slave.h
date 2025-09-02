#ifndef SERCOM0_SPI_SLAVE_H
#define SERCOM0_SPI_SLAVE_H

#include <Arduino.h>
/*
  SPI command info
*/
#define CMD_WR 0x00 // Command write code
#define CMD_RD 0x01 // Command read code

#define NUM_CMDS 5 // Number of commands

class SERCOM0_SPI_Slave {
  public:
    const uint16_t SPI_START_BYTES = 0xBC9E; // Start bytes for SPI communication
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
    
    // Constructor
    SERCOM0_SPI_Slave();

    // Methods
    void init(); // Initialize SPI slave
    void handle_receive(uint8_t data); // Handle received data

    // Variables
    volatile bool rd_data_ready;
    volatile uint8_t received_cmd;
    volatile uint8_t rd_data[16];
    
    bool wr_data_ready;
    uint8_t wr_data[8];

  private:
    // Functions
    void configure_pins();
    void configure_sercom();
};

#include "Sercom0_SPI_Slave.inl"

#endif