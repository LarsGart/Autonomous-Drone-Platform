#ifndef SPISLAVE_H
#define SPISLAVE_H

#include <Arduino.h>
/*
  SPI command info
*/
#define CMD_WR 0x00 // Command write code
#define CMD_RD 0x01 // Command read code

#define NUM_CMDS 5 // Number of commands

#define CMD_MOTOR_SPEEDS 0x03 // Command for motor speeds
#define MOTOR_SPEEDS_BYTE_LENGTH 9 // Number of bytes for motor speeds

class SPISlave {
  public:
    /*
      SPI table format:
      {rd_wr_command, data_size}
      - rd_wr_command: 0x00 for write, 0x01 for read
      - data_size: number of bytes this transaction will transfer
    */
    const uint8_t spi_cmd_info[NUM_CMDS][2] = {
      {CMD_RD, 3},                        // FIRMWARE_VERSION
      {CMD_WR, 1},                        // ESC_ARM_DISARM
      {CMD_WR, 1},                        // CRC_ENABLE_DISABLE
      {CMD_WR, MOTOR_SPEEDS_BYTE_LENGTH}, // MOTOR_SPEEDS
      {CMD_WR, 1}                         // MOTOR_STOP
    };
    
    // Constructor
    SPISlave();

    // Methods
    void init(); // Initialize SPI slave

  private:
    // Functions
    void configure_spi_pins(); // Configure SPI pins
    void initialize_sercom_irq(); // Initialize SERCOM interrupt
    void initialize_sercom(); // Initialize SERCOM
};

#endif