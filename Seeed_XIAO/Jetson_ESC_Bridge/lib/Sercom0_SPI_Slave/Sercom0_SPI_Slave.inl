#include "Sercom0_SPI_Slave.h"

/*
  Handle received data
*/
inline void SERCOM0_SPI_Slave::handle_receive(uint8_t data) {
  static enum {
    WAIT_FOR_START_1,
    WAIT_FOR_START_2,
    WAIT_FOR_CMD,
    WAIT_FOR_DATA
  } state = WAIT_FOR_START_1;

  static uint8_t data_index = 0;
  static uint8_t expected_data_size = 0;

  switch (state) {
    case WAIT_FOR_START_1:
      if (data == (SPI_START_BYTES >> 8)) {
        state = WAIT_FOR_START_2;
      }
      break;

    case WAIT_FOR_START_2:
      if (data == (SPI_START_BYTES & 0xFF)) {
        state = WAIT_FOR_CMD;
      } else {
        state = WAIT_FOR_START_1; // Reset if second start byte doesn't match
      }
      break;

    case WAIT_FOR_CMD:
      this->received_cmd = data;
      // Validate command and get expected data size
      if (this->received_cmd < NUM_CMDS) {
        expected_data_size = spi_cmd_info[this->received_cmd][1];
        data_index = 0;
        state = WAIT_FOR_DATA;
      } else {
        state = WAIT_FOR_START_1; // Invalid command, reset state
      }
      break;

    case WAIT_FOR_DATA:
      this->rd_data[data_index++] = data;
      if (data_index == expected_data_size) {
        this->rd_data_ready = true;
        state = WAIT_FOR_START_1;
      }
      break;
  }
}