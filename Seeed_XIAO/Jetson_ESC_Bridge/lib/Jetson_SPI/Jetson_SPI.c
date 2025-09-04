#include "Jetson_SPI.h"

// Initialize the SERCOM0 SPI slave instance
Jetson_SPI_t jetson_spi = {
  .rd_data_ready = false,
  .received_cmd = 0,
  .rd_data = {0},
  .wr_data_ready = false,
  .wr_data = {0}
};

// ----------------------------------------------------------------
// PRIVATE FUNCTIONS
// ----------------------------------------------------------------
/*
  @name configure_pins
  @brief Configures the pins for SERCOM0 SPI slave operation. This function sets the peripheral multiplexer
  for the SPI pins and sets the appropriate pin functions for MISO, MOSI, SCK, and SS.
*/
static void configure_pins(void) {
  // Enable pin mux
  PORT->Group[PORTA].PINCFG[PIN_PA04].bit.PMUXEN = 1;
  PORT->Group[PORTA].PINCFG[PIN_PA05].bit.PMUXEN = 1;
  PORT->Group[PORTA].PINCFG[PIN_PA06].bit.PMUXEN = 1;
  PORT->Group[PORTA].PINCFG[PIN_PA07].bit.PMUXEN = 1;

  // Select pin mux function
  PORT->Group[PORTA].PMUX[2].bit.PMUXE = MUX_PA04D_SERCOM0_PAD0;
  PORT->Group[PORTA].PMUX[2].bit.PMUXO = MUX_PA05D_SERCOM0_PAD1;
  PORT->Group[PORTA].PMUX[3].bit.PMUXE = MUX_PA06D_SERCOM0_PAD2;
  PORT->Group[PORTA].PMUX[3].bit.PMUXO = MUX_PA07D_SERCOM0_PAD3;
}

/*
  @name configure_sercom
  @brief Configures the SERCOM0 as an SPI slave. This function sets up the SERCOM0 registers, enables the clock,
  and configures the SPI interrupt.
*/
static void configure_sercom(void) {
  /*
    Reset SERCOM0 and wait for synchronization
  */
  SERCOM0->SPI.CTRLA.bit.SWRST = 1;
  while (SERCOM0->SPI.CTRLA.bit.SWRST || SERCOM0->SPI.SYNCBUSY.bit.SWRST);
  /*
    Enable SERCOM0 clocks
  */
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;                // Enable APBC clock for SERCOM0
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_SERCOM0_CORE) | // Select the SERCOM0 core clock
                      GCLK_CLKCTRL_GEN_GCLK0 |            // Use generic clock generator 0
                      GCLK_CLKCTRL_CLKEN;                 // Enable the clock
  while (GCLK->STATUS.bit.SYNCBUSY);
  /*
    Set up SERCOM0 SPI registers
  */
  SERCOM0->SPI.CTRLA.reg =  SERCOM_SPI_CTRLA_MODE_SPI_SLAVE | // Set SPI to slave mode
                            SERCOM_SPI_CTRLA_DOPO(0) |        // Set MISO to PAD0, SCK to PAD1, SS to PAD2
                            SERCOM_SPI_CTRLA_DIPO(3);         // Set MOSI to PAD3

  SERCOM0->SPI.CTRLB.bit.RXEN = 1; // Enable receiver
  while (SERCOM0->SPI.SYNCBUSY.bit.CTRLB);
  /*
    Configure SPI interrupts
  */
  // SERCOM0->SPI.INTENSET.reg = SERCOM_SPI_INTENSET_RXC | // Enable receive complete interrupt
  //                             SERCOM_SPI_INTENSET_TXC | // Enable transmit complete interrupt
  //                             SERCOM_SPI_INTENSET_DRE;  // Enable data register empty interrupt
  SERCOM0->SPI.INTENSET.reg = SERCOM_SPI_INTENSET_RXC; // Receive complete
  /*
    Enable SERCOM0 SPI
  */
  SERCOM0->SPI.CTRLA.bit.ENABLE = 1;
  while (SERCOM0->SPI.SYNCBUSY.bit.ENABLE);
}

// ----------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------
/*
  @name jetson_spi_init
  @brief Initializes the Jetson SPI. This function configures the pins, sets up the SERCOM0 registers,
  and enables the SPI interrupt.
*/
void jetson_spi_init(void) {
  NVIC_DisableIRQ(SERCOM0_IRQn);
  NVIC_ClearPendingIRQ(SERCOM0_IRQn);
  NVIC_SetPriority(SERCOM0_IRQn, 0);

  configure_pins();
  configure_sercom();

  NVIC_EnableIRQ(SERCOM0_IRQn);
}

// ----------------------------------------------------------------
// INTERUPT HANDLER
// ----------------------------------------------------------------
void SERCOM0_Handler(void) {
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

  if (SERCOM0->SPI.INTFLAG.bit.RXC) {
    uint8_t data = SERCOM0->SPI.DATA.reg; // Read data from the RX data buffer to clear the interrupt

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
        jetson_spi.received_cmd = data;
        // Validate command and get expected data size
        if (jetson_spi.received_cmd < NUM_CMDS) {
          expected_data_size = spi_cmd_info[jetson_spi.received_cmd][1];
          data_index = 0;
          state = WAIT_FOR_DATA;
        } else {
          state = WAIT_FOR_START_1; // Invalid command, reset state
        }
        break;

      case WAIT_FOR_DATA:
        jetson_spi.rd_data[data_index++] = data;
        if (data_index == expected_data_size) {
          jetson_spi.rd_data_ready = true;
          state = WAIT_FOR_START_1;
        }
        break;
    }
  }
}