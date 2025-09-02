#include "Sercom0_SPI_Slave.h"

SERCOM0_SPI_Slave::SERCOM0_SPI_Slave() {
  // Constructor implementation (if needed)
}

/*
  Configure pins as SERCOM0 pins
*/
void SERCOM0_SPI_Slave::configure_pins() {
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
  Configure SERCOM0 as SPI slave
*/
void SERCOM0_SPI_Slave::configure_sercom() {
  /*
    Reset SERCOM0 registers
  */
  // Disable SERCOM0
  SERCOM0->SPI.CTRLA.bit.ENABLE = 0;

  // Wait for disable synchronization
  while (SERCOM0->SPI.SYNCBUSY.bit.ENABLE);
  
  // Software reset SERCOM0
  SERCOM0->SPI.CTRLA.bit.SWRST = 1;
  
  // Wait for reset and synchronization
  while (SERCOM0->SPI.CTRLA.bit.SWRST || SERCOM0->SPI.SYNCBUSY.bit.SWRST);

  /*
    Initialize SERCOM0 clock
  */
  // Enable APBC clock for SERCOM0
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;

  // Set up clock to SERCOM0
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_SERCOM0_CORE) | // Select the SERCOM0 core clock
                      GCLK_CLKCTRL_GEN_GCLK0 |            // Use generic clock generator 0
                      GCLK_CLKCTRL_CLKEN;                 // Enable the clock
  
  // Wait for clock register synchronization
  while (GCLK->STATUS.bit.SYNCBUSY);

  /*
    Set up SERCOM0 SPI
  */
  // Set up SPI control A register
  SERCOM0->SPI.CTRLA.reg =  SERCOM_SPI_CTRLA_MODE_SPI_SLAVE | // Set SPI to slave mode
                            SERCOM_SPI_CTRLA_DOPO(0) |        // Set MISO to PAD0, SCK to PAD1, SS to PAD2
                            SERCOM_SPI_CTRLA_DIPO(3);         // Set MOSI to PAD3

  // Enable receiver
  SERCOM0->SPI.CTRLB.bit.RXEN = 1;

  // Wait for CTRLB register synchronization
  while (SERCOM0->SPI.SYNCBUSY.bit.CTRLB);

  // Set up SERCOM0 interrupts
  // SERCOM0->SPI.INTENSET.reg = SERCOM_SPI_INTENSET_RXC | // Enable receive complete interrupt
  //                             SERCOM_SPI_INTENSET_TXC | // Enable transmit complete interrupt
  //                             SERCOM_SPI_INTENSET_DRE;  // Enable data register empty interrupt
  SERCOM0->SPI.INTENSET.reg = SERCOM_SPI_INTENSET_RXC; // Enable only receive complete interrupt

  // Enable SERCOM0
  SERCOM0->SPI.CTRLA.bit.ENABLE = 1;

  // Wait for enable synchronization
  while (SERCOM0->SPI.SYNCBUSY.bit.ENABLE);
}

/*
  Initialize SPI slave
*/
void SERCOM0_SPI_Slave::init() {
  // Configure SPI pins
  this->configure_pins();

  // Disable SERCOM0 interrupt
  NVIC_DisableIRQ(SERCOM0_IRQn);

  // Clear pending interrupt
  NVIC_ClearPendingIRQ(SERCOM0_IRQn);

  // Set interrupt priority
  NVIC_SetPriority(SERCOM0_IRQn, 0);

  // Initialize SERCOM
  this->configure_sercom();

  // Enable SERCOM0 interrupt
  NVIC_EnableIRQ(SERCOM0_IRQn);
}